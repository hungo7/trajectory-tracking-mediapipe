#include <Arduino.h> 
#include <WiFi.h> 
#include <WiFiUdp.h> 
#include <PID_v1.h> 
#include <cmath> 
#include <Wire.h> 
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h> 

// ── CẤU HÌNH CHÂN PIN ─────────────────────────────────────────────────────── 
#define PIN_AIN1 27 
#define PIN_AIN2 26 
#define PIN_PWMA 25 
#define PIN_BIN1 14 
#define PIN_BIN2 12 
#define PIN_PWMB 13 
#define PIN_STBY 16 
#define ENC_L_A 34 
#define ENC_L_B 35 
#define ENC_R_A 23 
#define ENC_R_B 17 
#define I2C_SDA 21
#define I2C_SCL 22

// ── THÔNG SỐ CƠ KHÍ & ĐỊNH VỊ ─────────────────────────────────────────────── 
constexpr float IMU_SIGN        = 0.8585f;     // Hướng quay chuẩn của IMU Trục Z
constexpr float WHEEL_BASE      = 0.2000f; 
constexpr float WHEEL_RADIUS    = 0.0347f; 
constexpr float PPR             = 11.0f; 
constexpr float GEAR_RATIO      = 21.3f; 
constexpr float TICK_PER_REV    = PPR * GEAR_RATIO; 
constexpr float METERS_PER_TICK = (2.0f * PI * WHEEL_RADIUS) / TICK_PER_REV; 

// ── THÔNG SỐ ĐIỀU KHIỂN NÂNG CAO ──────────────────────────────────────────── 
constexpr float alpha = 0.3f;       // Hệ số bộ lọc thông thấp vận tốc
constexpr float Kv_R = 418.0f, Kstatic_R = 30.0f;   
constexpr float Kv_L = 400.0f, Kstatic_L = 55.0f;

// ── KHỞI TẠO ĐỐI TƯỢNG ────────────────────────────────────────────────────── 
WiFiUDP udp; 
Adafruit_MPU6050 mpu; 
IPAddress clientIp; 

// ── PID DẢI HẸP ───────────────────────────────────────────────────────────── 
double setL = 0, inL = 0, outL = 0; 
double setR = 0, inR = 0, outR = 0; 
double filteredInL = 0.0, filteredInR = 0.0;
PID pidL(&inL, &outL, &setL, 12.0, 1.0, 0.1, DIRECT); 
PID pidR(&inR, &outR, &setR, 12.0, 1.0, 0.1, DIRECT); 

// ── BIẾN TRẠNG THÁI TOÀN CỤC ──────────────────────────────────────────────── 
volatile long countL = 0, countR = 0; 
volatile long globalCountL = 0, globalCountR = 0; 

long lastGlobalL = 0, lastGlobalR = 0; 
float robot_x = 0, robot_y = 0, robot_theta = 0; 
float gyro_bias_z = 0; 
float v_robot = 0; 
bool imu_ready = false; 

unsigned long lastCtrl = 0; 
unsigned long lastOdom = 0, lastPkt = 0; 

// ── NGẮT ENCODER ──────────────────────────────────────────────────────────── 
void IRAM_ATTR isrL() { 
  if (!digitalRead(ENC_L_A)) return; 
  (digitalRead(ENC_L_B) ? (countL--, globalCountL--) : (countL++, globalCountL++));
} 
void IRAM_ATTR isrR() { 
  if (!digitalRead(ENC_R_A)) return; 
  (digitalRead(ENC_R_B) ? (countR++, globalCountR++) : (countR--, globalCountR--));
} 

// ── ĐIỀU KHIỂN CƠ SỞ MOTOR ────────────────────────────────────────────────── 
void setMotor(bool isRight, float pwm_output) { 
  int val = constrain((int)pwm_output, -255, 255); 
  bool fwd = (val >= 0); 
  if (!isRight) { 
    digitalWrite(PIN_BIN1, !fwd); 
    digitalWrite(PIN_BIN2, fwd); 
    ledcWrite(1, abs(val)); 
  } else { 
    digitalWrite(PIN_AIN1, fwd);  
    digitalWrite(PIN_AIN2, !fwd); 
    ledcWrite(0, abs(val)); 
  } 
} 

void stopMotors() { 
  ledcWrite(0, 0); ledcWrite(1, 0); 
  inL = 0; outL = 0; setL = 0; 
  inR = 0; outR = 0; setR = 0; 
  filteredInL = 0; filteredInR = 0;
  pidL.SetMode(MANUAL); pidR.SetMode(MANUAL); 
  v_robot = 0; 
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); 
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW); 
} 

// ── HÀM ĐỊNH VỊ ODOMETRY + BỘ LỌC BÙ 95/5 (TÁI CẤU TRÚC LÀM GỌN) ─────────────── 
void updateOdometry(float dt) {
  noInterrupts(); 
  long dL = countL; countL = 0;
  long dR = countR; countR = 0; 
  long currentGlobalL = globalCountL;
  long currentGlobalR = globalCountR;
  interrupts(); 

  // 1. Lọc thông thấp tính vận tốc hiển thị cho PID
  float rawInL = (dL * METERS_PER_TICK) / dt; 
  float rawInR = (dR * METERS_PER_TICK) / dt; 
  filteredInL = (alpha * rawInL) + ((1.0f - alpha) * filteredInL); 
  filteredInR = (alpha * rawInR) + ((1.0f - alpha) * filteredInR); 
  inL = filteredInL; 
  inR = filteredInR; 
  v_robot = (inL + inR) / 2.0f;

  // 2. Tính toán độ dịch chuyển dựa trên hiệu xung tích lũy tuyệt đối
  long deltaTickL = currentGlobalL - lastGlobalL;
  long deltaTickR = currentGlobalR - lastGlobalR;
  lastGlobalL = currentGlobalL;
  lastGlobalR = currentGlobalR;

  float ds_L = deltaTickL * METERS_PER_TICK;
  float ds_R = deltaTickR * METERS_PER_TICK;
  float delta_s = (ds_L + ds_R) / 2.0f;

  // 3. Thực thi Bộ lọc bù (Complementary Filter) trộn 95% IMU và 5% Encoder
  float delta_theta_enc = (ds_R - ds_L) / WHEEL_BASE;
  float delta_theta_imu = 0.0f;

  if (imu_ready) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      delta_theta_imu = IMU_SIGN * (g.gyro.z - gyro_bias_z) * dt;
    } else {
      delta_theta_imu = delta_theta_enc; // Fallback nếu lỗi đọc Bus I2C tức thời
    }
  } else {
    delta_theta_imu = delta_theta_enc;   // Fallback nếu không tìm thấy phần cứng IMU
  }

  // Kết hợp trộn tỉ lệ cấu hình toán học
  float delta_theta = (0.95f * delta_theta_imu) + (0.05f * delta_theta_enc);

  // 4. Tích phân động học cập nhật trạng thái robot
  float theta_avg = robot_theta + (delta_theta / 2.0f);
  robot_theta += delta_theta;
  robot_theta = atan2(sin(robot_theta), cos(robot_theta)); // Chuẩn hóa góc [-PI, PI]

  robot_x += delta_s * cos(theta_avg);
  robot_y += delta_s * sin(theta_avg);
}

// ── HÀM THỰC THI ĐIỀU KHIỂN MOTOR (TÁI CẤU TRÚC LÀM GỌN) ───────────────────── 
void controlMotors() {
  if (pidL.GetMode() != AUTOMATIC) return;

  pidL.Compute(); 
  pidR.Compute(); 
  
  // Tính toán tầng năng lượng Feedforward bánh Trái
  float pwm_ff_L = (setL * Kv_L); 
  if (setL > 0)       pwm_ff_L += Kstatic_L;
  else if (setL < 0)  pwm_ff_L -= Kstatic_L;
  else                pwm_ff_L = 0;
  
  // Tính toán tầng năng lượng Feedforward bánh Phải
  float pwm_ff_R = (setR * Kv_R); 
  if (setR > 0)       pwm_ff_R += Kstatic_R;
  else if (setR < 0)  pwm_ff_R -= Kstatic_R;
  else                pwm_ff_R = 0;

  // Xuất PWM tổng hợp (Feedforward + PID)
  setMotor(false, pwm_ff_L + outL); 
  setMotor(true, pwm_ff_R + outR); 
}

// ── SETUP HỆ THỐNG ────────────────────────────────────────────────────────── 
void setup() { 
  Serial.begin(115200); 
  Wire.begin(I2C_SDA, I2C_SCL); 
  Wire.setClock(400000); 
  
  if (mpu.begin()) { 
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG); 
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 

    Serial.println("Đang Calib IMU...");
    float sum_z = 0; 
    for(int i = 0; i < 200; i++) { 
      sensors_event_t a, g, temp; 
      mpu.getEvent(&a, &g, &temp); 
      sum_z += g.gyro.z; 
      delay(5); 
    } 
    gyro_bias_z = sum_z / 200.0f; 
    imu_ready = true; 
    Serial.println("Calib xong!");
  } else { 
    Serial.println("Không tìm thấy MPU6050!");
    imu_ready = false; 
  } 
  
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); 
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT); 
  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, HIGH); 
  
  ledcSetup(0, 20000, 8); ledcAttachPin(PIN_PWMA, 0); 
  ledcSetup(1, 20000, 8); ledcAttachPin(PIN_PWMB, 1); 
  
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT); 
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT); 
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING); 
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING); 
  
  pidL.SetSampleTime(20); pidL.SetOutputLimits(-120, 120); 
  pidR.SetSampleTime(20); pidR.SetOutputLimits(-120, 120); 
  
  stopMotors(); 
  WiFi.softAP("RobotAP", "12345678"); 
  udp.begin(1234); 
  lastPkt = lastOdom = millis(); 
  lastCtrl = micros(); 
} 

// ── VÒNG LẶP CHÍNH SIÊU TỐI GIẢN ───────────────────────────────────────────── 
void loop() { 
  // 1. Tiếp nhận cấu trúc gói tin điều khiển mạng UDP
  int packetSize = udp.parsePacket(); 
  if (packetSize > 0) { 
    char rxBuf[64]; 
    int len = udp.read(rxBuf, 63); 
    rxBuf[len] = '\0'; 
    float v_in, w_in; 
    if (sscanf(rxBuf, "%f,%f", &v_in, &w_in) == 2) { 
      clientIp = udp.remoteIP(); 
      lastPkt = millis(); 
      if (v_in == 0.0f && w_in == 0.0f) { 
        stopMotors(); 
      } else { 
        setL = v_in - (w_in * WHEEL_BASE / 2.0); 
        setR = v_in + (w_in * WHEEL_BASE / 2.0); 
        if (pidL.GetMode() == MANUAL) { 
          pidL.SetMode(AUTOMATIC); pidR.SetMode(AUTOMATIC); 
        } 
      } 
    } 
    else if (strncmp(rxBuf, "RESET", 5) == 0) { 
      stopMotors(); 
      robot_x = 0.0f; robot_y = 0.0f; robot_theta = 0.0f; 
      noInterrupts(); globalCountL = 0; globalCountR = 0; interrupts();
      lastGlobalL = 0; lastGlobalR = 0; 
      clientIp = udp.remoteIP(); lastPkt = millis(); 
    } 
  } 
  
  // Kiểm tra Watchdog ngắt kết nối an toàn 
  if (millis() - lastPkt > 500 && pidL.GetMode() == AUTOMATIC) { 
    stopMotors(); 
  } 
  
  // 2. Định kỳ chu kỳ thời gian thực (20ms) cho Định vị & Điều khiển
  unsigned long now_us = micros(); 
  if (now_us - lastCtrl >= 20000) { 
    float dt = (now_us - lastCtrl) / 1000000.0f; 
    lastCtrl = now_us; 
    
    updateOdometry(dt);  // Định vị, tính toán bộ lọc bù hình học
    controlMotors();     // Điều khiển động cơ kết hợp
    
    // ── THÊM ĐOẠN NÀY ĐỂ TEST VẬN TỐC ────────────────────────────────────────
    // Xuất dữ liệu để vẽ đồ thị so sánh trên Serial Plotter / Teleplot
    Serial.printf(">V_set_L:%.3f\n", setL);
    Serial.printf(">V_real_L:%.3f\n", inL);
    Serial.printf(">V_set_R:%.3f\n", setR);
    Serial.printf(">V_real_R:%.3f\n", inR);
    // ─────────────────────────────────────────────────────────────────────────

    // Serial Monitor liên tục hỗ trợ Debug kể cả khi dừng xe
    Serial.printf("X:%.3f | Y:%.3f | Theta(Rad):%.4f\n", robot_x, robot_y, robot_theta); 
  } 
  
  // 3. Định kỳ phát gói tin phản hồi vị trí về máy tính (50ms)
  if (millis() - lastOdom >= 50) { 
    lastOdom = millis(); 
    if (clientIp[0] != 0) { 
      udp.beginPacket(clientIp, 1235); 
      udp.printf("%.4f,%.4f,%.4f", robot_x, robot_y, robot_theta); 
      udp.endPacket(); 
    } 
  } 
}