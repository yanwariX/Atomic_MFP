// Atomic Motion + MultiFunPlayer
#include "M5Unified.h"
#include "M5AtomicMotion.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <ESPmDNS.h>

#define FIRMWARE_ID "ATOMIC_MFP_v1.0"
#define TCODE_VER   "TCode v0.3"

#define CHANNELS        10
#define STOP_RANGE_MIN  4990   // 停止下限
#define STOP_RANGE_MAX  5010   // 停止上限

//============================================================
//  WIFI接続設定 (WIFIを使用する場合はSSIDとPASSWORDを書き換えて下さい)
//============================================================
#define WIFI_SSID       "SSID"
#define WIFI_PASSWORD   "PASSWORD"
#define WIFI_HOSTNAME  "atomicmotion"   // MFP で指名するホスト名
#define UDP_PORT        8889   // MFP で指定する UDP ポート番号
#define USE_UDP_INPUT   1      // 0 にするとwifi設定無効化

//============================================================
//  出力デバイス有効/無効(1/0)
//============================================================
#define ENABLE_DC_MOTORS  1   // DCモーター
#define ENABLE_RC_SERVOS  1   // 180°RCサーボ

//============================================================
//  DCモーター速度設定
//============================================================
#define MAX_MOTOR_SPEED  127  // Atomic Motion の最大PWM（基本的には固定）
#define MIN_MOTOR_SPEED   50  // 動き出しの最低PWM（低速が速すぎる→下げる 低速で動かない→上げる）
#define SPEED_SAT_PCT    100.0f // funscript何%で最速に達するか(全体的に速くしたい場合下げる)

//============================================================
//  RC180°サーボ設定
//============================================================
#define RC_SERVO_CENTER_ANGLE   90    // 中心角度（通常は90）
#define RC_SERVO_SWING_DEG      20.0f // 可動角度（中心±○°）
#define RC_TRIM_0       0             // S1端子の中心位置の微調整（個体差で中心がずれる場合に調整）
#define RC_TRIM_1       0             // S3端子の中心位置の微調整（個体差で中心がずれる場合に調整）

//============================================================
//  Axisチャンネル割り当て(文字はL/R/V/A 数字は0～9)
//============================================================
#define AXIS_DC0  "A3"
#define AXIS_DC1  "A4"
#define AXIS_RC0  "A5"
#define AXIS_RC1  "A6"

//============================================================
//  Atomic Motion S端子割り当て(RC=180°サーボ)
//============================================================
#define AM_SERVO(ch) ((ch) - 1)
#define SERVO_CH_RC0 1
#define SERVO_CH_RC1 3

//============================================================
//  スムージング設定（UDP時のカクつき対策）
//============================================================
#define CONTROL_UPDATE_HZ        100      // 出力更新周波数(Hz). 50～200 くらい推奨
#define UDP_POLL_BUDGET_US       2000     // loop内でUDP受信処理に使う最大時間(µs)
#define SERIAL_POLL_BUDGET_BYTES 256      // loop内でシリアル受信処理する最大バイト数
#define RC_SERVO_SMOOTH_ALPHA    0.35f    // 0..1 大きいほど追従が速い(滑らかさは下がる) / 1.0で無効
#define I2C_CLOCK_HZ             400000   // Atomic Motion I2Cクロック

//============================================================
//  ATOM Lite LED設定
//============================================================
#define USE_STATUS_LED   1
#define ATOM_LED_PIN     27
#define ATOM_LED_COUNT   1
#define LED_BRIGHTNESS   1   // 0でLEDオフ

//============================================================
//  グローバルオブジェクト
//============================================================

M5AtomicMotion AtomicMotion;

#if USE_UDP_INPUT
WiFiUDP Udp;
uint8_t udpBuffer[512];
bool g_udpEnabled = false;   
#endif

//============================================================
//  状態LED
//============================================================

#if USE_STATUS_LED
Adafruit_NeoPixel StatusLED(ATOM_LED_COUNT, ATOM_LED_PIN, NEO_GRB + NEO_KHZ800);

static uint32_t makeColor(uint8_t r, uint8_t g, uint8_t b) {
  return StatusLED.Color(r, g, b);
}

static void showStatusColor(uint32_t color) {
  static uint32_t lastColor = 0;
  if (color == lastColor) return;
  lastColor = color;

  StatusLED.setBrightness(LED_BRIGHTNESS);
  StatusLED.setPixelColor(0, color);
  StatusLED.show();
}

static void initStatusLed() {
  StatusLED.begin();
  StatusLED.clear();
  StatusLED.show();

  // 起動時は未接続扱い(緑)
  showStatusColor(makeColor(0, 255, 0));
}

static void updateStatusLed(bool udpOk) {
  const uint32_t green  = makeColor(0, 255, 0);
  const uint32_t purple = makeColor(255, 0, 255);
  showStatusColor(udpOk ? purple : green);
}
#endif  // USE_STATUS_LED

//============================================================
//  Axis / TCode
//============================================================

class Axis {
public:
  Axis() : currentValue(5000) {}
  void Set(int value) { currentValue = constrain(value, 0, 9999); }
  int  Get() { return currentValue; }
private:
  int currentValue;
};

class TCode {
public:
  TCode(String firmware, String tcode) {
    firmwareID = firmware;
    tcodeID    = tcode;
    bufferString = "";
    for (int i = 0; i < CHANNELS; ++i) {
      axes[i] = Axis();
    }
  }

  void AxisInput(String axis, int value) {
    int channel = axis.substring(1).toInt();
    if (channel >= 0 && channel < CHANNELS) {
      axes[channel].Set(value);
    }
  }

  int AxisRead(String axis) {
    int channel = axis.substring(1).toInt();
    if (channel >= 0 && channel < CHANNELS) {
      return axes[channel].Get();
    }
    return 5000;
  }

  void SerialInput(uint8_t inByte) {
    char inChar = (char)inByte;

    if (inChar == '\n') {
      bufferString.trim();
      if (bufferString.length() > 0) {
        executeString(bufferString);
      }
      bufferString = "";

    } else {
      bufferString += inChar;
      if (bufferString.length() > 128) {
        bufferString = "";
      }
    }
  }

private:
  String firmwareID, tcodeID;
  String bufferString;
  Axis   axes[CHANNELS];

  void executeString(String buffer) {
    int idx;
    while ((idx = buffer.indexOf(' ')) > 0) {
      readCmd(buffer.substring(0, idx));
      buffer = buffer.substring(idx + 1);
      buffer.trim();
    }
    if (buffer.length() > 0) {
      readCmd(buffer);
    }
  }

  void readCmd(String cmd) {
    cmd.trim();
    if (cmd.length() < 2) return;

    String head = cmd.substring(0, 1);

    if (head == "D") {
      // デバイス情報(D0/D1)
      if (cmd == "D0") {
        Serial.println("D0 " + firmwareID);
      } else if (cmd == "D1") {
        Serial.println("D1 " + tcodeID);
      }
      return;
    }

    // Axisコマンド例: A0xxxx
    if (head == "A") {
      if (cmd.length() >= 6) {
        String axis = cmd.substring(0, 2);
        int value = cmd.substring(2).toInt();
        AxisInput(axis, value);
      }
    }
  }
};

TCode tcode(FIRMWARE_ID, TCODE_VER);

//============================================================
//  受信バイトをTCodeへ渡す
//============================================================

static inline void feedTCodeByte(uint8_t b) {
  tcode.SerialInput(b);
}

//============================================================
//  共通: 速度計算
//============================================================

// 停止レンジ判定
static inline bool isStopRange(int tcodeVal) {
  return (tcodeVal >= STOP_RANGE_MIN && tcodeVal <= STOP_RANGE_MAX);
}

// TCode(0..9999) → 速度[%] (-100..+100). 停止レンジは0%
static float getSpeedPercentSigned(int tcodeVal) {
  tcodeVal = constrain(tcodeVal, 0, 9999);

  if (isStopRange(tcodeVal)) {
    return 0.0f;
  }

  if (tcodeVal < STOP_RANGE_MIN) {
    // 0..STOP_RANGE_MIN-1 → -100..0
    return -100.0f * (STOP_RANGE_MIN - tcodeVal) / STOP_RANGE_MIN;
  }

  // STOP_RANGE_MAX+1..9999 → 0..+100
  return 100.0f * (tcodeVal - STOP_RANGE_MAX) / (10000.0f - STOP_RANGE_MAX);
}

//============================================================
//  DCモーター制御
//============================================================

// 0..100% → MotorSpeed(0..127)
static int rpmPercentToMotorSpeed(float rpmAbsPct) {
  if (rpmAbsPct <= 0.0f) {
    return 0; // 停止
  }

  const int minSpeed = constrain(MIN_MOTOR_SPEED, 0, MAX_MOTOR_SPEED);

  if (rpmAbsPct >= SPEED_SAT_PCT) {
    return MAX_MOTOR_SPEED;
  }

  float ratio  = rpmAbsPct / SPEED_SAT_PCT; // 0..1
  float scaled = minSpeed + ratio * (MAX_MOTOR_SPEED - minSpeed);
  int speed = (int)(scaled + 0.5f); // 四捨五入

  if (speed < minSpeed) speed = minSpeed;

  return constrain(speed, 0, MAX_MOTOR_SPEED);
}

// TCodeでモーター制御
static void controlMotor(uint8_t motorChannel, int tcodeValue) {
  float pctSigned = getSpeedPercentSigned(tcodeValue); // -100..100
  float absPct    = (pctSigned >= 0.0f) ? pctSigned : -pctSigned; // 0..100
  int   speedAbs  = rpmPercentToMotorSpeed(absPct);    // 0..127

  int speed = (pctSigned >= 0.0f) ? speedAbs : -speedAbs;
  AtomicMotion.setMotorSpeed(motorChannel, speed);
}

//============================================================
//  180°サーボ制御
//============================================================
#if ENABLE_RC_SERVOS

static void controlPosRcServo(uint8_t servoChannel, int tcodeValue, int trim) {
  // 停止レンジは0%、左右は-100..+100
  const float pctSigned = getSpeedPercentSigned(tcodeValue); // -100..+100

  const float center = (float)RC_SERVO_CENTER_ANGLE + (float)trim;
  const float offset = (pctSigned / 100.0f) * (float)RC_SERVO_SWING_DEG;

  float angle = center + offset;
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;

  AtomicMotion.setServoAngle(AM_SERVO(servoChannel), (int)(angle + 0.5f));
}

#endif  // ENABLE_RC_SERVOS

//============================================================
//  setup / loop
//============================================================

//  UDP入力時のカクつき対策
static int g_lastMotorSpeed[2] = { 9999, 9999 };               // 0..1
static int g_lastServoAngle[5] = { 9999, 9999, 9999, 9999, 9999 }; // index 1..4
static float g_servoAngleFilt[5] = { NAN, NAN, NAN, NAN, NAN };     // index 1..4

static inline void applyMotorSpeedCached(uint8_t motorChannel, int speed) {
  speed = constrain(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  if (motorChannel > 1) return;
  if (g_lastMotorSpeed[motorChannel] == speed) return;
  g_lastMotorSpeed[motorChannel] = speed;
  AtomicMotion.setMotorSpeed(motorChannel, speed);
}

static inline void applyServoAngleCached(uint8_t servoChannel, int angle) {
  angle = constrain(angle, 0, 180);
  if (servoChannel < 1 || servoChannel > 4) return;
  if (g_lastServoAngle[servoChannel] == angle) return;
  g_lastServoAngle[servoChannel] = angle;
  AtomicMotion.setServoAngle(AM_SERVO(servoChannel), angle);
}

// TCode(0..9999) -> MotorSpeed(-127..+127)
static inline int calcMotorSpeed(int tcodeValue) {
  float pctSigned = getSpeedPercentSigned(tcodeValue); // -100..100
  float absPct    = (pctSigned >= 0.0f) ? pctSigned : -pctSigned; // 0..100
  int   speedAbs  = rpmPercentToMotorSpeed(absPct);    // 0..127
  return (pctSigned >= 0.0f) ? speedAbs : -speedAbs;
}

// TCode(0..9999) -> RCサーボ角度(0..180)
static inline int calcRCServoAngle(int tcodeValue, int trim) {
  const float pctSigned = getSpeedPercentSigned(tcodeValue); // -100..+100
  const float center = (float)RC_SERVO_CENTER_ANGLE + (float)trim;
  const float offset = (pctSigned / 100.0f) * (float)RC_SERVO_SWING_DEG;

  float angle = center + offset;
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;

  return (int)(angle + 0.5f);
}

// 一次遅れ (alpha=1.0でスムージング無し)
static inline int smoothServoAngle(uint8_t servoChannel, int targetAngle, float alpha) {
  alpha = constrain(alpha, 0.0f, 1.0f);
  if (servoChannel < 1 || servoChannel > 4) return targetAngle;

  float &f = g_servoAngleFilt[servoChannel];
  if (isnan(f)) f = (float)targetAngle;
  f += alpha * ((float)targetAngle - f);

  int angle = (int)(f + 0.5f);
  return constrain(angle, 0, 180);
}

#if USE_UDP_INPUT
static void pollUdpWithBudget(uint32_t budgetUs) {
  if (!g_udpEnabled) return;
  const uint32_t startUs = micros();

  int packetSize = Udp.parsePacket();
  while (packetSize > 0) {
    int len = Udp.read(udpBuffer, sizeof(udpBuffer));
    for (int i = 0; i < len; ++i) {
      feedTCodeByte(udpBuffer[i]);
    }

    if ((uint32_t)(micros() - startUs) >= budgetUs) {
      break; // 受信を取り切らずに次loopへ (制御周期優先)
    }
    packetSize = Udp.parsePacket();
  }
}
#endif

static void updateOutputsFixedRate() {
#if ENABLE_DC_MOTORS
  applyMotorSpeedCached(0, calcMotorSpeed(tcode.AxisRead(AXIS_DC0)));
  applyMotorSpeedCached(1, calcMotorSpeed(tcode.AxisRead(AXIS_DC1)));
#endif

#if ENABLE_RC_SERVOS
  int rc0 = calcRCServoAngle(tcode.AxisRead(AXIS_RC0), RC_TRIM_0);
  int rc1 = calcRCServoAngle(tcode.AxisRead(AXIS_RC1), RC_TRIM_1);
  applyServoAngleCached(SERVO_CH_RC0, smoothServoAngle(SERVO_CH_RC0, rc0, RC_SERVO_SMOOTH_ALPHA));
  applyServoAngleCached(SERVO_CH_RC1, smoothServoAngle(SERVO_CH_RC1, rc1, RC_SERVO_SMOOTH_ALPHA));
#endif
}

void setup() {
  Serial.begin(115200);

  auto cfg = M5.config();
  M5.begin(cfg);

#if USE_STATUS_LED
  initStatusLed();
#endif

  // ボード種別からI2Cピンを決定
  uint8_t sda = 0;
  uint8_t scl = 0;
  m5::board_t board = M5.getBoard();

  if (board == m5::board_t::board_M5AtomLite ||
      board == m5::board_t::board_M5AtomMatrix ||
      board == m5::board_t::board_M5AtomEcho) {
    sda = 25;
    scl = 21;
  } else if (board == m5::board_t::board_M5AtomS3   ||
             board == m5::board_t::board_M5AtomS3R  ||
             board == m5::board_t::board_M5AtomS3Lite ||
             board == m5::board_t::board_M5AtomS3RExt ||
             board == m5::board_t::board_M5AtomS3RCam) {
    sda = 1;
    scl = 2;
  } else {

    sda = 25;
    scl = 21;
  }

  // Atomic Motion初期化
  while (!AtomicMotion.begin(&Wire, M5_ATOMIC_MOTION_I2C_ADDR, sda, scl, I2C_CLOCK_HZ)) {
    Serial.println("Atomic Motion begin failed");
    delay(500);
  }

#if ENABLE_DC_MOTORS
  // 念のため停止
  AtomicMotion.setMotorSpeed(0, 0);
  AtomicMotion.setMotorSpeed(1, 0);
#endif

#if ENABLE_RC_SERVOS
  // 180°サーボを中心へ
  AtomicMotion.setServoAngle(AM_SERVO(SERVO_CH_RC0), RC_SERVO_CENTER_ANGLE + RC_TRIM_0);
  AtomicMotion.setServoAngle(AM_SERVO(SERVO_CH_RC1), RC_SERVO_CENTER_ANGLE + RC_TRIM_1);
#endif

#if USE_UDP_INPUT
  // Wi‑Fi接続
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi connecting");

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    g_udpEnabled = true;

    // 省電力モードで制御周期が乱れやすいので無効化
    WiFi.setSleep(false);
    // mDNS: atomicmotion.local
    if (!MDNS.begin(WIFI_HOSTNAME)) {
      Serial.println("mDNS begin failed");
    } else {
      MDNS.addService("tcode", "udp", UDP_PORT);  
      Serial.print("mDNS: ");
      Serial.print(WIFI_HOSTNAME);
      Serial.println(".local");
    }

    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());

    Udp.begin(UDP_PORT);
    Serial.print("UDP port: ");
    Serial.println(UDP_PORT);
  } else {
    g_udpEnabled = false;
    Serial.println("WiFi connect failed, UDP disabled");
  }
#endif

#if USE_STATUS_LED
#if USE_UDP_INPUT
  updateStatusLed(g_udpEnabled);
#else
  updateStatusLed(false);
#endif
#endif

  Serial.println("Atomic MFP Ready!");
}

void loop() {
  M5.update();

  // シリアル受信(TCode) 
  int serialCount = 0;
  while (Serial.available() && serialCount < SERIAL_POLL_BUDGET_BYTES) {
    feedTCodeByte((uint8_t)Serial.read());
    ++serialCount;
  }

#if USE_UDP_INPUT
  // UDP受信(TCode) 
  if (g_udpEnabled) {
    pollUdpWithBudget(UDP_POLL_BUDGET_US);
  }
#endif

  // 出力更新は一定周期で実行
  static uint32_t lastUs = 0;
  const uint32_t periodUs = 1000000UL / (uint32_t)CONTROL_UPDATE_HZ;
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastUs) >= periodUs) {
    lastUs = nowUs;
    updateOutputsFixedRate();
  }

#if USE_STATUS_LED
#if USE_UDP_INPUT
  updateStatusLed(g_udpEnabled);
#else
  updateStatusLed(false);
#endif
#endif
}
