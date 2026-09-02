#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "wifi_config.h"


#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ---------------------------------------------------------------------------
// Transport: two esp_http_server instances
//
//   Port 80 (CONTROL_PORT) - GET /control?pan=1&tilt=-1&mood=happy
//   Port 81 (STREAM_PORT)  - GET /stream, multipart MJPEG
//
// A single httpd instance serves one request at a time, 
// and the stream handler never returns while a viewer is
// watching. A single server would therefore never answer a control request.
// The reference splits them for exactly this reason, and gives each its own
// ctrl_port as well.
// ---------------------------------------------------------------------------
httpd_handle_t streamHttpd = NULL;
httpd_handle_t controlHttpd = NULL;

Servo panServo;
Servo tiltServo;
int currentPan = SERVO_PAN_CENTER_ANGLE;
int currentTilt = SERVO_TILT_CENTER_ANGLE;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ---------------------------------------------------------------------------
// Servo output
// ---------------------------------------------------------------------------
volatile int targetPan = SERVO_PAN_CENTER_ANGLE;
volatile int targetTilt = SERVO_TILT_CENTER_ANGLE;

// Last commanded position, kept in RTC memory so it survives a reset.

#define SERVO_POS_MAGIC 0x5E70A1u
RTC_NOINIT_ATTR uint32_t servoPosMagic;
RTC_NOINIT_ATTR int lastPanPos;
RTC_NOINIT_ATTR int lastTiltPos;

// Relative commands accumulate against the TARGET, never against the actual position.
void setPanTarget(int angle) {
  targetPan = constrain(angle, SERVO_PAN_MIN_ANGLE, SERVO_PAN_MAX_ANGLE);
}

void setTiltTarget(int angle) {
  targetTilt = constrain(angle, SERVO_TILT_MIN_ANGLE, SERVO_TILT_MAX_ANGLE);
}

void writePan(int angle) {
  currentPan = angle;
  lastPanPos = angle; // remembered across a reset - see SERVO_POS_MAGIC above
  panServo.write(SERVO_PAN_INVERT ? (180 - angle) : angle);
}

void writeTilt(int angle) {
  currentTilt = angle;
  lastTiltPos = angle;
  tiltServo.write(SERVO_TILT_INVERT ? (180 - angle) : angle);
}

// Walks each axis toward its target a small step at a time. See the slewrate notes in wifi_config.h
void servoTask(void *pvParameters) {
  bool panGoesFirst = true;

  for (;;) {
    int wantPan = targetPan;
    int wantTilt = targetTilt;
    bool panNeeds = (currentPan != wantPan);
    bool tiltNeeds = (currentTilt != wantTilt);

    if (panNeeds && tiltNeeds) {
      if (panGoesFirst) {
        tiltNeeds = false;
      } else {
        panNeeds = false;
      }
      panGoesFirst = !panGoesFirst;
    }

    if (panNeeds) {
      int delta = constrain(wantPan - currentPan,
                            -SERVO_SLEW_DEG_PER_STEP, SERVO_SLEW_DEG_PER_STEP);
      writePan(currentPan + delta);
    }
    if (tiltNeeds) {
      int delta = constrain(wantTilt - currentTilt,
                            -SERVO_SLEW_DEG_PER_STEP, SERVO_SLEW_DEG_PER_STEP);
      writeTilt(currentTilt + delta);
    }

    vTaskDelay(pdMS_TO_TICKS(SERVO_SLEW_TICK_MS));
  }
}

// ---------------------------------------------------------------------------
// Mood-driven animated eyes (in the spirit of playfultechnology/esp32-eyes).
//
// Runs entirely inside eyesTask on its own core.
// ---------------------------------------------------------------------------
enum Mood {
  MOOD_NEUTRAL,
  MOOD_HAPPY,
  MOOD_CURIOUS,
  MOOD_ANGRY,
  MOOD_SLEEPY,
  MOOD_SURPRISED,
  MOOD_LOVE,
  MOOD_SAD,
  MOOD_SUSPICIOUS,
  MOOD_DIZZY,
  MOOD_WINK,
  MOOD_BORED,
  MOOD_EXCITED,
  MOOD_COUNT // keep last - number of moods, used for range checks
};

volatile int requestedMood = MOOD_NEUTRAL;
volatile int gazeHintX = 0; 
volatile int gazeHintY = 0;
volatile unsigned long lastCommandMillis = 0;

const unsigned long TRACKING_TIMEOUT_MS = 1500;
const unsigned long SLEEPY_TIMEOUT_MS = 60000; 

const int screenWidth = 128;
const int screenHeight = 64;
const int eyeWidth = 30;
const int eyeHeight = 40;
const int eyeGap = 16;
const int eyeCenterY = screenHeight / 2;
const int leftEyeCX = screenWidth / 2 - (eyeGap / 2 + eyeWidth / 2);
const int rightEyeCX = screenWidth / 2 + (eyeGap / 2 + eyeWidth / 2);


const char *const MOOD_NAMES[] = {
  "neutral", "happy", "curious", "angry", "sleepy", "surprised",
  "love", "sad", "suspicious", "dizzy", "wink", "bored", "excited"
};

static_assert(sizeof(MOOD_NAMES) / sizeof(MOOD_NAMES[0]) == MOOD_COUNT,
              "MOOD_NAMES must list every Mood, in enum order");

int moodFromName(const char *name) {
  for (int i = 0; i < MOOD_COUNT; i++) {
    if (!strcmp(name, MOOD_NAMES[i])) return i;
  }
  return MOOD_NEUTRAL; 
}

struct MoodStyle {
  float openScale;
  float widthScale;
  int yOffset;
  int blinkMinMs;
  int blinkMaxMs;
};

MoodStyle styleFor(Mood m) {
  switch (m) {
    case MOOD_HAPPY:      return {0.85f, 1.00f,  2, 2500, 5000};
    case MOOD_CURIOUS:    return {1.00f, 1.00f, -2, 1800, 3800};
    case MOOD_ANGRY:      return {0.75f, 1.00f,  0, 3000, 6000};
    case MOOD_SLEEPY:     return {0.30f, 1.00f,  6, 4000, 8000};
    case MOOD_SURPRISED:  return {1.15f, 0.85f, -2, 3500, 7000};
    // Hearts are drawn as a shape, so the lid opening just controls size.
    case MOOD_LOVE:       return {1.05f, 1.00f,  0, 2800, 5200};
    case MOOD_SAD:        return {0.80f, 1.00f,  3, 3200, 6000};
    // Narrow AND wide, so it does not read as merely sleepy.
    case MOOD_SUSPICIOUS: return {0.22f, 1.15f,  0, 3000, 7000};
    case MOOD_DIZZY:      return {1.00f, 1.00f,  0, 5000, 9000};
    case MOOD_WINK:       return {0.95f, 1.00f,  0, 2500, 5000};
    case MOOD_BORED:      return {0.45f, 1.00f,  4, 3500, 7000};
    // Starstruck: drawn as a shape, and blinks rarely so the sparkle shows.
    case MOOD_EXCITED:    return {1.10f, 1.00f, -1, 4000, 8000};
    default:              return {1.00f, 1.00f,  0, 2500, 6000};
  }
}

float animOpen = 1.0f;
float animGazeX = 0.0f;
float animGazeY = 0.0f;
float gazeTargetX = 0.0f;
float gazeTargetY = 0.0f;
int blinkPhase = 0; 
unsigned long blinkPhaseEnd = 0;
unsigned long nextBlinkAt = 0;
unsigned long nextSaccadeAt = 0;
uint32_t lastRenderKey = 0xFFFFFFFF;

void drawEye(int cx, int cy, int w, int h, Mood m, bool isLeft) {
  h = constrain(h, 0, 60);
  if (w < 4) return;

  if (h < 5) { // lids effectively shut - just a lash line
    u8g2.drawBox(cx - w / 2, cy - 1, w, 3);
    return;
  }

  if (m == MOOD_HAPPY) {
    int baseline = cy + h / 2;
    int x0 = cx - w / 2;
    for (int i = 0; i < w; i++) {
      float t = (float)(i - w / 2) / (w / 2.0f); // -1 .. 1
      int top = baseline - (int)(h * (1.0f - t * t));
      if (top < baseline) u8g2.drawVLine(x0 + i, top, baseline - top);
    }
    return;
  }

  if (m == MOOD_LOVE) {
    int r = max(3, w / 4);
    int lobeY = cy - h / 8;
    u8g2.drawDisc(cx - r, lobeY, r, U8G2_DRAW_ALL);
    u8g2.drawDisc(cx + r, lobeY, r, U8G2_DRAW_ALL);
    u8g2.drawTriangle(cx - 2 * r, lobeY, cx + 2 * r, lobeY, cx, cy + h / 2);
    return;
  }

  if (m == MOOD_EXCITED) {
    int hx = w / 2, hy = h / 2;
    int wx = max(2, w / 6), wy = max(2, h / 6);
    u8g2.drawTriangle(cx, cy - hy, cx - wx, cy, cx + wx, cy); // up
    u8g2.drawTriangle(cx, cy + hy, cx - wx, cy, cx + wx, cy); // down
    u8g2.drawTriangle(cx - hx, cy, cx, cy - wy, cx, cy + wy); // left
    u8g2.drawTriangle(cx + hx, cy, cx, cy - wy, cx, cy + wy); // right
    return;
  }

  if (m == MOOD_DIZZY) {
    int r = min(w, h) / 2;
    for (int o = 0; o <= 1; o++) {
      u8g2.drawLine(cx - r, cy - r + o, cx + r, cy + r + o);
      u8g2.drawLine(cx - r, cy + r + o, cx + r, cy - r + o);
    }
    return;
  }

  if (m == MOOD_WINK && !isLeft) {
    u8g2.drawBox(cx - w / 2, cy - 1, w, 3);
    return;
  }

  int x = cx - w / 2;
  int y = cy - h / 2;
  int r = min(8, min(w, h) / 2);
  u8g2.drawRBox(x, y, w, h, r);

  if (m == MOOD_ANGRY) {
    u8g2.setDrawColor(0);
    if (isLeft) {
      u8g2.drawTriangle(x + w + 1, y - 1, x + w + 1, y + h / 2, x - 1, y - 1);
    } else {
      u8g2.drawTriangle(x - 1, y - 1, x - 1, y + h / 2, x + w + 1, y - 1);
    }
    u8g2.setDrawColor(1);
  } else if (m == MOOD_SAD) {
    u8g2.setDrawColor(0);
    if (isLeft) {
      u8g2.drawTriangle(x - 1, y - 1, x - 1, y + h / 2, x + w + 1, y - 1);
    } else {
      u8g2.drawTriangle(x + w + 1, y - 1, x + w + 1, y + h / 2, x - 1, y - 1);
    }
    u8g2.setDrawColor(1);
  } else if (m == MOOD_SURPRISED && w > 14 && h > 14) {
    u8g2.setDrawColor(0);
    u8g2.drawRBox(x + 5, y + 5, w - 10, h - 10, max(2, r - 3));
    u8g2.setDrawColor(1);
  } else if (m == MOOD_BORED && h > 6) {
    u8g2.setDrawColor(0);
    u8g2.drawBox(x - 1, y - 1, w + 2, h / 3);
    u8g2.setDrawColor(1);
  }
}

void renderFace(Mood m, float open, float gx, float gy) {
  MoodStyle st = styleFor(m);
  int w = (int)(eyeWidth * st.widthScale);
  int h = (int)(eyeHeight * open);
  int cy = eyeCenterY + st.yOffset + (int)gy;
  int lx = leftEyeCX + (int)gx;
  int rx = rightEyeCX + (int)gx;

  u8g2.clearBuffer();
  if (m == MOOD_CURIOUS) {
    drawEye(lx, cy - 2, w, (int)(h * 1.12f), m, true);
    drawEye(rx, cy + 2, w, (int)(h * 0.88f), m, false);
  } else {
    drawEye(lx, cy, w, h, m, true);
    drawEye(rx, cy, w, h, m, false);
  }
  u8g2.sendBuffer();
}

void updateEyes() {
  unsigned long now = millis();
  unsigned long sinceCommand = now - lastCommandMillis;

  Mood m = (Mood)requestedMood;
  if (sinceCommand > SLEEPY_TIMEOUT_MS) m = MOOD_SLEEPY;
  MoodStyle st = styleFor(m);

  if (blinkPhase == 0) {
    if ((long)(now - nextBlinkAt) >= 0) {
      blinkPhase = 1;
      blinkPhaseEnd = now + 90;
    }
  } else if ((long)(now - blinkPhaseEnd) >= 0) {
    if (blinkPhase == 1) {
      blinkPhase = 2;
      blinkPhaseEnd = now + 130;
    } else {
      blinkPhase = 0;
      nextBlinkAt = now + random(st.blinkMinMs, st.blinkMaxMs);
    }
  }
  float openTarget = (blinkPhase == 1) ? 0.0f : st.openScale;

  if (sinceCommand < TRACKING_TIMEOUT_MS) {
    gazeTargetX = gazeHintX * 5.0f;
    gazeTargetY = gazeHintY * 3.0f;
    nextSaccadeAt = now + 900;
  } else if ((long)(now - nextSaccadeAt) >= 0) {
    gazeTargetX = (float)random(-6, 7);
    gazeTargetY = (float)random(-3, 4);
    nextSaccadeAt = now + random(1500, 4000);
  }

  animOpen += (openTarget - animOpen) * 0.35f;
  animGazeX += (gazeTargetX - animGazeX) * 0.20f;
  animGazeY += (gazeTargetY - animGazeY) * 0.20f;

  uint32_t key = ((uint32_t)(animOpen * 40.0f) << 20) ^
                 ((uint32_t)(animGazeX + 32.0f) << 12) ^
                 ((uint32_t)(animGazeY + 32.0f) << 4) ^
                 (uint32_t)m;
  if (key == lastRenderKey) return;
  lastRenderKey = key;

  renderFace(m, animOpen, animGazeX, animGazeY);
}

void eyesTask(void *pvParameters) {
  for (;;) {
    updateEyes();
    vTaskDelay(pdMS_TO_TICKS(40)); // ~25fps, about the most 400kHz I2C sustains
  }
}

// ---------------------------------------------------------------------------
// Why did we last reboot?
//
// With no serial attached during normal running, the OLED is the only place
// this can be reported
//
//   POWERON   - the rail actually collapsed. Servo inrush pulling the supply
//               down is the usual cause on this build.
//   BROWNOUT  - the detector fired, i.e. the disable in setup() is not taking
//               effect (it is armed before setup() runs).
//   PANIC     - a software crash, NOT power. A task stack overflow looks like
//               this, and would be pure coincidence with servo movement.
//   TASK WDT  - a task hogged its core without yielding.
//   SW        - our own ESP.restart(), e.g. the camera retry path below.
// ---------------------------------------------------------------------------
const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON-rail died";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_PANIC:     return "PANIC-crash!";
    case ESP_RST_TASK_WDT:  return "TASK WDT";
    case ESP_RST_INT_WDT:   return "INT WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_SW:        return "SW restart";
    case ESP_RST_EXT:       return "EXT reset pin";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    default:                return "unknown";
  }
}

void showStatus(const char *line1, const char *line2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 12, line1);
  u8g2.drawStr(0, 26, line2);
  u8g2.sendBuffer();
}

// How many times camera bring-up is retried before giving up and rebooting.
const int CAMERA_INIT_ATTEMPTS = 4;

#define CAM_FAIL_MAGIC 0xC0FFEEu
const uint32_t CAMERA_MAX_FAIL_REBOOTS = 3;
RTC_NOINIT_ATTR uint32_t camFailMagic;
RTC_NOINIT_ATTR uint32_t camFailCount;

esp_err_t initCamera(uint32_t xclkHz) {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = xclkHz;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;

  if (psramFound()) {
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  return esp_camera_init(&config);
}

// ---------------------------------------------------------------------------
// Camera bring-up, with retries.
//
//   0x105  ESP_ERR_NOT_FOUND     - sensor not detected. Power rail sagging at
//                                  boot, or a loose camera ribbon connector.
//   0x101  ESP_ERR_NO_MEM        - out of memory / PSRAM not found.
//   0x103  ESP_ERR_INVALID_STATE - already initialised.
// ---------------------------------------------------------------------------
esp_err_t bringUpCamera() {
  esp_err_t err = ESP_FAIL;

  for (int attempt = 1; attempt <= CAMERA_INIT_ATTEMPTS; attempt++) {
    bool slowClock = (attempt == CAMERA_INIT_ATTEMPTS);
    uint32_t xclk = slowClock ? 10000000 : 20000000;

    err = initCamera(xclk);
    if (err == ESP_OK) {
      Serial.printf("Camera up on attempt %d (xclk %u Hz)\n", attempt, xclk);
      if (slowClock) {
        showStatus("Camera OK - SLOW", "xclk 10MHz, low fps");
        delay(2500);
      }
      return ESP_OK;
    }

    Serial.printf("Camera init failed: 0x%x (attempt %d/%d, xclk %u Hz)\n",
                  err, attempt, CAMERA_INIT_ATTEMPTS, xclk);

    char detail[24];
    snprintf(detail, sizeof(detail), "0x%x  try %d/%d",
             err, attempt, CAMERA_INIT_ATTEMPTS);
    showStatus("Camera init FAILED", detail);
    esp_camera_deinit();
    delay(600);
  }
  return err;
}

// ---------------------------------------------------------------------------
// MJPEG stream server (port 80, GET /stream)
// ---------------------------------------------------------------------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  Serial.println("Stream client connected");
  char partBuf[64];

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
      break;
    }
    if (fb->format != PIXFORMAT_JPEG) {
      esp_camera_fb_return(fb);
      res = ESP_FAIL;
      break;
    }

    size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, partBuf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    esp_camera_fb_return(fb);

    if (res != ESP_OK) break;

  }

  Serial.println("Stream client disconnected");
  return res;
}

// ---------------------------------------------------------------------------
esp_err_t controlHandler(httpd_req_t *req) {
  char query[160];
  char val[24];

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (httpd_query_key_value(query, "pan", val, sizeof(val)) == ESP_OK) {
      int delta = atoi(val);
      if (delta != 0) {
        setPanTarget(targetPan + delta);
        gazeHintX = (delta > 0) ? 1 : -1;
      }
    }
    if (httpd_query_key_value(query, "tilt", val, sizeof(val)) == ESP_OK) {
      int delta = atoi(val);
      if (delta != 0) {
        setTiltTarget(targetTilt + delta);
        gazeHintY = (delta > 0) ? 1 : -1;
      }
    }
    if (httpd_query_key_value(query, "panabs", val, sizeof(val)) == ESP_OK) {
      setPanTarget(atoi(val));
    }
    if (httpd_query_key_value(query, "tiltabs", val, sizeof(val)) == ESP_OK) {
      setTiltTarget(atoi(val));
    }
    if (httpd_query_key_value(query, "mood", val, sizeof(val)) == ESP_OK) {
      requestedMood = moodFromName(val);
    }
    lastCommandMillis = millis();
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "ok", 2);
}

void startServers() {
  httpd_config_t streamConfig = HTTPD_DEFAULT_CONFIG();
  streamConfig.server_port = STREAM_PORT;
  streamConfig.ctrl_port = 32769;
  streamConfig.max_uri_handlers = 2;
  streamConfig.lru_purge_enable = true;

  httpd_uri_t streamUri = {};
  streamUri.uri = "/stream";
  streamUri.method = HTTP_GET;
  streamUri.handler = streamHandler;
  streamUri.user_ctx = NULL;

  if (httpd_start(&streamHttpd, &streamConfig) == ESP_OK) {
    httpd_register_uri_handler(streamHttpd, &streamUri);
    Serial.printf("MJPEG stream server on port %d (/stream)\n", STREAM_PORT);
  } else {
    Serial.println("Failed to start MJPEG stream server");
  }

  httpd_config_t controlConfig = HTTPD_DEFAULT_CONFIG();
  controlConfig.server_port = CONTROL_PORT;
  controlConfig.ctrl_port = 32768;
  controlConfig.max_uri_handlers = 4;
  controlConfig.lru_purge_enable = true;

  httpd_uri_t controlUri = {};
  controlUri.uri = "/control";
  controlUri.method = HTTP_GET;
  controlUri.handler = controlHandler;
  controlUri.user_ctx = NULL;

  if (httpd_start(&controlHttpd, &controlConfig) == ESP_OK) {
    httpd_register_uri_handler(controlHttpd, &controlUri);
    Serial.printf("Control server on port %d (/control)\n", CONTROL_PORT);
  } else {
    Serial.println("Failed to start control server");
  }
}

void setup() {
  // Two SG90s can brown out the onboard 3.3V regulator under load; disable
  // the brownout detector as a safety net. IMPORTANT: power the servos from
  // a separate 5V supply sharing GND with the ESP32-CAM, not from its
  // 5V/3.3V pin - the onboard regulator cannot supply servo stall current.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(300);

  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("Reset reason: %s (%d)\n", resetReasonName(resetReason), resetReason);

  // RTC memory is garbage on a cold boot, so validate it before trusting the
  // reboot counter.
  if (camFailMagic != CAM_FAIL_MAGIC) {
    camFailMagic = CAM_FAIL_MAGIC;
    camFailCount = 0;
  }

  // Likewise for the remembered servo positions. No valid record means a cold
  // power-on, so assume centre.
  if (servoPosMagic != SERVO_POS_MAGIC) {
    servoPosMagic = SERVO_POS_MAGIC;
    lastPanPos = SERVO_PAN_CENTER_ANGLE;
    lastTiltPos = SERVO_TILT_CENTER_ANGLE;
  }
  Serial.printf("Servo positions restored: pan=%d tilt=%d\n", lastPanPos, lastTiltPos);

  // Only count camera failures across OUR OWN restarts. Any other reset cause
  // - a servo browning out the rail, a crash - has nothing to do with the
  // camera, and letting those feed this counter would park the board on
  // "reseat ribbon" for a fault that isn't the ribbon. That trap is easy to
  // fall into here: a power reset is followed by a boot into a still-sagging
  // rail, which fails camera init for reasons that have nothing to do with
  // the camera itself.
  if (resetReason != ESP_RST_SW) {
    camFailCount = 0;
  }

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  u8g2.setBusClock(100000);
  u8g2.begin();

  showStatus("Last reset:", resetReasonName(resetReason));
  delay(2500);

  showStatus("Robot Pet booting", "Camera init...");

  delay(400);

  esp_err_t camErr = bringUpCamera();
  if (camErr == ESP_OK) {
    camFailCount = 0; // a good boot clears the reboot budget
  } else {
    char line[24];
    if (camFailCount >= CAMERA_MAX_FAIL_REBOOTS) {
      snprintf(line, sizeof(line), "0x%x-reseat ribbon", camErr);
      showStatus("Camera DEAD", line);
      while (true) delay(1000);
    }
    camFailCount++;
    snprintf(line, sizeof(line), "0x%x reboot %u/%u",
             camErr, camFailCount, CAMERA_MAX_FAIL_REBOOTS);
    showStatus("Camera FAILED", line);
    delay(2000);
    ESP.restart();
  }

  showStatus("Connecting WiFi", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - wifiStart > 20000) {
      showStatus("WiFi FAILED", "check credentials");
      delay(2000);
      ESP.restart();
    }
  }
  Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());

  char ipLine[32];
  snprintf(ipLine, sizeof(ipLine), "IP %s", WiFi.localIP().toString().c_str());

  char wifiLine[24];
  snprintf(wifiLine, sizeof(wifiLine), "WiFi OK %ddBm", WiFi.RSSI());
  showStatus(wifiLine, ipLine);
  delay(1500);

  startServers();

  // ---------------------------------------------------------------------
  // Servos last, and one at a time.
  //
  // attach() itself drives a servo to centre (ESP32Servo pulses 1500us by
  // default), so the horn lunges there from wherever it physically sits -
  // potentially 90 degrees of travel at full torque.
  // ---------------------------------------------------------------------
  // Timer 0 is already claimed by initCamera() (config.ledc_timer =
  // LEDC_TIMER_0) to generate the camera's XCLK. If the servo engine also
  // grabs timer 0, attach() reprograms it for 50Hz and corrupts XCLK / the
  // servo signal - so servos get timers 1 and 2 instead, never 0.
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  showStatus("Centering servos", "pan...");
  panServo.attach(SERVO_PAN_PIN, SERVO_PAN_MIN_PULSE_US, SERVO_PAN_MAX_PULSE_US);
  writePan(constrain(lastPanPos, SERVO_PAN_MIN_ANGLE, SERVO_PAN_MAX_ANGLE));
  setPanTarget(SERVO_PAN_CENTER_ANGLE);
  delay(700); // let pan settle, and the rail recover, before touching tilt

  showStatus("Centering servos", "tilt...");
  tiltServo.attach(SERVO_TILT_PIN, SERVO_TILT_MIN_PULSE_US, SERVO_TILT_MAX_PULSE_US);
  writeTilt(constrain(lastTiltPos, SERVO_TILT_MIN_ANGLE, SERVO_TILT_MAX_ANGLE));
  setTiltTarget(SERVO_TILT_CENTER_ANGLE);
  delay(700);

  randomSeed(micros());
  lastCommandMillis = millis();

  // Start the eyes on their own task now that setup() is done touching u8g2
  // directly (showStatus() calls above) - from here on only eyesTask draws
  // to the OLED, so no locking is needed between it and loop().

  xTaskCreatePinnedToCore(eyesTask, "eyes", 8192, NULL, 1, NULL, 0);

  // Servo slewing on core 0 as well. It only nudges an angle every 15ms, so
  // it is cheap, but it must not be starved by the stream server on core 1 -
  // a stalled ramp would leave the head halfway to where it was asked to go.
  xTaskCreatePinnedToCore(servoTask, "servo", 4096, NULL, 2, NULL, 0);
}

// Both HTTP servers run in their own httpd tasks and the eyes run in
// eyesTask, so loop() has nothing left to do. Nothing here needs servicing,
// which means nothing here can be starved.
void loop() {
  delay(1000);
}
