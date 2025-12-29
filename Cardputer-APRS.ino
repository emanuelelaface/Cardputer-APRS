#include <M5Cardputer.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <math.h>

// =====================
// DEFAULTS (first boot)
// =====================
// As requested: CALLSIGN + COMMENT empty by default
static const char* DEF_CALLSIGN = "";
static const char* DEF_DESTCALL = "APRS";
static const char* DEF_PATH     = "WIDE1-1";
static const char  DEF_SYM_TABLE = '/';
static const char  DEF_SYM_CODE  = '>';
static const char* DEF_COMMENT  = "";

// =====================
// LoRa (EU868)
// =====================
static const float   LORA_FREQ_MHZ = 868.0f;
static const float   LORA_BW_KHZ   = 125.0f;
static const uint8_t LORA_SF       = 12;
static const uint8_t LORA_CR       = 5;
static const uint8_t LORA_SYNC     = 0x12;
static const int8_t  LORA_PWR_DBM  = 14;
static const uint16_t LORA_PREAMBLE = 20;
static const float   LORA_TCXO_V   = 3.0f;
static const bool    LORA_LDO      = true;

// SX1262 pins (Cardputer-Adv + LoRa Cap)
static const uint8_t PIN_LORA_NSS  = 5;
static const uint8_t PIN_LORA_DIO1 = 4;
static const uint8_t PIN_LORA_RST  = 3;
static const uint8_t PIN_LORA_BUSY = 6;

// GNSS UART
static const int PIN_GPS_RX = 15;
static const int PIN_GPS_TX = 13;
static const uint32_t GPS_BAUD = 115200;

// Brightness
static const uint8_t BRIGHT_ON  = 80;
static const uint8_t BRIGHT_OFF = 0;

// Layout tuning
static const float TEXT_AREA_RATIO = 0.66f;          // smaller => bigger right plot
static const float LEFT_COL_RATIO_IN_TEXT = 0.58f;   // smaller => slightly smaller left column

// =====================
// APRS CONFIG (runtime + NVS)
// =====================
static Preferences prefs;
static bool cfgOk = false;

static String cfgCallsign;
static String cfgDestcall;
static String cfgPath;
static char   cfgSymTable = DEF_SYM_TABLE;
static char   cfgSymCode  = DEF_SYM_CODE;
static String cfgComment;

static void normalizeCfg() {
  cfgCallsign.trim(); cfgCallsign.toUpperCase();
  cfgDestcall.trim(); cfgDestcall.toUpperCase();
  cfgPath.trim();     cfgPath.toUpperCase();
  cfgComment.trim();

  // callsign/comment can be empty
  if (cfgDestcall.length() == 0) cfgDestcall = DEF_DESTCALL;
  if (cfgPath.length() == 0)     cfgPath = DEF_PATH;
  if (cfgSymTable == 0)          cfgSymTable = DEF_SYM_TABLE;
  if (cfgSymCode == 0)           cfgSymCode = DEF_SYM_CODE;
}

static void loadCfg() {
  prefs.begin("aprs", true);
  cfgOk       = prefs.getBool("ok", false);
  cfgCallsign = prefs.getString("call", DEF_CALLSIGN);
  cfgDestcall = prefs.getString("dest", DEF_DESTCALL);
  cfgPath     = prefs.getString("path", DEF_PATH);
  cfgSymTable = (char)prefs.getUChar("stab", (uint8_t)DEF_SYM_TABLE);
  cfgSymCode  = (char)prefs.getUChar("scode",(uint8_t)DEF_SYM_CODE);
  cfgComment  = prefs.getString("cmt",  DEF_COMMENT);
  prefs.end();

  normalizeCfg();
}

static void saveCfg() {
  normalizeCfg();
  prefs.begin("aprs", false);
  prefs.putBool("ok", true);
  prefs.putString("call", cfgCallsign);
  prefs.putString("dest", cfgDestcall);
  prefs.putString("path", cfgPath);
  prefs.putUChar("stab", (uint8_t)cfgSymTable);
  prefs.putUChar("scode",(uint8_t)cfgSymCode);
  prefs.putString("cmt",  cfgComment);
  prefs.end();
  cfgOk = true;
}

// =====================
// OBJECTS
// =====================
TinyGPSPlus gps;
HardwareSerial GPS(2);
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);

static M5Canvas mainCanvas(&M5Cardputer.Display);
static bool canvasReady = false;

static M5Canvas cfgCanvas(&M5Cardputer.Display);
static bool cfgCanvasReady = false;

// =====================
// FreeRTOS: non-blocking TX
// =====================
struct TxRequest { char frame[256]; };

static QueueHandle_t txQueue = nullptr;
static TaskHandle_t  txTaskHandle = nullptr;

static volatile bool txDoneFlag = false;
static volatile int  txLastCode = 0;
static volatile bool txBusy = false;

static void LoRaTxTask(void* pv) {
  (void)pv;
  TxRequest req;
  while (true) {
    if (xQueueReceive(txQueue, &req, portMAX_DELAY) == pdTRUE) {
      txBusy = true;
      int code = radio.transmit(req.frame);
      txLastCode = code;
      txDoneFlag = true;
      txBusy = false;
    }
  }
}

// =====================
// SATELLITES (NMEA GSV + GSA parser)
// =====================
struct SatInfo {
  uint16_t prn = 0;
  uint8_t  elev = 0;
  uint16_t az   = 0;
  uint8_t  snr  = 0;
  char     sys  = 'N';
  bool     used = false;
  uint32_t lastMs = 0;
};

static SatInfo sats[48];
static uint16_t usedPrn[32];
static uint8_t  usedCount = 0;
static uint8_t  fixTypeGsa = 0;

static char nmeaLine[128];
static uint8_t nmeaPos = 0;

static bool hex2u8(const char* p, uint8_t& out) {
  auto h = [](char c)->int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
  };
  int hi = h(p[0]), lo = h(p[1]);
  if (hi < 0 || lo < 0) return false;
  out = (uint8_t)((hi << 4) | lo);
  return true;
}

static bool nmeaChecksumOk(const char* s) {
  const char* star = strchr(s, '*');
  if (!star) return true;
  uint8_t want = 0;
  if (!hex2u8(star + 1, want)) return false;

  uint8_t x = 0;
  for (const char* p = s + 1; p < star; ++p) x ^= (uint8_t)(*p);
  return x == want;
}

static char talkerToSys(const char* msgId) {
  if (!msgId || strlen(msgId) < 2) return 'N';
  if (msgId[0] == 'G' && msgId[1] == 'P') return 'G';
  if (msgId[0] == 'G' && msgId[1] == 'L') return 'L';
  if (msgId[0] == 'G' && msgId[1] == 'A') return 'A';
  if (msgId[0] == 'G' && msgId[1] == 'B') return 'B';
  if (msgId[0] == 'B' && msgId[1] == 'D') return 'B';
  return 'N';
}

static bool prnIsUsed(uint16_t prn) {
  for (uint8_t i = 0; i < usedCount; i++) if (usedPrn[i] == prn) return true;
  return false;
}

static void clearUsedFlags() { for (auto &s : sats) s.used = false; }

static void upsertSat(char sys, uint16_t prn, uint8_t elev, uint16_t az, uint8_t snr) {
  if (!prn) return;

  int freeIdx = -1;
  int foundIdx = -1;
  uint32_t oldestMs = 0xFFFFFFFF;
  int oldestIdx = 0;

  for (int i = 0; i < (int)(sizeof(sats)/sizeof(sats[0])); i++) {
    if (sats[i].prn == prn && sats[i].sys == sys) { foundIdx = i; break; }
    if (sats[i].prn == 0 && freeIdx < 0) freeIdx = i;
    if (sats[i].prn != 0 && sats[i].lastMs < oldestMs) { oldestMs = sats[i].lastMs; oldestIdx = i; }
  }

  int idx = (foundIdx >= 0) ? foundIdx : ((freeIdx >= 0) ? freeIdx : oldestIdx);
  sats[idx].sys = sys;
  sats[idx].prn = prn;
  sats[idx].elev = (elev > 90) ? 90 : elev;
  sats[idx].az = (az >= 360) ? (az % 360) : az;
  sats[idx].snr = snr;
  sats[idx].lastMs = millis();
  sats[idx].used = prnIsUsed(prn);
}

static void expireSats(uint32_t maxAgeMs = 12000) {
  uint32_t now = millis();
  for (auto &s : sats) if (s.prn != 0 && (now - s.lastMs) > maxAgeMs) s = SatInfo{};
}

static void parseGSA(char sys, char** tok, int n) {
  if (n < 3) return;
  fixTypeGsa = (uint8_t)atoi(tok[2]);

  usedCount = 0;
  for (int i = 3; i <= 14 && i < n; i++) {
    if (tok[i] && tok[i][0]) {
      uint16_t prn = (uint16_t)atoi(tok[i]);
      if (prn && usedCount < sizeof(usedPrn)/sizeof(usedPrn[0])) usedPrn[usedCount++] = prn;
    }
  }
  clearUsedFlags();
  for (auto &s : sats) if (s.prn && prnIsUsed(s.prn)) s.used = true;
  (void)sys;
}

static void parseGSV(char sys, char** tok, int n) {
  if (n < 4) return;
  for (int i = 4; i + 3 < n; i += 4) {
    if (!tok[i] || !tok[i][0]) continue;
    uint16_t prn = (uint16_t)atoi(tok[i]);
    uint8_t  elev = tok[i+1] ? (uint8_t)atoi(tok[i+1]) : 0;
    uint16_t az   = tok[i+2] ? (uint16_t)atoi(tok[i+2]) : 0;
    uint8_t  snr  = tok[i+3] ? (uint8_t)atoi(tok[i+3]) : 0;
    upsertSat(sys, prn, elev, az, snr);
  }
}

static void handleNmeaLine(const char* line) {
  if (!line || line[0] != '$') return;
  if (!nmeaChecksumOk(line)) return;

  char buf[128];
  strncpy(buf, line, sizeof(buf)-1);
  buf[sizeof(buf)-1] = 0;

  char* tok[32];
  int nt = 0;
  char* p = buf;
  while (p && nt < 32) {
    char* comma = strchr(p, ',');
    char* star  = strchr(p, '*');
    char* cut = nullptr;
    if (comma && star) cut = (comma < star) ? comma : star;
    else cut = comma ? comma : star;

    if (cut) { *cut = 0; tok[nt++] = p; p = cut + 1; }
    else { tok[nt++] = p; break; }
  }
  if (nt < 1) return;

  const char* msg = tok[0] + 1;
  if (strlen(msg) < 5) return;

  char sys = talkerToSys(msg);
  const char* type = msg + 2;

  if (strncmp(type, "GSV", 3) == 0) parseGSV(sys, tok, nt);
  else if (strncmp(type, "GSA", 3) == 0) parseGSA(sys, tok, nt);
}

// =====================
// APRS
// =====================
static void formatAprsLatLon(double lat, double lon, char* outLat, size_t outLatSz, char* outLon, size_t outLonSz) {
  char ns = (lat >= 0) ? 'N' : 'S';
  char ew = (lon >= 0) ? 'E' : 'W';
  lat = fabs(lat);
  lon = fabs(lon);

  int latDeg = (int)lat;
  double latMin = (lat - latDeg) * 60.0;
  int lonDeg = (int)lon;
  double lonMin = (lon - lonDeg) * 60.0;

  snprintf(outLat, outLatSz, "%02d%05.2f%c", latDeg, latMin, ns);
  snprintf(outLon, outLonSz, "%03d%05.2f%c", lonDeg, lonMin, ew);
}

static String buildAprsTnc2Packet() {
  char latBuf[16], lonBuf[16];
  formatAprsLatLon(gps.location.lat(), gps.location.lng(), latBuf, sizeof(latBuf), lonBuf, sizeof(lonBuf));

  int course = gps.course.isValid() ? (int)lround(gps.course.deg()) : 0;
  int speed  = gps.speed.isValid()  ? (int)lround(gps.speed.knots()) : 0;

  char csBuf[16];
  snprintf(csBuf, sizeof(csBuf), "%03d/%03d", course % 360, speed);

  String altPart = "";
  if (gps.altitude.isValid()) {
    long altFt = lround(gps.altitude.meters() * 3.28084);
    if (altFt < 0) altFt = 0;
    char aBuf[16];
    snprintf(aBuf, sizeof(aBuf), "/A=%06ld", altFt);
    altPart = String(aBuf);
  }

  String payload = "!" + String(latBuf) + String(cfgSymTable) + String(lonBuf) + String(cfgSymCode)
                 + String(csBuf) + altPart;

  if (cfgComment.length()) payload += " " + cfgComment;

  return cfgCallsign + ">" + cfgDestcall + "," + cfgPath + ":" + payload;
}

// =====================
// UI (RUN MODE) + APP STATE
// =====================
static bool screenOff = false;

static String statusMsg = "ENTER=TX  L=Auto  S=Screen  C=Config";
static uint32_t statusUntilMs = 0;

static uint32_t lastUiMs = 0;

// Auto TX
static bool autoTxEnabled = false;
static uint32_t autoPeriodMs = 300000; // 5 minutes
static uint32_t nextAutoMs = 0;

// Layout
static int W, H;
static const int TOP_H = 14;
static const int BOT_H = 14;
static int mainY, mainH;

static int textW, plotX, plotW;
static int leftColW, midColW;

static const int ROWS = 6;
static int rowH;

static void setStatus(const String& s, uint32_t ms = 2500) {
  statusMsg = s;
  statusUntilMs = millis() + ms;
}

static String fmtFloat(double v, int prec) {
  if (!isfinite(v)) return "--";
  char b[32];
  dtostrf(v, 0, prec, b);
  return String(b);
}

static String batteryPctString() {
  int pct = M5.Power.getBatteryLevel();
  if (pct < 0) return "--";
  return String(pct) + "%";
}

static void computeLayout() {
  W = M5Cardputer.Display.width();
  H = M5Cardputer.Display.height();
  mainY = TOP_H;
  mainH = H - TOP_H - BOT_H;
  rowH = mainH / ROWS;

  textW = (int)(W * TEXT_AREA_RATIO);
  plotX = textW;
  plotW = W - textW;

  leftColW = (int)(textW * LEFT_COL_RATIO_IN_TEXT);
  midColW  = textW - leftColW;
}

static void drawTopBar() {
  M5Cardputer.Display.fillRect(0, 0, W, TOP_H, GREEN);
  M5Cardputer.Display.setTextColor(BLACK, GREEN);
  M5Cardputer.Display.setCursor(2, 2);
  M5Cardputer.Display.print("Cardputer GPS/APRS");
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

static void drawBottomBar() {
  M5Cardputer.Display.fillRect(0, H - BOT_H, W, BOT_H, DARKGREY);
  M5Cardputer.Display.setTextColor(WHITE, DARKGREY);
  M5Cardputer.Display.setCursor(2, H - BOT_H + 2);

  if (statusUntilMs && millis() > statusUntilMs) {
    statusUntilMs = 0;
    statusMsg = "ENTER=TX  L=Auto  S=Screen  C=Config";
  }
  M5Cardputer.Display.print(statusMsg);
  M5Cardputer.Display.setTextColor(WHITE, BLACK);
}

// ---- clip text so it never overwrites borders ----
static String clipToWidth(M5Canvas& c, const String& s, int maxPx) {
  if (maxPx <= 0) return "";
  if (c.textWidth(s) <= maxPx) return s;

  int n = s.length();
  while (n > 0) {
    String t = s.substring(0, n);
    if (c.textWidth(t) <= maxPx) return t;
    n--;
  }
  return "";
}

static void drawCellCanvas(int x, int y, int w, int h, const char* label, const String& value) {
  mainCanvas.fillRect(x + 1, y + 1, w - 2, h - 2, BLACK);
  mainCanvas.setTextColor(WHITE, BLACK);

  const int padL = 2;
  const int padR = 4;

  String head = String(label) + ":";
  mainCanvas.setCursor(x + padL, y + 2);
  mainCanvas.print(head);

  int headW = mainCanvas.textWidth(head);
  int maxValPx = (x + w - padR) - (x + padL + headW);
  if (maxValPx < 0) maxValPx = 0;

  String v = clipToWidth(mainCanvas, value, maxValPx);
  mainCanvas.setCursor(x + padL + headW, y + 2);
  mainCanvas.print(v);
}

static void drawSatPlotCanvas() {
  int px = plotX;
  int py = 0;
  int pw = plotW;
  int ph = mainH;

  mainCanvas.fillRect(px + 1, py + 1, pw - 2, ph - 2, BLACK);

  int cx = px + pw / 2;
  int cy = py + ph / 2;
  int r  = min(pw, ph) / 2 - 8;

  mainCanvas.drawCircle(cx, cy, r, WHITE);
  mainCanvas.drawCircle(cx, cy, r * 2 / 3, DARKGREY);
  mainCanvas.drawCircle(cx, cy, r * 1 / 3, DARKGREY);
  mainCanvas.drawLine(cx - r, cy, cx + r, cy, WHITE);
  mainCanvas.drawLine(cx, cy - r, cx, cy + r, WHITE);

  mainCanvas.setTextColor(WHITE, BLACK);
  mainCanvas.setCursor(cx - 3, cy - r - 10); mainCanvas.print("N");
  mainCanvas.setCursor(cx - 3, cy + r + 2);  mainCanvas.print("S");
  mainCanvas.setCursor(cx - r - 10, cy - 3); mainCanvas.print("W");
  mainCanvas.setCursor(cx + r + 2,  cy - 3); mainCanvas.print("E");

  expireSats();
  uint32_t now = millis();

  for (auto &s : sats) {
    if (!s.prn) continue;
    if (now - s.lastMs > 12000) continue;

    float elev = (float)s.elev;
    float az   = (float)s.az;

    float rr = (90.0f - elev) / 90.0f * r;
    float a  = az * (float)M_PI / 180.0f;
    int x = cx + (int)lround(rr * sinf(a));
    int y = cy - (int)lround(rr * cosf(a));

    uint16_t c;
    if (s.used) c = GREEN;
    else if (s.snr >= 35) c = CYAN;
    else if (s.snr >= 20) c = YELLOW;
    else c = ORANGE;

    mainCanvas.fillCircle(x, y, 3, c);
  }
}

static void renderMainCanvas() {
  mainCanvas.fillScreen(BLACK);

  mainCanvas.drawRect(0, 0, W, mainH, WHITE);
  mainCanvas.drawLine(leftColW, 0, leftColW, mainH, WHITE);
  mainCanvas.drawLine(textW,    0, textW,    mainH, WHITE);

  for (int r = 1; r < ROWS; r++) {
    int y = r * rowH;
    mainCanvas.drawLine(0, y, textW, y, WHITE);
    mainCanvas.drawLine(textW, y, W, y, WHITE);
  }

  auto yRow = [&](int r){ return r * rowH; };

  // Left column
  drawCellCanvas(0, yRow(0), leftColW, rowH, "Lat",  gps.location.isValid() ? fmtFloat(gps.location.lat(), 6) : "--");
  drawCellCanvas(0, yRow(1), leftColW, rowH, "Lon",  gps.location.isValid() ? fmtFloat(gps.location.lng(), 6) : "--");
  drawCellCanvas(0, yRow(2), leftColW, rowH, "Alt",  gps.altitude.isValid() ? (fmtFloat(gps.altitude.meters(), 1) + "m") : "--");
  drawCellCanvas(0, yRow(3), leftColW, rowH, "Spd",  gps.speed.isValid() ? (fmtFloat(gps.speed.kmph(), 1) + "km/h") : "--");
  drawCellCanvas(0, yRow(4), leftColW, rowH, "Date", (gps.date.isValid()
      ? (String(gps.date.day()) + "/" + String(gps.date.month()) + "/" + String(gps.date.year() % 100))
      : "--"));
  drawCellCanvas(0, yRow(5), leftColW, rowH, "Time", (gps.time.isValid()
      ? (String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()))
      : "--"));

  // Middle column
  int xMid = leftColW;
  bool fix = gps.location.isValid() && gps.location.age() < 3000;
  String fixStr = (fixTypeGsa == 3) ? "3D" : (fixTypeGsa == 2) ? "2D" : (fix ? "FIX" : "NO");

  int seen = 0, used = 0;
  uint32_t now = millis();
  for (auto &s : sats) {
    if (!s.prn) continue;
    if (now - s.lastMs > 12000) continue;
    seen++;
    if (s.used) used++;
  }

  String ageStr  = gps.location.isValid() ? (String((int)(gps.location.age() / 1000)) + "s") : "--";
  String autoStr = autoTxEnabled ? "ON" : "OFF";

  // IMPORTANT: when TX -> show only "TX..."
  String loraVal = txBusy ? "TX..." : (String("IDLE A:") + autoStr);

  drawCellCanvas(xMid, yRow(0), midColW, rowH, "Fix",  fixStr);
  drawCellCanvas(xMid, yRow(1), midColW, rowH, "Seen", String(seen));
  drawCellCanvas(xMid, yRow(2), midColW, rowH, "Used", String(used));
  drawCellCanvas(xMid, yRow(3), midColW, rowH, "Age",  ageStr);
  drawCellCanvas(xMid, yRow(4), midColW, rowH, "Batt", batteryPctString());
  drawCellCanvas(xMid, yRow(5), midColW, rowH, "LoRa", loraVal);

  drawSatPlotCanvas();
  mainCanvas.pushSprite(0, mainY);
}

static void toggleScreen() {
  screenOff = !screenOff;
  if (screenOff) {
    M5Cardputer.Display.fillScreen(BLACK);
    M5Cardputer.Display.setBrightness(BRIGHT_OFF);
  } else {
    M5Cardputer.Display.setBrightness(BRIGHT_ON);
    // caller redraws current mode
  }
}

// =====================
// CONFIG MODE
// =====================
enum AppMode { MODE_RUN, MODE_CONFIG };
static AppMode mode = MODE_RUN;

static int cfgStep = 0;     // 0..5
static String cfgInput;
static bool cfgDirty = true;

static const int CFG_STEPS = 6;

static const char* cfgLabel(int s) {
  switch (s) {
    case 0: return "Callsign";
    case 1: return "Destcall";
    case 2: return "Path";
    case 3: return "SymTable";
    case 4: return "SymCode";
    case 5: return "Comment";
    default: return "";
  }
}

static String cfgGetStepValue(int s) {
  switch (s) {
    case 0: return cfgCallsign;
    case 1: return cfgDestcall;
    case 2: return cfgPath;
    case 3: return String(cfgSymTable);
    case 4: return String(cfgSymCode);
    case 5: return cfgComment;
    default: return "";
  }
}

static void cfgApplyStepValue(int s, const String& in) {
  String v = in;
  v.trim();

  switch (s) {
    case 0: cfgCallsign = v; break;                  // may be empty
    case 1: if (v.length()) cfgDestcall = v; break;
    case 2: if (v.length()) cfgPath = v; break;
    case 3: if (v.length()) cfgSymTable = v[0]; break;
    case 4: if (v.length()) cfgSymCode  = v[0]; break;
    case 5: cfgComment = v; break;                   // may be empty
  }
  normalizeCfg();
}

static void enterConfig() {
  mode = MODE_CONFIG;
  autoTxEnabled = false;   // stop auto while editing
  cfgStep = 0;
  cfgInput = cfgGetStepValue(cfgStep);
  cfgDirty = true;
}

static void exitConfigSave() {
  cfgApplyStepValue(cfgStep, cfgInput);
  saveCfg();

  mode = MODE_RUN;
  setStatus("Config saved", 1500);
  drawTopBar();
  drawBottomBar();
}

static void cfgNext() {
  cfgApplyStepValue(cfgStep, cfgInput);
  cfgStep++;
  if (cfgStep >= CFG_STEPS) {
    exitConfigSave();
    return;
  }
  cfgInput = cfgGetStepValue(cfgStep);
  cfgDirty = true;
}

static void drawConfigScreen() {
  if (!cfgCanvasReady) return;

  cfgCanvas.fillScreen(BLACK);
  cfgCanvas.setTextSize(1);
  cfgCanvas.setTextColor(WHITE, BLACK);

  // header
  cfgCanvas.fillRect(0, 0, W, TOP_H, DARKGREY);
  cfgCanvas.setTextColor(WHITE, DARKGREY);
  cfgCanvas.setCursor(2, 2);
  cfgCanvas.print("APRS CONFIG ");
  cfgCanvas.print(cfgStep + 1);
  cfgCanvas.print("/");
  cfgCanvas.print(CFG_STEPS);
  cfgCanvas.setTextColor(WHITE, BLACK);

  int y = TOP_H + 6;

  // show all fields, highlight current
  for (int i = 0; i < CFG_STEPS; i++) {
    cfgCanvas.setCursor(4, y);
    cfgCanvas.print(i == cfgStep ? ">" : " ");
    cfgCanvas.print(cfgLabel(i));
    cfgCanvas.print(": ");
    cfgCanvas.print(cfgGetStepValue(i));
    y += 12;
  }

  // edit box
  y += 6;
  cfgCanvas.setCursor(4, y);
  cfgCanvas.print("Edit: ");
  y += 10;
  cfgCanvas.drawRect(4, y, W - 8, 16, WHITE);
  cfgCanvas.setCursor(8, y + 4);
  cfgCanvas.print(cfgInput);

  // footer (SHORT so it doesn't wrap)
  cfgCanvas.fillRect(0, H - BOT_H, W, BOT_H, DARKGREY);
  cfgCanvas.setTextColor(WHITE, DARKGREY);
  cfgCanvas.setCursor(2, H - BOT_H + 2);
  cfgCanvas.print("DEL=Back  ENTER=Next/Save");
  cfgCanvas.setTextColor(WHITE, BLACK);

  cfgCanvas.pushSprite(0, 0);
}

// =====================
// TX enqueue
// =====================
static bool enqueueAprsTx() {
  if (cfgCallsign.length() == 0) {
    setStatus("No CALLSIGN (press C)", 2500);
    return false;
  }

  if (!(gps.location.isValid() && gps.location.age() < 5000)) {
    setStatus("NO FIX (skipped)", 2000);
    return false;
  }

  String frame = buildAprsTnc2Packet();
  TxRequest req{};
  frame.toCharArray(req.frame, sizeof(req.frame));

  if (!txQueue) {
    setStatus("TX queue not ready", 2500);
    return false;
  }

  if (xQueueSend(txQueue, &req, 0) != pdTRUE) {
    setStatus("TX queue FULL", 2500);
    return false;
  }

  setStatus("TX queued...", 1200);
  return true;
}

// =====================
// SETUP/LOOP
// =====================
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setBrightness(BRIGHT_ON);

  Serial.begin(115200);
  GPS.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  computeLayout();

  mainCanvas.setColorDepth(16);
  canvasReady = mainCanvas.createSprite(W, mainH);
  if (canvasReady) mainCanvas.setTextSize(1);

  cfgCanvas.setColorDepth(8);
  cfgCanvasReady = cfgCanvas.createSprite(W, H);
  if (cfgCanvasReady) cfgCanvas.setTextSize(1);

  loadCfg();

  int state = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC,
                          LORA_PWR_DBM, LORA_PREAMBLE, LORA_TCXO_V, LORA_LDO);
  if (state != RADIOLIB_ERR_NONE) {
    setStatus(String("LoRa init ERR ") + String(state), 4000);
  } else {
    setStatus("ENTER=TX  L=Auto  S=Screen  C=Config", 2500);
  }

  txQueue = xQueueCreate(6, sizeof(TxRequest));
  if (!txQueue) {
    setStatus("Queue alloc ERR", 4000);
  } else {
    xTaskCreatePinnedToCore(
      LoRaTxTask,
      "LoRaTxTask",
      4096,
      nullptr,
      2,
      &txTaskHandle,
      0
    );
  }

  // first boot -> start in config
  if (!cfgOk) {
    enterConfig();
    drawConfigScreen();
  } else {
    drawTopBar();
    drawBottomBar();
  }
}

void loop() {
  M5Cardputer.update();

  // GNSS (always)
  while (GPS.available()) {
    char c = (char)GPS.read();
    gps.encode(c);

    if (c == '\n') {
      nmeaLine[nmeaPos] = 0;
      if (nmeaPos > 6) handleNmeaLine(nmeaLine);
      nmeaPos = 0;
    } else if (c != '\r') {
      if (nmeaPos < sizeof(nmeaLine) - 1) nmeaLine[nmeaPos++] = c;
      else nmeaPos = 0;
    }
  }

  // Keyboard: read ONCE per loop
  bool keyEvent = (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed());

  if (keyEvent) {
    auto st = M5Cardputer.Keyboard.keysState();

    // If screen is OFF: ONLY allow S to turn it back on (RUN behavior)
    if (screenOff) {
      for (auto ch : st.word) {
        if (ch == 's' || ch == 'S') {
          toggleScreen();
          // redraw whatever mode we are in
          if (mode == MODE_CONFIG) { cfgDirty = true; }
          else { drawTopBar(); drawBottomBar(); }
          return;
        }
      }
      return;
    }

    // =====================
    // CONFIG MODE: NO COMMAND KEYS (L/S/C do NOT trigger)
    // =====================
    if (mode == MODE_CONFIG) {
      bool changed = false;

      // printable chars (including 'L','S','C' as normal text)
      for (auto ch : st.word) {
        if (ch >= 32 && ch <= 126) {
          if (cfgStep <= 2) {
            if (cfgInput.length() < 16) { cfgInput += (char)ch; changed = true; }
          } else if (cfgStep == 3 || cfgStep == 4) {
            if (cfgInput.length() < 1) { cfgInput += (char)ch; changed = true; }
          } else {
            if (cfgInput.length() < 50) { cfgInput += (char)ch; changed = true; }
          }
        }

        // fallback backspace if it arrives as ASCII
        if (ch == 0x08 || ch == 0x7F) {
          if (cfgInput.length()) { cfgInput.remove(cfgInput.length() - 1); changed = true; }
        }
      }

      // DEL as backspace (preferred)
      if (st.del) {
        if (cfgInput.length()) { cfgInput.remove(cfgInput.length() - 1); changed = true; }
      }

      // ENTER -> next / save at end
      if (st.enter) {
        cfgNext();
        if (mode == MODE_RUN) return;
        changed = true;
      }

      if (changed) cfgDirty = true;
    }

    // =====================
    // RUN MODE: commands active
    // =====================
    else {
      for (auto ch : st.word) {
        if (ch == 's' || ch == 'S') {
          toggleScreen();
          if (!screenOff) { drawTopBar(); drawBottomBar(); }
        }

        if (ch == 'c' || ch == 'C') {
          enterConfig();
          cfgDirty = true;
          drawConfigScreen();
          return;
        }

        if (ch == 'l' || ch == 'L') {
          autoTxEnabled = !autoTxEnabled;
          if (autoTxEnabled) {
            nextAutoMs = millis();
            setStatus("Auto TX ON (5 min)", 2500);
          } else {
            setStatus("Auto TX OFF", 2000);
          }
          drawBottomBar();
        }
      }

      if (st.enter) {
        enqueueAprsTx();
        drawBottomBar();
      }
    }
  }

  // CONFIG redraw only when needed
  if (mode == MODE_CONFIG) {
    if (cfgCanvasReady && cfgDirty) {
      cfgDirty = false;
      drawConfigScreen();
    }
    return;
  }

  // Auto TX every 5 minutes (non-blocking)
  if (autoTxEnabled) {
    uint32_t now = millis();
    if ((int32_t)(now - nextAutoMs) >= 0) {
      enqueueAprsTx();
      nextAutoMs = now + autoPeriodMs;
      drawBottomBar();
    }
  }

  // TX result from task
  if (txDoneFlag) {
    txDoneFlag = false;
    int code = txLastCode;
    if (code == RADIOLIB_ERR_NONE) setStatus("APRS sent OK", 2500);
    else setStatus(String("TX ERR ") + String(code), 3500);
    drawBottomBar();
  }

  if (screenOff) return;

  // UI
  if (millis() - lastUiMs > 300) {
    lastUiMs = millis();
    if (canvasReady) renderMainCanvas();
    drawBottomBar();
  }
}
