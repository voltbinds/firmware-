// ============================================================
//  VoltBinds v1.4
//  ESP32-C3 OLED Board (0.42", SSD1306 72x40px)
//  SDA=GPIO5, SCL=GPIO6
//
//  Architektur:
//    IPowermeter  <- SonnenAdapter  (Status + Steuerung)
//                 <- ShellyAdapter  (Nur Smartmeter)
//    IStorage     <- MarstekAdapter (Modbus TCP only)
//    RuleEngine   arbeitet ausschliesslich mit den Interfaces
//
//  Features:
//    - Captive Portal (WLAN-Setup + Geraetekonfig)
//    - OTA ueber WLAN  (Passwort: voltbinds2026)
//    - Dashboard http://voltbinds.local
//    - OLED: 3 rotierende Screens (Grid/PV, SOC, Status)
//
//  Arduino IDE / PlatformIO:
//    Board:   ESP32C3 Dev Module
//    Libs:    ArduinoJson, ESPAsyncWebServer (mathieucarbou),
//             AsyncTCP (mathieucarbou), U8g2 (olikraus)
//    USB CDC On Boot: Enabled
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <U8g2lib.h>
#include <Wire.h>

// ============================================================
//  KONSTANTEN
// ============================================================
#define VB_VERSION     "1.4.1"
#define OTA_HOSTNAME     "voltbinds"
#define OTA_PASSWORD     "voltbinds2026"
#define AP_SSID          "VoltBinds-Setup"

#define OLED_SDA         5
#define OLED_SCL         6
#define OLED_ROTATE_MS   5000   // Display-Wechsel alle 5s

#define MAX_STORAGE      3
#define MARSTEK_PORT     502
#define OFFLINE_THR      5      // Fehler bis offline-Flag

// ============================================================
//  OLED  (SSD1306, 72x40px, I2C)
//  U8g2: voller Buffer, kein Page-Mode noetig auf 72x40
// ============================================================
// U8g2 Konstruktor: (rotation, reset, SCL, SDA)
// GPIO6 = SCL, GPIO5 = SDA  - Reihenfolge ist kritisch!
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0,
  /* reset=*/ U8X8_PIN_NONE,
  /* scl=*/   6,
  /* sda=*/   5);

// ============================================================
//  GLOBALE OBJEKTE
// ============================================================
WebServer  server(80);
DNSServer  dnsServer;
Preferences prefs;

// ============================================================
//  KONFIGURATIONSSTRUKTUR  (NVS-persistent)
// ============================================================
struct Config {
  // WiFi
  char wifi_ssid[64]      = "";
  char wifi_password[64]  = "";

  // Smartmeter: 0=Sonnenbatterie API, 1=Shelly 3EM
  int  meter_type         = 0;

  // Sonnenbatterie
  char sonnen_ip[32]      = "192.168.1.237";
  char sonnen_token[48]   = "";

  // Shelly 3EM (meter_type==1)
  char shelly_ip[32]      = "192.168.1.100";

  // Marstek (bis zu 3 Geraete)
  int  marstek_count      = 1;
  char marstek_ip[3][32]  = {"192.168.1.218", "", ""};

  // Regelparameter
  int   grid_tolerance_w          = 30;
  int   sonnen_charge_threshold_w = 4300;
  int   marstek_max_discharge_w   = 1800;
  int   marstek_max_charge_w      = 1800;
  int   marstek_min_soc           = 20;
  int   sonnen_min_soc            = 10;
  int   high_load_threshold_w     = 4500;
  int   marstek_ramp_w            = 30;
  int   startup_delay_s           = 90;
  float marstek_verbrauch_faktor  = 0.70f;
  int   einspeisung_reserve_w     = 80;
};
Config cfg;

// ============================================================
//  SYSTEMZUSTAND
// ============================================================
bool wifi_connected    = false;
bool setup_mode        = false;
bool controller_active = true;
bool startup_complete  = false;
unsigned long mode_change_time = 0;

// ============================================================
//  DATENMODELL
// ============================================================

// --- Powermeter ---
struct MeterData {
  float pv_w       = 0;    // PV-Produktion
  float grid_w     = 0;    // pos=Einspeisung, neg=Bezug
  float house_w    = 0;    // Hausverbrauch
  int   sonnen_soc = 0;    // SOC Sonnenbatterie (%)
  float sonnen_w   = 0;    // pos=Entladen, neg=Laden
};
MeterData meter;

// --- Marstek pro Geraet ---
struct MarstekState {
  int   soc         = 0;
  float power_w     = 0;   // Istwert (pos=Entladen, neg=Laden)
  int   current_w   = 0;   // Sollwert
  int   fail_count  = 0;
  bool  offline     = false;
  bool  manual_override = false;
  int   manual_power    = 0;
};
MarstekState mt[MAX_STORAGE];

// --- Regelung ---
int   marstek_target_w     = 0;
int   marstek_last_target  = 0;

// Verbrauchsringpuffer (5 x 5s = 25s Glaettung)
const int  CONS_BUF = 5;
float cons_buf[CONS_BUF] = {350, 350, 350, 350, 350};
int   cons_idx = 0;
float last_valid_cons_w = 350;

// ============================================================
//  LOGGING
// ============================================================
String logBuffer = "";
void addLog(const String& msg) {
  Serial.println(msg);
  logBuffer = String(millis() / 1000) + "s: " + msg + "\n" + logBuffer;
  if (logBuffer.length() > 3000) logBuffer = logBuffer.substring(0, 3000);
}

// ============================================================
//  NVS  laden / speichern
// ============================================================
void config_load() {
  prefs.begin("vb", true);
  prefs.getString("wifi_ssid",  cfg.wifi_ssid,     sizeof(cfg.wifi_ssid));
  prefs.getString("wifi_pw",    cfg.wifi_password,  sizeof(cfg.wifi_password));
  cfg.meter_type   = prefs.getInt("mtype",    cfg.meter_type);
  prefs.getString("sonnen_ip",  cfg.sonnen_ip,      sizeof(cfg.sonnen_ip));
  prefs.getString("sonnen_tok", cfg.sonnen_token,   sizeof(cfg.sonnen_token));
  prefs.getString("shelly_ip",  cfg.shelly_ip,      sizeof(cfg.shelly_ip));
  cfg.marstek_count = prefs.getInt("mt_cnt",  cfg.marstek_count);
  cfg.marstek_count = max(1, min(3, cfg.marstek_count));
  for (int i = 0; i < 3; i++)
    prefs.getString(("mt_ip" + String(i)).c_str(), cfg.marstek_ip[i], sizeof(cfg.marstek_ip[i]));
  cfg.grid_tolerance_w          = prefs.getInt("r_tol",   cfg.grid_tolerance_w);
  cfg.sonnen_charge_threshold_w = prefs.getInt("r_sbthr", cfg.sonnen_charge_threshold_w);
  cfg.marstek_max_discharge_w   = prefs.getInt("r_mtdis", cfg.marstek_max_discharge_w);
  cfg.marstek_max_charge_w      = prefs.getInt("r_mtchg", cfg.marstek_max_charge_w);
  cfg.marstek_min_soc           = prefs.getInt("r_mtsoc", cfg.marstek_min_soc);
  cfg.sonnen_min_soc            = prefs.getInt("r_sbsoc", cfg.sonnen_min_soc);
  cfg.high_load_threshold_w     = prefs.getInt("r_hlt",   cfg.high_load_threshold_w);
  cfg.marstek_ramp_w            = prefs.getInt("r_ramp",  cfg.marstek_ramp_w);
  cfg.startup_delay_s           = prefs.getInt("r_sdly",  cfg.startup_delay_s);
  cfg.marstek_verbrauch_faktor  = prefs.getFloat("r_fak", cfg.marstek_verbrauch_faktor);
  cfg.einspeisung_reserve_w     = prefs.getInt("r_eres",  cfg.einspeisung_reserve_w);
  marstek_last_target           = prefs.getInt("mt_last", 0);
  prefs.end();
}

void config_save() {
  prefs.begin("vb", false);
  prefs.putString("wifi_ssid",  cfg.wifi_ssid);
  prefs.putString("wifi_pw",    cfg.wifi_password);
  prefs.putInt("mtype",         cfg.meter_type);
  prefs.putString("sonnen_ip",  cfg.sonnen_ip);
  prefs.putString("sonnen_tok", cfg.sonnen_token);
  prefs.putString("shelly_ip",  cfg.shelly_ip);
  prefs.putInt("mt_cnt",        cfg.marstek_count);
  for (int i = 0; i < 3; i++)
    prefs.putString(("mt_ip" + String(i)).c_str(), cfg.marstek_ip[i]);
  prefs.putInt("r_tol",   cfg.grid_tolerance_w);
  prefs.putInt("r_sbthr", cfg.sonnen_charge_threshold_w);
  prefs.putInt("r_mtdis", cfg.marstek_max_discharge_w);
  prefs.putInt("r_mtchg", cfg.marstek_max_charge_w);
  prefs.putInt("r_mtsoc", cfg.marstek_min_soc);
  prefs.putInt("r_sbsoc", cfg.sonnen_min_soc);
  prefs.putInt("r_hlt",   cfg.high_load_threshold_w);
  prefs.putInt("r_ramp",  cfg.marstek_ramp_w);
  prefs.putInt("r_sdly",  cfg.startup_delay_s);
  prefs.putFloat("r_fak", cfg.marstek_verbrauch_faktor);
  prefs.putInt("r_eres",  cfg.einspeisung_reserve_w);
  prefs.end();
}

// ============================================================
//  INTERFACES
// ============================================================

// IPowermeter: abstrakt - SonnenAdapter und ShellyAdapter implementieren
class IPowermeter {
public:
  virtual bool  poll()              = 0;
  virtual float getGridW()    const = 0;   // pos=Einspeisung, neg=Bezug
  virtual float getPvW()      const = 0;
  virtual float getHouseW()   const = 0;
  virtual int   getSonnenSoc() const { return 0; }
  virtual float getSonnenW()   const { return 0; }
  virtual bool  setSonnenAuto()                { return false; }
  virtual bool  setSonnenDischarge(int watt)   { return false; }
  virtual ~IPowermeter() {}
};

// IStorage: abstrakt - MarstekAdapter implementiert
class IStorage {
public:
  virtual bool poll()                  = 0;   // SoC lesen
  virtual bool pollPower()             = 0;   // Istwatt lesen
  virtual bool setPower(int watt)      = 0;   // pos=Entladen, neg=Laden, 0=Stop
  virtual int  getSoc()  const         = 0;
  virtual float getPowerW() const      = 0;
  virtual bool isOnline() const        = 0;
  virtual ~IStorage() {}
};

// ============================================================
//  MODBUS TCP HILFSFUNKTIONEN
// ============================================================
bool modbus_read(const char* ip, uint16_t reg, uint16_t cnt, uint16_t* out) {
  WiFiClient c; c.setTimeout(1500);
  if (!c.connect(ip, MARSTEK_PORT)) return false;
  uint8_t req[12] = {0,1, 0,0, 0,6, 1, 3,
    (uint8_t)(reg>>8),(uint8_t)(reg&0xFF),
    (uint8_t)(cnt>>8),(uint8_t)(cnt&0xFF)};
  c.write(req, 12); delay(200);
  if (c.available() < 9) { c.stop(); return false; }
  uint8_t resp[256];
  int len = c.read(resp, sizeof(resp)); c.stop();
  if (len < 9 || resp[7] != 3) return false;
  for (int i = 0; i < cnt && (9+i*2+1) < len; i++)
    out[i] = (resp[9+i*2]<<8)|resp[10+i*2];
  return true;
}

bool modbus_write(const char* ip, uint16_t reg, uint16_t val) {
  WiFiClient c; c.setTimeout(1500);
  if (!c.connect(ip, MARSTEK_PORT)) return false;
  uint8_t req[12] = {0,1, 0,0, 0,6, 1, 6,
    (uint8_t)(reg>>8),(uint8_t)(reg&0xFF),
    (uint8_t)(val>>8),(uint8_t)(val&0xFF)};
  c.write(req, 12); delay(200);
  uint8_t resp[12];
  int len = c.read(resp, sizeof(resp)); c.stop();
  return (len >= 8 && resp[7] == 6);
}

// ============================================================
//  MARSTEK ADAPTER  (Modbus TCP only)
//  Implementiert IStorage
// ============================================================
class MarstekAdapter : public IStorage {
public:
  MarstekAdapter(int idx) : _idx(idx) {}

  // SoC lesen  (Register 37005)
  bool modbus_read_soc(int& soc_out) {
    uint16_t v = 0;
    if (!modbus_read(cfg.marstek_ip[_idx], 37005, 1, &v)) return false;
    if (v < 1 || v > 100) return false;
    soc_out = (int)v;
    return true;
  }

  // Power lesen  (Register 30001, int16: pos=Entladen, neg=Laden)
  bool modbus_read_power(float& pw_out) {
    uint16_t v = 0;
    if (!modbus_read(cfg.marstek_ip[_idx], 30001, 1, &v)) return false;
    pw_out = (float)(int16_t)v;
    return true;
  }

  // Entladen setzen
  bool modbus_discharge(int watt) {
    watt = constrain((watt/50)*50, 0, 2500);
    const char* ip = cfg.marstek_ip[_idx];
    if (!modbus_write(ip, 42000, 21930)) return false; delay(100);
    if (!modbus_write(ip, 42010, 2))     return false; delay(100);
    return modbus_write(ip, 42021, (uint16_t)watt);
  }

  // Laden setzen
  bool modbus_charge(int watt) {
    watt = constrain((watt/50)*50, 0, 2500);
    const char* ip = cfg.marstek_ip[_idx];
    if (!modbus_write(ip, 42000, 21930)) return false; delay(100);
    if (!modbus_write(ip, 42010, 1))     return false; delay(100);
    return modbus_write(ip, 42020, (uint16_t)watt);
  }

  // Stop / Auto
  bool modbus_stop() {
    const char* ip = cfg.marstek_ip[_idx];
    if (!modbus_write(ip, 42000, 21930)) return false; delay(100);
    return modbus_write(ip, 42010, 0);
  }

  // IStorage::poll  - SoC lesen via Modbus
  bool poll() override {
    if (strlen(cfg.marstek_ip[_idx]) < 7) return false;
    int soc_val = -1;
    bool ok = modbus_read_soc(soc_val);
    if (ok) {
      mt[_idx].soc = soc_val;
      mt[_idx].fail_count = 0;
      if (mt[_idx].offline) {
        mt[_idx].offline = false;
        addLog("Marstek " + String(_idx+1) + ": wieder ONLINE");
      }
      addLog("Marstek " + String(_idx+1) + ": SoC=" + String(soc_val) + "%");
    } else {
      mt[_idx].fail_count++;
      if (mt[_idx].fail_count >= OFFLINE_THR && !mt[_idx].offline) {
        mt[_idx].offline = true;
        addLog("Marstek " + String(_idx+1) + ": OFFLINE nach " + String(mt[_idx].fail_count) + " Fehlern");
      }
    }
    return ok;
  }

  // IStorage::pollPower
  bool pollPower() override {
    if (strlen(cfg.marstek_ip[_idx]) < 7) return false;
    float pw = 0;
    bool ok = modbus_read_power(pw);
    if (ok) mt[_idx].power_w = pw;
    return ok;
  }

  // IStorage::setPower  (pos=Entladen, neg=Laden, 0=Stop)
  bool setPower(int watt) override {
    if (strlen(cfg.marstek_ip[_idx]) < 7) return false;
    mt[_idx].current_w = watt;
    bool ok;
    if      (watt > 0) ok = modbus_discharge(watt);
    else if (watt < 0) ok = modbus_charge(abs(watt));
    else               ok = modbus_stop();
    addLog("Marstek " + String(_idx+1) + ": " + String(watt) + "W -> " + (ok ? "OK" : "FEHLER"));
    return ok;
  }

  int   getSoc()    const override { return mt[_idx].soc; }
  float getPowerW() const override { return mt[_idx].power_w; }
  bool  isOnline()  const override { return !mt[_idx].offline && strlen(cfg.marstek_ip[_idx]) > 6; }

private:
  int _idx;
};

// ============================================================
//  SONNENBATTERIE ADAPTER  (Powermeter + Storage)
//  Implementiert IPowermeter
// ============================================================
class SonnenAdapter : public IPowermeter {
public:
  bool poll() override {
    HTTPClient http;
    http.begin("http://" + String(cfg.sonnen_ip) + "/api/v2/status");
    http.addHeader("Auth-Token", cfg.sonnen_token);
    http.setTimeout(3000);
    int code = http.GET();
    if (code != 200) { http.end(); addLog("Sonnen Fehler HTTP " + String(code)); return false; }
    StaticJsonDocument<1024> doc;
    if (deserializeJson(doc, http.getStream()) != DeserializationError::Ok) {
      http.end(); addLog("Sonnen: JSON Fehler"); return false;
    }
    _pv    = doc["Production_W"]  | 0.0f;
    _grid  = doc["GridFeedIn_W"]  | 0.0f;   // pos=Einspeisung, neg=Bezug
    _house = doc["Consumption_W"] | 0.0f;
    _soc   = doc["USOC"]          | 0;
    _pw    = doc["Pac_total_W"]   | 0.0f;    // pos=Entladen, neg=Laden
    http.end();
    return true;
  }

  bool setSonnenAuto() override {
    HTTPClient http;
    http.begin("http://" + String(cfg.sonnen_ip) + "/api/v2/configurations");
    http.addHeader("Auth-Token", cfg.sonnen_token);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(3000);
    http.PUT("{\"EM_OperatingMode\": \"2\"}");
    http.end();
    addLog("Sonnen: Auto-Modus (OperatingMode=2)");
    return true;
  }

  bool setSonnenDischarge(int watt) override {
    HTTPClient http;
    http.begin("http://" + String(cfg.sonnen_ip) +
               "/api/v2/setpoint/discharge/" + String(watt));
    http.addHeader("Auth-Token", cfg.sonnen_token);
    http.setTimeout(3000);
    int code = http.POST(""); http.end();
    addLog("Sonnen Entladen: " + String(watt) + "W -> HTTP " + String(code));
    return (code == 200 || code == 201);
  }

  float getGridW()     const override { return _grid; }
  float getPvW()       const override { return _pv; }
  float getHouseW()    const override { return _house; }
  int   getSonnenSoc() const override { return _soc; }
  float getSonnenW()   const override { return _pw; }

private:
  float _pv = 0, _grid = 0, _house = 0, _pw = 0;
  int   _soc = 0;
};

// ============================================================
//  SHELLY 3EM ADAPTER  (Nur Smartmeter, keine Steuerung)
//  Implementiert IPowermeter
// ============================================================
class ShellyAdapter : public IPowermeter {
public:
  bool poll() override {
    HTTPClient http;
    http.begin("http://" + String(cfg.shelly_ip) + "/status");
    http.setTimeout(3000);
    int code = http.GET();
    if (code != 200) { http.end(); addLog("Shelly Fehler HTTP " + String(code)); return false; }
    StaticJsonDocument<768> doc;
    if (deserializeJson(doc, http.getStream()) != DeserializationError::Ok) {
      http.end(); addLog("Shelly: JSON Fehler"); return false;
    }
    http.end();
    float total = 0;
    JsonArray em = doc["emeters"];
    for (JsonObject phase : em) total += phase["power"].as<float>();
    // Shelly: positiv=Bezug -> umkehren auf VoltBinds-Konvention: pos=Einspeisung
    _grid = -total;
    addLog("Shelly Grid: " + String(_grid, 0) + "W");
    return true;
  }
  float getGridW()  const override { return _grid; }
  float getPvW()    const override { return 0; }   // Shelly kennt keine PV
  float getHouseW() const override { return 0; }   // Shelly kennt keinen Hausverbrauch
private:
  float _grid = 0;
};

// ============================================================
//  ADAPTER INSTANZEN
// ============================================================
SonnenAdapter sonnenAdapter;
ShellyAdapter shellyAdapter;
MarstekAdapter* marstekAdapters[MAX_STORAGE] = {nullptr, nullptr, nullptr};

IPowermeter* activeMeter = nullptr;

void adapters_init() {
  // Powermeter waehlen
  activeMeter = (cfg.meter_type == 1)
    ? (IPowermeter*)&shellyAdapter
    : (IPowermeter*)&sonnenAdapter;

  // Marstek-Instanzen anlegen
  for (int i = 0; i < MAX_STORAGE; i++) {
    delete marstekAdapters[i];
    marstekAdapters[i] = nullptr;
  }
  for (int i = 0; i < cfg.marstek_count; i++)
    marstekAdapters[i] = new MarstekAdapter(i);
}

// ============================================================
//  RINGPUFFER  (Verbrauchsglaettung)
// ============================================================
void cons_update(float v) {
  if (v > 50) {
    last_valid_cons_w = v;
    cons_buf[cons_idx] = v;
    cons_idx = (cons_idx + 1) % CONS_BUF;
  }
}
float cons_avg() {
  float s = 0; for (int i = 0; i < CONS_BUF; i++) s += cons_buf[i]; return s / CONS_BUF;
}

// ============================================================
//  REGELLOGIK  (RuleEngine - nur Interface-Zugriffe)
// ============================================================
void run_control() {
  // Controller AUS: diese Funktion wird im loop() gar nicht aufgerufen.
  // Einmaliger Stop erfolgt in handleSet() beim Umschalten.
  // Hier nur als Schutz falls doch aufgerufen:
  if (!controller_active) return;

  // Manual Override
  bool any_override = false;
  for (int i = 0; i < cfg.marstek_count; i++) {
    if (mt[i].manual_override && marstekAdapters[i]) {
      marstekAdapters[i]->setPower(mt[i].manual_power);
      any_override = true;
    }
  }
  if (any_override) return;

  // Startup Delay
  if (!startup_complete) {
    if (millis() < (unsigned long)cfg.startup_delay_s * 1000) {
      addLog("Startup: warte " + String(cfg.startup_delay_s - millis()/1000) + "s");
      for (int i = 0; i < cfg.marstek_count; i++)
        if (marstekAdapters[i]) marstekAdapters[i]->setPower(0);
      return;
    }
    startup_complete = true;
    // Sonnenbatterie auf Auto (nur wenn SonnenAdapter aktiv)
    activeMeter->setSonnenAuto();
    for (int i = 0; i < cfg.marstek_count; i++)
      mt[i].current_w = marstek_last_target;
    addLog("Startup: bereit!");
  }

  // Messwerte aus aktivem Meter holen
  float pv_w   = activeMeter->getPvW();
  float grid_w = activeMeter->getGridW();   // pos=Einspeisung, neg=Bezug
  float house_w= activeMeter->getHouseW();
  int   sb_soc = activeMeter->getSonnenSoc();
  float sb_w   = activeMeter->getSonnenW();  // pos=Entladen, neg=Laden

  // Meter-Daten global fuer OLED / Dashboard verfuegbar halten
  meter.pv_w       = pv_w;
  meter.grid_w     = grid_w;
  meter.house_w    = house_w;
  meter.sonnen_soc = sb_soc;
  meter.sonnen_w   = sb_w;

  // Verbrauchsringpuffer
  if (house_w > 50) cons_update(house_w);

  // PV-Hysterese: Tag / Nacht
  static bool pv_stable = false;
  static int  pv_cnt    = 0;
  bool pv_raw = (pv_w > 150);
  if (pv_raw != pv_stable) {
    pv_cnt++;
    if (pv_cnt >= (pv_raw ? 3 : 1)) { pv_stable = pv_raw; pv_cnt = 0; mode_change_time = millis(); }
  } else { pv_cnt = 0; }
  bool pv_active = pv_stable || (house_w == 0 && pv_w > 200);

  static bool last_pv = false;
  if (pv_active != last_pv) {
    last_pv = pv_active;
    addLog(pv_active ? "-> TAG-Modus" : "-> NACHT-Modus");
  }

  addLog("Regel: PV=" + String(pv_w,0) + "W Grid=" + String(grid_w,0) +
         "W Haus=" + String(house_w,0) + "W SB=" + String(sb_w,0) + "W");

  // ---- TAG ----
  if (pv_active) {
    if (millis() - mode_change_time < 10000) {
      // 10s Uebergangs-Rampe
      for (int i = 0; i < cfg.marstek_count; i++) {
        if (!marstekAdapters[i]) continue;
        if (mt[i].current_w > 0) {
          mt[i].current_w = max(0, mt[i].current_w - cfg.marstek_ramp_w);
          marstekAdapters[i]->setPower(mt[i].current_w);
        }
      }
      return;
    }

    float sb_chg = (sb_w < 0) ? abs(sb_w) : 0;

    if (sb_chg >= cfg.sonnen_charge_threshold_w || sb_soc >= 98) {
      // SB laedt stark / voll -> Marstek bei Ueberschuss laden
      float surplus = pv_w - house_w + abs(mt[0].power_w);
      if (surplus > cfg.grid_tolerance_w) {
        int chg = min((int)(surplus / max(1, cfg.marstek_count)), cfg.marstek_max_charge_w);
        addLog("Tag: SB voll -> Marstek laedt " + String(chg) + "W");
        for (int i = 0; i < cfg.marstek_count; i++) {
          if (!marstekAdapters[i]) continue;
          if (mt[i].soc < 99) marstekAdapters[i]->setPower(-chg);
          else                 marstekAdapters[i]->setPower(0);
        }
      } else if (grid_w < -cfg.grid_tolerance_w) {
        // Netzbezug trotz vollem SB -> Marstek hilft
        int help = min((int)abs(grid_w), cfg.marstek_max_discharge_w);
        addLog("Tag: Bezug -> Marstek " + String(help) + "W");
        for (int i = 0; i < cfg.marstek_count; i++) {
          if (!marstekAdapters[i]) continue;
          if (mt[i].soc > cfg.marstek_min_soc) marstekAdapters[i]->setPower(help / max(1, cfg.marstek_count));
          else                                  marstekAdapters[i]->setPower(0);
        }
      } else {
        for (int i = 0; i < cfg.marstek_count; i++)
          if (marstekAdapters[i]) marstekAdapters[i]->setPower(0);
      }
    } else {
      addLog("Tag: SB laedt -> Marstek Standby");
      for (int i = 0; i < cfg.marstek_count; i++)
        if (marstekAdapters[i]) marstekAdapters[i]->setPower(0);
    }
    return;
  }

  // ---- NACHT ----
  bool any_ok = false;
  for (int i = 0; i < cfg.marstek_count; i++)
    if (marstekAdapters[i] && marstekAdapters[i]->isOnline() && mt[i].soc > cfg.marstek_min_soc)
      any_ok = true;

  if (!any_ok) {
    addLog("Nacht: Marstek leer/offline -> SB alleine");
    for (int i = 0; i < cfg.marstek_count; i++)
      if (marstekAdapters[i]) marstekAdapters[i]->setPower(0);
    return;
  }

  // SB laedt -> Marstek stoppen (kein Energiekreisel)
  if (sb_w < -200) {
    addLog("Nacht: SB laedt -> Marstek gestoppt");
    for (int i = 0; i < cfg.marstek_count; i++)
      if (marstekAdapters[i]) marstekAdapters[i]->setPower(0);
    return;
  }

  float verbrauch = cons_avg();
  int target_w;
  if (verbrauch > cfg.high_load_threshold_w)
    target_w = min((int)(verbrauch - cfg.high_load_threshold_w), cfg.marstek_max_discharge_w);
  else
    target_w = (int)(verbrauch * cfg.marstek_verbrauch_faktor);
  target_w = constrain(target_w, 0, cfg.marstek_max_discharge_w);
  marstek_target_w = target_w;

  for (int i = 0; i < cfg.marstek_count; i++) {
    if (!marstekAdapters[i] || !marstekAdapters[i]->isOnline()) { mt[i].current_w = 0; continue; }
    if (mt[i].soc <= cfg.marstek_min_soc)  { marstekAdapters[i]->setPower(0); mt[i].current_w = 0; continue; }

    int t = target_w / max(1, cfg.marstek_count);
    // Soft-Stop nahe Min-SOC
    int soft = cfg.marstek_min_soc + 10;
    if (mt[i].soc <= soft)
      t = (int)(t * constrain((float)(mt[i].soc - cfg.marstek_min_soc) / 10.0f, 0.0f, 1.0f));
    // Soft-Start Rampe
    mt[i].current_w = (t > mt[i].current_w)
      ? min(t, mt[i].current_w + cfg.marstek_ramp_w)
      : t;
    if (mt[i].current_w < 0) mt[i].current_w = 0;
    marstekAdapters[i]->setPower(mt[i].current_w);
  }

  // Sonnenbatterie Maximalentladung setzen
  int sb_max = (int)(verbrauch * (1.0f - cfg.marstek_verbrauch_faktor)) + cfg.einspeisung_reserve_w;
  sb_max = constrain(sb_max, 0, 4500);
  if (sb_soc <= cfg.sonnen_min_soc) { sb_max = 0; addLog("Nacht: SB leer -> gestoppt"); }
  activeMeter->setSonnenDischarge(sb_max);

  if (target_w != marstek_last_target) {
    marstek_last_target = target_w;
    prefs.begin("vb", false);
    prefs.putInt("mt_last", marstek_last_target);
    prefs.end();
  }
  addLog("Nacht: Verbr=" + String(verbrauch,0) + "W MT=" + String(target_w) +
         "W SB-Max=" + String(sb_max) + "W");
}

// ============================================================
//  OLED DISPLAY  (72x40px, rotierend)
// ============================================================
// Screen 0: Energie-Uebersicht   (PV | Netz)
// Screen 1: Speicher-SOC         (Sonnen | Marstek 1..3)
// Screen 2: Status               (Modus | Controller AN/AUS | IP)

uint8_t oled_screen   = 0;
unsigned long oled_last_rotate = 0;

// Hilfsfunktion: Watt kompakt formatieren ("1.2kW" / "850W")
String fmtW(float w) {
  if (abs(w) >= 1000) return String(w / 1000.0f, 1) + "k";
  return String((int)w) + "W";
}

void oled_draw() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tr);   // kleinstmoegliche lesbare Schrift

  if (setup_mode) {
    // Setup-Screen: IP des AP
    u8g2.drawStr(0, 7,  "VoltBinds");
    u8g2.drawStr(0, 16, "Setup-Modus");
    u8g2.drawStr(0, 25, AP_SSID);
    u8g2.drawStr(0, 34, "192.168.4.1");
    u8g2.sendBuffer();
    return;
  }

  // Keine WiFi-Verbindung
  if (!wifi_connected) {
    u8g2.drawStr(0, 7,  "VoltBinds");
    u8g2.drawStr(0, 16, "Verbinde...");
    u8g2.sendBuffer();
    return;
  }

  switch (oled_screen) {
    // ---- Screen 0: Energie ----
    case 0: {
      // Zeile 1: Header
      u8g2.drawStr(0, 7, "PV");
      u8g2.drawStr(38, 7, "Netz");
      u8g2.drawHLine(0, 9, 72);

      // PV-Wert
      String pv_str = fmtW(meter.pv_w);
      u8g2.setFont(u8g2_font_7x13B_tr);
      u8g2.drawStr(0, 24, pv_str.c_str());

      // Grid-Wert  (pos=Einspeisung ^, neg=Bezug v)
      String gw_str = fmtW(abs(meter.grid_w));
      u8g2.drawStr(38, 24, gw_str.c_str());

      u8g2.setFont(u8g2_font_5x7_tr);
      // Richtungs-Pfeile
      if (meter.grid_w > cfg.grid_tolerance_w)       u8g2.drawStr(66, 24, "^");
      else if (meter.grid_w < -cfg.grid_tolerance_w) u8g2.drawStr(66, 24, "v");

      // Zeile 3: Haus + Modus
      String mode_str = meter.pv_w > 150 ? "Tag" : "Nacht";
      u8g2.drawStr(0, 35, ("H:" + fmtW(meter.house_w) + "  " + mode_str).c_str());
      break;
    }

    // ---- Screen 1: SOC ----
    case 1: {
      u8g2.drawStr(0, 7, "Speicher SOC");
      u8g2.drawHLine(0, 9, 72);

      // Sonnenbatterie
      u8g2.setFont(u8g2_font_5x7_tr);
      String sb_line = "SB: " + String(meter.sonnen_soc) + "%";
      if (meter.sonnen_w < -50)      sb_line += " <" + fmtW(abs(meter.sonnen_w));
      else if (meter.sonnen_w > 50)  sb_line += " >" + fmtW(meter.sonnen_w);
      u8g2.drawStr(0, 19, sb_line.c_str());

      // SOC-Balken Sonnenbatterie
      int sb_bar = (meter.sonnen_soc * 40) / 100;
      u8g2.drawFrame(0, 21, 40, 4);
      if (sb_bar > 0) u8g2.drawBox(0, 21, sb_bar, 4);

      // Marstek
      for (int i = 0; i < cfg.marstek_count && i < 2; i++) {
        int y = 30 + i * 8;
        String line = "M" + String(i+1) + ": ";
        if (mt[i].offline)     line += "offline";
        else {
          line += String(mt[i].soc) + "%";
          if      (mt[i].current_w > 50)   line += " >" + fmtW(mt[i].current_w);
          else if (mt[i].current_w < -50)  line += " <" + fmtW(abs(mt[i].current_w));
        }
        u8g2.drawStr(0, y + 7, line.c_str());
      }
      break;
    }

    // ---- Screen 2: Status ----
    case 2: {
      u8g2.drawStr(0, 7, "VoltBinds v" VB_VERSION);
      u8g2.drawHLine(0, 9, 72);
      // IP
      u8g2.drawStr(0, 18, WiFi.localIP().toString().c_str());
      // Controller AN/AUS
      u8g2.drawStr(0, 27, controller_active ? "Regelung: AN" : "Regelung: AUS");
      // Meter-Typ
      u8g2.drawStr(0, 36, cfg.meter_type == 0 ? "Meter: Sonnen" : "Meter: Shelly");
      break;
    }
  }
  u8g2.sendBuffer();
}

void oled_update() {
  // Rotating: alle OLED_ROTATE_MS Sekunden naechsten Screen
  if (millis() - oled_last_rotate > OLED_ROTATE_MS) {
    oled_last_rotate = millis();
    oled_screen = (oled_screen + 1) % 3;
  }
  oled_draw();
}

// ============================================================
//  WIFI
// ============================================================
void wifi_connect() {
  if (strlen(cfg.wifi_ssid) == 0) return;
  addLog("WiFi: verbinde mit " + String(cfg.wifi_ssid));
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.wifi_ssid, cfg.wifi_password);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(300); oled_draw();  // Display lebendig halten
  }
  wifi_connected = (WiFi.status() == WL_CONNECTED);
  if (wifi_connected) addLog("WiFi: IP=" + WiFi.localIP().toString());
  else                addLog("WiFi: Verbindung fehlgeschlagen");
}

void start_ap_mode() {
  setup_mode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, "");
  dnsServer.start(53, "*", WiFi.softAPIP());
  addLog("AP: " + String(AP_SSID) + " " + WiFi.softAPIP().toString());
}

// ============================================================
//  WEBSERVER: LANDING PAGE (Setup)
// ============================================================
void handleSetupPage() {
  String h = "";
  h += "<!DOCTYPE html><html lang=\"de\"><head>";
  h += "<meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
  h += "<title>VoltBinds Setup</title>";
  h += "<style>";
  h += "*{box-sizing:border-box;margin:0;padding:0}";
  h += "body{font-family:sans-serif;background:#0f1117;color:#e0e0e0;min-height:100vh;display:flex;align-items:center;justify-content:center;padding:16px}";
  h += ".card{background:#1a1d27;border-radius:12px;padding:28px;width:100%;max-width:480px}";
  h += "h1{font-size:1.35rem;color:#f5c518;margin-bottom:4px}";
  h += ".ver{color:#555;font-size:.78rem;margin-bottom:20px}";
  h += "h2{font-size:.82rem;color:#888;text-transform:uppercase;letter-spacing:.07em;margin:18px 0 8px;padding-top:14px;border-top:1px solid #23263a}";
  h += "label{display:block;font-size:.83rem;color:#aaa;margin:10px 0 3px}";
  h += "input,select{width:100%;padding:9px 12px;background:#0f1117;border:1px solid #2a2d3a;border-radius:7px;color:#e0e0e0;font-size:.88rem;outline:none}";
  h += "input:focus,select:focus{border-color:#f5c518}";
  h += ".note{font-size:.75rem;color:#555;margin-top:4px}";
  h += ".btn{width:100%;margin-top:22px;padding:12px;background:#f5c518;color:#0f1117;font-weight:700;font-size:.98rem;border:none;border-radius:8px;cursor:pointer}";
  h += ".btn:hover{background:#ffd740}";
  h += "</style></head><body><div class=\"card\">";
  h += "<h1>VoltBinds</h1>";
  h += "<div class=\"ver\">v" VB_VERSION " - Einrichtung</div>";
  h += "<form action=\"/save-setup\" method=\"POST\">";

  h += "<h2>WLAN</h2>";
  // Aktuelle IP anzeigen wenn bereits verbunden
  if (wifi_connected) {
    h += "<div style=\"background:#12141c;border-radius:7px;padding:9px 12px;margin-bottom:10px;font-size:.82rem;\">";
    h += "Verbunden &bull; IP: <strong style=\"color:#4caf50;\">";
    h += WiFi.localIP().toString();
    h += "</strong></div>";
  }
  h += "<label>WLAN-Netzwerk</label>";
  // WiFi-Scan und Dropdown
  int n_networks = WiFi.scanNetworks();
  h += "<select name=\"wifi_ssid\" required style=\"margin-bottom:4px;\">";
  h += "<option value=\"\">-- Netzwerk waehlen --</option>";
  for (int si = 0; si < n_networks && si < 20; si++) {
    String ssid_i = WiFi.SSID(si);
    int rssi_i    = WiFi.RSSI(si);
    String bars   = rssi_i > -60 ? "[****]" : rssi_i > -70 ? "[*** ]" : rssi_i > -80 ? "[**  ]" : "[*   ]";
    String sel_i  = (ssid_i == String(cfg.wifi_ssid)) ? " selected" : "";
    h += "<option value=\"" + ssid_i + "\"" + sel_i + ">" + bars + " " + ssid_i + " (" + String(rssi_i) + " dBm)</option>";
  }
  WiFi.scanDelete();
  h += "</select>";
  h += "<label>WLAN-Passwort</label>";
  h += "<input name=\"wifi_pw\" type=\"password\" placeholder=\"Passwort\">";

  h += "<h2>Smartmeter</h2>";
  h += "<label>Typ</label>";
  h += "<select name=\"meter_type\" onchange=\"document.getElementById('sh').style.display=this.value=='1'?'block':'none'\">";
  h += "<option value=\"0\""; h += (cfg.meter_type == 0 ? " selected" : ""); h += ">Sonnenbatterie (integriert)</option>";
  h += "<option value=\"1\""; h += (cfg.meter_type == 1 ? " selected" : ""); h += ">Shelly 3EM / Pro 3EM</option>";
  h += "</select>";
  h += "<div id=\"sh\" style=\"display:"; h += (cfg.meter_type == 1 ? "block" : "none"); h += "\">";
  h += "<label>Shelly IP-Adresse</label>";
  h += "<input name=\"shelly_ip\" type=\"text\" value=\""; h += String(cfg.shelly_ip); h += "\" placeholder=\"192.168.1.100\">";
  h += "</div>";

  h += "<h2>Sonnenbatterie</h2>";
  h += "<label>IP-Adresse</label>";
  h += "<input name=\"sonnen_ip\" type=\"text\" value=\""; h += String(cfg.sonnen_ip); h += "\" placeholder=\"192.168.1.237\">";
  h += "<label>Auth-Token</label>";
  h += "<input name=\"sonnen_token\" type=\"text\" value=\""; h += String(cfg.sonnen_token); h += "\" placeholder=\"ffe23d58-...\">";
  h += "<div class=\"note\">Token: Sonnenbatterie Web -&gt; Software -&gt; API</div>";

  h += "<h2>Marstek Venus</h2>";
  h += "<label>Anzahl Geraete</label>";
  h += "<select name=\"marstek_count\" onchange=\"updateMt(this.value)\">";
  h += "<option value=\"1\""; h += (cfg.marstek_count == 1 ? " selected" : ""); h += ">1 Geraet</option>";
  h += "<option value=\"2\""; h += (cfg.marstek_count == 2 ? " selected" : ""); h += ">2 Geraete</option>";
  h += "<option value=\"3\""; h += (cfg.marstek_count == 3 ? " selected" : ""); h += ">3 Geraete</option>";
  h += "</select>";
  h += "<div id=\"mt-ips\">";
  h += "<label>Marstek 1 - IP</label>";
  h += "<input name=\"mt_ip0\" type=\"text\" value=\""; h += String(cfg.marstek_ip[0]); h += "\" placeholder=\"192.168.1.218\">";
  if (cfg.marstek_count >= 2) {
    h += "<label>Marstek 2 - IP</label>";
    h += "<input name=\"mt_ip1\" type=\"text\" value=\""; h += String(cfg.marstek_ip[1]); h += "\" placeholder=\"192.168.1.219\">";
  }
  if (cfg.marstek_count >= 3) {
    h += "<label>Marstek 3 - IP</label>";
    h += "<input name=\"mt_ip2\" type=\"text\" value=\""; h += String(cfg.marstek_ip[2]); h += "\" placeholder=\"192.168.1.220\">";
  }
  h += "</div>";
  h += "<button class=\"btn\" type=\"submit\">Speichern &amp; Neustart</button>";
  h += "</form></div>";
  h += "<script>";
  h += "function updateMt(n){";
  h += "var d=document.getElementById('mt-ips');";
  h += "var ips=['192.168.1.218','192.168.1.219','192.168.1.220'];";
  h += "d.innerHTML='';";
  h += "for(var i=0;i<parseInt(n);i++){";
  h += "d.innerHTML+='<label>Marstek '+(i+1)+' - IP</label><input name=\"mt_ip'+i+'\" type=\"text\" placeholder=\"'+ips[i]+'\">';}}";
  h += "</script>";
  h += "</body></html>";
  server.send(200, "text/html", h);
}

void handleSaveSetup() {
  if (server.hasArg("wifi_ssid"))    strncpy(cfg.wifi_ssid,    server.arg("wifi_ssid").c_str(),    63);
  if (server.hasArg("wifi_pw"))      strncpy(cfg.wifi_password, server.arg("wifi_pw").c_str(),     63);
  cfg.meter_type = server.hasArg("meter_type") ? server.arg("meter_type").toInt() : 0;
  if (server.hasArg("sonnen_ip"))    strncpy(cfg.sonnen_ip,    server.arg("sonnen_ip").c_str(),    31);
  if (server.hasArg("sonnen_token")) strncpy(cfg.sonnen_token, server.arg("sonnen_token").c_str(), 47);
  if (server.hasArg("shelly_ip"))    strncpy(cfg.shelly_ip,    server.arg("shelly_ip").c_str(),    31);
  cfg.marstek_count = server.hasArg("marstek_count") ? constrain(server.arg("marstek_count").toInt(),1,3) : 1;
  for (int i = 0; i < 3; i++) {
    String key = "mt_ip" + String(i);
    if (server.hasArg(key)) strncpy(cfg.marstek_ip[i], server.arg(key).c_str(), 31);
    else memset(cfg.marstek_ip[i], 0, 32);
  }
  config_save();

  String h = "";
  h += "<!DOCTYPE html><html lang=\"de\"><head><meta charset=\"UTF-8\">";
  h += "<meta http-equiv=\"refresh\" content=\"6;url=http://voltbinds.local\">";
  h += "<title>VoltBinds</title>";
  h += "<style>body{font-family:sans-serif;background:#0f1117;color:#e0e0e0;display:flex;align-items:center;justify-content:center;min-height:100vh;text-align:center}";
  h += ".c{background:#1a1d27;padding:32px;border-radius:12px;max-width:340px}";
  h += "h1{color:#4caf50;margin-bottom:12px}p{color:#aaa;font-size:.9rem;line-height:1.6}</style>";
  h += "</head><body><div class=\"c\"><h1>Gespeichert!</h1>";
  h += "<p>VoltBinds verbindet sich mit<br><strong>";
  h += String(cfg.wifi_ssid);
  h += "</strong><br><br>Dann erreichbar unter<br><strong>http://voltbinds.local</strong></p>";
  h += "</div></body></html>";
  server.send(200, "text/html", h);
  delay(2000);
  ESP.restart();
}

void handleDashboard() {
  // Dashboard als PROGMEM String
  static const char DASH_HTML[] PROGMEM = R"HTML(<!DOCTYPE html> <html lang="de"><head> <meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"> <title>VoltBinds</title> <style> *{box-sizing:border-box;margin:0;padding:0}body{font-family:sans-serif;background:#0f1117;color:#e0e0e0;padding:14px;min-height:100vh}h1{color:#f5c518;font-size:1.2rem}.sub{color:#555;font-size:.75rem;margin-bottom:14px}.flow{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:14px}.fc{background:#1a1d27;border-radius:9px;padding:11px;text-align:center}.fc .val{font-size:1.25rem;font-weight:700;margin-top:3px}.fc .lbl{font-size:.68rem;color:#666;margin-top:2px}.pos{color:#4caf50}.neg{color:#ef5350}.neu{color:#f5c518}.sec{font-size:.72rem;color:#555;text-transform:uppercase;letter-spacing:.08em;margin:12px 0 6px}.bg{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:9px;margin-bottom:12px}.bt{background:#1a1d27;border-radius:9px;padding:11px}.bt .nm{font-size:.7rem;color:#888;margin-bottom:4px}.bt .sc{font-size:1.45rem;font-weight:700}.bar{height:5px;background:#23263a;border-radius:3px;margin-top:5px;overflow:hidden}.bf{height:100%;border-radius:3px;transition:width .4s}.bt .pw{font-size:.75rem;margin-top:5px}.bt .st{font-size:.7rem;margin-top:3px}.dot{display:inline-block;width:7px;height:7px;border-radius:50%;margin-right:4px}.dok{background:#4caf50}.der{background:#ef5350}.ctl{margin-top:8px;display:flex;gap:6px;flex-wrap:wrap;align-items:center}.ctl .pb{padding:5px 10px;border:1px solid #2a2d3a;background:#12141c;color:#e0e0e0;border-radius:6px;cursor:pointer;font-size:.78rem}.ctl .pb:hover{border-color:#f5c518;color:#f5c518}.ctl .pb.act{background:#f5c518;color:#0f1117;border-color:#f5c518;font-weight:700}.ctl .mi{width:72px;padding:5px 7px;background:#0f1117;border:1px solid #2a2d3a;border-radius:5px;color:#e0e0e0;font-size:.78rem;text-align:right}.ctl .cb{padding:5px 10px;background:#2a2d3a;color:#e0e0e0;border:none;border-radius:6px;cursor:pointer;font-size:.78rem}.ctl .cb:hover{background:#3a3d4a}.btn-row{display:flex;gap:7px;flex-wrap:wrap;margin-bottom:12px}.btn{padding:7px 13px;border:1px solid #23263a;background:#1a1d27;color:#e0e0e0;border-radius:7px;cursor:pointer;font-size:.8rem}.btn:hover{border-color:#f5c518;color:#f5c518}.btn.on{background:#f5c518;color:#0f1117;border-color:#f5c518;font-weight:700}.log{background:#08090e;border-radius:7px;padding:10px;font-family:monospace;font-size:.7rem;color:#777;max-height:180px;overflow-y:auto;white-space:pre-wrap;line-height:1.4;display:none} </style></head><body> <h1>VoltBinds</h1> <div class="sub" id="sl">...</div> <div class="flow"> <div class="fc"><div class="val pos" id="f-pv">-</div><div class="lbl">PV</div></div> <div class="fc"><div class="val" id="f-haus">-</div><div class="lbl">Haus</div></div> <div class="fc"><div class="val" id="f-grid">-</div><div class="lbl">Netz</div></div> </div> <div class="sec">Speicher</div> <div class="bg" id="batts"></div> <div class="sec">Steuerung</div> <div class="btn-row"> <button class="btn" id="b-an" onclick="setActive(1)">AN</button> <button class="btn" id="b-aus" onclick="setActive(0)">AUS</button> <button class="btn" onclick="toggleLog()">Log</button> <a class="btn" href="/setup">Geraete / WLAN</a> </div> <div class="log" id="log-box"></div> <script> var D={};function fw(v){var n=parseFloat(v);return(Math.abs(n)>=1000?(n/1000).toFixed(1)+'kW':Math.round(n)+'W');}function poll(){fetch('/api').then(function(r){return r.json();}).then(function(d){D=d;document.getElementById('f-pv').textContent=fw(d.pv_w||0);document.getElementById('f-haus').textContent=fw(d.house_w||0);var gw=parseFloat(d.grid_w||0);var ge=document.getElementById('f-grid');ge.textContent=(gw>50?'^ ':gw<-50?'v ':'')+fw(Math.abs(gw));ge.className='val '+(gw>50?'pos':gw<-50?'neg':'neu');renderBatts(d);var a=d.active;document.getElementById('b-an').className='btn'+(a?' on':'');document.getElementById('b-aus').className='btn'+(!a?' on':'');var up=parseInt(d.sys_uptime_s||0);document.getElementById('sl').textContent=(d.pv_active?'Tag':'Nacht')+' | '+Math.floor(up/3600)+'h'+Math.floor((up%3600)/60)+'m | IP: '+d.ip;}).catch(function(){});}function renderBatts(d){var b='';var sb=parseInt(d.sonnen_soc||0),sbw=parseFloat(d.sonnen_w||0);var sbc=sb>50?'#4caf50':sb>20?'#f5c518':'#ef5350';var sbpw=sbw>50?'<span class="pos">^ '+fw(sbw)+'</span>':sbw<-50?'<span class="neg">v '+fw(Math.abs(sbw))+'</span>':'<span style="color:#444">Standby</span>';b+='<div class="bt"><div class="nm">Sonnenbatterie</div><div class="sc" style="color:'+sbc+'">'+sb+'%</div><div class="bar"><div class="bf" style="width:'+sb+'%;background:'+sbc+'"></div></div><div class="pw">'+sbpw+'</div></div>';(d.marsteks||[]).forEach(function(m,i){var s=parseInt(m.soc||0),pw=parseFloat(m.power||0);var c=s>50?'#4caf50':s>20?'#f5c518':'#ef5350';var mpw=pw>50?'<span class="pos">^ '+fw(pw)+'</span>':pw<-50?'<span class="neg">v '+fw(Math.abs(pw))+'</span>':'<span style="color:#444">Standby</span>';var st=m.offline?'<span class="dot der"></span>Offline':'<span class="dot dok"></span>Online';var ov=m.override?'<span style="color:#f5c518"> [manuell]</span>':'';b+='<div class="bt"><div class="nm">Marstek '+(i+1)+ov+'</div><div class="sc" style="color:'+c+'">'+s+'%</div><div class="bar"><div class="bf" style="width:'+s+'%;background:'+c+'"></div></div><div class="pw">'+mpw+'</div><div class="st">'+st+'</div><div class="ctl"><button class="pb" onclick="setMt('+i+',-800)">-800W</button><button class="pb" onclick="setMt('+i+',0)">Stop</button><button class="pb" onclick="setMt('+i+',800)">+800W</button></div><div class="ctl"><input class="mi" id="mw'+i+'" type="number" placeholder="Watt" min="-2500" max="2500" step="50"><button class="cb" onclick="setMtManual('+i+')">Setzen</button>'+(m.override?'<button class="cb" onclick="setMt('+i+',0)" style="color:#f5c518">Freigeben</button>':'')+'</div></div>';});document.getElementById('batts').innerHTML=b;}function setMt(idx,w){fetch('/marstek?idx='+idx+'&power='+w).then(function(){poll();});}function setMtManual(idx){var el=document.getElementById('mw'+idx);if(!el)return;var v=parseInt(el.value);if(isNaN(v)){alert('Bitte einen Wert eingeben');return;}v=Math.max(-2500,Math.min(2500,v));if(Math.abs(v)>800){if(!confirm('Leistung '+v+'W liegt ueber 800W. Wirklich setzen?'))return;}fetch('/marstek?idx='+idx+'&power='+v).then(function(){el.value='';poll();});}function setActive(v){fetch('/set?active='+v).then(function(){poll();});}function toggleLog(){var l=document.getElementById('log-box');if(l.style.display==='block'){l.style.display='none';return;}fetch('/log').then(function(r){return r.text();}).then(function(t){l.textContent=t;l.style.display='block';l.scrollTop=0;});}poll();setInterval(poll,5000); </script></body></html>)HTML";
  server.send_P(200, "text/html", DASH_HTML);
}

// ============================================================
//  WEBSERVER: JSON API
// ============================================================
void handleApi() {
  StaticJsonDocument<768> doc;
  doc["pv_w"]        = meter.pv_w;
  doc["grid_w"]      = meter.grid_w;
  doc["house_w"]     = meter.house_w;
  doc["sonnen_soc"]  = meter.sonnen_soc;
  doc["sonnen_w"]    = meter.sonnen_w;
  doc["active"]      = controller_active;
  doc["pv_active"]   = (meter.pv_w > 50);
  doc["meter_type"]  = cfg.meter_type;
  doc["marstek_target_w"] = marstek_target_w;
  // Regelparameter nicht mehr im API (Settings entfernt)
  // System
  doc["sys_uptime_s"]  = millis() / 1000;
  doc["sys_heap_pct"]  = (int)(ESP.getFreeHeap() * 100 / ESP.getHeapSize());
  doc["sys_free_heap"] = ESP.getFreeHeap();
  // Marstek
  JsonArray arr = doc.createNestedArray("marsteks");
  for (int i = 0; i < cfg.marstek_count; i++) {
    JsonObject o = arr.createNestedObject();
    o["soc"]     = mt[i].soc;
    o["power"]   = mt[i].power_w;
    o["current"] = mt[i].current_w;
    o["offline"] = mt[i].offline;
    o["override"]= mt[i].manual_override;
    o["ip"]      = cfg.marstek_ip[i];
  }
  String out; serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

void handleSet() {
  if (server.hasArg("active")) {
    bool v = (server.arg("active") == "1");
    if (v && !controller_active) {
      // AN: Startwerte zuruecksetzen
      last_valid_cons_w = 350;
      addLog("Controller: AN");
    } else if (!v && controller_active) {
      // AUS: genau einmal Stop-Befehl per Modbus senden, dann Ruhe
      addLog("Controller: AUS -> sende Stop an alle Marstek");
      for (int i = 0; i < cfg.marstek_count; i++) {
        if (marstekAdapters[i]) {
          marstekAdapters[i]->setPower(0);
          mt[i].power_w   = 0;
          mt[i].current_w = 0;
        }
      }
    }
    controller_active = v;
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "OK");
}

void handleLog() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/plain; charset=utf-8",
    logBuffer.length() > 0 ? logBuffer : "Kein Log vorhanden.");
}

void handleMarstek() {
  if (!server.hasArg("idx") || !server.hasArg("power")) {
    server.send(400, "text/plain", "idx und power erforderlich"); return;
  }
  int idx   = constrain(server.arg("idx").toInt(), 0, cfg.marstek_count-1);
  int power = server.arg("power").toInt();
  if (power == 0) {
    mt[idx].manual_override = false;
    mt[idx].manual_power    = 0;
    if (marstekAdapters[idx]) marstekAdapters[idx]->setPower(0);
    addLog("Marstek " + String(idx+1) + ": Override aufgehoben");
  } else {
    mt[idx].manual_override = true;
    mt[idx].manual_power    = power;
    if (marstekAdapters[idx]) marstekAdapters[idx]->setPower(power);
    addLog("Marstek " + String(idx+1) + ": Override -> " + String(power) + "W");
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json",
    "{\"idx\":" + String(idx) + ",\"power\":" + String(power) +
    ",\"override\":" + String(mt[idx].manual_override ? "true" : "false") + "}");
}

void handleNotFound() {
  if (setup_mode) {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  } else {
    server.send(404, "text/plain", "Not found");
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // OLED initialisieren (so frueh wie moeglich)
  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(0, 10, "VoltBinds");
  u8g2.drawStr(0, 20, "v" VB_VERSION);
  u8g2.drawStr(0, 30, "Starte...");
  u8g2.sendBuffer();

  addLog("=== VoltBinds v" VB_VERSION " ===");
  config_load();
  adapters_init();

  wifi_connect();

  if (!wifi_connected) {
    start_ap_mode();
    server.on("/",           HTTP_GET,  handleSetupPage);
    server.on("/setup",      HTTP_GET,  handleSetupPage);
    server.on("/save-setup", HTTP_POST, handleSaveSetup);
    server.onNotFound(handleNotFound);
    server.begin();
    addLog("Setup-Modus aktiv");
    return;
  }

  // mDNS
  if (MDNS.begin(OTA_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    addLog("mDNS: http://voltbinds.local");
  }

  // OTA
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA
    .onStart([]()  { addLog("OTA: Update..."); })
    .onEnd([]()    { addLog("OTA: Fertig!"); })
    .onProgress([](unsigned int p, unsigned int t) {
      static int lp = -1; int pct = p*100/t;
      if (pct/10 != lp/10) { lp=pct; addLog("OTA: "+String(pct)+"%"); }
      oled_draw();  // Display waehrend OTA aktiv halten
    })
    .onError([](ota_error_t e) { addLog("OTA Fehler: " + String(e)); });
  ArduinoOTA.begin();
  addLog("OTA: bereit (PW: voltbinds2026)");

  // Web-Routen
  server.on("/",            HTTP_GET,  handleDashboard);
  server.on("/dashboard",   HTTP_GET,  handleDashboard);
  server.on("/setup",       HTTP_GET,  handleSetupPage);
  server.on("/save-setup",  HTTP_POST, handleSaveSetup);
  server.on("/api",         HTTP_GET,  handleApi);
  server.on("/log",         HTTP_GET,  handleLog);
  server.on("/set",         HTTP_GET,  handleSet);
  server.on("/marstek",     HTTP_GET,  handleMarstek);
  server.onNotFound(handleNotFound);
  server.begin();

  addLog("Webserver: http://" + WiFi.localIP().toString());
  addLog("Meter: " + String(cfg.meter_type == 0 ? "Sonnenbatterie" : "Shelly 3EM"));
  addLog("Marstek: " + String(cfg.marstek_count) + "x");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  // Setup-Modus
  if (setup_mode) {
    dnsServer.processNextRequest();
    server.handleClient();
    oled_update();
    return;
  }

  // WiFi-Reconnect
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long last_rc = 0;
    if (millis() - last_rc > 30000) {
      last_rc = millis();
      addLog("WiFi: Reconnect...");
      WiFi.reconnect();
    }
    server.handleClient();
    ArduinoOTA.handle();
    oled_update();
    return;
  }

  ArduinoOTA.handle();
  server.handleClient();

  unsigned long now = millis();

  // Smartmeter alle 5s - immer (auch bei Controller AUS)
  static unsigned long last_meter = 0;
  if (now - last_meter >= 5000) {
    last_meter = now;
    if (activeMeter) activeMeter->poll();
    // Regellogik nur wenn aktiv
    if (controller_active) run_control();
  }

  // Marstek SOC alle 30s - nur wenn Controller aktiv
  static unsigned long last_soc = 0;
  if (controller_active && now - last_soc >= 30000) {
    last_soc = now;
    for (int i = 0; i < cfg.marstek_count; i++)
      if (marstekAdapters[i]) marstekAdapters[i]->poll();
  }

  // Marstek Power-Istwert alle 5s - nur wenn Controller aktiv
  static unsigned long last_pw = 0;
  if (controller_active && now - last_pw >= 5000) {
    last_pw = now;
    for (int i = 0; i < cfg.marstek_count; i++)
      if (marstekAdapters[i]) marstekAdapters[i]->pollPower();
  }
  // Marstek-Werte werden einmalig in handleSet() zurueckgesetzt (nicht per Tick)

  // OLED Update
  oled_update();
}
