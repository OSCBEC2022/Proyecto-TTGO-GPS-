#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <NMEAGPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// =========================
// MPU6050 por I2C
// =========================
const int MPU_SDA = 21;
const int MPU_SCL = 22;
Adafruit_MPU6050 mpu;
bool mpuOK = false;

// =========================
// GPS (NEO-6M) por UART1
// =========================
static const int GPS_RX_PIN = 39;   // ESP32 RX <- TX del GPS
static const int GPS_TX_PIN = -1;   // no se usa
static const uint32_t GPS_BAUD = 9600;

// Chile: UTC-3 verano, UTC-4 invierno
int TZ_OFFSET_HOURS = -3;

// =========================
// Objetos GPS
// =========================
HardwareSerial GPSserial(1);
NMEAGPS gps;
gps_fix fix;

// Últimos datos válidos
NeoGPS::time_t lastDT;
bool lastDTok = false;

float lastLat = 0.0f;
float lastLon = 0.0f;
bool lastLocOk = false;

// Tiempos de diagnóstico
unsigned long lastCharMs = 0;   // último byte recibido desde GPS
unsigned long lastFixMs  = 0;   // último fix válido
unsigned long lastWarnMs = 0;   // última advertencia mostrada

bool gpsVioBytesAlgunaVez = false;

// =========================
// LMIC / TTN
// =========================

// APPEUI / JOINEUI en LSB
static const u1_t PROGMEM APPEUI[8] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// DEVEUI en LSB
static const u1_t PROGMEM DEVEUI[8] = {
  0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x53, 0x6E
};
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// APPKEY en MSB
static const u1_t PROGMEM APPKEY[16] = {
  0xB2, 0xA2, 0xF9, 0x8D, 0x43, 0x8C, 0xC1, 0x41,
  0x1B, 0xFD, 0xE3, 0xC7, 0x20, 0x13, 0x5E, 0x81
};
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

static osjob_t sendjob;
const unsigned TX_INTERVAL = 30;

// Adelanto
void do_send(osjob_t* j);

// =========================
// Utilidades GPS / Tiempo
// =========================
static inline void servicioGPS(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    while (gps.available(GPSserial)) {
      fix = gps.read();

      gpsVioBytesAlgunaVez = true;
      lastCharMs = millis();

      if (fix.valid.date && fix.valid.time) {
        lastDT = fix.dateTime;
        lastDTok = true;
        lastFixMs = millis();
      }

      if (fix.valid.location) {
        lastLat = fix.latitude();
        lastLon = fix.longitude();
        lastLocOk = true;
        lastFixMs = millis();
      }
    }
    delay(1);
  }
}

bool esBisiesto(int y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

int diasEnMes(int y, int m) {
  static const int dm[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2) return esBisiesto(y) ? 29 : 28;
  return dm[m - 1];
}

uint32_t unixTimeUTC(int year, int mon, int day, int hh, int mm, int ss) {
  uint32_t days = 0;
  for (int y = 1970; y < year; y++) days += esBisiesto(y) ? 366 : 365;
  for (int m = 1; m < mon; m++) days += diasEnMes(year, m);
  days += (day - 1);
  return (uint32_t)(days * 86400UL + hh * 3600UL + mm * 60UL + ss);
}

void iniciarGPS() {
  GPSserial.setRxBufferSize(2048);
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  lastCharMs = millis();
  lastWarnMs = millis();

  Serial.println(F("GPS UART1 iniciado a 9600 en RX=39."));
}

void actualizarGPS() {
  while (gps.available(GPSserial)) {
    fix = gps.read();

    gpsVioBytesAlgunaVez = true;
    lastCharMs = millis();

    if (fix.valid.date && fix.valid.time) {
      lastDT = fix.dateTime;
      lastDTok = true;
      lastFixMs = millis();
    }

    if (fix.valid.location) {
      lastLat = fix.latitude();
      lastLon = fix.longitude();
      lastLocOk = true;
      lastFixMs = millis();
    }
  }

  if (millis() > 5000 &&
      (millis() - lastCharMs > 5000) &&
      (millis() - lastWarnMs > 5000)) {

    if (!gpsVioBytesAlgunaVez) {
      Serial.println(F("⚠️ Aun no llegan bytes del GPS."));
      Serial.println(F("   Revisa TX del GPS -> RX=39, GND, VCC y baud 9600."));
    } else {
      Serial.println(F("⚠️ GPS sin bytes recientes por UART, pero puede existir ultimo fix valido."));
      Serial.print(F("Edad ultimo byte GPS (ms): "));
      Serial.println(millis() - lastCharMs);

      if (lastFixMs > 0) {
        Serial.print(F("Edad ultimo fix GPS (ms): "));
        Serial.println(millis() - lastFixMs);
      } else {
        Serial.println(F("Aun no existe fix valido."));
      }
    }

    lastWarnMs = millis();
  }
}

bool obtenerTsDesdeGPS(uint32_t &tsOut) {
  if (!lastDTok) return false;

  int y  = 2000 + lastDT.year;
  int m  = lastDT.month;
  int d  = lastDT.date;
  int hh = lastDT.hours;
  int mm = lastDT.minutes;
  int ss = lastDT.seconds;

  tsOut = unixTimeUTC(y, m, d, hh, mm, ss);
  return true;
}

bool obtenerFechaCL(uint32_t &dateOut) {
  if (!lastDTok) return false;

  int y  = 2000 + lastDT.year;
  int m  = lastDT.month;
  int d  = lastDT.date;
  int hh = lastDT.hours;

  hh += TZ_OFFSET_HOURS;

  if (hh < 0) {
    hh += 24;
    d--;
  } else if (hh >= 24) {
    hh -= 24;
    d++;
  }

  if (d < 1) {
    m--;
    if (m < 1) {
      m = 12;
      y--;
    }
    d = diasEnMes(y, m);
  } else {
    int dim = diasEnMes(y, m);
    if (d > dim) {
      d = 1;
      m++;
      if (m > 12) {
        m = 1;
        y++;
      }
    }
  }

  dateOut = (uint32_t)(y * 10000UL + m * 100UL + d);
  return true;
}

bool obtenerHoraCL(uint32_t &timeOut) {
  if (!lastDTok) return false;

  int hh = lastDT.hours;
  int mm = lastDT.minutes;
  int ss = lastDT.seconds;

  hh += TZ_OFFSET_HOURS;
  while (hh < 0) hh += 24;
  while (hh >= 24) hh -= 24;

  timeOut = (uint32_t)(hh * 10000UL + mm * 100UL + ss);
  return true;
}

bool obtenerLatLonDesdeGPS(float &latOut, float &lonOut) {
  if (!lastLocOk) return false;
  latOut = lastLat;
  lonOut = lastLon;
  return true;
}

// =========================
// MPU
// =========================
bool existeI2C(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void escanearI2C() {
  Serial.println(F("Escaneando bus I2C..."));
  bool found = false;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("Dispositivo I2C encontrado en 0x"));
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found = true;
    }
    delay(2);
  }

  if (!found) {
    Serial.println(F("No se encontraron dispositivos I2C."));
  }
}

void iniciarMPU() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(100000);
  delay(300);

  escanearI2C();

  bool ok68 = existeI2C(0x68);
  bool ok69 = existeI2C(0x69);

  if (ok68) {
    Serial.println(F("Intentando iniciar MPU6050 en direccion 0x68..."));
    if (mpu.begin(0x68, &Wire)) {
      mpuOK = true;
    }
  }

  if (!mpuOK && ok69) {
    Serial.println(F("Intentando iniciar MPU6050 en direccion 0x69..."));
    if (mpu.begin(0x69, &Wire)) {
      mpuOK = true;
    }
  }

  if (!mpuOK) {
    Serial.println(F("⚠️ No se pudo iniciar el MPU6050."));
    Serial.println(F("   Revisa direccion I2C (0x68/0x69), sensor real y pines SDA/SCL."));
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println(F("✅ MPU6050 detectado correctamente."));
}

void leerMPU(int16_t &ax, int16_t &ay, int16_t &az) {
  ax = 0;
  ay = 0;
  az = 0;

  if (!mpuOK) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ax = (int16_t)(a.acceleration.x * 100.0f);
  ay = (int16_t)(a.acceleration.y * 100.0f);
  az = (int16_t)(a.acceleration.z * 100.0f);
}

// =========================
// Eventos LMIC
// =========================
void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(F(": "));

  switch (ev) {
    case EV_JOINING:
      Serial.println(F("Intentando unirse (Join Request enviado)..."));
      break;

    case EV_JOINED:
      Serial.println(F("¡CONECTADO A TTN! 🟢"));
      LMIC_setLinkCheckMode(0);
      break;

    case EV_JOIN_FAILED:
      Serial.println(F("Fallo en la union. Revisa las llaves."));
      break;

    case EV_TXCOMPLETE:
      Serial.println(F("Datos recibidos por el servidor 🚀"));
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;

    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("No hubo respuesta (Timeout). ¿Está el Gateway en la misma SubBanda?"));
      break;

    default:
      Serial.print(F("Evento LMIC: "));
      Serial.println((unsigned) ev);
      break;
  }
}

// =========================
// Envío LoRaWAN
// =========================
void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("Esperando envio anterior..."));
    return;
  }

  servicioGPS(300);
  actualizarGPS();

  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  leerMPU(ax, ay, az);

  float lat = 0.0f, lon = 0.0f;
  uint32_t fechaCL = 0, horaCL = 0, tsUTC = 0;

  bool gpsLocOK   = obtenerLatLonDesdeGPS(lat, lon);
  bool gpsFechaOK = obtenerFechaCL(fechaCL);
  bool gpsHoraOK  = obtenerHoraCL(horaCL);
  bool gpsTsOK    = obtenerTsDesdeGPS(tsUTC);

  int32_t lat_i = gpsLocOK ? (int32_t)(lat * 100000.0f) : 0;
  int32_t lon_i = gpsLocOK ? (int32_t)(lon * 100000.0f) : 0;

  // Payload:
  // [0..1]   ax
  // [2..3]   ay
  // [4..5]   az
  // [6..9]   lat
  // [10..13] lon
  static uint8_t payload[14];

  payload[0] = (ax >> 8) & 0xFF;
  payload[1] = ax & 0xFF;

  payload[2] = (ay >> 8) & 0xFF;
  payload[3] = ay & 0xFF;

  payload[4] = (az >> 8) & 0xFF;
  payload[5] = az & 0xFF;

  payload[6]  = (lat_i >> 24) & 0xFF;
  payload[7]  = (lat_i >> 16) & 0xFF;
  payload[8]  = (lat_i >> 8) & 0xFF;
  payload[9]  = lat_i & 0xFF;

  payload[10] = (lon_i >> 24) & 0xFF;
  payload[11] = (lon_i >> 16) & 0xFF;
  payload[12] = (lon_i >> 8) & 0xFF;
  payload[13] = lon_i & 0xFF;

  LMIC_setTxData2(1, payload, sizeof(payload), 0);

  Serial.println(F("Enviando paquete de GPS + MPU..."));

  if (mpuOK) {
    Serial.print(F("AX: "));
    Serial.println(ax);
    Serial.print(F("AY: "));
    Serial.println(ay);
    Serial.print(F("AZ: "));
    Serial.println(az);
  } else {
    Serial.println(F("MPU no disponible, enviando AX/AY/AZ en 0."));
  }

  if (gpsLocOK) {
    Serial.print(F("Lat: "));
    Serial.println(lat, 6);
    Serial.print(F("Lon: "));
    Serial.println(lon, 6);
  } else {
    Serial.println(F("GPS sin ubicacion valida aun"));
  }

  if (gpsFechaOK && gpsHoraOK) {
    Serial.print(F("Fecha CL: "));
    Serial.println(fechaCL);
    Serial.print(F("Hora CL: "));
    Serial.println(horaCL);
  } else {
    Serial.println(F("GPS sin fecha/hora valida aun"));
  }

  if (gpsTsOK) {
    Serial.print(F("Timestamp UTC: "));
    Serial.println(tsUTC);
  }

  Serial.print(F("Edad ultimo byte GPS (ms): "));
  Serial.println(millis() - lastCharMs);

  if (lastFixMs > 0) {
    Serial.print(F("Edad ultimo fix GPS (ms): "));
    Serial.println(millis() - lastFixMs);
  } else {
    Serial.println(F("Aun no existe fix valido."));
  }
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);

  Serial.println(F("--- INICIANDO TTGO PARA UG67 ---"));

  iniciarGPS();
  iniciarMPU();

  os_init();
  LMIC_reset();

  // AU915 / Chile
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
  LMIC_selectSubBand(1);

  do_send(&sendjob);
}

// =========================
// Loop
// =========================
void loop() {
  actualizarGPS();
  os_runloop_once();
}