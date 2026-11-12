// =====================================================================================
// UoT MIE1050 - ELEVATOR RIDE DATA LOGGER  ("Which elevator am I in?", Part 1)
// Based on the course firmware UoT-MIE1050 v1.2.0 (same board, pins and libraries).
//
// What it does:
//   Streams one CSV line per accelerometer sample over USB serial. The laptop program
//   elevator_logger.py reads the stream, detects each ride's start and stop, saves one
//   segment per ride and collects the ride metadata. The board itself does not decide
//   what a ride is; it only measures.
//
//   BMA400 accelerometer : 100 Hz, +-2 g, on-chip low-pass at 48 Hz
//   BME680 pressure/T/RH : 20 Hz, gas heater OFF (the heater made each reading take
//                          >150 ms and warms the sensor)
//   Not used here        : ultrasonic sensor, ADC channels (mic/EMF/light), AHT20.
//                          They block the loop and are not needed for elevator features.
//
// Serial commands (sent by elevator_logger.py, or type them in the Serial Monitor):
//   start      -> beep, then stream data until "stop"
//   stop       -> stop streaming, double beep
//   led 1 / led 0 -> LED on/off (the laptop turns it on while it sees a ride)
//   id         -> report firmware version and whether both sensors were found
//
// The screen and buzzer are not used while streaming: drawing takes tens of ms and
// would drop samples, and the buzzer would shake the accelerometer.
//
// CSV columns while streaming:
//   t_us, ax_g, ay_g, az_g, p_pa, temp_c, rh_pct, baro_seq
//   t_us     = microseconds since "start" (board clock, 64-bit so it never wraps)
//   a*_g     = acceleration in g (includes gravity, about 1 g on the vertical axis)
//   p_pa     = most recent pressure reading in Pa (the library reports whole Pa,
//              about 8 cm of height)
//   baro_seq = counts up each time a NEW pressure/T/RH reading arrives. Pressure is
//              slower than the accelerometer, so its value repeats over ~5 rows; keep
//              only the rows where baro_seq changes to get the true 20 Hz series.
//
// Lines starting with # are status messages, not data.
// =====================================================================================

#include <SPI.h>                              // SPI library
#include <Wire.h>                             // I2C library
#include "esp_timer.h"                        // 64-bit microsecond clock
#include <Adafruit_Sensor.h>                  // Core sensor library
#include <Adafruit_ST7789.h>                  // Hardware-specific library for ST7789
#include "Adafruit_BME680.h"                  // Hardware-specific library for BME680
#include "SparkFun_BMA400_Arduino_Library.h"  // Hardware-specific library for BMA400

//Pin Definitions (same as course firmware)
#define TFT_CS 38
#define TFT_RST -1
#define TFT_DC 48

#define LED_PIN 19
#define BUZZER_PIN 7

//Other Constants
#define BME_Addr 0x76

//Logger settings
#define FW_VERSION "ELEVLOG 2.0"
#define SERIAL_BAUD 115200            // if you raise ACCEL_HZ above 100, raise this too (and --baud in the script)
#define ACCEL_HZ 100                  // accelerometer rows per second
#define ACCEL_PERIOD_US (1000000UL / ACCEL_HZ)
#define BARO_PERIOD_MS 50             // pressure/T/RH reading every 50 ms = 20 Hz (one reading takes ~42 ms)

//Location in memory to store SPI object and display object
SPIClass* fspi = NULL;  //SPI
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//Location In Memory to store Sensor Objects
Adafruit_BME680 bme;
BMA400 accelerometer;

bool bmaOk = false;  // true if the sensor answered at boot
bool bmeOk = false;

//Latest pressure/T/RH reading
uint32_t baroP = 0;        // Pa
float baroT = 0;           // deg C
float baroRH = 0;          // %
uint32_t baroSeq = 0;      // counts new readings
bool baroBusy = false;     // a reading is in progress
uint32_t nextBaroMs = 0;   // when to start the next reading

//Streaming state
bool recording = false;     // true while streaming
int64_t startUs = 0;        // board clock when streaming started
uint32_t nextSampleUs = 0;  // micros() when the next accelerometer sample is due
uint32_t nSamples = 0;      // rows sent since "start"
uint32_t nLate = 0;         // samples skipped because the loop fell behind (should stay 0)

String cmdBuf = "";             // characters received over serial, until Enter
uint32_t nextIdleDrawMs = 0;    // idle screen refresh timer

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(SERIAL_BAUD);

  //Init Buzzer and LED GPIO
  digitalWrite(LED_PIN, LOW);  //Turn off before setting to output to prevent flickering at boot
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  //Init Display
  fspi = new SPIClass(FSPI);
  fspi->begin(36, 37, 35);
  tft.init(135, 240);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  //Init all sensors
  Wire.begin(16, 15);     //I2C Bus #1 Init (For BME and BMA)
  Wire.setClock(400000);  //Fast I2C so each sensor read takes well under 1 ms

  //Init BME680
  bmeOk = bme.begin(BME_Addr);
  if (!bmeOk) {
    Serial.println("# ERROR,BME680 not found - check wiring");
  } else {
    bme.setTemperatureOversampling(BME680_OS_2X);
    bme.setHumidityOversampling(BME680_OS_1X);
    bme.setPressureOversampling(BME680_OS_16X);  // lowest pressure noise
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);  // light smoothing (settles in ~0.2 s)
    bme.setGasHeater(0, 0);                      // heater OFF: fast readings, no self-heating
  }

  //Init BMA400
  bmaOk = (accelerometer.beginI2C(BMA400_I2C_ADDRESS_DEFAULT) == BMA400_OK);
  if (!bmaOk) {
    Serial.println("# ERROR,BMA400 not found - check wiring and I2C address");
  } else {
    // The chip samples at 200 Hz and low-pass filters at 0.24 x 200 = 48 Hz, just under
    // half of our 100 Hz logging rate, so vibration above 50 Hz cannot fold back into the data.
    accelerometer.setRange(BMA400_RANGE_2G);                      // elevators stay far below 2 g; best resolution (~1 mg)
    accelerometer.setODR(BMA400_ODR_200HZ);
    accelerometer.setDataSource(BMA400_DATA_SRC_ACCEL_FILT_1);
    accelerometer.setFilter1Bandwidth(BMA400_ACCEL_FILT1_BW_1);   // 0.24 x ODR
    accelerometer.setOSR(BMA400_ACCEL_OSR_SETTING_3);             // lowest noise
  }

  nextBaroMs = millis();
  drawIdleScreen();
  printId();
  Serial.println("# Commands: start | stop | led 0/1 | id");
}

//loop function runs over and over again forever
void loop() {
  handleSerialCommands();
  serviceBaro();

  if (recording) {
    if ((int32_t)(micros() - nextSampleUs) >= 0) {  // next 10 ms slot reached
      logSample();
      nextSampleUs += ACCEL_PERIOD_US;
      // If something held the loop up for more than one slot, skip the missed slots
      // (they show up as a gap in t_us) instead of sending a burst of late samples.
      int32_t behind = (int32_t)(micros() - nextSampleUs);
      if (behind > (int32_t)ACCEL_PERIOD_US) {
        uint32_t missed = behind / ACCEL_PERIOD_US;
        nLate += missed;
        nextSampleUs += missed * ACCEL_PERIOD_US;
      }
    }
  } else if ((int32_t)(millis() - nextIdleDrawMs) >= 0) {
    nextIdleDrawMs = millis() + 1000;
    updateIdleValues();  // the screen is only touched when NOT recording (drawing is slow)
  }
}

// ===================== Sensors =====================

// Keeps the BME680 measuring in the background without ever blocking the loop:
// start a reading, come back when it is ready, store it, start the next one on schedule.
void serviceBaro() {
  if (!bmeOk) return;

  if (baroBusy) {
    if (bme.remainingReadingMillis() == 0) {  // 0 = reading is ready to collect
      if (bme.endReading()) {
        baroP = bme.pressure;
        baroT = bme.temperature;
        baroRH = bme.humidity;
        baroSeq++;
      }
      baroBusy = false;
    }
  } else if ((int32_t)(millis() - nextBaroMs) >= 0) {
    nextBaroMs += BARO_PERIOD_MS;
    if ((int32_t)(millis() - nextBaroMs) > 0) nextBaroMs = millis() + BARO_PERIOD_MS;  // fell behind: resync
    if (bme.beginReading() != 0) baroBusy = true;
  }
}

// Read the accelerometer and send one CSV row
void logSample() {
  uint64_t t = (uint64_t)(esp_timer_get_time() - startUs);
  accelerometer.getSensorData();
  Serial.printf("%llu,%.4f,%.4f,%.4f,%lu,%.2f,%.2f,%lu\n",
                (unsigned long long)t,
                accelerometer.data.accelX, accelerometer.data.accelY, accelerometer.data.accelZ,
                (unsigned long)baroP, baroT, baroRH, (unsigned long)baroSeq);
  nSamples++;
}

// ===================== Streaming control =====================

void startRecording() {
  if (recording) stopRecording();
  if (!bmaOk) {
    Serial.println("# ERROR,cannot stream: BMA400 not found");
    return;
  }
  drawRecScreen();
  tone(BUZZER_PIN, 1500, 80);  // one beep = streaming
  delay(200);                  // let the beep finish so the buzzer does not shake the accelerometer data

  Serial.printf("# START,%d\n", ACCEL_HZ);
  Serial.println("t_us,ax_g,ay_g,az_g,p_pa,temp_c,rh_pct,baro_seq");
  nSamples = 0;
  nLate = 0;
  startUs = esp_timer_get_time();
  nextSampleUs = micros();
  recording = true;
}

void stopRecording() {
  if (!recording) return;
  recording = false;
  Serial.printf("# STOP,%lu,%lu\n", (unsigned long)nSamples, (unsigned long)nLate);
  digitalWrite(LED_PIN, LOW);
  tone(BUZZER_PIN, 1500, 80);  // two beeps = stopped
  delay(160);
  tone(BUZZER_PIN, 1500, 80);
  delay(160);
  drawIdleScreen();
}

void printId() {
  Serial.printf("# %s,bma400=%d,bme680=%d,accel_hz=%d\n", FW_VERSION, bmaOk, bmeOk, ACCEL_HZ);
}

// ===================== Serial commands =====================

// Read characters from serial; act on a full line when Enter is received
void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuf.length() > 0) processCommand(cmdBuf);
      cmdBuf = "";
    } else if (cmdBuf.length() < 40) {
      cmdBuf += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "stop" || cmd == "s") {
    if (recording) stopRecording();
    else Serial.println("# IDLE");
    return;
  }
  if (cmd == "id") {
    printId();
    return;
  }
  if (cmd.startsWith("start")) {
    startRecording();
    return;
  }
  if (cmd.startsWith("led")) {
    digitalWrite(LED_PIN, cmd.substring(3).toInt() > 0 ? HIGH : LOW);
    return;
  }
  Serial.printf("# unknown command: %s\n", cmd.c_str());
}

// ===================== Display =====================

void drawIdleScreen() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setFont();  // built-in font
  tft.setTextWrap(false);
  tft.setTextSize(2);

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 0);
  tft.print("ELEVATOR LOGGER");

  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(0, 20);
  tft.print("IDLE - not streaming");
  if (!bmaOk || !bmeOk) {
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(0, 118);
    tft.print(!bmaOk ? "BMA400 MISSING" : "BME680 MISSING");
  }
  nextIdleDrawMs = millis();
}

// Live readings once per second while idle, to check the board is alive and sitting still
void updateIdleValues() {
  if (bmaOk) accelerometer.getSensorData();
  tft.setFont();
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);  // second colour = background, overwrites old digits
  tft.setCursor(0, 44);
  tft.printf("X %+.3f g  ", accelerometer.data.accelX);
  tft.setCursor(0, 62);
  tft.printf("Y %+.3f g  ", accelerometer.data.accelY);
  tft.setCursor(0, 80);
  tft.printf("Z %+.3f g  ", accelerometer.data.accelZ);
  tft.setCursor(0, 98);
  tft.printf("%lu Pa %.1f C  ", (unsigned long)baroP, baroT);
}

void drawRecScreen() {
  tft.fillScreen(ST77XX_RED);
  tft.setFont();
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 20);
  tft.print("STREAMING");
  tft.setTextSize(2);
  tft.setCursor(10, 70);
  tft.print("to laptop");
  tft.setCursor(10, 100);
  tft.print("LED on = ride seen");
}
