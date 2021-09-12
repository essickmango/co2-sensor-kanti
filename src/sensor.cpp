#include "sensor.h"
#include "buzzled.h"
#include "display.h"
#include "configuration.h"
#include <Ticker.h>


///////////////////
// LOLIN BUTTONS //
///////////////////

#include <LOLIN_I2C_BUTTON.h>

I2C_BUTTON u8g2Button = I2C_BUTTON();


/////////////////////////
// MHZ19X (CO2-sensor) //
/////////////////////////

#include <ErriezMHZ19B.h>
#include <SoftwareSerial.h>

SoftwareSerial mhzSerial(MHZ19X_TX_PIN, MHZ19X_RX_PIN);
ErriezMHZ19B mhz19x(&mhzSerial);

Ticker printDotTicker;

void setupMhz19x()
{
  // Initialize serial port to print diagnostics and CO2 output
  Serial.println(F("\nSetting up Erriez MH-Z19X CO2 Sensor"));

  // Initialize senor software serial at fixed 9600 baudrate
  mhzSerial.begin(9600);

  // Optional: Detect MH-Z19X sensor (check wiring / power)
  u8g2.print(F("mh-z19x"));
  u8g2.sendBuffer();
  while (!mhz19x.detect())
  {
    Serial.println(F("Detecting MH-Z19X sensor..."));
    delay(2000);
  };
  u8g2.print(F("...ok"));
  newLine();
  u8g2.sendBuffer();
  delay(3000);

  u8g2.clearBuffer();
  moveCursorTop();
  u8g2.print(F("mh-z19x info"));
  underline(64);
  newLine();
  u8g2.sendBuffer();

  // Optional: Print firmware version
  char firmwareVersion[5];
  Serial.print(F("  Firmware: "));
  mhz19x.getVersion(firmwareVersion, sizeof(firmwareVersion));
  int firmwareVersionInteger = atoi(firmwareVersion);
  Serial.println(firmwareVersionInteger);
  u8g2.print(F("firmware: "));
  u8g2.print(firmwareVersion);
  newLine();
  u8g2.sendBuffer();

  // Optional: Set CO2 range 2000ppm or 5000ppm (default) once
  // Serial.print(F("Set range..."));
  // mhz19x.setRange2000ppm();
  mhz19x.setRange5000ppm();

  // Optional: Print operating range
  int range = mhz19x.getRange();
  Serial.print(F("  Range: "));
  Serial.print(range);
  Serial.println(F("ppm"));
  u8g2.print(F("range: "));
  u8g2.print(range);
  u8g2.print(F(" ppm"));
  newLine();
  u8g2.sendBuffer();

  // Optional: Set automatic calibration on (true) or off (false) once
  // Serial.print(F("Set auto calibrate..."));
  mhz19x.setAutoCalibration(true);

  // Optional: Print Automatic Baseline Calibration status
  int8_t autoCalibration = mhz19x.getAutoCalibration();
  Serial.print(F("  Auto calibrate: "));
  Serial.println(autoCalibration ? F("on") : F("off"));
  u8g2.print(F("auto cal: "));
  u8g2.print(autoCalibration ? F("on") : F("off"));
  newLine();
  u8g2.sendBuffer();

  // Sensor requires 3 minutes warming-up after power-on
  u8g2.print(F("warming up..."));
  newLine();
  u8g2.sendBuffer();
  printDotTicker.attach(15, printDot);
  setLed(yellow, ledOn);
  printDot();
  while (mhz19x.isWarmingUp())
  {
    delay(100);
  };
  setLed(yellow, ledOff);
  printDotTicker.detach();
  u8g2.print(F("...ok"));
  u8g2.sendBuffer();
}

int16_t readMHZ19X() {
  return mhz19x.readCO2();
}

Co2Exposure getExposure(int16_t mhzCo2)
{
  if (mhzCo2 < CO2_LOW_MEDIUM_THRESH)
  {
    return co2Low;
  }
  else if ((CO2_LOW_MEDIUM_THRESH <= mhzCo2) && (mhzCo2 < CO2_MEDIUM_HIGH_THRESH))
  {
    return co2MediumLow;
  }
  else if ((CO2_MEDIUM_HIGH_THRESH <= mhzCo2) && (mhzCo2 < CO2_HIGH_THRESH))
  {
    return co2MediumHigh;
  }
  else
  {
    return co2High;
  }
}


void printMhz19xErrorCode(int16_t result)
{
  // Print error code
  switch (result)
  {
  case MHZ19B_RESULT_ERR_CRC:
    Serial.println(F("CRC error"));
    break;
  case MHZ19B_RESULT_ERR_TIMEOUT:
    Serial.println(F("RX timeout"));
    break;
  default:
    Serial.print(F("Error: "));
    Serial.println(result);
    break;
  }
}



////////////
// BME280 //
////////////

#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

void setupBme()
{
  u8g2.print(F("bme280"));
  u8g2.sendBuffer();
  delay(1000);

  unsigned status;

  // default settings
  status = bme.begin(0x76);
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status)
  {
    u8g2.print(F("...ERROR!"));
    u8g2.sendBuffer();
    newLine();
    ledTicker.attach_ms(50, toggleErrorLed);
    while (1)
      delay(100);
  }
  u8g2.print(F("...ok"));
  u8g2.sendBuffer();
  newLine();
}


float bmePressure()
{
  return bme.readPressure() / 100.0F;
}

float bmeAltitude(float sealevelPressure)
{
  return bme.readAltitude(sealevelPressure);
}

float bmeTemperature()
{
  return bme.readTemperature();
}

float bmeHumidity()
{
  return bme.readHumidity();
}


///////////
// SHT31 //
///////////

#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31();

void setupSht31()
{
  u8g2.print(F("sht3x"));
  u8g2.sendBuffer();
  if (!sht31.begin(0x44))
  {
    u8g2.print(F("...ERROR!"));
    u8g2.sendBuffer();
    newLine();
    ledTicker.attach_ms(50, toggleErrorLed);
    while (1)
      delay(100);
  }
  u8g2.print(F("...ok"));
  u8g2.sendBuffer();
  newLine();
}

float sht31Temperature()
{
  return sht31.readTemperature();
}

float sht31Humidity()
{
  return sht31.readHumidity();
}

///////////
// SHTC3 //
///////////

#include "Adafruit_SHTC3.h"
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

void setupShtc3()
{
  u8g2.print(F("shtc3"));
  u8g2.sendBuffer();
  if (!shtc3.begin())
  {
    u8g2.print(F("...ERROR!"));
    u8g2.sendBuffer();
    newLine();
    ledTicker.attach_ms(50, toggleErrorLed);
    while (1)
      delay(100);
  }
  u8g2.print(F("...ok"));
  u8g2.sendBuffer();
  newLine();
}

void shtc3GetEvent(sensors_event_t* humidity, sensors_event_t* temperature)
{
  shtc3.getEvent(humidity, temperature);
}

