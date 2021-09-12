#include "sensor.h"
#include "buzzled.h"
#include "display.h"
#include "configuration.h"

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <U8g2lib.h>
#include <LOLIN_I2C_BUTTON.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

#include <ErriezMHZ19B.h>
#include <SoftwareSerial.h>
#include <Ticker.h>
Ticker printValuesTicker;
Ticker readSensorsTicker;
Ticker alarmEndTicker;

const char firmwareVersion[] = "v0.1.6";
// TODO: 1) move BME280 setup and sht3x setup to individual functions
//       2) implement faster setup and shorter read-intervalls for mh-z19c

enum alarmState
{
  alarmOff,
  alarmOn
};
alarmState alarm = alarmOff;

bool mediumLowAlarmHasFired = false;
bool mediumHighAlarmHasFired = false;
bool highAlarmHasFired = false;

bool alarmMuted = false;
void updateAlarm();
void reenableAlarmCallback();


// BME
float bmePress = 0.0;
float bmeAlt = 0.0;
float bmeTemp = 0.0;
float bmeHum = 0.0;

// SHT
float shtTemp = 0.0;
float shtHum = 0.0;

// MHZ19X
int mhzCo2 = 0;
Co2Exposure co2Level = co2Low;

void readSensors();
void readSensorsCallback();
bool readSensorsFlag = false;

// other stuff
void updateLeds();
void handleButtons();

enum DisplayState
{
  printValuesState,
  showAlarmMutedState,
  showAlarmReenabledState,
};
DisplayState state = printValuesState;

void printValues();
void printValuesCallback();
bool printValuesFlag = false;

void setup()
{
  setupLeds();
  delay(2000);
  setAllLeds(ledOff);

  // start blinking status led
  ledTicker.attach(1, toggleStatusLed);

  // test buzzer
  pinMode(buzzerPin, OUTPUT);
  analogWriteFreq(beepFrequency);
  analogWrite(buzzerPin, 64); // analogWrite takes values between 0 and 255
  delay(200);
  analogWrite(buzzerPin, LOW);

  // disable wifi
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  // setup serial
  Serial.begin(9600);
  while (!Serial)
    ; // time to get serial running

  // setup display
  setupDisplay(firmwareVersion);
  newLine();

  #ifdef SHT3_ACTIVE
    setupSht31();
  #endif // SHC3_ACTIVE

  #ifdef SHTC3_ACTIVE
    setupShtc3();
  #endif // SHTC3_ACTIVE

  #ifdef BME_ACTIVE
    setupBme();
  #endif // BME_ACTIVE

  setupMhz19x();

  // Detach for any errors that occured during setup.
  ledTicker.detach();

  delay(2000);

  printValuesTicker.attach(displayRefreshSeconds, printValuesCallback);
  readSensorsTicker.attach(sensorRefreshSeconds, readSensorsCallback);
  readSensorsFlag = true;
  printValuesFlag = true;
  readSensors();
  printValues();
}


////////////////
// LOOP STUFF //
////////////////

void loop()
{
  readSensors();
  updateLeds();
  updateAlarm();
  handleButtons();
  printValues();
}


void updateAlarm()
{
  switch (co2Level)
  {
  case co2Low:
    mediumLowAlarmHasFired = false;
    mediumHighAlarmHasFired = false;
    highAlarmHasFired = false;
    break;
  case co2MediumLow:
    if (!mediumLowAlarmHasFired)
    {
      mediumLowAlarmHasFired = true;
      mediumHighAlarmHasFired = false;
      highAlarmHasFired = false;
      if (numberOfMediumLowBeeps > 0 && !alarmMuted)
      {
        initBeep(numberOfMediumLowBeeps);
      }
    }
    break;
  case co2MediumHigh:
    if (!mediumHighAlarmHasFired)
    {
      mediumHighAlarmHasFired = true;
      highAlarmHasFired = false;
      if (numberOfMediumHighBeeps > 0 && !alarmMuted)
      {
        initBeep(numberOfMediumHighBeeps);
      }
    }
    break;
  case co2High:
    if (!highAlarmHasFired)
    {
      highAlarmHasFired = true;
      if (numberOfHighBeeps > 0 && !alarmMuted)
      {
        initBeep(numberOfHighBeeps);
      }
    }
    break;
  default:
    setLed(green, ledOn);
    setLed(yellow, ledOn);
    setLed(red, ledOn);
    break;
  }
}


void updateLeds()
{
  switch (co2Level)
  {
  case co2Low:
    setLed(green, ledOn);
    setLed(yellow, ledOff);
    setLed(red, ledOff);
    break;
  case co2MediumLow:
    setLed(green, ledOff);
    setLed(yellow, ledOn);
    setLed(red, ledOff);
    break;
  case co2MediumHigh:
    setLed(green, ledOff);
    setLed(yellow, ledOn);
    setLed(red, ledOff);
    break;
  case co2High:
    setLed(green, ledOff);
    setLed(yellow, ledOff);
    setLed(red, ledOn);
    break;
  default:
    setLed(red, ledOn);
    setLed(yellow, ledOff);
    setLed(red, ledOn);
    break;
  }
}


void readSensors()
{
  if (readSensorsFlag)
  {
    mhzCo2 = readMHZ19X();
    co2Level = getExposure(mhzCo2);

    #ifdef BME_ACTIVE
      bmePress = bmePressure();
      bmeAlt = bmeAltitude(SEALEVELPRESSURE_HPA);
      bmeTemp = bmeTemperature();
      bmeHum = bmeHumidity();
    #endif // BME_ACTIVE

    #ifdef SHT3_ACTIVE
      shtTemp = sht31Temperature();
      shtHum = sht31Humidity();
    #endif // SHT3_ACTIVE

    #ifdef SHTC3_ACTIVE
      sensors_event_t humidity, temp;
      shtc3GetEvent(&humidity, &temp);
      shtTemp = temp.temperature;
      shtHum = humidity.relative_humidity;
    #endif // SHTC3_ACTIVE

    readSensorsFlag = false;
  }
}

void readSensorsCallback()
{
  readSensorsFlag = true;
}

void handleButtons()
{
  if (u8g2Button.get() == 0) {
    switch (u8g2Button.BUTTON_A)
    {
      case KEY_VALUE_DOUBLE_PRESS:
        if (!alarmMuted) {
          alarmMuted = true;
          alarmEndTicker.once(alarmMuteSeconds, reenableAlarmCallback);
          printValuesTicker.detach();
          printValuesTicker.attach(displayRefreshSeconds, printValuesCallback);
          state = showAlarmMutedState;
          printValuesFlag = true;
        }
        else {
          alarmMuted = false;
          alarmEndTicker.detach();
          printValuesTicker.detach();
          printValuesTicker.attach(displayRefreshSeconds, printValuesCallback);
          state = showAlarmReenabledState;
          printValuesFlag = true;
        }
        break;
      default:
        break;
    }
  }
}

void reenableAlarmCallback()
{
  alarmMuted = false;
}

void printValues()
{
  if (printValuesFlag)
  {
    switch (state)
    {
      case printValuesState:
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_helvB14_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
        updateFontParameters();
        moveCursorTop();
        if (mhzCo2 < 1000)
        {
          u8g2.print(F(" "));
        }
        u8g2.print(mhzCo2);
        u8g2.setFont(u8g2_font_tom_thumb_4x6_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
        updateFontParameters();
        u8g2.print(" ppm");
        moveCursor(0, 25);

        #ifdef BME_ACTIVE
          u8g2.print(bmeTemp, 1);
          u8g2.print(F(" °C  "));
        #endif // BME_ACTIVE

        #if defined(SHT3_ACTIVE) || defined(SHTC3_ACTIVE)
          // they move to the left if no BME is active
          u8g2.print(shtTemp, 1);
          u8g2.print(F(" °C"));
        #endif // SHT
        newLine();

        #ifdef BME_ACTIVE
          u8g2.print(bmeHum, 1);
          u8g2.print(F(" %   "));
        #endif // BME_ACTIVE

        #if defined(SHT3_ACTIVE) || defined(SHTC3_ACTIVE)
          // they move to the left if no BME is active
          u8g2.print(shtHum, 1);
          u8g2.print(F(" %"));
        #endif // SHT

        #ifdef BME_ACTIVE
          newLine();

          u8g2.print(bmePress, 1);
          u8g2.print(F(" hPa"));
          newLine();

          u8g2.print(bmeAlt, 1);
          u8g2.print(F(" m"));
          newLine();
        #endif // BME_ACTIVE

        u8g2.sendBuffer();
        break;

      case showAlarmMutedState:
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_helvB08_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
        updateFontParameters();
        moveCursorTop();
        u8g2.print(F("Alarm muted"));
        newLine();
        u8g2.print(F("for "));
        u8g2.print(alarmMuteSeconds / 60, 0);
        newLine();
        u8g2.print(F("minutes."));
        u8g2.sendBuffer();
        state = printValuesState;
        break;

      case showAlarmReenabledState:
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_helvB08_tf); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
        updateFontParameters();
        moveCursorTop();
        u8g2.print(F("Alarm"));
        newLine();
        u8g2.print(F("reenabled"));
        u8g2.sendBuffer();
        state = printValuesState;
        break;
    }
    printValuesFlag = false;
  }
}

void printValuesCallback(void)
{
  printValuesFlag = true;
}
