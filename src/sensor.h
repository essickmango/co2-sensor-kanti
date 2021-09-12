#ifndef SENSOR_H
#define SENSOR_H

///////////////////
// LOLIN BUTTONS //
///////////////////

#include <LOLIN_I2C_BUTTON.h>

extern I2C_BUTTON u8g2Button;


/////////////////
// MHZ19X: CO2 //
/////////////////

#include <ErriezMHZ19B.h>

enum Co2Exposure
{
  co2Low,
  co2MediumLow,
  co2MediumHigh,
  co2High
};

extern ErriezMHZ19B mhz19x;

void setupMhz19x();
void printMhz19xErrorCode(int16_t);
int16_t readMHZ19X();
Co2Exposure getExposure(int16_t);



////////////
// BME280 //
////////////

#include <Adafruit_BME280.h>

void setupBme();
float bmePressure();
float bmeAltitude(float);
float bmeTemperature();
float bmeHumidity();


//////////
// SHT31 //
//////////

void setupSht31();
float sht31Temperature();
float sht31Humidity();


///////////
// SHTC3 //
///////////

void setupShtc3();
void shtc3GetEvent(sensors_event_t*, sensors_event_t*);

#endif