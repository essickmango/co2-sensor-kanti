#ifndef CONFIGURATION_H
#define CONFIGURATION_H

static const int buzzerPin = D8;
static const unsigned int beepFrequency = 255; // 0-255
static const unsigned long beepDuration = 500;
static const int numberOfMediumLowBeeps = 0;
static const int numberOfMediumHighBeeps = 2;
static const int numberOfHighBeeps = 3;


// #define BME_ACTIVE // not available on old board
// #define SHT3_ACTIVE // not available on old board
#define SHTC3_ACTIVE // not available on new board


////////////
// MHZ19X //
////////////
#define CO2_LOW_MEDIUM_THRESH  (int16_t) 800
#define CO2_MEDIUM_HIGH_THRESH (int16_t)1000
#define CO2_HIGH_THRESH        (int16_t)1200

#define MHZ19X_TX_PIN D6
#define MHZ19X_RX_PIN D7


////////////
// BME280 //
////////////
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)


#endif