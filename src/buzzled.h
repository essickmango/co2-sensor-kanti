#ifndef BUZZLED_H
#define BUZZLED_H

#include <Arduino.h>
#include <Ticker.h>

enum LedName
{
  green,
  yellow,
  red,
  numberOfLeds
};

enum LedState
{
  ledOn,
  ledOff
};

extern Ticker ledTicker;

void setupLeds();
void setLed(LedName name, LedState state);
void setAllLeds(LedState state);
void toggleStatusLed();
void toggleErrorLed();

void initBeep(int repetitions);
void friendlyBeep();
void friendlyBeep(int repetitions);
void playTone(int _pin, unsigned int frequency, unsigned long duration);
void stopTone();

#endif