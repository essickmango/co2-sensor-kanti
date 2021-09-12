#include "buzzled.h"
#include "configuration.h"
#include <Arduino.h>
#include <Ticker.h>

const int ledPin[numberOfLeds] {D3, D4, D5};
Ticker ledTicker;

void setupLeds()
{
  for (int i = 0; i < numberOfLeds; ++i)
  {
    pinMode(ledPin[i], OUTPUT);
    setLed(LedName(i), ledOn);
  }
}

void setAllLeds(LedState state) {
  for (int i = 0; i < numberOfLeds; ++i)
  {
    setLed(LedName(i), state);
  }
}

void setLed(LedName name, LedState state)
{
  digitalWrite(ledPin[name], state);
}

void toggleStatusLed(void)
{
  static bool state = true;
  digitalWrite(ledPin[yellow], state);
  state = !state;
}

void toggleErrorLed(void)
{
  static bool state = true;
  digitalWrite(ledPin[red], state);
  state = !state;
}


////////////
// BUZZER //
////////////

Ticker beepTicker;
Ticker stopToneTicker;
static int beepCnt = 0;

void initBeep(int repetitions)
{
  beepTicker.attach(1, friendlyBeep, repetitions);
}

void friendlyBeep(int repetitions)
{
  if (repetitions > beepCnt)
  {
    playTone(buzzerPin, beepFrequency, beepDuration);
    ++beepCnt;
  }
  else
  {
    beepTicker.detach();
    beepCnt = 0;
  }
}

void friendlyBeep()
{
  playTone(buzzerPin, beepFrequency, beepDuration);
}

void playTone(int _pin, unsigned int frequency, unsigned long duration)
{
  pinMode(_pin, OUTPUT);
  analogWriteFreq(frequency);
  analogWrite(_pin, 512);
  stopToneTicker.once_ms(duration, stopTone);
}

void stopTone()
{
  analogWrite(buzzerPin, 0);
}