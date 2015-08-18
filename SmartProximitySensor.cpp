#include <Zumo32U4.h>
#include "SmartProximitySensor.h"

class SmartProximitySensors
{
public:
  const uint16_t pulseOnTimeUs = 421;
  const uint16_t pulseOffTimeUs = 578;

  void init()
  {
    // Pull up the output of the front sensor.
    FastGPIO::Pin<IO_F1>::setInputPulledUp();

    // Turn off the line sensor emitter.
    FastGPIO::Pin<11>::setOutputLow();
    delayMicroseconds(pulseOffTimeUs);
  }

  bool senseDir(uint16_t brightness, Zumo32U4IRPulses::Direction dir)
  {
    Zumo32U4IRPulses::start(dir, brightness);
    delayMicroseconds(pulseOnTimeUs);
    bool result = !FastGPIO::Pin<IO_F1>::isInputHigh();
    Zumo32U4IRPulses::stop();
    delayMicroseconds(pulseOffTimeUs);
    return result;
  }

  bool senseLeft(uint16_t brightness)
  {
    return senseDir(brightness, Zumo32U4IRPulses::Left);
  }

  bool senseRight(uint16_t brightness)
  {
    return senseDir(brightness, Zumo32U4IRPulses::Right);
  }

} proxSensors;

const uint16_t brightnessMin = 2;
const uint16_t brightnessMax = 200;

uint16_t brightnessLeft = brightnessMax;
uint16_t brightnessRight = brightnessMax;
bool detectedLastTimeLeft = false;
bool detectedLastTimeRight = false;
uint8_t okLeft = 0;
uint8_t okRight = 0;
bool objectSeen = 0;

void senseInit()
{
  proxSensors.init();
  senseReset();
}

void senseReset()
{
  brightnessLeft = brightnessMax;
  brightnessRight = brightnessMax;
  okLeft = 0;
  okRight = 0;
  objectSeen = 0;
}

static void senseRight()
{
  if (detectedLastTimeRight)
  {
    brightnessRight -= 2;
    brightnessRight = constrain(brightnessRight, brightnessMin, brightnessMax);
  }
  detectedLastTimeRight = proxSensors.senseRight(brightnessRight);
  if (detectedLastTimeRight)
  {
    okRight = 4;
  }
  else
  {
    if (okRight > 0) { okRight--; }
    brightnessRight += 2;
    brightnessRight = constrain(brightnessRight, brightnessMin, brightnessMax);
  }
}

static void senseLeft()
{
  if (detectedLastTimeLeft)
  {
    brightnessLeft -= 2;
    brightnessLeft = constrain(brightnessLeft, brightnessMin, brightnessMax);
  }
  detectedLastTimeLeft = proxSensors.senseLeft(brightnessLeft);
  if (detectedLastTimeLeft)
  {
    okLeft = 4;
  }
  else
  {
    if (okLeft > 0) { okLeft--; }
    brightnessLeft += 2;
    brightnessLeft = constrain(brightnessLeft, brightnessMin, brightnessMax);
  }
}

void sense()
{
  // If we try to sense both sides in this function, that produces
  // asymmetric results that have big effects on the performance of
  // our algorithms which are looking for any difference between the
  // two brightnesses.
  static bool senseToggle = 0;
  if (senseToggle)
  {
    senseRight();
  }
  else
  {
    senseLeft();
  }
  senseToggle ^= 1;

  objectSeen = (okLeft && brightnessLeft <= 190) || (okRight && brightnessRight <= 190);
}

void senseTest()
{
  lcd.clear();
  while(1)
  {
    sense();
    lcd.gotoXY(0, 0);
    lcd.print(brightnessLeft);
    lcd.print(F("  "));
    lcd.gotoXY(5, 0);
    lcd.print(okLeft);
    lcd.gotoXY(0, 1);
    lcd.print(brightnessRight);
    lcd.print(F("  "));
    lcd.gotoXY(5, 1);
    lcd.print(okLeft);

    ledYellow(objectSeen);
  }
}
