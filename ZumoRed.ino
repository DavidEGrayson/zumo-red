#include <Wire.h>
#include <Zumo32U4.h>

const uint16_t brightnessMin = 2;
const uint16_t brightnessMax = 200;

const char beep1[] PROGMEM = "!>c32";

Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
bool motorsEnabled = false;

uint16_t brightnessLeft = 50;
uint16_t brightnessRight = 50;
bool detectedLastTimeLeft = false;
bool detectedLastTimeRight = false;
uint8_t okLeft = 0;
uint8_t okRight = 0;

class SmartProximitySensors
{
public:
  const uint16_t pulseOnTimeUs = 421;
  const uint16_t pulseOffTimeUs = 578;

  bool frontLeft = 0;
  bool frontRight = 0;

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

void setup()
{
  proxSensors.init();
}

void loop()
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

  if (motorsEnabled)
  {
    if (brightnessLeft < brightnessRight && okLeft)
    {
      int32_t speed = (brightnessRight - brightnessLeft) * (int32_t)800 / brightnessLeft;
      speed = constrain(speed, 200, 400);
      motors.setSpeeds(-speed, speed);
    }
    else if (brightnessLeft > brightnessRight && okRight)
    {
      int32_t speed = (brightnessLeft - brightnessRight) * (int32_t)800 / brightnessRight;
      speed = constrain(speed, 200, 400);
      motors.setSpeeds(speed, -speed);
    }
    else
    {
      motors.setSpeeds(0, 0);
    }
  }
  else
  {
    motors.setSpeeds(0, 0);
  }

  if (buttonA.getSingleDebouncedPress())
  {
    motorsEnabled = true;
    buzzer.playFromProgramSpace(beep1);
    delay(500);
  }
}
