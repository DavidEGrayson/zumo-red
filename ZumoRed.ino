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

class SmartProximitySensors
{
public:
  const uint16_t pulseOnTimeUs = 421;
  const uint16_t pulseOffTimeUs = 578;

  bool frontLeft = 0;
  bool frontRight = 0;

  uint16_t brightness = 50;

  void init()
  {
    // Pull up the output of the front sensor.
    FastGPIO::Pin<IO_F1>::setInputPulledUp();

    // Turn off the line sensor emitter.
    FastGPIO::Pin<11>::setOutputLow();
    delayMicroseconds(pulseOffTimeUs);
  }

  uint8_t sense()
  {
    Zumo32U4IRPulses::start(Zumo32U4IRPulses::Left, brightness);
    delayMicroseconds(pulseOnTimeUs);
    frontLeft = !FastGPIO::Pin<IO_F1>::isInputHigh();
    Zumo32U4IRPulses::stop();
    delayMicroseconds(pulseOffTimeUs);

    Zumo32U4IRPulses::start(Zumo32U4IRPulses::Right, brightness);
    delayMicroseconds(pulseOnTimeUs);
    frontRight = !FastGPIO::Pin<IO_F1>::isInputHigh();
    Zumo32U4IRPulses::stop();
    delayMicroseconds(pulseOffTimeUs);
  }

} proxSensors;

void setup()
{
  proxSensors.init();
}

void loop()
{
  if (proxSensors.frontLeft == proxSensors.frontRight)
  {
      if (proxSensors.frontLeft)
      {
        proxSensors.brightness -= 2;
      }
      else
      {
        proxSensors.brightness += 2;
      }
      proxSensors.brightness = constrain(proxSensors.brightness,
        brightnessMin, brightnessMax);
  }

  proxSensors.sense();

  lcd.gotoXY(0, 0);
  lcd.print(proxSensors.brightness);
  lcd.print(F("  "));
  lcd.gotoXY(5, 0);
  lcd.print(proxSensors.frontLeft);
  lcd.gotoXY(7, 0);
  lcd.print(proxSensors.frontRight);

  if (motorsEnabled)
  {
    if (proxSensors.frontLeft == proxSensors.frontRight)
    {
      motors.setSpeeds(0, 0);
    }
    else if (proxSensors.frontLeft)
    {
      motors.setSpeeds(-400, 400);
    }
    else
    {
      motors.setSpeeds(400, -400);
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
