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

uint16_t brightnessLow = 50;
uint16_t brightnessHigh = 50;

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

  uint8_t sense(uint16_t brightness)
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
  bool leftCloser = false;
  bool rightCloser = false;

  proxSensors.sense(brightnessLow);
  if (proxSensors.frontLeft == proxSensors.frontRight)
  {
      if (proxSensors.frontLeft)
      {
        brightnessLow -= 2;
      }
      else
      {
        brightnessLow += 2;
      }
  }
  else
  {
    leftCloser = proxSensors.frontLeft;
    brightnessLow -= 2;
  }
  brightnessLow = constrain(brightnessLow, brightnessMin, brightnessMax);

  proxSensors.sense(brightnessHigh);
  if (proxSensors.frontLeft == proxSensors.frontRight)
  {
      if (proxSensors.frontLeft)
      {
        brightnessHigh -= 2;
      }
      else
      {
        brightnessHigh += 2;
      }
  }
  else
  {
    rightCloser = proxSensors.frontRight;
    brightnessHigh += 2;
  }
  brightnessHigh = constrain(brightnessHigh, brightnessMin, brightnessMax);

  lcd.gotoXY(0, 0);
  lcd.print(brightnessLow);
  lcd.print(F("  "));
  lcd.gotoXY(0, 1);
  lcd.print(brightnessHigh);
  lcd.print(F("  "));

  if (motorsEnabled)
  {
    if (brightnessHigh > brightnessLow && (leftCloser ^ rightCloser))
    {
      int32_t speed = (brightnessHigh - brightnessLow) * (int32_t)1600 / brightnessLow;
      speed = constrain(speed, 200, 400);
      if (leftCloser)
      {
        motors.setSpeeds(-speed, speed);
      }
      else
      {
        motors.setSpeeds(speed, -speed);
      }
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
