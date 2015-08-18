#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"

// This enum lists the top-level states that the robot can be in.
enum State
{
  StatePausing,
  StateWaiting,
  StateCentering,
  StateScanning,
  StateDriving,
  StateBacking,
};

enum Direction
{
  DirectionLeft,
  DirectionRight,
};

// When the reading on a line sensor goes below this value, we
// consider that line sensor to have detected the white border at
// the edge of the ring.  This value might need to be tuned for
// different lighting conditions, surfaces, etc.
const uint16_t lineSensorThreshold = 1000;

// The speed that the robot uses when backing up.
const uint16_t reverseSpeed = 300;

// The speed that the robot uses when turning.
const uint16_t turnSpeedHigh = 400;
const uint16_t turnSpeedLow = 100;

// The speed that the robot usually uses when moving forward.
// You don't want this to be too fast because then the robot
// might fail to stop when it detects the white border.
const uint16_t forwardSpeed = 200;

// The speed the robot goes when driving to the center initially.
const uint16_t driveCenterSpeed = 400;

// These two variables specify the speeds to apply to the motors
// when veering left or veering right.  While the robot is
// driving forward, it uses its proximity sensors to scan for
// objects ahead of it and tries to veer towards them.
const uint16_t veerSpeedLow = 0;
const uint16_t veerSpeedHigh = 250;

// The speed that the robot drives when it detects an opponent in
// front of it, either with the proximity sensors or by noticing
// that it is caught in a stalemate (driving forward for several
// seconds without reaching a border).  400 is full speed.
const uint16_t rammingSpeed = 400;

// The amount of time to spend backing up after detecting a
// border, in milliseconds.
const uint16_t reverseTime = 400;

// The minimum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMin = 200;

// The maximum amount of time to spend scanning for nearby
// opponents, in milliseconds.
// const uint16_t scanTimeMax = 2100;

// The maximum number of degrees to turn while scanning for the opponent.
const uint16_t scanDegreesMax = 360 * 2 + 90;

// The amount of time to wait between detecting a button press
// and actually starting to move, in milliseconds.  Typical robot
// sumo rules require 5 seconds of waiting.
const uint16_t waitTime = 5000;

// If the robot has been driving forward for this amount of time,
// in milliseconds, without reaching a border, the robot decides
// that it must be pushing on another robot and this is a
// stalemate, so it increases its motor speed.
const uint16_t stalemateTime = 4000;

// The number of encoder ticks to travel when we want to go from the
// edge to the center.
const uint16_t edgeToCenterEncoderTicks = 2180;

const uint16_t brightnessMin = 2;
const uint16_t brightnessMax = 200;

const char beep1[] PROGMEM = "!>c32";

Zumo32U4Buzzer buzzer;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4LCD lcd;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
L3G gyro;
bool motorsEnabled = false;
unsigned int lineSensorValues[3];

uint16_t brightnessLeft = brightnessMax;
uint16_t brightnessRight = brightnessMax;
bool detectedLastTimeLeft = false;
bool detectedLastTimeRight = false;
uint8_t okLeft = 0;
uint8_t okRight = 0;
bool objectSeen = 0;

State state = StatePausing;

// scanDir is the direction the robot should turn the next time
// it scans for an opponent.
Direction scanDir = DirectionLeft;

// The time, in milliseconds, that we entered the current top-level state.
uint16_t stateStartTime;

// The time, in milliseconds, that the LCD was last updated.
uint16_t displayTime;

// This gets set to true whenever we change to a new state.
// A state can read and write this variable this in order to
// perform actions just once at the beginning of the state.
bool justChangedState;

// This gets set whenever we clear the display.
bool displayCleared;

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
  senseReset();
  turnSensorSetup();
  proxSensors.init();
  lineSensors.initThreeSensors();
  changeState(StatePausing);

  //senseTest();
}

void senseReset()
{
  brightnessLeft = brightnessMax;
  brightnessRight = brightnessMax;
  okLeft = 0;
  okRight = 0;
  objectSeen = 0;
}

void sense()
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


// Gets the amount of time we have been in this state, in
// milliseconds.  After 65535 milliseconds (65 seconds), this
// overflows to 0.
uint16_t timeInThisState()
{
  return (uint16_t)(millis() - stateStartTime);
}

// Changes to a new state.  It also clears the LCD and turns off
// the LEDs so that the things the previous state were doing do
// not affect the feedback the user sees in the new state.
void changeState(uint8_t newState)
{
  state = (State)newState;
  justChangedState = true;
  stateStartTime = millis();
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
  lcd.clear();
  displayCleared = true;
}

// Returns true if the display has been cleared or the contents
// on it have not been updated in a while.  The time limit used
// to decide if the contents are staled is specified in
// milliseconds by the staleTime parameter.
bool displayIsStale(uint16_t staleTime)
{
  return displayCleared || (millis() - displayTime) > staleTime;
}

// Any part of the code that uses displayIsStale to decide when
// to update the LCD should call this function when it updates the
// LCD.
void displayUpdated()
{
  displayTime = millis();
  displayCleared = false;
}

void loop()
{
  bool buttonPress = buttonA.getSingleDebouncedPress();

  if (state == StatePausing)
  {
    // In this state, we just wait for the user to press button
    // A, while displaying the battery voltage every 100 ms.

    motors.setSpeeds(0, 0);

    if (justChangedState)
    {
      justChangedState = false;
      lcd.print(F("Press A"));
    }

    if (displayIsStale(100))
    {
      displayUpdated();
      lcd.gotoXY(0, 1);
      lcd.print(readBatteryMillivolts());
      lcd.print(F("     "));
    }

    if (buttonPress)
    {
      // The user pressed button A, so go to the waiting state.
      changeState(StateWaiting);
    }
  }
  else if (buttonPress)
  {
    // The user pressed button A while the robot was running, so pause.
    changeState(StatePausing);
  }
  else if (state == StateWaiting)
  {
    // In this state, we wait for a while and then move on to the
    // scanning state.

    motors.setSpeeds(0, 0);

    uint16_t time = timeInThisState();

    if (time < waitTime)
    {
      // Display the remaining time we have to wait.
      uint16_t timeLeft = waitTime - time;
      lcd.gotoXY(0, 0);
      lcd.print(timeLeft / 1000 % 10);
      lcd.print('.');
      lcd.print(timeLeft / 100 % 10);
    }
    else
    {
      // We have waited long enough.  Start moving.
      changeState(StateCentering);
    }
  }
  else if (state == StateCentering)
  {
    if (justChangedState)
    {
      justChangedState = false;
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
    }

    motors.setSpeeds(driveCenterSpeed, driveCenterSpeed);

    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();
    if (counts > (int16_t)edgeToCenterEncoderTicks * 2)
    {
      changeState(StateScanning);
    }

    if (displayIsStale(100))
    {
      displayUpdated();
      lcd.gotoXY(0, 1);
      lcd.print(counts);
      lcd.print(F("     "));
    }
  }
  else if (state == StateBacking)
  {
    // In this state, the robot drives in reverse.

    if (justChangedState)
    {
      justChangedState = false;
      lcd.print(F("back"));
    }

    motors.setSpeeds(-reverseSpeed, -reverseSpeed);

    // After backing up for a specific amount of time, drive in
    // reverse.
    if (timeInThisState() >= reverseTime)
    {
      changeState(StateScanning);
    }
  }
  else if (state == StateScanning)
  {
    // In this state the robot rotates in place and tries to find
    // its opponent.

    static uint16_t degreesTurned;
    static uint32_t angleBase;

    if (justChangedState)
    {
      justChangedState = false;
      senseReset();
      turnSensorReset();
      degreesTurned = 0;
      angleBase = 0;
      lcd.print(F("scan"));
    }

    if (scanDir == DirectionRight)
    {
      motors.setSpeeds(turnSpeedHigh, -turnSpeedLow);
    }
    else
    {
      motors.setSpeeds(-turnSpeedLow, turnSpeedHigh);
    }

    // Use the gyro and some static variables to figure out how far we
    // have turned while in this state.
    turnSensorUpdate();
    uint32_t angle1;
    if (scanDir == DirectionRight)
    {
      angle1 = -turnAngle;
    }
    else
    {
      angle1 = turnAngle;
    }
    if ((int32_t)(angle1 - angleBase) > turnAngle45)
    {
      //buzzer.playFromProgramSpace(beep1);
      angleBase += turnAngle45;
      degreesTurned += 45;
    }

    sense();

    uint16_t time = timeInThisState();

    if (degreesTurned >= scanDegreesMax)
    {
      // We have not seen anything for a while, so start driving.
      changeState(StateDriving);
    }
    else if (time > scanTimeMin)
    {
      // If we detect anything with the front sensor, then start
      // driving forwards.
      if (objectSeen)
      {
        changeState(StateDriving);
      }
    }
  }
  else if (state == StateDriving)
  {
    // In this state we drive forward while also looking for the
    // opponent using the proximity sensors and checking for the
    // white border.

    if (justChangedState)
    {
      justChangedState = false;
      lcd.print(F("drive"));
    }

    // Check for borders.
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[0] < lineSensorThreshold)
    {
      scanDir = DirectionRight;
      changeState(StateBacking);
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
      scanDir = DirectionLeft;
      changeState(StateBacking);
    }

    // Read the proximity sensors to see if know where the
    // opponent is.
    sense();

    if ((objectSeen && brightnessLeft < 40 && brightnessRight < 40)
      || timeInThisState() > stalemateTime)
    {
      // The front sensor is getting a strong signal, or we have
      // been driving forward for a while now without seeing the
      // border.  Either way, there is probably a robot in front
      // of us and we should switch to ramming speed to try to
      // push the robot out of the ring.
      motors.setSpeeds(rammingSpeed, rammingSpeed);

      // Turn on the red LED when ramming.
      ledRed(1);
    }
    else if (!objectSeen)
    {
      // We don't see anything with the front sensor, so just
      // keep driving forward.  Also monitor the side sensors; if
      // they see an object then we want to go to the scanning
      // state and turn torwards that object.

      motors.setSpeeds(forwardSpeed, forwardSpeed);

      /**
      if (proxSensors.countsLeftWithLeftLeds() >= 2)
      {
        // Detected something to the left.
        scanDir = DirectionLeft;
        changeState(StateScanning);
      }

      if (proxSensors.countsRightWithRightLeds() >= 2)
      {
        // Detected something to the right.
        scanDir = DirectionRight;
        changeState(StateScanning);
      }
      **/

      ledRed(0);
    }
    else
    {
      // We see something with the front sensor but it is not a
      // strong reading.

      if (brightnessLeft > brightnessRight && okRight)
      {
        // The right-side reading is stronger, so veer to the right.
        motors.setSpeeds(veerSpeedHigh, veerSpeedLow);
      }
      else if (brightnessLeft < brightnessRight && okLeft)
      {
        // The left-side reading is stronger, so veer to the left.
        motors.setSpeeds(veerSpeedLow, veerSpeedHigh);
      }
      else
      {
        // Just drive forward.
        motors.setSpeeds(forwardSpeed, forwardSpeed);
      }
      ledRed(0);
    }
  }
}
