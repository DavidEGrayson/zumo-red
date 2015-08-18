#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"
#include "SmartProximitySensor.h"

// This enum lists the top-level states that the robot can be in.
enum State
{
  StatePausing,
  StateWaiting,
  StateTurningToCenter,
  StateDrivingToCenter,
  StateScanning,
  StateDriving,
  StateBorderAnalyze,
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
const uint16_t reverseSpeed = 400;

// The speed that the robot uses when turning.
const uint16_t turnSpeedHigh = 400;
const uint16_t turnSpeedLow = 100;

// The speed that the robot usually uses when moving forward.
// You don't want this to be too fast because then the robot
// might fail to stop when it detects the white border.
const uint16_t forwardSpeed = 200;

// The speed we drive when analyzing the white border.
const uint16_t analyzeSpeed = 100;

// The speed the robot goes when driving to the center initially.
const uint16_t driveCenterSpeed = 400;

// The speed used when turning towards the center.
const uint16_t turnCenterSpeed = 200;

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

// The number of encoder ticks to travel when backing away from the
// edge.
const uint16_t reverseEncoderTicks = 900;

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

State state = StatePausing;

// scanDir is the direction the robot should turn the next time
// it scans for an opponent.
Direction scanDir = DirectionLeft;

Direction turnCenterDir;
uint32_t turnCenterAngle;

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

void setup()
{
  senseInit();
  turnSensorSetup();
  lineSensors.initThreeSensors();
  changeState(StatePausing);

  //senseTest();
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
      delay(500); changeState(StateDriving);  senseReset(); // TMPHAX
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
      changeState(StateDrivingToCenter);
    }
  }
  else if (state == StateTurningToCenter)
  {
    if (justChangedState)
    {
      justChangedState = false;
      turnSensorReset();
      lcd.print(F("turncent"));
    }

    if (turnCenterDir == DirectionLeft)
    {
      motors.setSpeeds(-turnCenterSpeed, turnCenterSpeed);
    }
    else
    {
      motors.setSpeeds(turnCenterSpeed, -turnCenterSpeed);
    }

    turnSensorUpdate();
    
    uint32_t angle = turnAngle;
    if (turnCenterDir == DirectionRight) { angle = -angle; }
    if (angle > turnCenterAngle && angle < turnAngle45 * 7)
    {
      changeState(StateDrivingToCenter);
    }
  }
  else if (state == StateDrivingToCenter)
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
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      lcd.print(F("back"));
    }

    motors.setSpeeds(-reverseSpeed, -reverseSpeed);

    // After backing up for a specific distance, start scanning.
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();
    if (-counts > (int16_t)reverseEncoderTicks * 2)
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
      turnCenterDir = DirectionRight;
      changeState(StateBorderAnalyze);
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
      turnCenterDir = DirectionLeft;
      changeState(StateBorderAnalyze);
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
    else if (objectSeen)
    {
    }
    else if (!objectSeen)
    {
      // We don't see anything with the front sensor.

      motors.setSpeeds(forwardSpeed, forwardSpeed);

      /** TODO: veer towards objects using the IR sensors!
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
  else if (state == StateBorderAnalyze)
  {
    if (justChangedState)
    {
      justChangedState = false;
      lcd.print(F("analyze"));
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
    }

    motors.setSpeeds(analyzeSpeed, analyzeSpeed);

    // Check the middle line sensor.
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[1] < lineSensorThreshold)
    {
      /** tmphax to show encoder counts and angle
      turnSensorReset();
      int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();
      lcd.clear();
      lcd.print(counts);
      buzzer.playFromProgramSpace(beep1);
      motors.setSpeeds(0, 0);
      while(1)
      {
        turnSensorUpdate();
        lcd.gotoXY(0, 1);
        lcd.print((((int32_t)turnAngle >> 16) * 360) >> 16);
        lcd.print(F("   "));
      }
      **/

      int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();
      if (counts < 0) { counts = 0; }

      turnCenterAngle = (turnAngle45 * 4) - 0x517CC1B7 * atan(counts/440);

      // tmphax to show calculated angle
      /**
      motors.setSpeeds(0, 0);
      lcd.clear();
      lcd.print(((turnCenterAngle >> 16) * 360) >> 16);
      delay(10000); **/

      changeState(StateTurningToCenter);
    }
  }
}
