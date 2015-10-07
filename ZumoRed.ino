#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"
#include "SmartProximitySensor.h"
#include "RobotState.h"

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
const uint16_t turnSpeedLow = 400;

// The speed that the robot usually uses when moving forward.
const uint16_t forwardSpeed = 400;

// The speed we drive when analyzing the white border.
const uint16_t analyzeSpeed = 100;

// The speed used when turning towards the center.
const uint16_t turnCenterSpeed = 400;

// The speed that the robot drives when it thinks it is pushing or
// about to push an opponent.
const uint16_t rammingSpeed = 400;

// The speed used on the non-dominant wheel to turn during ramming.
const uint16_t rammingSpeedLow = 200;

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
const uint16_t stalemateTime = 1500;

// The number of encoder ticks to travel when we want to go from the
// edge to the center.
const uint16_t edgeToCenterEncoderTicks = 2180;

// The number of encoder ticks to travel when backing away from the
// edge.
const uint16_t reverseEncoderTicks = 400;

// The number of encoder ticks of distance separating the middle and
// side line sensors.
const uint16_t sensorDistance = 440;

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

// scanDir is the direction the robot should turn the next time
// it scans for an opponent.
Direction scanDir = DirectionLeft;

Direction turnCenterDir;
uint32_t turnCenterAngle;

uint16_t borderAnalyzeEncoderCounts;

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

bool lastStopAtEdge;

// Gets the amount of time we have been in this state, in
// milliseconds.  After 65535 milliseconds (65 seconds), this
// overflows to 0.
uint16_t timeInThisState()
{
  return (uint16_t)(millis() - stateStartTime);
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

uint32_t calculateTurnCenterAngle(uint16_t counts)
{
  // This fudge factor helps us account for the fact that we have
  // simple turning algorithms that make us overshoot the threshold.
  float fudge = 0.9;

  // The number 0x28BE60DB is the conversion factor needed to convert
  // from radians (the unit returned by atan) to our internal angle
  // units.  It is approximately 0x80000000 divided by pi.

  return ((uint32_t)turnAngle45 * 4 * fudge) -
    (uint32_t)((0x28BE60DB * fudge) * atan((double)counts / sensorDistance));
}

extern RobotState * robotState;

// Changes to a new state.  It also clears the LCD and turns off
// the LEDs so that the things the previous state were doing do
// not affect the feedback the user sees in the new state.
void changeState(RobotState & state)
{
  justChangedState = true;
  stateStartTime = millis();
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
  lcd.clear();
  displayCleared = true;
  robotState = &state;
}

void changeStateToPausing();
void changeStateToWaiting();
void changeStateToTurningToCenter();
void changeStateToDriving();
void changeStateToPushing();
void changeStateToBacking();
void changeStateToScanning();
void changeStateToAnalyzingBorder();

// In this state, we just wait for the user to press button
// A, while displaying the battery voltage every 100 ms.
class StatePausing : public RobotState
{
public:
  void setup()
  {
    lastStopAtEdge = true;
    motors.setSpeeds(0, 0);
    lcd.print(F("Press A"));
  }

  void loop()
  {
    if (displayIsStale(100))
    {
      displayUpdated();
      lcd.gotoXY(0, 1);
      lcd.print(readBatteryMillivolts());
      lcd.print(F("     "));
    }

    if (buttonA.getSingleDebouncedPress())
    {
      buzzer.playFromProgramSpace(beep1);
      // The user pressed button A, so go to the waiting state.
      changeStateToWaiting();
    }
  }
} statePausing;
void changeStateToPausing() { changeState(statePausing); }
RobotState * robotState = &statePausing;

class StateWaiting : public RobotState
{
  void setup()
  {
    motors.setSpeeds(0, 0);
  }

  void loop()
  {
    // In this state, we wait for a while and then move on to the
    // scanning state.

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
      changeStateToDriving();
    }
  }
} stateWaiting;
void changeStateToWaiting() { changeState(stateWaiting); }

class StateTurningToCenter : public RobotState
{
  void setup()
  {
    turnSensorReset();
    lcd.print(F("turncent"));
  }

  void loop()
  {
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
      changeStateToDriving();

      /** // Uncomment to test the analyzing algorithms.
      motors.setSpeeds(0, 0);
      lcd.clear();
      lcd.print(borderAnalyzeEncoderCounts);
      lcd.gotoXY(0, 1);
      lcd.print((((uint32_t)turnCenterAngle >> 16) * 360) >> 16);
      while(1){ }
      **/
    }
  }
} stateTurningToCenter;
void changeStateToTurningToCenter() { changeState(stateTurningToCenter); }

// In this state we drive forward while
// - looking for the opponent using the proximity sensors
// - veering towards the opponent if it is seen
// - checking for the white border
// - stopping if we have driven enough to get into the center
class StateDriving : public RobotState
{
  void setup()
  {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    motors.setSpeeds(forwardSpeed, forwardSpeed);
    // senseReset(); // not needed because we were scanning earlier
    lcd.print(F("drive"));
  }

  void loop()
  {
    // If we have driven far enough to get into the center, then start
    // scanning.  You can point the robot at the center if you want it to go there,
    // or you can point it at a nearby part of the border.  This behavior is
    // dangerous because we might be really close to the edge, so we only do it
    // if our last stopping point was at an edge.
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();

    if (lastStopAtEdge && counts > (int16_t)edgeToCenterEncoderTicks * 2)
    {
      changeStateToScanning();
    }

    // Check for the white border.
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[0] < lineSensorThreshold)
    {
      turnCenterDir = DirectionRight;
      changeStateToAnalyzingBorder();
      return;
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
      turnCenterDir = DirectionLeft;
      changeStateToAnalyzingBorder();
      return;
    }

    // Read the proximity sensors to sense the opponent.
    sense();
    if (objectSeen)
    {
      changeStateToPushing();
      return;
    }
  }
} stateDriving;
void changeStateToDriving() { changeState(stateDriving); }

class StatePushing : public RobotState
{
  void setup()
  {
    lcd.print(F("PUSH"));
  }

  void loop()
  {
    ledRed(1);

    // Read the proximity sensors to sense the opponent.
    sense();

    ledYellow(objectSeen);

    // Within the first second, we try to steer towards the enemy.
    // After that, we are probably locked in a stalemate and we should
    // ensure our motors are running at full power.
    if (objectSeen && timeInThisState() < stalemateTime)
    {
      if (brightnessLeft > brightnessRight)
      {
        // Swerve to the right.
        motors.setSpeeds(rammingSpeed, rammingSpeedLow);
      }
      else
      {
        motors.setSpeeds(rammingSpeedLow, rammingSpeed);
      }
    }
    else
    {
      motors.setSpeeds(rammingSpeed, rammingSpeed);
    }

    // Check for the white border.
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[0] < lineSensorThreshold)
    {
      turnCenterDir = DirectionRight;
      changeStateToAnalyzingBorder();
      return;
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
      turnCenterDir = DirectionLeft;
      changeStateToAnalyzingBorder();
      return;
    }
  }
} statePushing;
void changeStateToPushing() { changeState(statePushing); }

// In this state, the robot drives in reverse.
class StateBacking : public RobotState
{
  void setup()
  {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    motors.setSpeeds(-reverseSpeed, -reverseSpeed);
    lcd.print(F("back"));
  }

  void loop()
  {
    // After backing up for a specific distance, start scanning.
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();
    if (-counts > (int16_t)reverseEncoderTicks * 2)
    {
      changeStateToTurningToCenter();
    }
  }
} stateBacking;
void changeStateToBacking() { changeState(stateBacking); }

// In this state the robot rotates in place and tries to find
// its opponent.
class StateScanning : public RobotState
{
  uint16_t degreesTurned;
  uint32_t angleBase;

  void setup()
  {
    lastStopAtEdge = false;
    degreesTurned = 0;
    angleBase = 0;
    turnSensorReset();

    senseReset();

    if (scanDir == DirectionRight)
    {
      motors.setSpeeds(turnSpeedHigh, -turnSpeedLow);
    }
    else
    {
      motors.setSpeeds(-turnSpeedLow, turnSpeedHigh);
    }

    lcd.print(F("scan"));
  }

  void loop()
  {
    // Use the gyro to figure out how far we have turned while in this
    // state.
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
      angleBase += turnAngle45;
      degreesTurned += 45;
    }

    sense();

    uint16_t time = timeInThisState();

    if (degreesTurned >= scanDegreesMax)
    {
      // We have not seen anything for a while, so start driving.
      changeStateToDriving();
    }
    else if (time > scanTimeMin)
    {
      // If we detect anything with the front sensor, then go push it.
      if (objectSeen)
      {
        changeStateToPushing();
      }
    }
  }
} stateScanning;
void changeStateToScanning() { changeState(stateScanning); }

class StateAnalyzingBorder : public RobotState
{
  void setup()
  {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    motors.setSpeeds(analyzeSpeed, analyzeSpeed);
    lcd.print(F("analyze"));
    lastStopAtEdge = true;
  }

  void loop()
  {
    if (timeInThisState() < 1000)
    {
      // For the first second of this state, drive slowly.
      motors.setSpeeds(analyzeSpeed, analyzeSpeed);
    }
    else
    {
      // The state lasted too long, so we are probably pushing against
      // a robot and should drive at ramming speed.  Changing to the
      // Pushing state wouldn't help because it would detect the
      // border and immediate go back here.
      motors.setSpeeds(rammingSpeed, rammingSpeed);
    }

    // Check the encoders.
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();

    // Check the middle line sensor.
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[1] < lineSensorThreshold)
    {
      if (counts < 0) { counts = 0; }

      borderAnalyzeEncoderCounts = counts;
      turnCenterAngle = calculateTurnCenterAngle(counts);

      changeStateToBacking();
    }

    // Make sure we don't travel too far.
    if (counts > 1600)
    {
      turnCenterAngle = turnAngle45 * 3;
      changeStateToBacking();
    }
  }
} stateAnalyzingBorder;
void changeStateToAnalyzingBorder() { changeState(stateAnalyzingBorder); }

void setup()
{
  senseInit();
  turnSensorSetup();
  lineSensors.initThreeSensors();
  changeStateToPausing();

  //senseTest();
}

void loop()
{
  if (justChangedState)
  {
    justChangedState = false;
    robotState->setup();
  }

  robotState->loop();
}
