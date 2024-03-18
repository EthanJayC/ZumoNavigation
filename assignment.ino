#include <Wire.h>
#include <Zumo32U4.h>

//Threshold for line sensors detecting light
#define QTR_THRESHOLD     1000  //ms

//Tuned to suit different motors
//0 is stop, 400 full speed
#define REVERSE_SPEED     120  // all values in ms
#define TURN_SPEED        100
#define FORWARD_SPEED     100
#define REVERSE_DURATION  200 
#define TURN_DURATION     300 
#define UNSTUCK_DURATION 1500 
#define STOP_N_DROP 0
#define STUCK_TIMER 0
#define HIT_INTERVAL 1000

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

// A sensors reading must be greater than or equal to this
// threshold in order for the program to consider that sensor as
// seeing an object.
const uint8_t sensorThreshold = 6;

bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;
static uint8_t houseCount = 2;
unsigned long leftHitReset = 0;
unsigned long rightHitReset = 0;

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

  // Reads the line sensor and gets its values
    static uint16_t leftSensorNum = 0;
    static uint16_t rightSensorNum = 0;

void waitForButtonAndCountDown()
{
  ledYellow(1);
  lcd.clear();
  lcd.print(F("Press A"));

  buttonA.waitForButton();

  ledYellow(0);
  lcd.clear();

  // Play audible countdown.
  for (int i = 0; i < 3; i++)
  {
    delay(500);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(500);
  buzzer.playNote(NOTE_G(4), 500, 15);
  delay(500);
}

void objectDetected(){
    ledGreen(1);
  lcd.clear();
  lcd.print("STOP");
    motors.setSpeeds(STOP_N_DROP, STOP_N_DROP);
    delay(750);
    lcd.clear();
  lcd.println("Reverse");
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(500);
    lcd.clear();
  lcd.println("Spin");
    motors.setSpeeds(-REVERSE_SPEED, REVERSE_SPEED);
    delay(1700);
    lcd.clear();
    ledGreen(0);
};


void setup()
{
  proxSensors.initFrontSensor();
  lineSensors.initThreeSensors();
  waitForButtonAndCountDown();
  Serial.begin(9600);
}

void loop()
{
  if (buttonA.isPressed())
  {
    // If button is pressed, stop and wait for another press to
    // go again.
    motors.setSpeeds(0, 0);
    buttonA.waitForRelease();
    waitForButtonAndCountDown();
  }


  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  // Determine if an object is visible or not.
  bool objectSeen = 0;
  objectSeen = leftValue >= sensorThreshold && rightValue >= sensorThreshold;

  lineSensors.read(lineSensorValues);

  //if object is detected, stop, reverse and move on
  if(objectSeen) 
  {
    houseCount--;
    objectDetected();
  if (houseCount == 0)
  {
    motors.setSpeeds(0, 0);

    buzzer.playNote(NOTE_G(2), 100, 15);
    delay(200);
    buzzer.playNote(NOTE_G(2), 100, 15);
    delay(200);
    buzzer.playNote(NOTE_G(2), 100, 15);
    delay(200);
    buzzer.playNote(NOTE_G(3), 1000, 15);
    while(1 == 1){
      // this is to keep the robot stationary after discovering 2 houses.
      // this implementation will stay until I can get it to retrace it steps, bug-free.
    }
  }
  } 
  // checks if a second has passed since the last hit, resets if over 1 second
  // this is to help prevent the robot doing a full turn when it's not stuck
  else if (millis() - leftHitReset < HIT_INTERVAL && millis() - rightHitReset < HIT_INTERVAL) 
  {
      leftHitReset = 0;
      rightHitReset = 0;
  }
  else if (leftSensorNum >= 3 && rightSensorNum >= 3)  
  {
    // if both sensors tap and reverse 3x each then we do a lil swirl
    motors.setSpeeds(REVERSE_SPEED, -REVERSE_SPEED);
    delay(UNSTUCK_DURATION);
    leftSensorNum = 0;
    rightSensorNum = 0;
    //sensors reset so we can repeat this statement
  }
  else if (lineSensorValues[0] > QTR_THRESHOLD)
  {
    // If leftmost sensor detects line, reverse and turn to the
    // right.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    
    Serial.println("left value: ");
    Serial.println(leftSensorNum);
    leftHitReset = millis();
    leftSensorNum++;
  }
  else if (lineSensorValues[NUM_SENSORS - 1] > QTR_THRESHOLD)
  {
    // If rightmost sensor detects line, reverse and turn to the
    // left.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    
    Serial.println("right value: ");
    Serial.println(rightSensorNum);
    rightHitReset = millis();
    rightSensorNum++;
  }
  else
  {
    // All else is good, moves forwards
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    ledRed(0);
  }
  }
