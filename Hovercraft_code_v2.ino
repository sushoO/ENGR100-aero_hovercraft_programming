#include <Wire.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Servo.h>
#include <string.h>

/* Team 12 - Azerbaijan Puckers Hovercraft Software 
==== WIRING ====
10DOF Breakout - SCL to A4 and SDA to A5, VIN to 5V and GND to GND
Rudder Servo - D3
Receiver (CH 3) - 5V to 5V, GND to GND, SIG to D10
Motor Controller - PWR to 5V, GND to GND, SIG to D11
================
*/

/* Much of this code is taken from the documentation within the libraries that
are included above. 

References:
https://cdn-learn.adafruit.com/downloads/pdf/adafruit-10-dof-imu-breakout-lsm303-l3gd20-bmp180.pdf
and the Examples (documentation) within the Adafruit_10DOF library
*/

const int clyPin = 4; //ch3
const int crxPin = 3; //ch2
const int cryPin = 2; //ch1
//int clyValue; // (1830 max, 920 min, 0 is 0)

Servo rudderServo;
int servoPos = 0;

int sensorStatus;
// float headings[3]; // iter 0 = angle about x-axis, iter 2 = angle about y-axis, iter 3 = angle about z-axis 

const int liftMotorPin = 11;
const int thrustMotorPin = 12;
//int motorValue = 0;

const int timersLength=100;

long timers[timersLength];
const int LIGHT = 0;
bool lighton=false;

  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

void add_timer_value(int i) {
//  for(int i=0;i<timersLength;i++) {
//    if(strcmp("\n",timers[i].name)!=0) {
//      strcpy(name,timers[i].name);
//      timers[i].time = 0;
//      return;
//    }
//  }
    timers[i] = 0;
}

void set_timer_value(int i, long msduration) {
//  for(int i=0;i<timersLength;i++) {
//    if(strcmp(name,timers[i].name)==0) {
//      timers[i].time = millis() + msduration;
//      return;
//    }
//  }
    timers[i] = millis() + msduration;
}

bool check_timer(int i) {
    return timers[i] <= millis() && timers[i] != 0;
}

int initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM3 detected ... Check your wiring!"));
    return 0;
  }
  else if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    return 0;
  }
  else if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    return 0;
  } else {return 1;}
}

void getPitchAndRoll() {
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation)) {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
  }  
}

void getHeading() {
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)){
    /* 'orientation' should have valid .heading data now */
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
  }
}

struct controls {
  int ly;
  int rx;
  int ry;
};

struct controls getControllerAll() {
  controls values = {.ly = (int) pulseIn(clyPin, HIGH), .rx = (int) pulseIn(crxPin, HIGH), .ry = (int) pulseIn(cryPin, HIGH)};
  Serial.print("clyValue = ");Serial.print(values.ly);Serial.print("\n");
  Serial.print("crxValue = ");Serial.print(values.rx);Serial.print("\n");
  Serial.print("cryValue = ");Serial.print(values.ry);Serial.print("\n");
  return values;
}

int getControllerCly() {
  int clyValue = pulseIn(clyPin, HIGH);
  Serial.print("clyValue = ");Serial.print(clyValue);Serial.print("\n");
  return clyValue;
}

int getControllerCrx() {
  int crxValue = pulseIn(crxPin, HIGH);
  Serial.print("crxValue = ");Serial.print(crxValue);Serial.print("\n");
  return crxValue;
}

int getControllerCry() {
  int cryValue = pulseIn(cryPin, HIGH);
  Serial.print("cryValue = ");Serial.print(cryValue);Serial.print("\n");
  return cryValue;
}

float controllerToMotor(int controllerValue) {
  /*The function to relate controller value to motor value is:
  f(x) = 1.80136 * 1.00271 ^ (x) - 1.80135 where x is the inputted controller value*/
  
  int motorValue = 1.80 * (pow(1.00271, controllerValue)) - 1.80;

  if (motorValue <= 5) motorValue = 0;
  if (motorValue >= 255) motorValue = 255;
  
  Serial.print("Motor Value: ");
  Serial.print(motorValue);
  Serial.print("\n");
  return motorValue;
}

float controllerToServo(int controllerValue) {//1900-1300
  /*The function to relate controller value to motor value is:
  f(x) = 1.80136 * 1.00271 ^ (x) - 1.80135 where x is the inputted controller value*/
  
  int servoValue = ((float)controllerValue) * 2.0f/11.0f - 1800.0f/11.0f;

  if (servoValue <= 55) servoValue = 50;
  if (servoValue >= 130) servoValue = 130;
  
  Serial.print("Servo Value: ");
  Serial.print(servoValue);
  Serial.print("\n");
  return servoValue;
}

float servoDegreeToMicro(int degree) { //maps degree [centered at 90] to a specific micro value to be more accurate [NEEDS IMPLEMENTATION]
  return degree;
}

//==== AUTOMATIC CONTROL ====//
struct corrections {
  int servoDegree; //centered at 90 degrees
  int thrustPower; //0-255
};

bool isControllerInUse() { //determines if the controller is in use, as if so the automatic correction should not happen [NEEDS IMPLEMENTATION]
  return true;
}




void findCorrections() {  // find how much we need to move in a direction to correct heading
  
  
  
  
}
  
void correctiveMeasures() { // figure out what we should set servoPos/motor power to in order to fix heading

  
  
  
}

//===== MAIN BODY CODE =====//

void setup() {
  pinMode(clyPin, INPUT);
  pinMode(crxPin, INPUT);
  pinMode(cryPin, INPUT);
  rudderServo.attach(9);
  pinMode(liftMotorPin, OUTPUT);
  Serial.begin(9600);
  sensorStatus = initSensors();
  add_timer_value(LIGHT);
  set_timer_value(LIGHT, 500);
  pinMode(13, OUTPUT);
}

void loop() {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  //flashing light
  if(check_timer(LIGHT)) {
    //toggle light
    lighton = !lighton;
    if(lighton) digitalWrite(13, HIGH);
    else digitalWrite(13, LOW);
    set_timer_value(LIGHT, 500);
  }

  //get sensor values
  if (sensorStatus == 1) {
    getPitchAndRoll();
    getHeading();
  } else {
    Serial.print("sensors are broken\n");
  }
  
  //automatic or manual control
  if(controllerInUse()) { //if controller in use then only do manual
    analogWrite(liftMotorPin, controllerToMotor(getControllerCly()));
    analogWrite(thrustMotorPin, controllerToMotor(getControllerCry()));
    rudderServo.write(servoDegreeToMicro(controllerToServo(getControllerCrx())));
  } else { //otherwise do automatic adjustment
    findCorrections();
    correctiveMeasures();
  }
  
}
