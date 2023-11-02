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

const int ch3Pin = 10;
int ch3Value; // (1830 max, 920 min, 0 is 0)

Servo rudderServo;
int servoPos = 0;

int sensorStatus;
// float headings[3]; // iter 0 = angle about x-axis, iter 2 = angle about y-axis, iter 3 = angle about z-axis 

const int motorPin = 11;
int motorValue = 0;

struct timerelement {
  char* name;
  long time;
};

const int timersLength=100;

timerelement timers[timersLength];

  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

void add_timer_value(char* name) {
  for(int i=0;i<timersLength;i++) {
    if(strcmp("\n",timers[i].name)!=0) {
      strcpy(name,timers[i].name);
      timers[i].time = 0;
      return;
    }
  }
}

void set_timer_value(char* name, long msduration) {
  for(int i=0;i<timersLength;i++) {
    if(strcmp(name,timers[i].name)==0) {
      timers[i].time = millis() + msduration;
      return;
    }
  }
}

bool check_timer(char* name) {
  for(int i=0;i<timersLength;i++) {
    if(strcmp(name,timers[i].name)==0) return timers[i].time <= millis() && timers[i].time != 0;
  }
  return false;
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

int getController() {
  ch3Value = pulseIn(ch3Pin, HIGH);
  Serial.print("ch3Value = ");
  Serial.print(ch3Value);
  Serial.print("\n");
  return ch3Value;
  }

float readController(int controllerValue) {
  /*The function to relate controller value to motor value is:
  f(x) = 1.80136 * 1.00271 ^ (x) - 1.80135 where x is the inputted controller value*/
  
  motorValue = 1.80 * (pow(1.00271, controllerValue)) - 1.80;

  if (motorValue <= 5) motorValue = 0;
  if (motorValue >= 255) motorValue = 255;
  
  Serial.print("Motor Value: ");
  Serial.print(motorValue);
  Serial.print("\n");
  return motorValue;
}

//==== AUTOMATIC CONTROL ====

// will probably not have to use PID??? Needs testing
void findCorrections() {  // find how much we need to move in a direction to correct heading
  
  
  
  
  }
void correctiveMeasures() { // figure out what we should set servoPos/motor power to in order to fix heading

  
  
  
  }

// ==========================

void setup() {
  pinMode(ch3Pin, INPUT);
  rudderServo.attach(6);
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  sensorStatus = initSensors();
  add_timer_value("light");
  set_timer_value("light", 500);
  pinMode(13, OUTPUT);
}

void loop() {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;
  
  if(check_timer("light")) {
    digitalWrite(13, HIGH);
    //toggle light here
  } else {
    digitalWrite(13, LOW);
  }
  
  if (sensorStatus == 1) {
    getPitchAndRoll();
    getHeading();
    analogWrite(motorPin, readController(getController()));
  } else {
    Serial.print("sensors are broken\n");
    analogWrite(motorPin, readController(getController()));}
}

