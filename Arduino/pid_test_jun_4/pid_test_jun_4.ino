#include "PIDController.hpp"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include "imu_util.h"
#include <ctime>
#include <Wire.h>

bool zero = true;
bool calibrate = true;
int zeroTime = 50;
float degToRad = 57.295779513;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(11,0x28,&Wire2);

// Global variable gyro and angle for angular velocity and rotation angle 
imu::Vector<3> gyro;
imu::Vector<3> angle;

///// Wheelchair Joystick ////////////////////
// For analog reading 
int linearPin = A0; // forward and Backward motion
int turningPin = A1;   // Turning motion
int centerReferencePin = A2; // Reference when the joystick is at natural position (not used)
int minJoy = 300; 
int maxJoy = 470;
// Digital Potentiometer
int midlinear = 168;
int midturn = 165;
const int shutDownPin = 2;
//////////////////////////////////////////////////////////
// MAPPING the joystick analog read to digital potentiometers
int mapFromLinearJoystick(int a){
  // for linear motion
  int output = map(a, minJoy, maxJoy, 55, 255);
 
  return output;
}

int mapFromTurnJoystick(int a){
  // for turning
  int output = map(a, minJoy, maxJoy, 87, 255);
 
  return output;
}
/////////////////////////////////////////////////////////
////////// Check for Joystick input ///////////

bool joystickOn(){
  
  int a = mapFromLinearJoystick(analogRead(linearPin));
  int b = mapFromTurnJoystick(analogRead(turningPin));

  if(( a > (midlinear-2) || a < (midlinear+2)) and ( b < (midturn+2) or b > (midturn-2))){
    return false;
  }
  else{
    return true;
  }
}

////////////////////////////////////////////////////////

unsigned long startTime;
unsigned long currentTime;
unsigned long timenow;
int turnspeedplot;
// PID controller setup (Cascaded PID where gyroSetpoint is the output of the OuterController)
float gyroSetpoint, gyroMeasure;
bool justonce = true;
char keyinput = '0'; //incoming serial data

float rotateby = 0; // amount of degree you want to rotate;

float desiredAngle = 0.0; // Desired turn angle in degrees
float currentAngle = 0.0; // Current angle from IMU
float motorSpeed = 0.0; // Speed correction for motors

//controller type 
float max_speed = 80; // 60 in case of wheelchair (profile 1 or 4)
// PID controller setup (Cascaded PID where motor speed (pwm signal) is the output of the OuterController(Angle controller))
float angleSetpoint, angleMeasure;
int ControllerOutput;

//controller type 
pid_controller_t controllertype = typePID;
// PID OuterLoop
float kp_angle = 6;
float ki_angle = 0.0005;
float kd_angle = 1;

// PID Controller for outerLoop, angleSetpoint is the desired Angle which in deg. The measurement is orientation in deg and the output is desired motor Controller Speed
PIDController<float,int> motorPID(&angleSetpoint, &angleMeasure, &ControllerOutput,kp_angle,ki_angle,kd_angle,controllertype);
// INCOMING KEYBOARD DATA FSM ///////////
double keyboardInput(char input){

  if((input == 'a')or(input == 'A')){
        rotateby -= 30;
  }
  else if((input == 'd')or(input == 'D')){
    rotateby += 30;
  }
  else if(input == 'r'){
    justonce = true;
  }
  else{
    rotateby += 0;
    Serial.println("Wrong input");
    Serial.println("");
  }

  return rotateby;

}
////////////////////////////////////////
double wrapAngle(double angle){
  // Wrap the angle rotation by +- 180 deg
  if(angle > 180){
    angle -= 360;
  }
  else if(angle < -180){
    angle += 360 ;
  }
  else{
    angle += 0 ;
  }  
 return angle;
}

float previousAngle = 0.0;
int completeRotations = 0;

float getUnwrappedAngle(float newAngle) {
    float delta = newAngle - previousAngle;

    if (delta > 180) {
        // Counter-clockwise crossing 0
        completeRotations -= 1;
    } else if (delta < -180) {
        // Clockwise crossing 360
        completeRotations += 1;
    }

    previousAngle = newAngle;
    return newAngle + completeRotations * 360;
}
///////////////////////////////////////////


//// UTILITY FOR MOTOR CONTROL //////////

void stop(){

  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(midlinear);
  Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(midturn);
  turnspeedplot = midturn;
  Wire.endTransmission();


}

void goforward(int speed){
  
  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(midlinear + speed);
  Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(midturn);
  Wire.endTransmission();

}

void turn(int speed){
  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(midlinear);
  Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(midturn-speed);
  turnspeedplot = midturn - speed;
  Wire.endTransmission();
}




void setup() {
    stop();
    pinMode(LED_BUILTIN, OUTPUT);
    ///////////////////////////////////////////////
    // imu Calibration
    ///////////////////////////////////////////////
    Serial.begin(115200);
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");

  if(!bno.begin()){
    // Send imu_status as not connected
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if(calibrate) {
    long bnoID;
    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    bool foundCalib = imu_calibrate_check(&sensor, &bno, &bnoID);

    if(!foundCalib){
      //Send sensor status as not calibrated
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
    }
    else{
      // retrieve calibration data
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      imu_retrieve_offsets(&calibrationData, &bno);
    }

    delay(1000);

    //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);

    sensors_event_t event;

    if(foundCalib){
      // set sensor status as calibrated
      while(!bno.isFullyCalibrated())
      {
        Serial.println("Move sensor slightly to calibrate magnetometers");
        digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
        imu_calibrate(&bno, &event);
      }
      digitalWrite(LED_BUILTIN, LOW);

    }
    else
    {
      while(!bno.isFullyCalibrated())
      {
        Serial.println("Please Calibrate Sensor: ");
        imu_calibrate(&bno, &event);
      }
    }

    adafruit_bno055_offsets_t newCalib;
    imu_store_offsets(&bno, &newCalib, &sensor, &bnoID);
    Serial.println("Data stored to EEPROM.");


  }
  else {
    bno.setExtCrystalUse(true);
  }

  if(zero) {
      // set imu status to zeroing
       Serial.println("Zeroing... Please do not move the device");
      delay(1000);
    }
    
  bno.setMode(OPERATION_MODE_NDOF);

  delay(500); // IMU Calibration done
  

  /////////////////////////////////////////////////////////////
  //////////// PID INITIALIZATION ///////////////////////////
  ////////////////////////////////////////////////////////////
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  angle = bno.getQuat().toEuler();
  // Initialize the PID controller
  bool check = motorPID.setup();
  Serial.println(check);
  Serial.println("");
 // check = OuterLoop.setup();
  Serial.println(check);
  Serial.println("");
  //motorPID.setControllerType(Icontroller);
  //OuterLoop.setControllerType(Ocontroller);
  // Set the sample time (in milliseconds)
  motorPID.setSampleTime(1);
  //OuterLoop.setSampleTime(1);

  // Set output limits for turning speed
  motorPID.setOutputLimits(-max_speed, max_speed); // speed backward to speed forward
  
  //OuterLoop.setOutputLimits(-maxAcceleration, maxAcceleration); // angular acceleration to reach the desired limits deg/s

  // Initialize the motor control pins as outputs
  // set the shutDownPin as an output
  pinMode(shutDownPin, OUTPUT);
  //Shutdown the pots to start with
  digitalWrite(shutDownPin, HIGH);
  Wire.begin();

  stop();
  Serial.println("Starting measurement...");
  startTime = millis();

} //SETUP END

// previous angle
float prevAngle = 0;
// error of inner loop (velocity control)
float error = 0;
// error of outerloop (angle control)
float errorO = 0;
//time difference
float tau = 0;

void loop() {

  if(joystickOn()){

    int pot1 = mapFromLinearJoystick(analogRead(linearPin));
    int pot2 = mapFromTurnJoystick(analogRead(turningPin));
    Wire.beginTransmission(0x2C);
    Wire.write(0x00);  // the other one: 0x00, 0x80
    Wire.write(pot1);
    Wire.endTransmission();
    Wire.beginTransmission(0x2C);
    Wire.write(0x80);  // the other one: 0x00, 0x80
    Wire.write(pot2);
    Wire.endTransmission();

    delay(1);

  }
  else{

    currentTime = millis();
    tau = (currentTime - startTime);
    if(tau > 1){
      // check if tau is greater than 10ms
        startTime = millis();
        // get the current value of angle (orientation)
        angle = bno.getQuat().toEuler();
        // z axis is the yaw ( x is forward in wheelchair frame and y can be derived using right hand rule)
        float z = angle.x() * degToRad;
        // get the current gyroscope reading
        gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

        // Check for serial data (keyboard input for rotation)
        if(Serial.available() > 0 ) {
        
        keyinput = Serial.read(); //read the incoming key
        Serial.println("Keyinput ");
        Serial.print(keyinput); //print the serial input

        rotateby = keyboardInput(keyinput); // function to rotate
        Serial.println("Rotateby: ");
        Serial.print(rotateby); 
        
        desiredAngle = rotateby; //function to wrap the angle 
        Serial.println("Desired Angle: ");
        Serial.print(desiredAngle);
        Serial.println("");
      }

    if(justonce == true){
        int pot1 = midlinear;//129;// 5.924
        int pot2 = midturn;//128;// 5.920
        previousAngle = 0.0;
        completeRotations = 0;
        Wire.beginTransmission(0x2C);
        Wire.write(0x00);  // the other one: 0x00, 0x80
        Wire.write(pot1);
        Wire.endTransmission();
        Wire.beginTransmission(0x2C);
        Wire.write(0x80);  // the other one: 0x00, 0x80
        Wire.write(pot2);
        rotateby = getUnwrappedAngle((float)z);
        desiredAngle = rotateby;
        angleMeasure = desiredAngle;
        angleSetpoint = desiredAngle;
        //gyroMeasure = 0.0;//(float)gyro.x();
        
        // Compute the PID controller output
        //OuterLoop.compute();
        //gyroSetpoint = (float)OuterControllerOutput; //setting gyro setpoint in deg/s
        //motorPID.compute();
        stop();
        justonce = false;
      }
      //current angle to give to outerloop controller
      angleMeasure = getUnwrappedAngle((float)z); // this is in degrees
      //current angular velocity as input to the innerloop controller (why is there a 100 here? i forgot)
      //gyroMeasure = (float)gyro.x();// (angleMeasure - prevAngle)*1000/tau;//(float)gyro.x();
      // angle setpoint to the outerloop controller which is the desired angle in steps of 10degs
      angleSetpoint = desiredAngle;
      // first compute the outerloop and 
      //OuterLoop.compute();
      // error of outer loop
      //errorO = OuterLoop.geterror();
      //output of outer loop is desired angular velocity which we give as gyroscope setpoint to the innerloop controller
      //gyroSetpoint = (float)OuterControllerOutput;//errorO; 
      // compute the inner loop controller
      
      // error of the innerloop controller
      error = angleMeasure - angleSetpoint;//motorPID.geterror();
    
      //if((abs(errorO) > 2)){
      // Apply the controller output to the motors to turn
      if (abs(error) > 1) {
        // Turn 
          motorPID.compute();
          turn(ControllerOutput);
          //Serial.print("here 1");
        }else{

        stop();

        }
      
            //print values for debugging
      timenow = micros();
      Serial.print("Time: ");
      Serial.print(timenow);
      Serial.print(", setAng: ");
      Serial.print(angleSetpoint);
      Serial.print(", CurrAng: ");
      Serial.print(angleMeasure);
      Serial.print(", Error: ");
      Serial.print(error);
      Serial.print(", MotorSpe: ");
      Serial.print(turnspeedplot);
      Serial.print(", tau: ");
      Serial.print(tau);
      Serial.println("");
      prevAngle = angleMeasure;
    }
    //}


    // Delay for a bit before the next loop iteration

    delay(1);

    }
    
}


