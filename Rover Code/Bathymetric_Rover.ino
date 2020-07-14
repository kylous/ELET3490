#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <ESP32Servo.h>
#include <ping1d.h>
#include <TinyGPS++.h>

//Define Hardware Serial Port Pins//
#define RXD 3                                   //RX0 - Ping Sonar
#define TXD 1                                   //TX0 - Ping Sonar
#define RXD1 18                                 //RX1 - GPS
#define TXD1 19                                 //TX1 - GPS
#define RXD2 16                                 //RX2 - Radio
#define TXD2 17                                 //TX2 - Radio
static Ping1D ping{Serial};                     //UART 0
#define GPS Serial1                             //UART 1
#define HC12 Serial2                            //UART 2

//Define BLDC Motor Pins//
#define leftM 32                                //Left Motor
#define rightM 33                               //Right Motor

//Define Battery Voltage Testing Pins//
#define battery1 35                             //ADC 7
#define battery2 34                             //ADC 6

//Define Delays
#define radioDelay 500
#define imuDelay 100

//////Initializing//////

////Components////
Adafruit_BNO055 myIMU = Adafruit_BNO055();      //BNO-055 9-Degrees Of Freedom Inertial Measurement Unit
TinyGPSPlus gps;                                //NEO-6M GPS Module
Servo leftMotor;                                //2200KV Motor Attached to 30A Electronic Speed Controller
Servo rightMotor;                               //2200KV Motor Attached to 30A Electronic Speed Controller

////Variables////

//System
double heading,relBearing,bearing=0,millisOld;    //Rover current heading, Relative bearing to target, Target bearing, Timer
int leftSpeed,rightSpeed,rSpeedMap,lSpeedMap,opMode=0,startup=0,armSpeed; //Left & Right Motor Speed and speed mapping, Operation Mode, Start-up mode 
float vVal1,vVal2,voltage1,voltage2,vIn=3.3;    //ADC Val for Battery 1 voltage,ADC Val for Battery 2 voltage,Calculated voltages for Batteries 1&2, Voltage supply
float TR1 = 100000.0,TR2 = 10000.0,vDivider;             //Resistor Values for voltage dividers

//Ping Sonar
int depth;                                    //Depth Reading
int confidence;                                 //Confidence in depth readings


//GPS
int sats;                                       //Number of Satellites
float HDOP;                                     //Horizontal Diminution of Precision
double latitude,longitude,targetLat,targetLon,velocity,distance;     //Latitude and Longitude in degrees, Speed in meters per second


//Radio
String message,command,newTarget;               //Telemetry data message being sent to base station, Command message being received from base station
int incoming;                                   //Incoming message integer buffer

//IMU
uint8_t sys, gyro, accel, mg=0;                 //Calibration variables for IMU System, 3-Axis Accelerometer, 3-Axis Gyroscope, 3-Axis Magnetometer
int8_t temp;                                    //IMU Temperature
float wQuat,xQuat,yQuat,zQuat;                  //Quaternion Values
float thetaM,phiM,thetaFold=0,thetaFnew,phiFold=0,phiFnew,thetaG=0,phiG=0,theta,phi,thetaRad,phiRad,Xm,Ym,dt;   //Tilt Compensation Variables

void setup() {
  //Attach Motors
  leftMotor.attach(leftM,1000,2000);
  rightMotor.attach(rightM, 1000, 2000);
  leftMotor.setPeriodHertz(50);
  rightMotor.setPeriodHertz(50);

  //Start Serial Devices
  Serial.begin(115200);
  GPS.begin(9600, SERIAL_8N1, RXD1, TXD1);
  HC12.begin(115200, SERIAL_8N1, RXD2, TXD2);

  //Start I2C Device
  myIMU.begin();
  myIMU.setExtCrystalUse(true);

  //Error Message for Ping Sonar Failure
  while (!ping.initialize()){
    message = "Sonar not initialized";
    sendMessage(message);
  }

  //Calculate vDivider Value
  vDivider = TR1/(TR1+TR2);
  
  //Start Timer
  millisOld=millis();
}

void loop() {
  
  switch(opMode){

    //Testing System Operation
    case 0:{

      armSpeed = 0;
      leftMotor.write(armSpeed);
      rightMotor.write(armSpeed);

    
      armSpeed = 50;
      leftMotor.write(armSpeed);
      rightMotor.write(armSpeed);


      armSpeed = 0;
      leftMotor.write(armSpeed);
      rightMotor.write(armSpeed);
      
      getIMU();
      for(double start = millis(); millis() - start < 1000;){
        getDepth();
        getGPS();
      }
      getVoltage();

      
      message = (String)opMode+","+(String)sys+","+(String)gyro+","+(String)accel+","+(String)mg+","+(String)sats+","+(String)HDOP+","+(String)heading+","+(String)bearing+","+(String)velocity+","+(String)wQuat+","+(String)xQuat+","+(String)yQuat+","+(String)zQuat+","+(String)voltage1+","+voltage2+"\r\n";
      sendMessage(message);
      getCommand();
    }
    break;

    //Mission Mode
    case 1:{
      getIMU();
      getDepth();
      getGPS();
      getVoltage();

      relBearing = bearing - heading;
      leftSpeed = calcLeftMotor(relBearing);
      rightSpeed = calcRightMotor(relBearing);
      lSpeedMap = map(leftSpeed, 0, 255, 0, 180);
      rSpeedMap = map(rightSpeed, 0, 255, 0, 180);

      //Wait 5 seconds before beginning movement
      if (startup == 0){
        for(double start = millis(); millis() - start < 5000;){
          leftMotor.write(0);
          rightMotor.write(0);
        }
        startup = 1;
      }

      //Begin Movement
      leftMotor.write(lSpeedMap);
      rightMotor.write(rSpeedMap);

      message = (String)opMode+","+(String)sys+","+(String)gyro+","+(String)accel+","+(String)mg+","+(String)sats+","+(String)HDOP+","+(String)heading+","+(String)bearing+","+(String)velocity+","+(String)wQuat+","+(String)xQuat+","+(String)yQuat+","+(String)zQuat+","+(String)voltage1+","+voltage2+"\r\n";
      sendMessage(message);
      getCommand();
    }
    break;
  }

}


//Send a message string to base station via HC12 Radio module
void sendMessage(String message){
  HC12.print(message);
  delay(radioDelay);
  HC12.print(latitude,6);
  delay(1);
  HC12.print(",");
  delay(1);
  HC12.print(longitude,6);
  delay(1);
  HC12.print(",");
  delay(1);
  HC12.print((String)depth+","+(String)confidence+"\r\n");
  delay(radioDelay);
}

//Receive a command from base station via HC12 Radio module
void getCommand(){
  
  while(HC12.available()>0){
    int incoming = HC12.read();
    if (isDigit(incoming)){
       newTarget += (char)incoming;
    }
    else{
       command += (char)incoming;
    }
  }
  
  if(!newTarget.length() == 0){
    bearing = newTarget.toInt();
    if (bearing > 180.00){
      bearing = bearing - 360.0;
    }
  }

  if(command.startsWith("A")){
    opMode = 1;
  }
  if(command.startsWith("B")){
    opMode = 0;
  }

  command = "";
  newTarget = "";
}

//Get Inertial Measurement Unit Readings - Orientation and Tilt Compensated Compass
void getIMU(){

  //Device Calibration Levels
  myIMU.getCalibration(&sys, &gyro, &accel, &mg);

  //Tilt Compensated Compass
  imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag =myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
  phiM=-atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
  phiFnew=.95*phiFold+.05*phiM;
  thetaFnew=.95*thetaFold+.05*thetaM;

  dt=(millis()-millisOld)/1000.;
  millisOld=millis();
  theta=(theta+gyr.y()*dt)*.95+thetaM*.05;
  phi=(phi-gyr.x()*dt)*.95+ phiM*.05;
  thetaG=thetaG+gyr.y()*dt;
  phiG=phiG-gyr.x()*dt;

  phiRad=phi/360*(2*3.14);
  thetaRad=theta/360*(2*3.14);

  Xm=mag.x()*cos(thetaRad)-mag.y()*sin(phiRad)*sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
  Ym=mag.y()*cos(phiRad)+mag.z()*sin(phiRad);

  heading=atan2(Ym,Xm)/(2*3.14)*360;

  phiFold=phiFnew;
  thetaFold=thetaFnew;

  //Orientation
  imu::Quaternion quat=myIMU.getQuat();
  wQuat = quat.w();
  xQuat = quat.x();
  yQuat = quat.y();
  zQuat = quat.x();
  
  delay(100);
}

//Get Depth Readings from Sonar
void getDepth(){
  ping.update();
  depth = ping.distance();
  confidence = ping.confidence();
}

//Get GPS Readings
void getGPS(){

  while(GPS.available()>0){
    gps.encode(GPS.read());
  }
  if(gps.location.isValid()){
    distance = getDistance(targetLat,targetLon,gps.location.lat(),gps.location.lng());
    velocity = gps.speed.mps();
    sats = gps.satellites.value();
    HDOP = (gps.hdop.value()/100);
    latitude = (gps.location.lat());
    longitude = (gps.location.lng());
  }
    
}

//Calculate distance between two coordinates
double getDistance(double lat1, double lon1, double lat2, double lon2){
  // Conversion factor from degrees to radians (pi/180)
  const double toRadian = 0.01745329251;

  // First coordinate (Radians)
  double lat1_r = lat1 * toRadian;
  double lon1_r = lon1 * toRadian;

  // Second coordinate (Radians)
  double lat2_r = lat2 * toRadian;  
  double lon2_r = lon2 * toRadian;

  // Delta coordinates 
  double deltaLat_r = (lat2 - lat1) * toRadian;
  double deltaLon_r = (lon2 - lon1) * toRadian;

  // Distance
  double a = sin(deltaLat_r/2)*sin(deltaLat_r/2) + cos(lat1_r) * cos(lat2_r) * sin(deltaLon_r/2) * sin(deltaLon_r/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double distance1 = 6371 * c * 1000;

  return distance1;
}

//Calculate battery voltages
void getVoltage(){
  vVal1 = analogRead(battery1);
  vVal2 = analogRead(battery2);
  voltage1 = ((vVal1*vIn)/4095)/vDivider;
  voltage2 = ((vVal2*vIn)/4095)/vDivider;
}

//Calculate necessary speed value for differential motor system - left motor
int calcLeftMotor(double relBearing){
  if(relBearing > 0){
    return 200;
  }
  else{
    return 1.452 * relBearing + 200;
  }
}

//Calculate necessary speed value for differential motor system - right motor
int calcRightMotor(double relBearing){
  if(relBearing <= 0){
    return 200;
  }
  else{
    return -1.452 * relBearing + 200;
  }
}
