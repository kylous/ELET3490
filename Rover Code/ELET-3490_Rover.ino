#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPS.h>
#include <ping1d.h>
#include <utility/imumaths.h>
#include <math.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)
#define RXD1 18                                     //Mapping RX1 to GPIO-18 for GPS
#define TXD1 19                                     //Mapping TX1 to GPIO-19 for GPS
#define RXD2 16                                     //RX2 - Radio
#define TXD2 17                                     //TX2 - Radio
#define GPS Serial1                                 //Hardware serial 1 on ESP32 - GPS
#define HC12 Serial2                                //Hardware serial 2 on the ESP32 - Radio
#define voltMeter1 35                               //ADC-7 on the ESP32 - Battery Voltage Tester
#define voltMeter2 34                               //ADC-6 on the ESP32 - Battery Voltage Tester

Adafruit_BNO055 myIMU = Adafruit_BNO055();          //Define IMU Object
TinyGPS gps;                                        //Define GPS Object
Ping1D ping { Serial };                             //Define Sonar Object

//----Variables----//

int opMode = 1;                                     //Operation mode variable
float volts1,volts2;                                //Voltage level variable
String commandMessage;                              //Base station command
String message;                                     //Telemetry operation data - approximately 56 bytes / 448 bits
String sysMessage;                                  //Telemetry system data - approximately 54 bytes / 432 bits

//IMU Variables//
int8_t temp;                                        //IMU temperature
String w,x,y,z;                                     //String format message variables for IMU-Quaternion(w,x,y,x)
String accel,gy,mg2,sys;                            //String format message variables for IMU-Calibration(accel,gyro,mg,system)
int target;                                         //Compass target bearing
float thetaM,phiM,thetaFold=0,thetaFnew,phiFold=0;
float phiFnew,thetaG=0,phiG=0,theta,phi;
float thetaRad,phiRad;
float Xm,Ym,psi, heading;
float dt,upperThresh,lowerThresh;
String inString;
float initV = 0.0;
float velocity;
float accVx,accVy,accVz,accRes;

//Sonar Variables//
float depth;                                        //Sonar Depth
int confidence;                                     //Sonar Confidence


//GPS Variables//
unsigned long start, chars;                         //GPS sentence reading timer variable and character variable
unsigned short sentences, failed;                   //GPS sentence variable and failed reading variable
char gpsChar;                                       //Character variable for incoming GPS sentences
bool newData = false;                               //Boolean variable for new GPS data
String lat,lon,sat,prec;                            //String format message variable for GPS-Latitude,Longitude,Number of Satellites and Precision
float flat, flon, latVal, lonVal, precVal;
int satVal;
unsigned long age;

unsigned long millisOld;
unsigned long timeNow;


void setup() {
  
  Serial.begin(115200);                               //UART0 - Ping Sonar @115200 BAUD
  GPS.begin(9600, SERIAL_8N1, RXD1, TXD1);          //UART1 - GPS @9600 BAUD
  HC12.begin(115200, SERIAL_8N1, RXD2, TXD2);       //UART2 - Radio @115200 BAUD

  myIMU.begin();                                    //Initialize IMU
  delay(1000);                                      //Give IMU 1 second to boot
  myIMU.setExtCrystalUse(true);                     //Set BNO-055 to External Oscillator

  while (!ping.initialize()){                       //Initialize Sonar
    HC12.println("Sonar not initialized");
  }

  millisOld=millis();
}

void loop() {

  switch(opMode){
    case 0:                                         //System Test Mode
      systemsCheck();
      getSensorReadings();
      coms();                              
      break;
      
    case 1:                                         //Waypoint Mission Mode
      systemsCheck();
      getSensorReadings();
//      getNavSolution(x,y);
//      navigateTo(x,y);
      coms();
      break;

    case 2:                                         //Emergency Return - GPS
//      goHomeGPS();
      coms();
      break;

    case 3:                                         //Emergency Return - IMU
//      goHomeIMU();
      coms();
      break;

    case 4:                                         //System Failure
//      goHome();
      coms();
      
  }

  

}

//Perform a systems diagnostic check//
void systemsCheck(){
  uint8_t system, gyro, accel, mg = 0;                //IMU calibration
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  temp = myIMU.getTemp();
  getVolts();
  getGPS();
//System telemetry message in form Voltage-1,Voltage-2,Temperature,Calibration(System),Calibration(Gyro),Calibration(Accel),Calibration(mg),GPS(lat),GPS(lon),GPS(#Sats),GPS(Prec)
  sysMessage = "S," + (String)volts1 + "," + (String)volts2 + "," + temp + "," + (String)system + "," + (String)gyro + "," +  (String)accel + "," + (String)mg + lat +"," + lon + "," + sat + "," + prec + "\r\n";
  HC12.println(sysMessage);
}


//Determine Sensor Readings//
void getSensorReadings(){

  //Sonar Reading
  ping.update();
  depth = ping.distance();
  confidence = ping.confidence();

  //IMU Reading
  imu::Quaternion quat = myIMU.getQuat();
  w = (String)quat.w();
  x = (String)quat.x();
  y = (String)quat.y();
  z = (String)quat.z();
  getCompass();
  getVelocity();

  //GPS Reading
  getGPS();
}


//Determines the current GPS positioning//
void getGPS(){
   for (start = millis(); millis() - start<1000;){
    while(GPS.available()){
      gpsChar = GPS.read();
      if(gps.encode(gpsChar)){
        newData = true;
      }
    }
  }
  if (newData){
    gps.f_get_position(&flat, &flon, &age);
    if(flat == TinyGPS::GPS_INVALID_F_ANGLE)
      latVal = 0.0;
    else
      latVal = flat, 6;
    
    if(flon == TinyGPS::GPS_INVALID_F_ANGLE)
      lonVal = 0.0;
    else
      lonVal = flon, 6;
    
    if(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES)
      satVal = 0;
    else
      satVal = gps.satellites();
    
    if(gps.hdop() == TinyGPS::GPS_INVALID_HDOP)
      precVal = 0;
    else
      precVal = ((float)gps.hdop())/100;
  }
  gps.stats(&chars, &sentences, &failed);

  lat = String(latVal,6);
  lon = String(lonVal,6);
  sat = (String)satVal;
  prec = (String)precVal;
}


//Determines the current heading of the rover//
void getCompass(){

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
   
  psi=atan2(Ym,Xm)/(2*3.14)*360;

  heading = psi;

  upperThresh = target + 5;
  lowerThresh = target - 5;

  
  phiFold=phiFnew;
  thetaFold=thetaFnew;
}


//Calculates the velocity of the rover//
void getVelocity(){
  timeNow = millis() - millisOld;
  imu::Vector<3> accV =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accVx = accV.x();
  accVy = accV.y();
  accRes = sqrt((accVx*accVx)+(accVy*accVy));
  velocity = initV + (accRes*timeNow);
  millisOld = timeNow;
  initV = velocity;
}


//Base-to-Rover and Rover-to-Base Communication//
void coms(){

//Operation telemetry message in form:
//GPS(Lat),GPS(Lon),IMU(Quat-w),IMU(Quat-x),IMU(Quat-y),IMU(Quat-z),IMU(Mag-heading),Velocity,Depth

  message = "M,"+ lat + "," + lon + "," + w + "," + x + "," + y + "," + z + "," + heading + "," + velocity + "," + depth + "\r\n";
  HC12.println(message);                          //Sending Data

  if (HC12.available()){                          //Receiving Data
    commandMessage = HC12.read();
//    parseCommand(commandMessage); 
    HC12.println("ACK");                          //Send Acknowledgement Reply
  }
}


//Determines battery voltages
void getVolts(){
  int sensorValue1 = analogRead(voltMeter1);
  int sensorValue2 = analogRead(voltMeter2);

  volts1 = sensorValue1 * (5.0/4095.0);
  volts2 = sensorValue2 * (5.0/4095.0);
}


//Calculates necessary course correction for new target//
//void getNavSolution(x,y){
//  
//}


//Navigate to argument coordinates based on navigation solution//
//void navigateTo(x,y){
//
//}


//System failure return to home feature//
//void goHome(){
//
//}


//IMU failure return to home via GPS feature//
//void goHomeGPS(){
//
//}


//GPS failure return to home via IMU feature//
//void goHomeIMU(){
//
//}



//Parse Incoming Commands//
//void parseCommand(commandMessage){
//  if (commandMessage == "C0"){
//    opMode = 0;
//  }
//  if (commandMessage == "C1"){
//    opMode = 1;
//  }
//  if (commandMessage == "C2"){
//    opMode = 2;
//  }
//  if (commandMessage == "C3"){
//    opMode = 3;
//  }
//  if (commandMessage == "C4"){
//    opMode = 4;
//  }
//  if (commandMessage == ""){
//    opMode = 1;
//  }
//}
