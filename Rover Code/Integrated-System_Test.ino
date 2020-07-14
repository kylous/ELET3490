#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <TinyGPS.h>

 
 
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define RXD1 18       //RX1 - GPS
#define TXD1 19       //TX1 - GPS
#define RXD2 16       //RX2 - Radio
#define TXD2 17       //TX2 - Radio
#define GPS Serial1   //Hardware serial 1 on ESP32 - GPS
#define HC12 Serial2  //Hardware serial 2 on the ESP32 - Radio

Adafruit_BNO055 myIMU = Adafruit_BNO055();
TinyGPS gps;

String lat;
String lon;
String sat;
String prec;
String compass;
String w;
String x;
String y;
String z;
String acc;
String gy;
String mg2;
String sys;
String message;

float flat, flon, latVal, lonVal;
int satVal, precVal;
unsigned long age;
    
void setup() {

Serial.begin(115200);                             //UART0 - Ping Sonar @115200 BAUD
GPS.begin(9600, SERIAL_8N1, RXD1, TXD1);          //UART1 - GPS @9600 BAUD
HC12.begin(115200, SERIAL_8N1, RXD2, TXD2);       //UART2 - Radio @115200 BAUD

myIMU.begin();                                    //Initialize IMU
delay(1000);                                      //Give IMU 1s to boot
int8_t temp=myIMU.getTemp();                      //Get Temp from BNO-055
myIMU.setExtCrystalUse(true);                     //Set BNO-055 to External Oscillator
}
 
void loop() {
  
uint8_t system, gyro, accel, mg = 0;
myIMU.getCalibration(&system, &gyro, &accel, &mg);
imu::Quaternion quat=myIMU.getQuat();


bool newData = false;
unsigned long chars;
unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPS.available())
    {
      char c = GPS.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
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
      precVal = gps.hdop();
  }

  gps.stats(&chars, &sentences, &failed);




lat = String(latVal,6);
lon = String(lonVal,6);
sat = (String)satVal;
prec = (String)precVal;
//compass = (String)mag;
w = (String)quat.w();
x = (String)quat.x();
y = (String)quat.y();
z = (String)quat.z();
acc = (String)accel;
gy = (String)gyro;
mg2 = (String)mg;
sys = (String)system; 


if(chars == 0)
  message = "-99,-99,-99,-99,"+w +"," +x +"," +y +"," +z +"," +acc +"," +gy +"," +mg2 + "," + sys +"\n";
else
  message = lat +"," +lon +"," +sat +"," +prec +"," +w +"," +x +"," +y +"," +z +"," +acc +"," +gy +"," +mg2 + "," + sys +"\n";

  
HC12.println(message);
Serial.println(message);

delay(BNO055_SAMPLERATE_DELAY_MS);
}
