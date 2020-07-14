#include <Wire.h>


#define RXD2 16  //(RX2)
#define TXD2 17 //(TX2)
#define HC12 Serial2  //Hardware serial 2 on the ESP32
#define LED_BUILTIN 2

void setup() 
{
  Serial.begin(115200);                            // Serial port to computer
  HC12.begin(115200, SERIAL_8N1, RXD2, TXD2);      // Serial port to HC12
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() 
{
  while (Serial.available()) 
  {      
//     If we have data from Serial monitor
    HC12.write(Serial.read());      // Send that data to HC-12
  }  
  if (HC12.available()) 
  {        
    // If HC-12 has data
    Serial.write(HC12.read());      // Send the data to Serial monitor
    digitalWrite(LED_BUILTIN, HIGH);
  }
  digitalWrite(LED_BUILTIN, LOW);

}
