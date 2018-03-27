#include <SoftwareSerial.h>


#define esp FALSE //comment out to remove esp msgs

//*-- Hardware Serial
#define _baudrate 9600

//*-- Software Serial
//
#define _rxpin      9
#define _txpin      10
SoftwareSerial esp( _rxpin, _txpin ); // RX, TX

//*-- IoT Information
#define SSID "#########"
#define PASS "########"
#define server "serever_url" // Localhost IP Address: 184.106.153.149

// GET 
String GET = "GET /firebaseEco.php";

/*
  This is a simple code to test BH1750FVI Light senosr
  communicate using I2C Protocol
  this library enable 2 slave device address
  Main address  0x23
  secondary address 0x5C
  connect this sensor as following :
  VCC >>> 3.3V
  SDA >>> A4
  SCL >>> A5
  addr >> A3
  Gnd >>>Gnd

  Written By : Mohannad Rawashdeh
 
*/

// First define the library :
#include <SoftwareSerial.h>

//*-- Hardware Serial
#define _baudrate 9600

//*-- Software Serial
//
#define _rxpin      9
#define _txpin      10

#include <BH1750FVI.h> // Sensor Library
#include <Wire.h> // I2C Library

#include <dht.h>

dht DHT;

#define DHT11_PIN 7



#define CO2_SENSOR_PWM_PIN 3


unsigned long duration;
long ppm;
/*MySensor gw;
MyMessage msg(CHILD_ID, V_LEVEL);
*/
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------MH-Z14


// A simple data logger for the Arduino analog pins

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  10000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 10000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

/* the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

// The analog pins that connect to the sensors
#define photocellPin 0           // analog 0
#define tempPin 1                // analog 1
#define BANDGAPREF 14            // special indicator that we want to measure the bandgap

#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!
#define bandgap_voltage 1.1      // this is not super guaranteed but its not -too- off
*/



//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------Logger

BH1750FVI LightSensor;




void resetEsp() {

esp.println("AT+CWMODE=3");

delay(500);

if(esp.find("OK") ) {
  Serial.println("Module Reset");
  }

   esp.println("AT+RST");
  delay(2000);
    if(esp.find("OK") ) {
      Serial.println("Module Reset");

      esp.println("AT+CIPMUX=1");
      if(esp.find("OK") ) {
        Serial.println("Module Mul");

     
      }else{
        Serial.println("Module Mul failed");
      }
    }else{
        Serial.println("Module Reset failed");
    }
}
boolean connectWiFi()
{
  //Connect to Router with AT+CWJAP="SSID","Password";
  // Check if connected with AT+CWJAP?
  String cmd="AT+CWJAP=\""; // Join accespoint
  cmd+=SSID;
  cmd+="\",\"";
  cmd+=PASS;
  cmd+="\"";
  sendesp(cmd);
  delay(4000);
  if(esp.find("OK"))
  {
    Serial.println("RECEIVED: OK");
    return true;
  }
  else
  {
    Serial.println("RECEIVED: Error CWJAP");
    return false;
  }

 /* cmd = "AT+CIPMUX=0";// Set Single connection
  sendesp( cmd );
  if( esp.find( "Error") )
  {
    Serial.print( "RECEIVED: Error" );
    return false;
  }*/
}

void sendesp(String cmd)
{
  Serial.print("SEND: ");
  esp.println(cmd);
  Serial.println(cmd);
}


void setup() {
   Serial.begin( _baudrate );
    LightSensor.begin();
  pinMode(CO2_SENSOR_PWM_PIN, INPUT);
   LightSensor.SetAddress(Device_Address_H);//Address 0x5C
  esp.begin( _baudrate );

  sendesp("AT");
  delay(5000);
  if(esp.find("OK"))
  {
    Serial.println("RECEIVED: OK\nData ready to sent!");
    //connectWiFi();
  }
  /*reset();
  if(connectWiFi()){
    Serial.println("Connected");
   }else{
    Serial.println("Connect failed");
   }
*/
LightSensor.SetMode(Continuous_H_resolution_Mode);
 
  Serial.println("Running...");
}

void loop() {

  resetEsp();
  if(connectWiFi()){
    Serial.println("Connected");
   }else{
    Serial.println("Connect failed");
   }
int chk = DHT.read11(DHT11_PIN);
  
 Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  delay(1000);
  // put your main code here, to run repeatedly:
  uint16_t lux = LightSensor.GetLightIntensity();// Get Lux value
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lux");
  while(digitalRead(CO2_SENSOR_PWM_PIN) == HIGH) {;}
  //wait for the pin to go HIGH and measure HIGH time
  duration = pulseIn(CO2_SENSOR_PWM_PIN, HIGH, 5000);
  ppm = 5000 * ((duration/1000) - 2)/1000;
  Serial.print("CO2 :");
  Serial.print(ppm);
  Serial.println(" PPM");
  //Serial.println(ppm);
  String temp=(String)DHT.temperature;
  String hum=(String)DHT.humidity;
  String light=(String)lux;
  String co2=((String)(ppm/10000));

 
  
  updateTS(temp,hum,light,co2);
  delay(3000); //
}
//----- update the  Thingspeak string with 3 values
void updateTS( String T,String H,String L,String C)
{
  // ESP8266 Client
  String cmd = "AT+CIPSTART=4,\"TCP\",\"";// Setup TCP connection
  cmd += server;
  cmd += "\",80";
  sendesp(cmd);
  //delay(2000);
  if( esp.find( "Error" ) )
  {
    Serial.print( "RECEIVED: Error\nExit1" );
    return;
  }

  cmd = GET + "?t=" + T +"&h="+ H +"&l="+ L +"&c="+ C +" HTTP/1.1\r\nHost: "+server+"\r\n\r\n\r\n";
  esp.print( "AT+CIPSEND=4," );
  esp.println( cmd.length() );
  if(esp.find( ">" ) )
  {
    //esp.print(">");
    esp.print(cmd);
    Serial.print(cmd);

    if( esp.find("OK") )
      {
        Serial.println( "RECEIVED IN: OK" );
         while (esp.available()) {

            String tmpResp = esp.readString();

            Serial.println(tmpResp);

        }
      }
   else
       {
    Serial.println( "RECEIVED: Error\nExit2" );
      }
  }
  else
  {
    sendesp( "AT+CIPCLOSE" );//close TCP connection
  }
 
}





