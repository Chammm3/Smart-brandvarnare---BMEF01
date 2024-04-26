/*
  If the IP address of your board is yourAddress:
    http://192.168.4.1/H turns the LED on
    http://192.168.4.1/L turns it off
 */
#include <Arduino.h>
#include <SPI.h>
#include <WiFiS3.h>
//#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WiFiServer.h>
#include <RTC.h>
#include <Wire.h>
#include <MAX30105.h>
#include <Adafruit_INA219.h>


Adafruit_INA219 ina219;


int buzzerPin = 8;
MAX30105 particleSensor;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "SECRET_SSID";        // your network SSID (name)
char pass[] = "SECRET_PASS";        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)

int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);
void printWiFiStatus();

int counter1=0;
int counter2=0;

//Battery
///////////////////////////////
float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0; // needed
  float loadvoltage = 0;
  float power_mW = 0;
  float batteryvoltage = 0;
  float batteryresistance = 2.85;
  float batterymaxvoltage = 9.3;
  float batteryprecent = 0;
  String batteryPrecent="";
/////////////////////////////////


void setup() {
  
  //RTC
  ////////////////////////////
  RTC.begin();
  Serial.begin(9600);

  RTCTime startTime(30, Month::APRIL, 2024, 12, 00, 00, DayOfWeek::MONDAY, SaveLight::SAVING_TIME_ACTIVE);

  RTC.setTime(startTime);
  /////////////////////////////////////////
  


// ALARM
///////////////////////////////////////////
  pinMode (buzzerPin, OUTPUT);
  
  Serial.println("Initializing...");

  // Initialize sensor
  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }


  //The LEDs are very low power and won't affect the temp reading much but
  //you may want to turn off the LEDs to avoid any local heating
  particleSensor.setup(0); //Configure sensor. Turn off LEDs
  //particleSensor.setup(); //Configure sensor. Use 25mA for LED drive

  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.

/////////////////////////////////


// Communication
//Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Access Point Web Server");
  Serial.println("Starting");
  

  pinMode(led, OUTPUT);      // set the LED pin mode

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  WiFi.config(IPAddress(192,48,56,2));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
  ////////////////////////////

//Battery
//////////////////////////////////
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
   // while (1) { delay(10); }
  }





}


void loop() {
  float temperature = particleSensor.readTemperature();
  if(temperature > 28){ // if the temperature exceed 24c the speaker will make a constant sound. 
      digitalWrite (buzzerPin, HIGH);
  } else {
      digitalWrite (buzzerPin, LOW);
  }

  if(counter1>=100)
  {
    counter1=0;
     Serial.print("temperatureC=");
    Serial.print(temperature, 4);
    Serial.println();

  }
//Serial.println("test");
  
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      delayMicroseconds(10);                // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<p style=\"font-size:7vw;\">Click <a href=\"/H\">here</a> turn the LED on<br></p>");
            client.print("<p style=\"font-size:7vw;\">Click <a href=\"/L\">here</a> turn the LED off<br></p>");

            client.print("<p style=\"font-size:7vw;\">Click <a href=\"/T\">here</a> TEST<br></p>");
            client.print("<p style=\"font-size:7vw;\">Click <a href=\"/B\">here</a> CHECK BATTERY<br></p>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(led, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(led, LOW);                // GET /L turns the LED off
        }
        if (currentLine.endsWith("GET /T")) {
          digitalWrite(buzzerPin, HIGH);
          delay(500);
          digitalWrite(buzzerPin, LOW);             // GET /T - test the buzzer
        }
        if (currentLine.endsWith("GET /B")) {
          batteryPrecent=String(batteryprecent);
          
          client.print("<p style=\"font-size:7vw;\">Battery:"+ batteryPrecent + "</p>");           // GET /B - display battery percent
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }


  // Battery
  //////////////////////////////////////
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();   // needed
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);   // needed
  batteryvoltage = (current_mA/1000)*batteryresistance;  // needed
  batteryprecent = abs((batteryvoltage/batterymaxvoltage) * 100);  // needed


  //////////////////////////////////////

  counter1++;
  counter2++;
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}

