#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP32Servo.h>
#include "DHT.h"
//#include <WebServer.h>

#define DHTPIN 0          // Pin connected to the DHT sensor
#define DHTTYPE DHT22

const char* ssid = "S22ultra";
const char* password = "yashpatel";
WiFiServer server(1234);  // Define the server port


float minTemp = 0.0;
float maxTemp = 0.0;

float minHumi = 0.0;
float maxHumi = 0.0;

const int servoPin = 25;
Servo myServo;
int motorpin = 27;
const int irSensorPin = 35;
const int ledPin = 4; 

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  setupWifi();
  server.begin();
  Serial.print("Server started on IP address: ");
  Serial.println(WiFi.localIP());
  myServo.attach(servoPin);
  dht.begin();
  //server.on("/set_temperature", handleSetTemperature);
  //Serial.println("HTTP server started");
  pinMode(motorpin, OUTPUT);
  pinMode(ledPin, OUTPUT); 
}

void loop() {

WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected");
    while (client.connected()) {
      if (client.available()) {
        String data = client.readStringUntil('\n');  // Read data from the client
        //Serial.print("Received data: ");
        //Serial.println(data);

        Serial.println(data);
      
        // Process the received data here
     
         //String data = client.readStringUntil('\n');
        
         //String data = client.readStringUntil('\n');
         //int commaIndex = data.indexOf('+');
    if (data.startsWith("Humidity:")) {
      String minHumistr = data.substring(9); // 12 is the length of "Temperature:"
      int commaIndex = minHumistr.indexOf(',');

      if (commaIndex != -1) {
      String maxHumistr = minHumistr.substring(commaIndex + 1);
      minHumistr = minHumistr.substring(0, commaIndex);

      
      minHumi = minHumistr.toFloat();
      maxHumi = maxHumistr.toFloat();
      Serial.print("Minimum Humidity set to: ");
      Serial.println(minHumi);
      Serial.print("Maximum Humidity set to: ");
      Serial.println(maxHumi);
      }
    }

    if (data.startsWith("Temperature:")) {
      String minTempstr = data.substring(12); // 12 is the length of "Temperature:"
      int commaIndex = minTempstr.indexOf(',');

      if (commaIndex != -1) {
      String maxTempstr = minTempstr.substring(commaIndex + 1);
      minTempstr = minTempstr.substring(0, commaIndex);

      
      minTemp = minTempstr.toFloat();
      maxTemp = maxTempstr.toFloat();
      Serial.print("Minimum Temperature set to: ");
      Serial.println(minTemp);
      Serial.print("Maximum Temperature set to: ");
      Serial.println(maxTemp);
         
       //int separator_index = data.indexOf(',');
        //if (separator_index != -1) {
          //Serial.print("get data:");
          /*Serial.println(data);
          String minTempstr  = data.substring(0, data.indexOf(','));
          //Serial.print(data);
          minTemp = minTempstr.toFloat();
          data.remove(0, data.indexOf(',') + 1);
          //separator_index = data.indexOf(',');
          //Serial.print(data);
          
          String maxTempstr = data.substring(data.indexOf(',') + 1);
          //Serial.print(maxTempstr);
          maxTemp = maxTempstr.toFloat();
           Serial.print("Minimum Temperature set to: ");
          Serial.println(minTemp);
            Serial.print("Maximum Temperature set to: ");
            Serial.println(maxTemp);*/

          //if (separator_index = -1) {
            //String maxTempstr = data.substring(0);
            //maxTemp = maxTempstr.toFloat();
           // data.remove(0, separator_index + 1);
           // separator_index = data.indexOf(',');
            //Serial.print("Maximum Temperature set to: ");
            //Serial.println(maxTemp);
    }
    }
  
    if (data.startsWith("LED:")) {
      String LEDcount = data.substring(4); // 12 is the length of "Temperature:"
      //int commaIndex = minTempstr.indexOf(',');
      LEDcount = LEDcount.substring(0);
      int count = LEDcount.toInt();
      blinkLed(count);
        Serial.print("LED count: ");
      Serial.println(count);
    }

        if (data == "Turn on Servo") {
          myServo.write(90);  // Set the servo angle to 90 degrees
          delay(5000);        // Wait for 1 second
          myServo.write(0);   // Set the servo angle to 0 degrees
          //delay(5000);
      // Code to turn on the LED
      Serial.println("Servo motor on");
    }
    else {
      myServo.write(0); 
      // Code to turn off the LED
      Serial.println("Servo motor off");
    } 
          
        }
      
      
       
      
       
        // Example: Send the IR sensor and temperature values
        //int irValue = analogRead(A0);
        float tempValue = dht.readTemperature();  // Gets the values of the temperature
	      float humvalue = dht.readHumidity();
        String sendDataTE = "Temperature:" + String(tempValue);
        String sendDataHU = "Humidity:" + String(humvalue);
        //client.println(sendDataTE);
        //client.println(sendDataHU);
        Serial.println(tempValue);
        Serial.println(humvalue);
        
         if (tempValue >= minTemp && tempValue <= maxTemp){
          Serial.print("motor stop");
          String motor = "Motor:" + String(0);
          //client.println(motor);
           digitalWrite(motorpin,LOW); // Turn on motor

           
        }
        else if (tempValue < minTemp || tempValue > maxTemp){
          Serial.print("motor start");
          String motor = "Motor:" + String(1);
          client.println(motor);
          digitalWrite(motorpin,HIGH);
        }
         

        int sensorValue = analogRead(irSensorPin);
        //Serial.print("sensorvalue:");
        //Serial.println(sensorValue);
  
  // Convert the analog value to distance using calibration values
        float distance = map(sensorValue, 0, 4095, 80, 10);
        Serial.print("Distance:");
        Serial.println(distance);
        if (distance <= 40.00)      {
         int irValue = 1;
         String sendDataIR = "IR Value:" + String(irValue);
         client.println(sendDataIR);
         //delay(500);
         //Serial.print("Distance:");
         //Serial.println(distance);

        }
        //int irValue = 1;
        //delay(8000);
        //float tempValue,humvalue = getTemperature(); // Replace this with your temperature reading code
        String sendData = "Temperature: " + String(tempValue) + ", Humidity: " + String(humvalue) + "n";
       // String sendDataIR = "IR Value:" + String(irValue);
        //String sendDataTemp = "Temperature:" + String(tempValue);
        //String sendDataHum = "Humidity:" + String(humvalue);
       Serial.println(sendData) ;
       //client.print("IR Value: ");
       //client.println(irValue);
     // client.println(sendDataIR);
      //client.println("\n");
//client.println(sendDataIR);
      delay(2000);
      client.println(sendData);
      //client.println("\n");
      //client.println(sendDataTemp);
      //client.println(sendDataHum);
        
        /*client.print("IR Value: ");
        client.println(irValue);
        client.print("Temperature: ");
        client.println(tempValue);
        client.print("Humidity: ");
        client.println(humvalue);
        client.println();*/

        
      //else{
        //client.stop();
        //Serial.println("Client disconnected");
      delay(500);
       
    }
      }
  }
// Check if the temperature is within the defined range
  //float tempValue = dht.readTemperature();
   
void blinkLed(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

  


void setupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}



