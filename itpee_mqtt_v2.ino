/*
  MQTT Client  Light sensor sender/receiver
  This sketch demonstrates an MQTT client that connects to a broker, subscribes to a topic,
  and both listens for messages on that topic and sends messages to it, a random number between 0 and 255.
  When the client receives a message, it parses it, and PWMs the built-in LED.
  Uses a TCS34725 light sensor to read lux and color temperature
  This sketch uses https://public.cloud.shiftr.io as the MQTT broker, but others will work as well.
  See https://tigoe.github.io/mqtt-examples/#broker-client-settings for connection details. 
Libraries used:
  * http://librarymanager/All#WiFiNINA or
  * http://librarymanager/All#WiFi101 
  * http://librarymanager/All#ArduinoMqttClient
  * http://librarymanager/All#Adafruit_TCS34725 (for the sensor)
  the arduino_secrets.h file:
  #define SECRET_SSID ""    // network name
  #define SECRET_PASS ""    // network password
  #define SECRET_MQTT_USER "public" // broker username
  #define SECRET_MQTT_PASS "public" // broker password
  created 11 June 2020
  updated 4 Jan 2023
  by Tom Igoe
*/

#include <WiFiNINA.h>  // use this for Nano 33 IoT, MKR1010, Uno WiFi
// #include <WiFi101.h>    // use this for MKR1000
#include <ArduinoMqttClient.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "arduino_secrets.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX


// initialize WiFi connection as SSL:
// WiFiSSLClient wifi;
//
WiFiClient wifi;
MqttClient mqttClient(wifi);

// for timestamping
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// details for MQTT client:
// char broker[] = "public.cloud.shiftr.io";
char broker[] = "test.mosquitto.org";
int port = 1883;
char topic[] = "conndev/joshjoshjosh";
String clientID = "joshjoshjosh";

// for sensor
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;  // Result data class structure, 1356 byes of RAM

int imageResolution = 0;  //Used to pretty print output
int imageWidth = 0;       //Used to pretty print output

//josh code
bool isDoorOpen;
int doorSum;
int doorAvg;
bool doorLatch;
bool entryLatch;
bool isPersonDetected;
bool personLatch;
bool exitLatch;
unsigned long timeAtPersonDetection;
unsigned long timeAtDoorOpen;

bool isOccupied = false;

int greenled = 2;  // digi pin 2
int redled = 3;    // digi pin 3


String macAddr;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // wait for serial monitor to open:
  if (!Serial) delay(3000);

  // get MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  // put it in a string:
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) macAddr += "0";
    macAddr += String(mac[i], HEX);
  }
  Serial.println();

  // set the credentials for the MQTT client:
  mqttClient.setId(clientID);
  // if needed, login to the broker with a username and password:
  // disabled for mosquitto
  // mqttClient.setUsernamePassword(SECRET_MQTT_USER, SECRET_MQTT_PASS);

  // for sensor
  Serial.println("ITPee by Josh Josh Josh");

  Wire.begin();           //This resets to 100kHz I2C
  Wire.setClock(400000);  //Sensor has max I2C freq of 400kHz

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false) {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1)
      ;
  }

  myImager.setResolution(4 * 4);  //Enable all 64 pads

  imageResolution = myImager.getResolution();  //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);          //Calculate printing width

  myImager.setRangingFrequency(3);  // 3Hz

  myImager.startRanging();

  // for LEDs
  pinMode(greenled, OUTPUT);
  pinMode(redled, OUTPUT);
}

void loop() {
  //if you disconnected from the network, reconnect:
  if (WiFi.status() != WL_CONNECTED) {
    connectToNetwork();
    // skip the rest of the loop until you are connected:
    return;
  } else {
    timeClient.begin();
    // timeClient.setTimeOffset(-5);
  }

  // if not connected to the broker, try to connect:
  if (!mqttClient.connected()) {
    Serial.println("attempting to connect to broker");
    connectToBroker();
  }
  // poll for new messages from the broker:
  // mqttClient.poll();

  // inserting sensor code
  //Poll sensor for new data
  if (myImager.isDataReady() == true) {
    if (myImager.getRangingData(&measurementData))  //Read distance data into array
    {
      // for clock
      timeClient.update();

      doorSum = 0;
      isPersonDetected = false;
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
        for (int x = imageWidth - 1; x >= 0; x--) {
          // Serial.print("\t");                                // uncomment to see grid or measurements
          // Serial.print(measurementData.distance_mm[x + y]);  // uncomment to see grid or measurements
          // Serial.print(",");

          if (y == 0) {
            // detect door open or closed by averaging
            doorSum += measurementData.distance_mm[x + y];
          } else if (y == imageWidth || y == 2 * imageWidth) {
            // detect for people
            if (measurementData.distance_mm[x + y] < 950) {
              isPersonDetected = true;
            };
          }
        }
        // Serial.println();  // uncomment to see grid or measurements
      }
      // Serial.println();  // uncomment to see grid or measurements

      // door logic here
      if (isPersonDetected && !personLatch) {
        Serial.println("person detected, waiting two seconds for door to open");
        personLatch = true;
        timeAtPersonDetection = millis();
      }

      if (millis() - timeAtPersonDetection > 2000) {
        personLatch = false;
      }
      // detect door open
      doorAvg = doorSum / 4;  // change for diff resolutions
      if (doorAvg > 900) {
        // Serial.println("door is opened");
        isDoorOpen = true;
        timeAtDoorOpen = millis();
      } else {
        isDoorOpen = false;
      }

      if (isDoorOpen && !doorLatch) {
        doorLatch = true;
      }

      // time out to allow for door to close after someone has gone in without setting off exit condition
      if (millis() - timeAtDoorOpen > 5000 && entryLatch) {
        entryLatch = false;
        // if door is still open, then cleaners?
      }
      // same but for coming out
      if (millis() - timeAtDoorOpen > 5000 && exitLatch) {
        exitLatch = false;
      }

      // door open and close condition
      // if person is detected AND door is not opened == someone came and saw it was locked
      // what to do if someone comes right when someone is leaving? maybe same as regular someone went in?

      // if door is open AND no person detected = someone has left
      if (isDoorOpen && !isPersonDetected && !exitLatch && !entryLatch) {
        Serial.println("someone has left the bathroom");
        digitalWrite(greenled, HIGH);
        digitalWrite(redled, LOW);
        exitLatch = true;
        isOccupied = false;
        String message = "exited";
        sendMqttMessage(message);
      }
      // if person is detected AND door is opened <2 seconds after detection (AND closed with no person detected?) = someone went in
      if (personLatch && isDoorOpen && !entryLatch && !exitLatch) {
        Serial.println("someone has entered the bathroom");
        digitalWrite(redled, HIGH);
        digitalWrite(greenled, LOW);
        personLatch = false;
        entryLatch = true;
        isOccupied = true;
        String message = "entered";
        sendMqttMessage(message);
      }
    }
  }
  delay(5);  //Small delay between polling
  // end sensor code
}

void sendMqttMessage(String status) {
  String message = "{\"status\": \"STATUS\", \"epoch-time\": \"TIME\"}";
  message.replace("STATUS", status);
  message.replace("TIME", String(timeClient.getEpochTime()));
  // include the MAC address as a unique ID for this client:
  // message.replace("ID", macAddr);

  if (mqttClient.connected()) {
    // start a new message on the topic:
    mqttClient.beginMessage(topic);
    // print the body of the message:
    mqttClient.print(message);
    // send the message:
    mqttClient.endMessage();
    // send a serial notification:
    Serial.print("published a message: ");
    Serial.println(message);
  }
}

boolean connectToBroker() {
  // if the MQTT client is not connected:
  if (!mqttClient.connect(broker, port)) {
    // print out the error message:
    Serial.print("MOTT connection failed. Error no: ");
    Serial.println(mqttClient.connectError());
    // return that you're not connected:
    return false;
  }

  // set the message receive callback:
  mqttClient.onMessage(onMqttMessage);
  // subscribe to a topic:
  Serial.print("Subscribing to topic: ");
  Serial.println(topic);
  mqttClient.subscribe(topic);

  // once you're connected, you
  // return that you're connected:
  return true;
}

void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.println("Received a message with topic ");
  Serial.print(mqttClient.messageTopic());
  Serial.print(", length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");
  String incoming = "";
  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    incoming += (char)mqttClient.read();
  }

  // print the incoming message:
  Serial.println(incoming);
}


void connectToNetwork() {
  // try to connect to the network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Attempting to connect to: " + String(SECRET_SSID));
    //Connect to WPA / WPA2 network:
    WiFi.begin(SECRET_SSID, SECRET_PASS);
    delay(2000);
  }
  // print IP address once connected:
  Serial.print("Connected. My IP address: ");
  Serial.println(WiFi.localIP());
}