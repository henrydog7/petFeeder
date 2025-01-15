#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>




#include "arduino_secrets.h"
#if defined(ESP32)
#include <ESP32Servo.h>

#else
#include "Servo.h"
#endif
#include "thingProperties.h"

int lastMicro = 0;

Servo myServo = Servo();
bool pos = 0;
int posO[2] = { 0, 180 };

#include <SD.h>
SDFile myFile;
int chipSelect = 5;


#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_GIGA) || defined(ARDUINO_OPTA)
#include <WiFi.h>
#elif defined(ARDUINO_PORTENTA_C33)
#include <WiFiC3.h>
#elif defined(ARDUINO_UNOR4_WIFI)
#include <WiFiS3.h>
#endif

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h

const char *ssid = "the big sky";      // Change this to your WiFi SSID
const char *password = "47%Chickens";  // Change this to your WiFi password
// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate
//    flashed in the WiFi module.

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "lbalog.local";
int port = 5050;
const char topic[] = "Image";

const long interval = 10000;
unsigned long previousMillis = 0;

int count = 0;
int servoPin = 18;


void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

#if defined(esp32)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);            // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000);  // attaches the servo on pin 18 to the servo object
#endif

  myServo.attach(3);
  initProperties();
  Serial.println("Connect to WiFI and IOT Cloud");
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  //attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(500);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  // You can provide a unique client ID, if not set the library uses Arduino-millis()
  // Each client must have a unique client ID
  // mqttClient.setId("clientId");

  // You can provide a username and password for authentication
  // mqttClient.setUsernamePassword("username", "password");

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1)
      ;
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true)
      ;
  }

  Serial.println("initialization done.");

  mqttClient.poll();

  myFile = SD.open("cat.jpg");
  if (myFile) {
    Serial.println("cat.jpg:");
    mqttClient.beginMessage(topic);
    char myByte;
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      myByte = myFile.read();
      Serial.write(myByte);
      mqttClient.print(myByte);
    }
    // close the file:
    myFile.close();
    mqttClient.endMessage();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening cat.txt");
    while (true)
      ;
  }
}

void loop() {
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();

  ArduinoCloud.update();
  //localTime += micros()-lastMicro;
  lastMicro = micros();
  Serial.println(localTime);
  // Your code here
  if (feedTime.isActive()) {
    Serial.println("Feeding");
    myServo.attach(posO[pos != 0]);
    pos = posO[pos != 0];
  }

  // to avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
  // see: File -> Examples -> 02.Digital -> BlinkWithoutDelay for more info
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    Serial.print("Sending message to topic: ");
    Serial.println(topic);
    Serial.print("hello ");
    Serial.println(count);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print("hello ");
    mqttClient.print(count);
    mqttClient.endMessage();

    Serial.println();

    count++;
  }
}



void onLocalTimeChange() {
  // Add your code here to act upon LocalTime change
}

/*
  Since FeedTime is READ_WRITE variable, onFeedTimeChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onFeedTimeChange() {
  // Add your code here to act upon FeedTime change
}

/*
  Since FoodIsEmpty is READ_WRITE variable, onFoodIsEmptyChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onFoodIsEmptyChange() {
  // Add your code here to act upon FoodIsEmpty change
}


/*
  Since IsCat is READ_WRITE variable, onIsCatChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onIsCatChange() {
  // Add your code here to act upon IsCat change
}
