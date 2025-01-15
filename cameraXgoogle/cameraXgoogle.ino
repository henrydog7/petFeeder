/******************************************************************************
   Copyright 2019 Google
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

 *****************************************************************************/

/*********
  This demo does the following:
    1. Loads small image from OV2640 camera
    2. Optionally base64-encodes payload data
    3. Transmits encoded bytes to Google Cloud

  Portions of the code used here are based on the following tutorial:
  https://randomnerdtutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/

  ...and bearing the following notice to be distributed code:
  """
  IMPORTANT!!!
   - Select Board "ESP32 Wrover Module"
   - Select the Partion Scheme "Huge APP (3MB No OTA)
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  """
*********/
#if defined(ESP8266) or defined(ARDUINO_SAMD_MKR1000)
#define __SKIP_ESP32__
#endif

#if defined(ESP32)
#define __ESP32_MQTT__
#endif

#ifdef __SKIP_ESP32__

#if defined(_ARDUINO_R4_WIFI_)
#define __SKIP_ESP32__
#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>

#include "fb_gfx.h"
#include "img_converters.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems

#endif


#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
// #include "memorysaver.h"
//This demo can only work on OV2640_MINI_2MP platform.
#if !(defined OV2640_MINI_2MP)
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define BMPIMAGEOFFSET 66


#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}


#endif

#ifdef __ESP32_MQTT__

#include "esp_camera.h"
#include <esp_wifi.h>
#include "esp_timer.h"
#include "fb_gfx.h"
#include "img_converters.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "Arduino.h"
#include "SPIFFS.h"
#include <WiFi.h>

#include "base64.h"
#include "esp32-mqtt.h"


#define CAMERA_LED_GPIO 13
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMEAR_MODEL_M5STACK_PSRAM_ALT
#define ARDUCAM_MREN318



#if defined(ARDUCAM_MREN318)
#define BMPIMAGEOFFSET 66
const char bmp_header[BMPIMAGEOFFSET] PROGMEM =
{
  0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
  0x00, 0x00
};
// set pin 7 as the slave select for the digital pot:
const int CS = 7;
bool is_header = false;
int mode = 0;
uint8_t start_capture = 0;

#if defined (OV2640_MINI_2MP)
ArduCAM myCAM( OV2640, CS );
#else
ArduCAM myCAM( OV5642, CS );
#endif
uint8_t read_fifo_burst(ArduCAM myCAM);

#else
#error "Camera model not selected"
#endif

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  uint8_t vid, pid;
  uint8_t temp;

  Serial.begin(115200);

  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;//FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera init


  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  // initialize SPI:
  SPI.begin();
  //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  while (1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55) {
      Serial.println(F("ACK CMD SPI interface Error! END"));
      delay(1000); continue;
    } else {
      Serial.println(F("ACK CMD SPI interface OK. END")); break;
    }
  }

#if defined (OV2640_MINI_2MP)
  while (1) {
    //Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) {
      Serial.println(F("ACK CMD Can't find OV2640 module! END"));
      delay(1000); continue;
    }
    else {
      Serial.println(F("ACK CMD OV2640 detected. END")); break;
    }
  }
#else
  while (1) {
    //Check if the camera module type is OV5642
    myCAM.wrSensorReg16_8(0xff, 0x01);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x42)) {
      Serial.println(F("ACK CMD Can't find OV5642 module! END"));
      delay(1000); continue;
    }
    else {
      Serial.println(F("ACK CMD OV5642 detected. END")); break;
    }
  }
#endif



  //Change to JPEG capture mode and initialize the OV5642 module
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
#if defined (OV2640_MINI_2MP)
  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
#else
  myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
  myCAM.OV5642_set_JPEG_size(OV5642_320x240);
#endif
  delay(1000);
  myCAM.clear_fifo_flag();
#if !(defined (OV2640_MINI_2MP))
  myCAM.write_reg(ARDUCHIP_FRAMES, 0x00);
#endif
}


//pinMode(CAMERA_LED_GPIO, OUTPUT);
//digitalWrite(CAMERA_LED_GPIO, HIGH);

WiFi.disconnect(true);


Serial.println("Starting network");
//digitalWrite(CAMERA_LED_GPIO, LOW);
delay(500);
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);
Serial.println("Connecting to WiFi");
bool on = false;
//digitalWrite(CAMERA_LED_GPIO, HIGH);
while (WiFi.status() != WL_CONNECTED) {
  WiFi.waitForConnectResult();
}
Serial.println("\nConnected!");
Serial.print("IP address: ");
Serial.println(WiFi.localIP());

setupCloudIoT();

digitalWrite(CAMERA_LED_GPIO, LOW);

if (!SPIFFS.begin(true)) {
  Serial.println("An Error has occurred while mounting SPIFFS");
  return;
}
}


/**
   This function uses SPIFFS as swap space for temporarily storing the
   temporary base64-encoded image.
*/
void publishTelemetryFromFile() {
  File file = SPIFFS.open("/b64image.txt", FILE_READ);
  if (!file) {
    Serial.println("There was an error opening the file for read");
    return;
  } else {
    Serial.println("Publishing data using temp file");
  }
  char* data = (char*)heap_caps_malloc(file.size(), MALLOC_CAP_8BIT);

  int i = 0;
  while (file.available()) {
    data[i++] = file.read();
  }
  Serial.println(String(i) + " bytes read");

  delay(10);
  mqtt->loop();
  mqtt->publishTelemetry(data, file.size());
  mqtt->loop();
  delay(10);
  file.close();
  Serial.println("Done publish.");
}


/**
   Captures an image using the camera library and transmits it to Google
   Cloud IoT Core.
*/
void transmitImage() {
  digitalWrite(CAMERA_LED_GPIO, HIGH);

  // Retrieve camera framebuffer
  camera_fb_t * fb = NULL;
  uint8_t* _jpg_buf = NULL;
  esp_err_t res = ESP_OK;
  size_t frame_size = 0;

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    res = ESP_FAIL;
  } else {
    if (fb->width > 400) {
      Serial.println(fb->format);
      Serial.println(fb->len);
      if (fb->format != PIXFORMAT_JPEG) {
        Serial.println("Compressing");
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &frame_size);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.println("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        frame_size = fb->len;
        _jpg_buf = fb->buf;
      }
    }
  }
  if (res != ESP_OK) {
    ESP_LOGW(TAG, "Camera capture failed with error = %d", err);
    return;
  }

  publishTelemetry((char*)_jpg_buf, frame_size);
  digitalWrite(CAMERA_LED_GPIO, LOW);
}

// The MQTT callback function for commands and configuration updates
// Place your message handler code here.
void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  // Uncomment to transmit image when receiving commands
  // Note: If your device is named command, this will send images on all
  //       messages such as configuration change which is sent on connect.
  if (topic.lastIndexOf("/command") > 0) {
    Serial.println("Transmit image on receieve command");
    transmitImage();
  }
}
///////////////////////////////



int imageWaitMillis = 30000;
unsigned long lastTransmit = millis();
void loop() {
  /*
    // Transmit every 30 seconds?
    if (millis() > lastTransmit + imageWaitMillis) {
    transmitImage();
    lastTransmit = millis();
    }
  */


  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
  delay(1000);
  Serial.println(F("ACK CMD switch to OV2640_320x240 END"));
  temp = 0xff;

  

  // Transmit anytime there's serial input
  if (Serial.available() > 0) {
    // Clear any outstanding input, just one image per head banger
    while (Serial.available() > 0) {
      Serial.read();
    }

  if (start_capture == 1) {
      myCAM.flush_fifo();
      myCAM.clear_fifo_flag();
      //Start capture
      myCAM.start_capture();
      start_capture = 0;
    }
    if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
      Serial.println(F("ACK CMD CAM Capture Done. END"));
      delay(50);
      read_fifo_burst(myCAM);
      //Clear the capture done flag
      myCAM.clear_fifo_flag();
    }
    
    transmitImage();
  }
  delay(10);  // <- fixes some issues with WiFi stability

  if (!mqttClient->connected()) {
    connect();
  } else {
    mqtt->loop();
  }
}
#endif







uint8_t read_fifo_burst(ArduCAM myCAM) {
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  length = myCAM.read_fifo_length();
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE)  //512 kb
  {
    Serial.println(F("ACK CMD Over size. END"));
    return 0;
  }
  if (length == 0)  //0 kb
  {
    Serial.println(F("ACK CMD Size is 0. END"));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();  //Set fifo burst mode
  temp = SPI.transfer(0x00);
  length--;
  while (length--) {
    temp_last = temp;
    temp = SPI.transfer(0x00);
    if (is_header == true) {
      Serial.write(temp);
    } else if ((temp == 0xD8) & (temp_last == 0xFF)) {
      is_header = true;
      Serial.println(F("ACK IMG END"));
      Serial.write(temp_last);
      Serial.write(temp);
    }
    if ((temp == 0xD9) && (temp_last == 0xFF))  //If find the end ,break while,
      break;
    delayMicroseconds(15);
  }
  myCAM.CS_HIGH();
  is_header = false;
  return 1;
}
