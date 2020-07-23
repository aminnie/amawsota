/**************************************************************************************************
   AWS S3-based OTA Update Utility
   Date: 07/11/2020
   Purpose: Perform an OTA update from an Amazon S3 bucket (HTTP Only)
            This code eventually will be added to the main sensor logic
            to enable the board to update itself to the latest code based
            on a command via the https://amsensor.net website.

   Optionally: Erase the NVS (non-volatile storage) - only if sensor is reporting NVS issues
**************************************************************************************************/

#include <WiFi.h>
#include <Update.h>

#include <nvs.h>
#include <nvs_flash.h>

WiFiClient client;

// Please note:
// Uncomment the sensor board you plan to update below 
// Only one board should be uncommented for a build
// Every controller board is different in terms of pinouts, OLED screen supported or not,
// and sensors. Please uncomment the correct one.

#define HILETGO     // Heltec_WiFi_Kit_32 driver
//#define TTGOCAM     // TTGO LoRa32-OLED V1 driver, board with Camera and PIR
//#define TTGODISP    // TTGO LoRa32-OLED V1 driver, board with color screen only
//#define DEVKITC       // ESP32 Dev Module driver. Espressif Reference board  

// *** Update the SSID and PWD that the sensor board is to connect to in your home.
// *** ESP32 boards, and most other controllers, connect to 2.4GHz only 
const char* SSID = "HOME-D55B-2.4";
const char* PWD = "fancy7520entire";

// AMSensor S3 Bucket Config
String host = "http://amsensorota.s3-website-us-west-2.amazonaws.com";  // Host => bucket-name.s3.region.amazonaws.com
int port = 80;                                                          // HTTPS = 443. As of today, HTTPS not working

// Test application for Heltec: Wifi Scanner Demo BIN file download location
//String bin = "/AMWiFiScan.ino.heltec_wifi_kit_32.bin"; // bin file name with a slash in front.
//String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/AMWiFiScan.ino.heltec_wifi_kit_32.bin";

// Sensor BIN files and download location
#ifdef HILETGO
  char manuf[] = "HL1";
  String bin = "/amsensor-iot.ino.heltec_wifi_kit_32.bin"; // bin file name with a slash in front.
  String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/amsensor-iot.ino.heltec_wifi_kit_32.bin";
#endif
#ifdef TTGOCAM
  char manuf[] = "TTC";
  String bin = "/amsensor-iot.ino.ttgo-lora32-v1.bin"; // bin file name with a slash in front.
  String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/amsensor-iot.ino.ttgo-lora32-v1.bin";
#endif
#ifdef TTGODISP
  char manuf[] = "TTD";
  String bin = "/amsensor-iot.ino.ttgo-display-v1.bin"; // bin file name with a slash in front.
  String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/amsensor-iot.ino.ttgo-display-v1.bin";
#endif
#ifdef DEVKITC
  char manuf[] = "ESP";
  String bin = "/amsensor-iot.ino.esp32.bin"; // bin file name with a slash in front.
  String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/amsensor-iot.ino.esp32.bin";
#endif

// Variables to validate S3 responses
long contentLength = 0;
bool isValidContentType = false;
int retrycnt = 0;

// Utility to extract header value from headers
String getHeaderValue(String header, String headerName) {

  return header.substring(strlen(headerName.c_str()));
}


/**************************************************************************************************
* NVS Factory Reset 
* https://github.com/espressif/arduino-esp32/issues/1941
**************************************************************************************************/
void clearNVS() {
    int err;

    err = nvs_flash_init();
    Serial.println("nvs_flash_init: " + err);
    
    err = nvs_flash_erase();
    Serial.println("nvs_flash_erase: " + err);
 }
 

/**************************************************************************************************
* OTA Logic 
**************************************************************************************************/
void execOTA() {
  
  Serial.println("OTA Attempt: " + retrycnt);
  Serial.println("Connecting to: " + String(host));
  
  // Connect to S3
  if (client.connect(host.c_str(), port)) {
    
    Serial.println("Connection succeeded: " + String(host));
    Serial.println("Fetching Bin: " + String(bin));

    // Get the contents of the bin file
    client.print(String("GET ") + bin1 + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("Client Timeout !");
        client.stop();
        return;
      }
    }
    
    while (client.available()) {
      
      String line = client.readStringUntil('\n');       // Read to end of line
      Serial.println(line);
      line.trim();                                      // Remove space, to check if the line is end of headers

      // If the the line is empty, this is end of headers. 
      // Break the while and feed the remaining `client` data to the Update.writeStream();
      if (!line.length()) {
        break;                  // Headers ended. Get the OTA started
      }

      // Check if the HTTP Response is HHTP 200 else break and Exit Update
      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
          Serial.println("Status code other than HTTP 200 received. Exiting OTA Update.");
          break;
        }
      }

      // Extract headers here, startinf with content length
      if (line.startsWith("Content-Length: ")) {
        contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Received " + String(contentLength) + " bytes from server");
      }

      // Next, extract content type
      if (line.startsWith("Content-Type: ")) {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Received payload: " + contentType);
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
      }
    }
  } else {
    // Connect attempt to S3 failed. Consider retry?
    Serial.println("Connection to S3 (" + String(host) + ") failed. Please validate your setup");
    // retry execOTA(); ??
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength: " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // Check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canbegin = Update.begin(contentLength);

    // If good, begin OTA
    if (canbegin) {
      Serial.println("Starting OTA. This may take 2 - 5 mins to complete. You may not see much feedback as the update is written to the new boot partition. Hang in there!... ");

      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        Serial.println("ESP32 Boot partitiation updated. Wrote: " + String(written) + " bytes successfully");
      }
      else {
        Serial.println("Wrote only: " + String(written) + "/" + String(contentLength) + " bytes. Retrying..." );

        while (retrycnt++ < 10) {
          execOTA();
        }
      }

      if (Update.end()) {
        
        Serial.println("Sensor OTA done!");
        
        if (Update.isFinished()) {
          Serial.println("Sensor update completed successfully! Rebooting...");
          ESP.restart();
        }
        else {
          Serial.println("Update not finished? Something went wrong!. Please retry");
        }
      }
      else {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    }
    else {
      
      // Not enough space to begin OTA. Research partitions and space availability for your sensor device
      Serial.println("Not enough space to begin OTA");
      client.flush();
    }
  }
  else {

    Serial.println("There was no content in the HTTP response");
    client.flush();
  }
}


/*********************************************************************************
* Get ESP32 Chip ID and Sensor Key to register
* Chip ID is essentially its MAC address(length: 6 bytes)
* We prefix with three letters to indicate sensor board and board version 
*********************************************************************************/
uint64_t chipid;       // ESP32 Mac Address
char schipId[20];      // Derived THINGNAME e.g. "HL1-1234-C5C2DDBC"

char* getThingname() {

  chipid = ESP.getEfuseMac();     
  snprintf(schipId, 20, "%s-%04X-%08X", manuf, (uint16_t)(chipid>>32), (uint32_t)chipid);
  
  return schipId;
}


/**************************************************************************************************
* Setup 
**************************************************************************************************/
void setup() {

  Serial.begin(115200);
  delay(10);

  //clearNVS();     // Only if sensor is reporting NVS issues and constantly rebooting

  Serial.println("*** Please ensure you have updated your SSID and PWD in this sketch ***");
  
  Serial.println("Connecting to: " + String(SSID));

  // Connect to provided SSID and PSWD
  WiFi.begin(SSID, PWD);

  // Wait for connection to establish
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("."); // Keep the serial monitor lit!
    delay(500);
  }

  // Connection Succeed
  Serial.println("");
  Serial.println("Connected to: " + String(SSID));

  Serial.println();
  Serial.println("***************************************************************");
  Serial.print("If this is a new sensor, please register with this sensor key: ");
  Serial.println(getThingname());
  Serial.println("Login into the sensor application at https://amsensor/net, and go to menu "); 
  Serial.println("option Location, and click the Add Sensor button below the sensor list");
  Serial.println("***************************************************************");
  Serial.println();

  // Execute OTA Update
  execOTA();

}

void loop() {
  
}

/*
 * Example: Serial Monitor log for this sketch
 * 
 * 

 *** Please ensure you have updated your SSID and PWD in the sketch ***
22:19:45.782 -> Connecting to HOME-NETWORK-2.4
22:19:45.849 -> .
22:19:46.390 -> Connected to HOME-NETWORK-2.4
22:19:46.390 -> 
22:19:46.390 -> ***************************************************************
22:19:46.390 -> If new sensor, register with this sensor key: TTD-F4A6-F1AB6224
22:19:46.390 -> Login into sensor application, and go to menu Location, and
22:19:46.390 -> click Add Sensor button under the sensor list
22:19:46.390 -> ***************************************************************
22:19:46.390 -> 
22:19:46.390 -> OTA Attempt: 
22:19:46.390 -> Connecting to: http://amsensorota.s3-website-us-west-2.amazonaws.com
22:19:46.558 -> Connection succeeded: http://amsensorota.s3-website-us-west-2.amazonaws.com
22:19:46.558 -> Fetching Bin: /amsensor-iot.ino.ttgo-display-v1.bin
22:19:46.763 -> HTTP/1.1 200 OK

22:19:46.763 -> x-amz-id-2: nph3HHJJtKp1eMusD1jHmYjh2iai0hK4n5so1cn02jg5VJoQhkqJ28z72sT8Wgn2dKEE1x7oQSI=

22:19:46.763 -> x-amz-request-id: 8A40027008D091D3

22:19:46.763 -> Date: Sun, 05 Jul 2020 03:19:47 GMT

22:19:46.763 -> Last-Modified: Sun, 05 Jul 2020 03:15:15 GMT

22:19:46.763 -> ETag: "0e1a59cc7554a1a0c1b84fc0b1a83fbe"

22:19:46.763 -> Content-Type: application/octet-stream

22:19:46.763 -> Received application/octet-stream payload.
22:19:46.763 -> Content-Length: 962896

22:19:46.763 -> Received 962896 bytes from server
22:19:46.763 -> Server: AmazonS3

22:19:46.763 -> Connection: close

22:19:46.763 -> 

22:19:46.763 -> contentLength: 962896, isValidContentType : 1
22:19:46.763 -> Starting OTA. This may take 2 - 5 mins to complete. You may not see much feedback as the update is written to the new boot partition. Hang in there!... 
22:20:07.455 -> Boot partitiation updated. Wrote: 962896 bytes successfully
22:20:08.000 -> OTA done!
22:20:08.000 -> Update successfully completed. Rebooting...
22:20:08.042 -> ets Jun  8 2016 00:22:57
22:20:08.042 -> 
22:20:08.042 -> rst:0xc (SW_CPU_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
22:20:08.042 -> configsip: 0, SPIWP:0xee
22:20:08.042 -> clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
22:20:08.042 -> mode:DIO, clock div:1
22:20:08.042 -> load:0x3fff0018,len:4
22:20:08.042 -> load:0x3fff001c,len:1044
22:20:08.042 -> load:0x40078000,len:8896
22:20:08.042 -> load:0x40080400,len:5816
22:20:08.042 -> entry 0x400806ac
22:20:09.397 -> *** Sensor booting! ***
22:20:09.397 -> Initializing TTGO T-Display
22:20:12.528 -> Sensor Key: TTD-F4A6-F1AB6224
22:20:12.631 -> Config read from EEPROM:
22:20:12.631 ->   Status Update interval:1
22:20:12.631 ->   Alarm arm interval:30
22:20:12.631 ->   Alert to Clear cycle count:1
22:20:12.631 ->   Alarm arm status:0
22:20:12.631 ->   Alarm enable/disable:1
22:20:12.631 -> Reading EEPROM into sConfig
22:20:12.631 -> Display EEPROM Contents:
22:20:12.631 ->    EEPROM CFGFLG:42
22:20:12.631 ->    EEPROM CBUTTON1:1
22:20:12.631 ->    EEPROM CBUTTON2:0
22:20:12.631 ->    EEPROM CBUTTON3:0
22:20:12.631 ->    EEPROM CPIR:0
22:20:12.631 ->    EEPROM CTMPHUM:0
22:20:12.631 ->    EEPROM CWATER:1
22:20:12.631 ->    EEPROM CBUZZER:0
22:20:12.631 ->    EEPROM CSIREN:0
22:20:12.631 ->    EEPROM CSTATUSINTV:1
22:20:12.631 ->    EEPROM CALARMINTV:30
22:20:12.631 ->   Alert to Clear cycle count:1
22:20:12.631 ->   Alarm arm status:0
22:20:12.631 -> Attach SSID Connect Button
22:20:12.631 -> Attach Panic Button
22:20:12.631 -> Initializing Relays
22:20:12.631 -> Attempting to connect to SSID: 
22:20:12.903 -> Press Alarm button to create new SSID credentials: 5
22:20:13.957 -> Press Alarm button to create new SSID credentials: 4
22:20:15.008 -> Press Alarm button to create new SSID credentials: 3
22:20:16.089 -> Press Alarm button to create new SSID credentials: 2
22:20:17.142 -> Press Alarm button to create new SSID credentials: 1
22:20:18.300 -> Attempting to connect to WiFi network
22:20:18.300 -> Connected to HOME-DDBB-2.4
22:20:18.300 -> IP address: 10.0.0.220
22:20:18.300 -> WiFi Diag: Mode: STA
22:20:18.300 -> Channel: 6
22:20:18.300 -> SSID (13): HOME-DDBB-2.4
22:20:18.300 -> Passphrase (15): passphrase
22:20:18.300 -> BSSID set: 0
22:20:23.407 -> Attempting MQTT Connect...
22:20:26.915 -> MQTT Connected!
22:20:29.160 -> MQTT RX subscribed: rx/amsensor/hiletgo/TTD-F4A6-F1AB6224
22:20:29.229 -> Attempting NTP connect to read current time
22:20:29.365 -> Buzzer initialized
22:20:29.739 -> Sending [tx/amsensor/prd/v1/TTD-F4A6-F1AB6224]: {"state":{"reported":{"alarmtriggered":1,"clearby":0,"alarmstatus":20,"button":{"button1":false,"button2":false,"button3":false},"pirsensor":{"activated":false,"alarm":false},"watersensor":{"alarm":false},"tempsensor":{"alarm":false},"thsensor":{"temp":20,"rhum":50},"coordinates":{"lat":42.19475,"long":-87.94712},"msgid":1,"sensorid":"TTD-F4A6-F1AB6224","uptime":0,"buildno":"Ver:2020-07-04 ","userid":0,"time":43}}}

    If you see the first "Sending" message, your sensor is not up. If you log into https://amsensor.net
    you still be able to send commands to it and receive the corresponding confirmations in the 
    https://amsensor.net/dashboard or https://amsensor.net/sensorreports pages, as well as on the sensor
    screen.
    
 * 
 */
