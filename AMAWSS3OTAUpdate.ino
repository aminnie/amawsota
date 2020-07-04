/**************************************************************************************************
   AWS S3-based OTA Update Utility
   Date: 07/04/2020
   Purpose: Perform an OTA update from an Amazon S3 bucket (HTTP Only)
**************************************************************************************************/

#include <WiFi.h>
#include <Update.h>

WiFiClient client;

#define HILETGO

// *** Your SSID and PWD that the sensor board needs to connect to ***/
const char* SSID = "xxx";
const char* PWD = "yyy";

// AMSensor S3 Bucket Config
String host = "http://amsensorota.s3-website-us-west-2.amazonaws.com";  // Host => bucket-name.s3.region.amazonaws.com
int port = 80;                                                          // HTTPS = 443. As of today, HTTPS not working

// Test application: Wifi Scanner Demo BIN file download location
//String bin = "/AMWiFiScan.ino.heltec_wifi_kit_32.bin"; // bin file name with a slash in front.
//String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/AMWiFiScan.ino.heltec_wifi_kit_32.bin";

// AMSensor BIN files and download location
#ifdef HILETGO
  char manuf[] = "HL1";
  String bin = "/amsensor-iot.ino.heltec_wifi_kit_32.bin"; // bin file name with a slash in front.
  String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/amsensor-iot.ino.heltec_wifi_kit_32.bin";
#endif
//#ifdef TTGOCAM
//  char manuf[] = "TTC";
//  String bin = "/amsensor-iot.ino.heltec_wifi_kit_32.bin"; // bin file name with a slash in front.
//  String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/amsensor-iot.ino.heltec_wifi_kit_32.bin";
//#endif
//#ifdef TTGODISP
//  char manuf[] = "TTD";
//  String bin = "/amsensor-iot.ino.heltec_wifi_kit_32.bin"; // bin file name with a slash in front.
//  String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/amsensor-iot.ino.heltec_wifi_kit_32.bin";
//#endif
//#ifdef DEVKITC
//  char manuf[] = "ESP";
//  String bin = "/amsensor-iot.ino.heltec_wifi_kit_32.bin"; // bin file name with a slash in front.
//  String bin1 = "http://amsensorota.s3-website-us-west-2.amazonaws.com/amsensor-iot.ino.heltec_wifi_kit_32.bin";
//#endif

// Variables to validate S3 responses
long contentLength = 0;
bool isValidContentType = false;
int retrycnt = 0;

// Utility to extract header value from headers
String getHeaderValue(String header, String headerName) {

  return header.substring(strlen(headerName.c_str()));
}


/**************************************************************************************************
* OTA Logic 
**************************************************************************************************/
void execOTA() {
  Serial.println("OTA Attempt: " + retrycnt);
  
  Serial.println("Connecting to: " + String(host));
  
  // Connect to S3
  if (client.connect(host.c_str(), port)) {
    
    // Connection Succeed.
    Serial.println("Connection succeeded: " + String(host));
    // Fecthing the bin
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
      
      String line = client.readStringUntil('\n');       // Read line till /n

      Serial.println(line);
      
      line.trim();              // Remove space, to check if the line is end of headers

      // If the the line is empty, this is end of headers. 
      // Break the while and feed the remaining `client` data to the Update.writeStream();
      if (!line.length()) {
        break;                  // Headers ended. Get the OTA started
      }

      // Check if the HTTP Response is HHTP 200 else break and Exit Update
      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
          Serial.println("Received status code other than HTTP 200 from server. Exiting OTA Update.");
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
        Serial.println("Received " + contentType + " payload.");
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
      }
    }
  } else {
    // Connect attempt to S3 failed. Consider retry?
    Serial.println("Connection to " + String(host) + " failed. Please check your setup");
    // retry execOTA(); ??
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength: " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // Check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If good, begin OTA
    if (canBegin) {
      Serial.println("Starting OTA. This may take 2 - 5 mins to complete. You may not see much feedback as the update is written to the new boot partition. Hang in there!... ");

      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        Serial.println("Boot partitiation updated. Wrote: " + String(written) + " bytes successfully");
      }
      else {
        Serial.println("Wrote only: " + String(written) + "/" + String(contentLength) + " bytes. Retrying..." );

        while (retrycnt++ < 10) {
          execOTA();
        }
      }

      if (Update.end()) {
        
        Serial.println("OTA done!");
        
        if (Update.isFinished()) {
          Serial.println("Update successfully completed. Rebooting...");
          ESP.restart();
        }
        else {
          Serial.println("Update not finished? Something went wrong!");
        }
      } else {
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
*********************************************************************************/
uint64_t chipid;       // ESP32 Mac Address
char schipId[20];      // Derived THINGNAME e.g. "HL1-1234-C5C2DDBC"

char* getThingname() {

  chipid = ESP.getEfuseMac();   //The chip ID is essentially its MAC address(length: 6 bytes)
  snprintf(schipId, 20, "%s-%04X-%08X", manuf, (uint16_t)(chipid>>32), (uint32_t)chipid);
  
  return schipId;
}


/**************************************************************************************************
* Setup 
**************************************************************************************************/
void setup() {

  Serial.begin(115200);
  delay(10);

  Serial.println("*** Please ensure you have updated your SSID and PWD in the sketch ***");
  
  Serial.println("Connecting to " + String(SSID));

  // Connect to provided SSID and PSWD
  WiFi.begin(SSID, PWD);

  // Wait for connection to establish
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("."); // Keep the serial monitor lit!
    delay(500);
  }

  // Connection Succeed
  Serial.println("");
  Serial.println("Connected to " + String(SSID));

  Serial.println();
  Serial.println("***************************************************************");
  Serial.print("If new sensor, register with this sensor key: ");
  Serial.println(getThingname());
  Serial.println("Login into sensor application, and go to menu Location, and"); 
  Serial.println("click Add Sensor button under the sensor list");
  Serial.println("***************************************************************");
  Serial.println();

  // Execute OTA Update
  execOTA();

}

void loop() {
  
}

/*
 * Serial Monitor log for this sketch
 * 
 * If the OTA succeeded, it would load the preference sketch, with a small modification. i.e.
 * Print `OTA Update succeeded!! This is an example sketch : Preferences > StartCounter`
 * And then keeps on restarting every 10 seconds, updating the preferences
 * 
 * 
      rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
      configsip: 0, SPIWP:0x00
      clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
      mode:DIO, clock div:1
      load:0x3fff0008,len:8
      load:0x3fff0010,len:160
      load:0x40078000,len:10632
      load:0x40080000,len:252
      entry 0x40080034
      Connecting to SSID
      ......
      Connected to SSID
      Connecting to: bucket-name.s3.ap-south-1.amazonaws.com
      Fetching Bin: /StartCounter.ino.bin
      Got application/octet-stream payload.
      Got 357280 bytes from server
      contentLength : 357280, isValidContentType : 1
      Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!
      Written : 357280 successfully
      OTA done!
      Update successfully completed. Rebooting.
      ets Jun  8 2016 00:22:57
      
      rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
      configsip: 0, SPIWP:0x00
      clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
      mode:DIO, clock div:1
      load:0x3fff0008,len:8
      load:0x3fff0010,len:160
      load:0x40078000,len:10632
      load:0x40080000,len:252
      entry 0x40080034
      
      OTA Update succeeded!! This is an example sketch : Preferences > StartCounter
      Current counter value: 1
      Restarting in 10 seconds...
      E (102534) wifi: esp_wifi_stop 802 wifi is not init
      ets Jun  8 2016 00:22:57
      
      rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
      configsip: 0, SPIWP:0x00
      clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
      mode:DIO, clock div:1
      load:0x3fff0008,len:8
      load:0x3fff0010,len:160
      load:0x40078000,len:10632
      load:0x40080000,len:252
      entry 0x40080034
      
      OTA Update succeeded!! This is an example sketch : Preferences > StartCounter
      Current counter value: 2
      Restarting in 10 seconds...

      ....
 * 
 */
