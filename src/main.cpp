/*
 *   Copyright (c) 2021 Benoni Jiang
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include "Version.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>

#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

#include <ESP8266mDNS.h>
// #include <ESP8266WebServer.h>

// #include <ESP8266HTTPUpdateServer.h>

#include <ArduinoOTA.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#include <EEPROM.h>

#include <Wire.h>
//#include <ArduinoJson.h>

#include "SimpleButton.h"

using namespace simplebutton;

#include "multilog.h"

#include "jled.h"

#include <LittleFS.h>
#include <ESPAsyncWiFiManager.h>       //https://github.com/tzapu/WiFiManager

//for LED status
#include <Ticker.h>
Ticker ticker;

// #include <IotWebConf.h>
// #include <IotWebConfTParameter.h>

#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <DNSServer.h>

void tick()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (AsyncWiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

// #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
#define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
// #define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE            // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT           // Support for overall task timeout
// #define _TASK_OO_CALLBACKS      // Support for dynamic callback method binding
#include <TaskScheduler.h>

#ifndef STASSID
#define STASSID "LvShun"
#define STAPSK  "lvshun123"
#endif

//StaticJsonBuffer<200> jsonBuffer;
//
//JsonObject& root = jsonBuffer.createObject();
//JsonObject&
const String ret1 = ", \"door\":1, \"controlPin\":2, \"statusPin\":1, \"action\":";

//const String ret2 =

const int relay_pin = 4;
const int status_pin = 5;

WiFiUDP ntpUDP;

const long utcOffsetInSeconds = 0;
const unsigned long utcupdateInterval = 60000;

NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds, utcupdateInterval);

#define Version "V2.3.2"

ESP8266WiFiMulti WiFiMulti;

// ESP8266WebServer server(80);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
DNSServer dns;

// ESP8266HTTPUpdateServer httpUpdater;

const char* username = "login";
const char* password = "Passw0rd";

#ifdef _DEBUG_
#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);
#else
#define _PP(a)
#define _PL(a)
#endif

// Scheduler
Scheduler ts;

/*
   Approach 1: LED is driven by the boolean variable; false = OFF, true = ON
*/
#define PERIOD1 500
#define DURATION 10000
void blink1CB();
Task tBlink1 ( PERIOD1 * TASK_MILLISECOND, DURATION / PERIOD1, &blink1CB, &ts, true );
#define PERIOD2 200
#define DURATION2 5000
void blinkwififails();
Task tBlink2 ( PERIOD2 * TASK_MILLISECOND, DURATION2 / PERIOD2, &blinkwififails, &ts, false );


// This example is now configured to use the automated signing support
// present in the Arduino IDE by having a "private.key" and "public.key"
// in the sketch folder.  You can also programmatically enable signing
// using the method shown here.

// This key is taken from the server public certificate in BearSSL examples
// You should make your own private/public key pair and guard the private
// key (never upload it to the 8266).

void upgrade(String host, String port);
void handleLED();
void sendresponder(int code, String xx, String action, String status, String message);

ButtonPullup* b = NULL;

// auto led_breathe = JLed(relay_pin).Breathe(1500).Repeat(6).DelayAfter(500);
auto led_breathe = JLed(relay_pin).Blink(200, 200).Forever();

MultiLog xMultiLog(IPAddress(192,168,0,255), 34200, &Serial);

FS& gfs = LittleFS;
//FS& gfs = SDFS;

void handleRoot();

void configSaved();

// bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

static const char chooserValues[] [128] = {"tcp", "http", "mqtt", "websockets", "modbus"};
static const char chooserNames[] [128] = {"TCP", "HTTP",  "MQTT", "WEBSOCKETS", "MODBUS TCP"};

// IotWebConf iotWebConf("test", &)

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //  digitalWrite(status_pin, HIGH);
  pinMode(relay_pin, OUTPUT);
  //  digitalWrite(LED_BUILTIN,HIGH);
  // b = new Button(status_pin, true);
  pinMode(status_pin, INPUT_PULLUP);
  if (digitalRead(status_pin) == LOW) {
    ticker.attach(0.05, tick);
    for (int i = 0; i <10 ;i++) {
      delay(1000);
      if( i > 9 ) {
        ticker.detach();
        // wifiManager.resetSettings();
        delay(3000);
        // ESP.reset();
      }
    }
  }

  b = new ButtonPullup(status_pin);
  b->setOnDoubleClicked([] () {
    // Serial.println("DoubleClick");
    xMultiLog.println("DoubleClick");

  }, 100, 1000);

  b->setOnClicked([] () {
    xMultiLog.println("Click");
  }, 500);

  b->setOnHolding([] () {
    xMultiLog.println("Holding");
  }, 3000);
  if (digitalRead(relay_pin) == 1) {
          digitalWrite(relay_pin, LOW);
          }

  Serial.begin(115200);
  // Serial.setDebugOutput(true);

  Serial.println("Project version: " + String(VERSION));
  Serial.println("Build timestamp:" + String(BUILD_TIMESTAMP));
  Serial.print("firmware version : ");
  Serial.println(Version);

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  //set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  //WiFiManager
  AsyncWiFiManager wifiManager(&server, &dns);
  //Local intialization. Once its business is done, there is no need to keep it around
  //reset settings - for testing
  // wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  ticker.detach();
  //keep LED on
  digitalWrite(BUILTIN_LED, LOW);

  // WiFi.mode(WIFI_STA);
  // WiFiMulti.addAP(STASSID, STAPSK);
  // WiFiMulti.addAP("AGV-roaming", "roam5678");
  // while (WiFi.status() != WL_CONNECTED) {
  //   Serial.print(".");
  //   delay(500);
  //   WiFiMulti.run();
  // }
  

  //delay(3000);
  // httpUpdater.setup(&server);
  // upgrade("192.168.0.142", "8080");
  timeClient.begin();
  timeClient.update();
  if (MDNS.begin("esp8266")) {              // Start the mDNS responder for esp8266.local
    Serial.println("mDNS responder started");
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("Error setting up MDNS responder!");
  }
  if ( !gfs.begin() ) {
    xMultiLog.println("Fs mount Fails");
  } else {
    xMultiLog.println("FS mount OK");
  }
  xMultiLog.println(gfs.exists("/img"));

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(gfs, "/index.html", "text/html", false); });

  server.serveStatic("/", LittleFS, "/");

  // server.on("/door.lc", HTTP_POST, handleLED);
  // server.on("/door.lc", HTTP_GET, handleLED);
  // Start ElegantOTA
  AsyncElegantOTA.begin(&server);
  server.begin();

  Serial.print("Open http://");
  Serial.print(WiFi.localIP());
  Serial.println("/ in your browser to see it working");

  // ArduinoOTA
  //   .onStart([]() {
  //     String type;
  //     if (ArduinoOTA.getCommand() == U_FLASH)
  //       type = "sketch";
  //     else // U_SPIFFS
  //       type = "filesystem";

  //     // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  //     Serial.println("Start updating " + type);
  //   })
  //   .onEnd([]() {
  //     Serial.println("\nEnd");
  //   })
  //   .onProgress([](unsigned int progress, unsigned int total) {
  //     Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  //   })
  //   .onError([](ota_error_t error) {
  //     Serial.printf("Error[%u]: ", error);
  //     if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  //     else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  //     else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  //     else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  //     else if (error == OTA_END_ERROR) Serial.println("End Failed");
  //   });
  // tcp.begin();
  // dav.begin(&server, &gfs);
  ArduinoOTA.begin();
}


void loop() {
  // wait for WiFi connection
  
  // MDNS.update();
  // server.handleClient();
  AsyncElegantOTA.loop();
  ArduinoOTA.handle();
  // ts.execute();
  b->update();
  led_breathe.Update();
  // dav.handleClient();

//  Serial.print(timeClient.getHours());
//  Serial.print(":");
//  Serial.print(timeClient.getMinutes());
//  Serial.print(":");
//  Serial.println(timeClient.getSeconds());
  
//  delay(1000);
}

inline void LEDOn() {
  digitalWrite( LED_BUILTIN, HIGH );
}

inline void LEDOff() {
  digitalWrite( LED_BUILTIN, LOW );
}

// === 1 =======================================
bool LED_state = false;
void blink1CB() {
  if ( tBlink1.isFirstIteration() ) {
    _PP(millis());
    _PL(": Blink1 - simple flag driven");
    LED_state = false;
  }

  if ( LED_state ) {
    LEDOff();
    LED_state = false;
  }
  else {
    LEDOn();
    LED_state = true;
  }

  if ( tBlink1.isLastIteration() ) {
    // tBlink2.restartDelayed( 2 * TASK_SECOND );
    LEDOff();
  }
}

inline void relay_on() {
  digitalWrite( relay_pin,1);
}

inline void relay_off() {
  digitalWrite(relay_pin, LOW);
}

bool relay_state;
void blinkwififails() {
  if (relay_state) {
    relay_off();
    relay_state = false;
  } else {
    relay_on();
    relay_state = true;
  }
}

// void handleLED() {                          // If a POST request is made to URI /LED
//   if (!server.authenticate(username, password)) {
//       return server.requestAuthentication();
//     }
//   //digitalWrite(led,!digitalRead(led));      // Change the state of the LED
// //  server.sendHeader("Location","/");        // Add a header to respond with a new location for the browser to go to the home page again
// //  server.send(303);                         // Send it back to the browser with an HTTP status 303 (See Other) to redirect
//   Serial.println(server.uri());
//   Serial.println(server.arg(1));
//   if (server.argName(0) == "action") {
//     String arg = server.arg(0);
//     if (arg == "open") {
//       if (0) {
//         sendresponder(400, "-3", "open", "open", "Door is alread open");
//         }
//       else {
//       digitalWrite(relay_pin,HIGH);
//       sendresponder(200, "0", "open", "opening", "Door opened");
//       }
//       }
//     else if (arg == "close") {
//       if (0) {
// //        digitalWrite(LED_BUILTIN,HIGH);
//         sendresponder(400, "-4", "close", "closed", "Door is already closed");
        
//         }
//         else {
//           digitalWrite(relay_pin,LOW);
//           sendresponder(200, "0", "close", "closing", "Door Closed");
//           }    
//       }
//     else if (arg == "status") {
//         if (digitalRead(relay_pin) == 1) {
//           sendresponder(200, "0", "status", "opened", "");
//           }
//          else {
//           sendresponder(200, "0", "status", "closed", "");
//           }
//       }
//     else if (arg == "reboot") {
//         sendresponder(200, "0", "reboot", "none", "device will reboot");
//         if (digitalRead(relay_pin) == 1) {
//           digitalWrite(relay_pin, LOW);
//           }
//         delay(1000);
//         ESP.restart();
//       }
//     else if (arg == "upgrade") {
//       String host;
//         if (server.args() >1 && server.argName(1) == "host") {
          
// //        sendresponder(200, "0", "upgrade", "none", "device will upgrade, dont shutdown power!");
// //        delay(1000);
//         host = server.arg(1);
//         upgrade(host, "8080");
//         }
//         else if (server.args() >2 && server.argName(2) == "port") {
//           upgrade(host, server.arg(2));
//           }
//         else {
//           upgrade("192.168.0.142", "8080");
//           }
//       }
//      else {
      
//       }
//     }
// }
// void sendresponder(int code, String error, String action, String doorstatus, String mes) {
//   String ret = "{\"error\":"+error;
//   ret += ", \"door\":1, \"controlPin\":2, \"statusPin\":1, \"action\":\"";
//   ret += action;
//   if (mes != "") {
//     ret += "\", \"message\":\"";
//     ret += mes;
//     }
//   ret += "\", \"status\":\"";
//   ret += doorstatus;
//   ret += "\", \"relay\":\"0\", \"hold time\":\"forever\", \"device id\":\"";
//   ret += ESP.getChipId();
//   ret = ret+"\", \"firmware version\": \""+Version;
//   ret += "\", \"UTC Time\":\"";
//   ret = ret+timeClient.getHours()+":"+timeClient.getMinutes()+":"+timeClient.getSeconds();
//   ret += "\"}";
  
//    server.send(code, "application/json", ret);
//  }

 void upgrade(String host, String port) {
 if ((WiFiMulti.run() == WL_CONNECTED)) {
    String url = "http://"+host+":"+port+"/esp8266.bin";
    String mes = "device will upgrade from "+url+",dot shutdown power!";
    sendresponder(200, "0", "upgrade", "none", mes);

    WiFiClient client;

    #if MANUAL_SIGNING
    // Ensure all updates are signed appropriately.  W/o this call, all will be accepted.
    Update.installSignature(hash, sign);
    #endif
    // If the key files are present in the build directory, signing will be
    // enabled using them automatically

    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);

    

    t_httpUpdate_return ret = ESPhttpUpdate.update(client, url, Version);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
    client.stop();
  } 
  
 }