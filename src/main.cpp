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
#include <ArduinoJson.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#include <EEPROM.h>

#include <Wire.h>
//#include <ArduinoJson.h>

#include "SimpleButton.h"

using namespace simplebutton;

#include "multilog.h"

#include "jled.h"

#include "FastLED.h"
#define DATA_PIN 12
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

#include <LittleFS.h>
#include <ESPAsyncWiFiManager.h>       //https://github.com/tzapu/WiFiManager
#include <asyncHTTPrequest.h>

//for LED status
#include <Ticker.h>
Ticker ticker;

// #include <IotWebConf.h>
// #include <IotWebConfTParameter.h>

#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <DNSServer.h>

// #include <IotWebConf.h>

void tick()
{
  //toggle state
  int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
}

char http_server[40] = "192.168.0.89";
char http_port[6] = "8081";
char device_number[6] = "1";

bool shouldSaveConfig = false;

MultiLog xMultiLog(IPAddress(192,168,0,255), 34200, &Serial);

//gets called when WiFiManager enters configuration mode
void configModeCallback (AsyncWiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void saveConfigCallback() {
  xMultiLog.println("Should save config");
  shouldSaveConfig = true;
}

asyncHTTPrequest request;

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
const int button1_pin = 14;
const int button2_pin = 12;
const int button3_pin = 13;
bool button3_status = false;

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
#define DURATION 100000
void blink1CB();
Task tBlink1 ( PERIOD1 * TASK_MILLISECOND, DURATION / PERIOD1, &blink1CB, &ts, true );
#define PERIOD2 200
#define DURATION2 5000
void blinkwififails();
Task tBlink2 ( PERIOD2 * TASK_MILLISECOND, DURATION2 / PERIOD2, &blinkwififails, &ts, false );


#include <WS2812FX.h>

#define LED_COUNT 1
#define LED_PIN 15

WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

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

ButtonPullup* sensor = NULL;
ButtonPullup* b = NULL;
ButtonPullup* b2 = NULL;
ButtonPullup* b3 = NULL;
// ButtonPullup* b4 = NULL;

// auto led_breathe = JLed(relay_pin).Breathe(1500).Repeat(6).DelayAfter(500);
auto led_breathe = JLed(relay_pin).Blink(200, 200).Forever();


FS& gfs = LittleFS;
//FS& gfs = SDFS;

void handleRoot();

void configSaved();

// bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

static const char chooserValues[] [128] = {"tcp", "http", "mqtt", "websockets", "modbus"};
static const char chooserNames[] [128] = {"TCP", "HTTP",  "MQTT", "WEBSOCKETS", "MODBUS TCP"};

// IotWebConf iotWebConf("test", &)
void sendRequest(String button, String action);
void requestCB(void* optParm, asyncHTTPrequest* request, int readyState);

#include <WebSocketsClient.h>

#include <Hash.h>

WebSocketsClient webSocket;

#define USE_SERIAL xMultiLog

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

	switch(type) {
		case WStype_DISCONNECTED:
			USE_SERIAL.printf("[WSc] Disconnected!\n");
			break;
		case WStype_CONNECTED: {
			USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

			// send message to server when Connected
			webSocket.sendTXT("Connected");
		}
			break;
		case WStype_TEXT:
			USE_SERIAL.printf("[WSc] get text: %s\n", payload);

			// send message to server
			// webSocket.sendTXT("message here");
			break;
		case WStype_BIN:
			USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
			hexdump(payload, length);

			// send data to server
			// webSocket.sendBIN(payload, length);
			break;
        case WStype_PING:
            // pong will be send automatically
            USE_SERIAL.printf("[WSc] get ping\n");
            break;
        case WStype_PONG:
            // answer to a ping we send
            USE_SERIAL.printf("[WSc] get pong\n");
            break;
    }

}

int carry_times = 1;
bool carry_start = false;

// mode number

int carry_mode = 0;

void sendWebSocket(String name, String action) {
  int device_num = 0;
  if (name == "sensor" and carry_start == true) {
    if (carry_mode % 4 == 1) {
      carry_times += 1;
      if (carry_times % 2 == 0) {
        device_num = 1;
      } else {
        device_num = 2;
      }
    } else if (carry_mode % 4 == 2) {
      device_num = 2;
    } else if (carry_mode % 4 == 3) {
      device_num = 3;
    }
  }
  String txt = String("device_number: ") + device_num + " button name: " + name + " action: " + action;
  webSocket.sendTXT(txt.c_str());
}


void setMode(int a = 0) {
  if (button3_status == false and a != 0) {
    carry_mode += 1;
    carry_times = 1;
  } else if (button3_status == true) {
    carry_start = true;
    ws2812fx.setColor(GREEN);
  }
  if (button3_status == false) {
    ws2812fx.setMode(FX_MODE_STATIC);
    if (carry_mode % 4 == 1) {
      ws2812fx.setColor(PINK);
    } else if(carry_mode % 4 == 2) {
      ws2812fx.setColor(YELLOW);
    } else if (carry_mode % 4 == 3) {
      ws2812fx.setColor(PURPLE);
    } else {
      ws2812fx.setColor(ORANGE);
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //  digitalWrite(status_pin, HIGH);
  pinMode(relay_pin, OUTPUT);
  //  digitalWrite(LED_BUILTIN,HIGH);
  // b = new Button(status_pin, true);
  pinMode(status_pin, INPUT_PULLUP);
  pinMode(button1_pin, INPUT_PULLUP);
  pinMode(button2_pin, INPUT_PULLUP);
  pinMode(button3_pin, INPUT_PULLUP);
  // pinMode
  // if (digitalRead(status_pin) == LOW) {
  //   ticker.attach(0.05, tick);
  //   for (int i = 0; i <10 ;i++) {
  //     delay(1000);
  //     if( i > 9 ) {
  //       ticker.detach();
  //       // wifiManager.resetSettings();
  //       delay(3000);
  //       // ESP.reset();
  //     }
  //   }
  // }

  b = new ButtonPullup(button1_pin);
  // b->setOnDoubleClicked([] () {
  //   // Serial.println("DoubleClick");
  //   xMultiLog.println("Button1 DoubleClick");

  // }, 100, 1000);

  b->setOnClicked([] () {
    if (button3_status == true) {
      carry_start = false;
      ws2812fx.setColor(ORANGE);
    }
    sendWebSocket("stop", "click");
    xMultiLog.println("Button1 Click");
  }, 50);

  b->setOnHolding([] () {
    sendWebSocket("stop", "holding");
    xMultiLog.println("Button1 Holding");
  }, 3000);
  sensor = new ButtonPullup(status_pin);
  // b->setOnDoubleClicked([] () {
  //   // Serial.println("DoubleClick");
  //   xMultiLog.println("Button1 DoubleClick");

  // }, 100, 1000);

  sensor->setOnClicked([] () {
    // sendRequest("sensor", "click");
    if (button3_status == true) {
      sendWebSocket("sensor", "click");
      xMultiLog.println("Sonsor Click");
    }
    
  }, 50);

  sensor->setOnHolding([] () {
    // sendRequest("sensor", "hold");
    if (button3_status == true) {
      sendWebSocket("sensor", "holding");
      xMultiLog.println("Sonsor Holding");
    }
  }, 3000);
  b2 = new ButtonPullup(button2_pin);
  // b2->setOnDoubleClicked([] () {
  //   // Serial.println("2DoubleClick");
  //   xMultiLog.println("Button2 DoubleClick");

  // }, 100, 1000);

  b2->setOnClicked([] () {
    setMode(1);
    sendWebSocket("start", "click");
    xMultiLog.println("Button2 Click");
  }, 50);

  b2->setOnHolding([] () {
    sendWebSocket("start", "holding");
    xMultiLog.println("Button2 Holding");
  }, 3000);
  b3 = new ButtonPullup(button3_pin);
  // b2->setOnDoubleClicked([] () {
  //   // Serial.println("2DoubleClick");
  //   xMultiLog.println("Button2 DoubleClick");

  // }, 100, 1000);

  b3->setOnClicked([] () {
    sendWebSocket("mode", "click");
    xMultiLog.println("Button3 Click");
  }, 50);

  b3->setOnHolding([] () {
    sendWebSocket("mode", "holding");
    xMultiLog.println("Button3 Holding");
  }, 3000);

  b3->setOnPushed([] () {
    button3_status = true;
    ws2812fx.setColor(BLUE);
    ws2812fx.setMode(FX_MODE_STATIC);
  });
  b3->setOnReleased([] () {
    button3_status = false;
    setMode();
    // ws2812fx.setColor(ORANGE);
    // ws2812fx.setMode(FX_MODE_STATIC);
  });

  

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
  pinMode(LED_BUILTIN, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  if (gfs.begin()) {
    if (gfs.exists("config.json")) {
      File configFile = gfs.open("config.json", "r");
      if (configFile) {
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        // doc.
        DeserializationError error = deserializeJson(json, buf.get());
        // json.printTo(Serial);
        if (!error) {
          strcpy(http_server, json["http_server"]);
          strcpy(http_port, json["http_port"]);
          strcpy(device_number, json["device_number"]);
        } else
        {
          Serial.println("Failed to parse json config");
        }
        
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  AsyncWiFiManagerParameter custom_http_server("server", "http server", http_server, 40);
  AsyncWiFiManagerParameter custom_http_port("port","http port", http_port, 5);
  AsyncWiFiManagerParameter custom_device_number("device number", "device number", device_number, 5);
  //WiFiManager
  AsyncWiFiManager wifiManager(&server, &dns);
  //Local intialization. Once its business is done, there is no need to keep it around
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  
  // reset settings - for testing
  // wifiManager.resetSettings();
  
  if (digitalRead(button1_pin) == 0 && digitalRead(button2_pin == 0)) {
          //reset settings - for testing
    wifiManager.resetSettings();
  }
  wifiManager.addParameter(&custom_http_server);
  wifiManager.addParameter(&custom_http_port);
  wifiManager.addParameter(&custom_device_number);

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
  digitalWrite(LED_BUILTIN, LOW);

  strcpy(http_server, custom_http_server.getValue());
  strcpy(http_port, custom_http_port.getValue());
  strcpy(device_number, custom_device_number.getValue());

  if (shouldSaveConfig) {
    xMultiLog.println("save config");
    DynamicJsonDocument doc(1024);
    // DeserializationError error = deserializeJson(doc, json);
    doc["http_server"] = http_server;
    doc["http_port"] = http_port;
    doc["device_number"] = device_number;

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
      xMultiLog.println("failed to open config file for writing");
    }
    serializeJson(doc, xMultiLog);
    serializeJson(doc, configFile);
    configFile.close();
  }

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

  // request.setDebug(true);
  // request.onReadyStateChange(requestCB);
  // if ( !gfs.begin() ) {
  //   xMultiLog.println("Fs mount Fails");
  // } else {
  //   xMultiLog.println("FS mount OK");
  // }
  // xMultiLog.println(gfs.exists("/img"));

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

  // server address, port and URL
	webSocket.begin(http_server, 8081, "/call_client");

	// event handler
	webSocket.onEvent(webSocketEvent);

	// use HTTP Basic Authorization this is optional remove if not needed
	// webSocket.setAuthorization("user", "Password");

	// try ever 5000 again if connection has failed
	webSocket.setReconnectInterval(5000);
  
  // start heartbeat (optional)
  // ping server every 15000 ms
  // expect pong from server within 3000 ms
  // consider connection disconnected if pong is not received 2 times
  webSocket.enableHeartbeat(15000, 3000, 2);

  ArduinoOTA.begin();

  // FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

  ws2812fx.init();
  ws2812fx.setBrightness(100);
  ws2812fx.setSpeed(1000);
  ws2812fx.setMode(FX_MODE_BREATH);
  ws2812fx.start();
}


void loop() {
  // wait for WiFi connection
  
  // MDNS.update();
  // server.handleClient();
  AsyncElegantOTA.loop();
  ArduinoOTA.handle();
  // ts.execute();
  b->update();
  b2->update();
  b3->update();
  sensor->update();
  ws2812fx.service();
  webSocket.loop();
  // led_breathe.Update();
  // dav.handleClient();

//  Serial.print(timeClient.getHours());
//  Serial.print(":");
//  Serial.print(timeClient.getMinutes());
//  Serial.print(":");
//  Serial.println(timeClient.getSeconds());
  
//  delay(1000);
}

void sendRequest(String button, String action)
{ 
  static bool requestOpenResult = false;

  if (request.readyState() == 0  || request.readyState() == 4) {
    // String url = String.format("http://%s:%d/api/v1/call_client?button=%s&action=%s", )
    String url = String("http://") + http_server + ":" + http_port + "/api/v1/call_client?device_number=" + device_number + "&button=" + button + "&action=" + action;
    requestOpenResult = request.open("GET", url.c_str());
    if (requestOpenResult) {
      request.send();
    } else {
      xMultiLog.println("Can't send bad request");
    }
  }
}

void requestCB(void* optParm, asyncHTTPrequest* request, int readyState) 
{
  // (void) optParm;
  
  if (readyState == 4) 
  {
    xMultiLog.println("\n**************************************");
    xMultiLog.println(request->responseText());
    xMultiLog.println("**************************************");
    
    // request->setDebug(false);
  }
}

inline void LEDOn() {
  // digitalWrite( LED_BUILTIN, HIGH );
  // leds[0] = CRGB::Red;
  leds[0] = CRGB::Blue;
  FastLED.show();
}

inline void LEDOff() {
  // digitalWrite( LED_BUILTIN, LOW );
  leds[0] = CRGB::Black;
  FastLED.show();
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