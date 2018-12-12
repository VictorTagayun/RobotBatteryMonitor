/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on ESP8266 chip.

  Note: This requires ESP8266 support package:
    https://github.com/esp8266/Arduino

  Please be sure to select the right ESP8266 module
  in the Tools -> Board menu!

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <BlynkSimpleEsp8266.h>
#include <ArduinoOTA.h>

ESP8266WiFiMulti WiFiMulti;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "215df2de13a54c86b54cbb0fcbef1730";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "SingaporePolice";
char pass[] = "123Qweasd";

void setup()
{
  // We start by connecting to a WiFi network
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("SingaporeMilitia", "123Qweasd");
  WiFiMulti.addAP("SingaporeMilitia_plus", "123Qweasd");
  WiFiMulti.addAP("SingaporePolice", "123Qweasd");
  WiFiMulti.addAP("AntipoloPolice", "TAGAYUNFAMILY");
  WiFiMulti.addAP("VicenteTagayun", "27Author");

  // Debug console
  Serial.begin(9600);

  OTA();

  Blynk.config(auth);
}

void loop()
{
  Blynk.run();
  ArduinoOTA.handle();
}

void OTA()
{
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

int sensorValueInt = 0;        // value read from the pot
float sensorValueFloat = 0;

void ReadBattLevel() {
  // read the analog in value:
  sensorValueInt = analogRead(A0);
  sensorValueFloat = (float(sensorValueInt) / 1023) / 10 * (10 + 22 + 20);
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  //analogWrite(analogOutPin, outputValue);

  // print the results to the Serial Monitor:
  Serial.print("sensorValueInt = ");
  Serial.println(sensorValueInt, DEC);
  Serial.print("sensorValueFloat = ");
  Serial.println(sensorValueFloat);
  //Serial.print("\t output = ");
  //Serial.println(outputValue);

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(200);
}
