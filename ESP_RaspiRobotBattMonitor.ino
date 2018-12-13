/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <BlynkSimpleEsp8266.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include "ThingSpeak.h"

ESP8266WiFiMulti WiFiMulti;

int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;
#define SECRET_CH_ID 650731     // replace 0000000 with your channel number
#define SECRET_WRITE_APIKEY "VYBJ5FSSJEV343N8"   // replace XYZ with your channel write API Key

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

char auth[] = "215df2de13a54c86b54cbb0fcbef1730";

int ETA1096_Enable_pin = 2;
int CW2015_Alert_pin = 15;
int VinCharging5V_pin = 13;
int VoutCharger5V_pin = 12;
int IP5305_Key_pin = 14;

bool ETA1096_Enable;
bool CW2015_Enable;
bool VinCharging5V;
bool VoutCharger5V;
bool IP5305_Key;

float Batt_Voltage_ADC, Batt_Voltage_I2C, Batt_Percent_I2C;
int ChargingInput, LobattAlert;

BlynkTimer timer1, timer2;

void setup()
{
  Wire.begin();

  // Debug console
  Serial.begin(9600);

  //   We start by connecting to a WiFi network
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("SingaporeMilitia", "123Qweasd");
  WiFiMulti.addAP("SingaporeMilitia_plus", "123Qweasd");
  WiFiMulti.addAP("SingaporePolice", "123Qweasd");
  WiFiMulti.addAP("AntipoloPolice", "TAGAYUNFAMILY");
  WiFiMulti.addAP("VicenteTagayun", "27Author");
  Serial.println();
  Serial.println();
  Serial.print("Wait for WiFi... ");
  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    // need to add cntr timeout
  }
  Serial.println("");
  Serial.print("WiFi connected to : ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Blynk.config(auth);
  // Blynk status
  // Init the the buttons

  ThingSpeak.begin(client);
  // ThingSpeak status

  pinMode(ETA1096_Enable_pin, OUTPUT);
  pinMode(CW2015_Alert_pin, INPUT);
  pinMode(VinCharging5V_pin, INPUT);
  pinMode(VoutCharger5V_pin, INPUT);
  pinMode(IP5305_Key_pin, OUTPUT);

  digitalWrite(ETA1096_Enable_pin, HIGH);
  digitalWrite(IP5305_Key_pin, HIGH);

  //  Blynk.begin(auth, ssid, pass);

  //  OTA();

  // Batt monitor reset
  write_mode(0x0);

  timer1.setInterval(20000L, ThingSpeakDatalog);
  timer2.setInterval(1000L, Datalog_1S);

  //delay(20000);
}

void loop()
{
  Blynk.run();
  timer1.run(); // Initiates BlynkTimer
  timer2.run(); // Initiates BlynkTimer

  //  ArduinoOTA.handle();
}

float ReadAnalogBattLevel()
{
  int BattValueInt = 0;
  float BattValueFloat = 0;

  BattValueInt = analogRead(A0);
  BattValueFloat = (float(BattValueInt) / 1023) / 10 * (10 + 22 + 20);

  return BattValueFloat;
}

float read_battery_voltage()
{
  float battery_voltage;
  char batt_lsb, batt_msb;
  int result;
  bool error_read_battery_voltage = false;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x02));
  Wire.endTransmission(false);
  result = Wire.requestFrom(0x62, 1, true);
  if (result != 1) {
    Serial.print("batt_msb nok : ");
    Serial.println(result, HEX);
  }
  batt_msb = Wire.read(); // receive a byte as character

  battery_voltage = batt_msb << 8;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x03));
  Wire.endTransmission(false);
  result = Wire.requestFrom(0x62, 1, true);
  if (result != 1) {
    Serial.print("batt_lsb nok : ");
    Serial.println(result, HEX);
  }
  batt_lsb = Wire.read(); // receive a byte as character

  battery_voltage += batt_lsb;
  battery_voltage = battery_voltage * 305 / 1000000;

  return battery_voltage;
}

float read_battery_percentage()
{
  float battery_percentage;
  char percent_lsb, percent_msb;
  int result;
  bool error_read_battery_percentage = false;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x04));
  Wire.endTransmission(false);
  result = Wire.requestFrom(0x62, 1, true);
  if (result != 1) {
    Serial.print("percent_msb nok : ");
    Serial.println(result, HEX);
  }
  percent_msb = Wire.read(); // receive a byte as character

  battery_percentage = percent_msb;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x05));
  Wire.endTransmission(false);
  result = Wire.requestFrom(0x62, 1, true);
  if (result != 1) {
    Serial.print("percent_lsb nok : ");
    Serial.println(result, HEX);
  }
  percent_lsb = Wire.read(); // receive a byte as character

  battery_percentage += (float(percent_lsb) / 256);

  return battery_percentage;
}

char read_version()
{
  Wire.beginTransmission(0x62);
  Wire.write(byte(0x00));
  Wire.endTransmission(false);
  int result = Wire.requestFrom(0x62, 1, true);
  if (result != 1) {
    Serial.print("read_mode nok : ");
    Serial.println(result, HEX);
  }

  char ic_version = Wire.read(); // receive a byte as character

  return ic_version;
}

char read_mode()
{
  Wire.beginTransmission(0x62);
  Wire.write(byte(0x0a));
  Wire.endTransmission(false);
  int result = Wire.requestFrom(0x62, 1, true);
  if (result != 1) {
    Serial.print("read_mode nok : ");
    Serial.println(result, HEX);
  }
  char ic_mode = Wire.read(); // receive a byte as character
  return ic_mode;
}

void write_mode(char ic_mode)
{
  Wire.beginTransmission(0x62);
  Wire.write(byte(0x0a));
  Wire.write(ic_mode);
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("write_mode nok : ");
    Serial.println(result, HEX);
  }
}
void ThingSpeakDatalog()
{
  // set the fields with the values
  ThingSpeak.setField(1, Batt_Voltage_ADC);
  ThingSpeak.setField(2, Batt_Voltage_I2C);
  ThingSpeak.setField(3, Batt_Percent_I2C);
  ThingSpeak.setField(4, ChargingInput);
  ThingSpeak.setField(5, LobattAlert);
  ThingSpeak.setField(6, VoutCharger5V);

  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial.println("ThingSpeakDtalog update successful.");
  }
  else {
    Serial.println("Problem updating ThingSpeakDtalog. HTTP error code " + String(x));
  }
}

void Datalog_1S()
{
  Batt_Voltage_ADC = ReadAnalogBattLevel();
  Batt_Voltage_I2C = read_battery_voltage();
  Batt_Percent_I2C = read_battery_percentage();
  ChargingInput = digitalRead(VinCharging5V_pin);
  LobattAlert = digitalRead(CW2015_Alert_pin);
  VoutCharger5V = digitalRead(VoutCharger5V_pin);


  Serial.print("Battery Analog Voltage = ");
  Serial.println(Batt_Voltage_ADC);
  Serial.print("Battery Voltage : ");
  Serial.println(Batt_Voltage_I2C);
  Serial.print("Battery Percentage : ");
  Serial.println(Batt_Percent_I2C);

  if (ChargingInput)
  {
    Serial.println("VinCharging5V_pin high");
  } else
  {
    Serial.println("VinCharging5V_pin low");
  }

  if (LobattAlert)
  {
    Serial.println("CW2015_Alert_pin high");
  } else
  {
    Serial.println("CW2015_Alert_pin low");
  }

  if (VoutCharger5V)
  {
    Serial.println("VoutCharger5V_pin high");
  } else
  {
    Serial.println("VoutCharger5V_pin low");
  }

  Serial.println("=============");
}

BLYNK_WRITE(V1) // Boost Output Enable (toggle)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(ETA1096_Enable_pin, HIGH);
    Serial.println("ETA1096_Enable_pin HIGH");
  } else
  {
    digitalWrite(ETA1096_Enable_pin, LOW);
    Serial.println("ETA1096_Enable_pin LOW");
  }

}

BLYNK_WRITE(V2) // Charger key (toggle)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(IP5305_Key_pin, HIGH);
    Serial.println("IP5305_Key_pin HIGH");
  } else
  {
    digitalWrite(IP5305_Key_pin, LOW);
    Serial.println("IP5305_Key_pin LOW");
  }
}

BLYNK_WRITE(V3) // Charger key Output off (30sec low)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(IP5305_Key_pin, LOW);
    delay(30000);
    digitalWrite(IP5305_Key_pin, HIGH); // this code powers on again the charger output
    Serial.println("IP5305_Key_pin (30sec low)");
  }
}

BLYNK_WRITE(V4) // Charger key (short)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(IP5305_Key_pin, LOW);
    delay(100);
    digitalWrite(IP5305_Key_pin, HIGH);
    Serial.println("IP5305_Key_pin (short)");
  }
}

BLYNK_WRITE(V5) // Charger key (long)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(IP5305_Key_pin, LOW);
    delay(3000);
    digitalWrite(IP5305_Key_pin, HIGH);
    Serial.println("IP5305_Key_pin (long)");
  }
}

BLYNK_WRITE(V6) // Charger key (2 short)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(IP5305_Key_pin, LOW);
    delay(100);
    digitalWrite(IP5305_Key_pin, HIGH);
    delay(700);
    digitalWrite(IP5305_Key_pin, LOW);
    delay(100);
    digitalWrite(IP5305_Key_pin, HIGH);
    Serial.println("IP5305_Key_pin (2 short)");
  }
}
