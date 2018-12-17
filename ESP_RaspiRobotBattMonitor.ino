/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial1

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <BlynkSimpleEsp8266.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include "ThingSpeak.h"
#include <EEPROM.h>

ESP8266WiFiMulti WiFiMulti;

int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;
#define SECRET_CH_ID 650731     // replace 0000000 with your channel number
#define SECRET_WRITE_APIKEY "VYBJ5FSSJEV343N8"   // replace XYZ with your channel write API Key

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

char auth[] = "215df2de13a54c86b54cbb0fcbef1730";

WidgetTerminal terminal(V12);

int ETA1096_Enable_pin = 3;
int CW2015_Alert_pin = 15;
int VinCharging5V_pin = 13;
int VoutCharger5V_pin = 12;
int IP5305_Key_pin = 14;
int Rpi_Tx_pin = 3;
int Tx_pin = 1;
int Counter_100msec;

bool ETA1096_Enable;
bool CW2015_Enable;
bool I2C_enable = true;
bool VinCharging5V;
bool VoutCharger5V;
bool IP5305_Key;
bool command_sent;
bool Rx;
bool Rpi_Tx;
bool first_power_on = true;
bool Counter_1sec_stat;
bool charger_output_off_stat;
bool sudo_halt_stat;
bool serial_enable;
bool terminal_enable;

char power_stat;
float Batt_Voltage_ADC, Batt_Voltage_I2C, Batt_Percent_I2C;
int ChargingInput, LobattAlert;

BlynkTimer timerSerialDatalog_3s, timerThingSpeakDatalog, timerCheckInputSignals, timerCheckCommandSent, timer_100msec;

void setup()
{
  // Check eeprom last saved power stat
  EEPROM.begin(1);
  read_power_stat();

  Wire.begin();

  Serial.begin(115200);
  Serial.swap(); // Serial.set_tx(2); // 
  Serial.println("pi");
  delay(1000);
  Serial.println("dwango");
  delay(1000);
  Serial.println("2778kulitlikotvj");

  Serial1.begin(115200);

  //Serial.swap();

  //   We start by connecting to a WiFi network
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("SingaporeMilitia", "123Qweasd");
  WiFiMulti.addAP("SingaporeMilitia_plus", "123Qweasd");
  WiFiMulti.addAP("SingaporePolice", "123Qweasd");
  WiFiMulti.addAP("AntipoloPolice", "TAGAYUNFAMILY");
  WiFiMulti.addAP("VicenteTagayun", "27Author");
  Serial1.println();
  Serial1.println();
  Serial1.print("Wait for WiFi ");
  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial1.print(".");
    delay(500);
    // need to add cntr timeout
  }
  Serial1.println("");
  Serial1.print("WiFi connected to : ");
  Serial1.println(WiFi.SSID());
  Serial1.print("IP address: ");
  Serial1.println(WiFi.localIP());

  Blynk.config(auth);
  // Blynk status
  // Init the the buttons

  terminal.clear();

  ThingSpeak.begin(client);
  // ThingSpeak status

  //  pinMode(ETA1096_Enable_pin, OUTPUT);
  //  pinMode(IP5305_Key_pin, OUTPUT);
  //  pinMode(CW2015_Alert_pin, INPUT);
  //  pinMode(VinCharging5V_pin, INPUT);
  //  pinMode(VoutCharger5V_pin, INPUT);
  //  pinMode(Tx_pin, INPUT);

  // check current raspi stat
  //  if (digitalRead(ETA1096_Enable_pin))
  //  {
  //    digitalWrite(ETA1096_Enable_pin, HIGH);
  //    while (!digitalRead(VoutCharger5V))
  //    {
  //      charger_key_2_short();
  //      //      Serial.println("charger_key_2_short");
  //      delay(3000);
  //    }
  //  } else
  //  {
  //    digitalWrite(ETA1096_Enable_pin, LOW);
  //    digitalWrite(IP5305_Key_pin, LOW);
  //  }


  //  Blynk.begin(auth, ssid, pass);

  OTA();

  // Batt monitor reset
  write_mode(0x0);

  timerThingSpeakDatalog.setInterval(30000L, ThingSpeakDatalog);
  timerSerialDatalog_3s.setInterval(3000L, SerialDatalog_3s);
  timer_100msec.setInterval(100L, Count_100msec);

  //delay(20000);
}

void loop()
{
  Blynk.run();
  if (1) // serial_enable
  {
    timerSerialDatalog_3s.run();
    timerThingSpeakDatalog.run(); // Initiates BlynkTimer
  }
  timer_100msec.run();

  ArduinoOTA.handle();
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
    Serial1.print("batt_msb nok : ");
    Serial1.println(result, HEX);
  }
  batt_msb = Wire.read(); // receive a byte as character

  battery_voltage = batt_msb << 8;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x03));
  Wire.endTransmission(false);
  result = Wire.requestFrom(0x62, 1, true);
  if (result != 1) {
    Serial1.print("batt_lsb nok : ");
    Serial1.println(result, HEX);
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
    Serial1.print("percent_msb nok : ");
    Serial1.println(result, HEX);
  }
  percent_msb = Wire.read(); // receive a byte as character

  battery_percentage = percent_msb;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x05));
  Wire.endTransmission(false);
  result = Wire.requestFrom(0x62, 1, true);
  if (result != 1) {
    Serial1.print("percent_lsb nok : ");
    Serial1.println(result, HEX);
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
    Serial1.print("read_mode nok : ");
    Serial1.println(result, HEX);
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
    Serial1.print("read_mode nok : ");
    Serial1.println(result, HEX);
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
    Serial1.print("write_mode nok : ");
    Serial1.println(result, HEX);
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
    Serial1.println("ThingSpeakDtalog update successful.");
  }
  else {
    Serial1.println("Problem updating ThingSpeakDtalog. HTTP error code " + String(x));
  }
}

void SerialDatalog_3s()
{
  Batt_Voltage_ADC = ReadAnalogBattLevel();
  if (I2C_enable)
  {
    Batt_Voltage_I2C = read_battery_voltage();
    Batt_Percent_I2C = read_battery_percentage();
    Serial1.println(">>>>>>>>>>>>> I2C enabled");
  } else
  {
    Serial1.println(">>>>>>>>>>>>> I2C disabled");
  }
  ChargingInput = digitalRead(VinCharging5V_pin);
  LobattAlert = digitalRead(CW2015_Alert_pin);
  VoutCharger5V = digitalRead(VoutCharger5V_pin);
  ETA1096_Enable = digitalRead(ETA1096_Enable_pin);
  IP5305_Key = digitalRead(IP5305_Key_pin);
  Rpi_Tx = digitalRead(Rpi_Tx_pin);

  Serial1.print("Battery Analog Voltage = ");
  Serial1.println(Batt_Voltage_ADC);
  Serial1.print("Battery Voltage        = ");
  Serial1.println(Batt_Voltage_I2C);
  Serial1.print("Battery Percentage     = ");
  Serial1.println(Batt_Percent_I2C);

  if (ChargingInput)
  {
    Serial1.println("VinCharging5V_pin  HIGH");
  } else
  {
    Serial1.println("VinCharging5V_pin  LOW");
  }

  if (LobattAlert)
  {
    Serial1.println("CW2015_Alert_pin   HIGH");
  } else
  {
    Serial1.println("CW2015_Alert_pin   LOW");
  }

  if (VoutCharger5V)
  {
    Serial1.println("VoutCharger5V_pin  HIGH");
  } else
  {
    Serial1.println("VoutCharger5V_pin  LOW");
  }

  if (ETA1096_Enable)
  {
    Serial1.println("ETA1096_Enable_pin HIGH");
  } else
  {
    Serial1.println("ETA1096_Enable_pin LOW");
  }

  if (IP5305_Key)
  {
    Serial1.println("IP5305_Key_pin     HIGH");
  } else
  {
    Serial1.println("IP5305_Key_pin     LOW");
  }

  if (Rx)
  {
    Serial1.println("Rpi_Tx_pin         HIGH");
  } else
  {
    Serial1.println("Rpi_Tx_pin         LOW");
  }

  Serial1.println("===============================");
}

BLYNK_WRITE(V1) // Boost Output Enable (toggle)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(ETA1096_Enable_pin, HIGH);
    Serial1.println(">>>>>>>>>>>>>>> ETA1096_Enable_pin HIGH");
  } else
  {
    digitalWrite(ETA1096_Enable_pin, LOW);
    Serial1.println(">>>>>>>>>>>>>>> ETA1096_Enable_pin LOW");
  }
}

BLYNK_WRITE(V2) // Charger key (toggle)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(IP5305_Key_pin, HIGH);
    Serial1.println(">>>>>>>>>>>>>>> IP5305_Key_pin HIGH");
  } else
  {
    digitalWrite(IP5305_Key_pin, LOW);
    Serial1.println(">>>>>>>>>>>>>>> IP5305_Key_pin LOW");
  }
}

BLYNK_WRITE(V3) // Charger key Output off (30sec low)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(IP5305_Key_pin, LOW);
    Serial1.println(">>>>>>>>>>>>>>> IP5305_Key_pin (30sec LOW)");
    Counter_100msec = 0;
    charger_output_off_stat = true;
  }
}

void charger_output_off()
{
  if (charger_output_off_stat)
  {
    if (Counter_100msec == 30)
    {
      digitalWrite(IP5305_Key_pin, HIGH); // this code powers on again the charger output
      Serial1.println(">>>>>>>>>>>>>>> IP5305_Key_pin 30 secs HIGH");
      charger_output_off_stat = false;
    }
  }
}

BLYNK_WRITE(V4) // Charger key (short)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(IP5305_Key_pin, LOW);
    BlynkDelay(100);
    digitalWrite(IP5305_Key_pin, HIGH);
    Serial1.println(">>>>>>>>>>>>>>> IP5305_Key_pin (short)");
  }
}

BLYNK_WRITE(V5) // Charger key (long)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    charger_key_long();
  }
}

void charger_key_long()
{
  digitalWrite(IP5305_Key_pin, LOW);
  BlynkDelay(3000);
  digitalWrite(IP5305_Key_pin, HIGH);
  Serial1.println(">>>>>>>>>>>>>>> IP5305_Key_pin (long)");
}

BLYNK_WRITE(V6) // Charger key (2 short)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    charger_key_2_short();
  }
}

void charger_key_2_short()
{
  digitalWrite(IP5305_Key_pin, LOW);
  BlynkDelay(100);
  digitalWrite(IP5305_Key_pin, HIGH);
  BlynkDelay(700);
  digitalWrite(IP5305_Key_pin, LOW);
  BlynkDelay(100);
  digitalWrite(IP5305_Key_pin, HIGH);
  Serial1.println(">>>>>>>>>>>>>>> IP5305_Key_pin (2 short)");
}

BLYNK_WRITE(V7) // Sudo Halt
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    //    Serial.println(">>>>>>>>>>>>>>> Sudo Halt");
    Serial1.println("sudo poweroff"); // did not restart
    Serial1.println("sudo halt");
    // Serial.println("sudo shutdown"); // did not restart, but need to wait for 1minute to shutdown
    //    Counter_100msec = 0;
    //    sudo_halt_stat = true;
    //    I2C_enable = false;
  }
}

void SudoHaltCmd()
{
  if (sudo_halt_stat)
  {
    if (digitalRead(Tx_pin)) // change to timer
    {
      Counter_100msec = 0 ;
    }  else
    {
      if (Counter_100msec == 100)
      {
        digitalWrite(ETA1096_Enable_pin, LOW);
        sudo_halt_stat = false;
        I2C_enable = true;
        Serial1.println(">>>>>>>>>>>>>>> ETA1096_Enable_pin = LOW"); // test if serial can power on raspi
      }
    }
  }
}

BLYNK_WRITE(V8) // raspi login
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    Serial1.println("pi"); // did not restart
    delay(1000);
    Serial1.println("dwango");
    delay(1000);
    Serial1.println("2778kulitlikotvj");
  }
}

BLYNK_WRITE(V9) // Reset
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    ESP.restart();
  }
}

BLYNK_WRITE(V10) // i2c enable
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    if (I2C_enable)
      I2C_enable = false;
    else
      I2C_enable = true;
  }
}

BLYNK_WRITE(V11) // serial enable
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    if (serial_enable)
      serial_enable = false;
    else
      serial_enable = true;
  }
}

// You can send commands from Terminal to your hardware. Just use
// the same Virtual Pin as your Terminal Widget
BLYNK_WRITE(V12)
{

  // if you type "Marco" into Terminal Widget - it will respond: "Polo:"
  if (String("Marco") == param.asStr()) {
    terminal.println("You said: 'Marco'") ;
    terminal.println("I said: 'Polo'") ;
  } else {

    // Send it back
    terminal.print("You said:");
    terminal.write(param.getBuffer(), param.getLength());
    terminal.println();
  }

  // Ensure everything is sent
  terminal.flush();
}
void ChargerOutOnOff(bool On_Off)
{
  if (On_Off) // power on charger output
  {
    VoutCharger5V = digitalRead(VoutCharger5V_pin);
    while (VoutCharger5V)
    {
      charger_key_2_short();
      BlynkDelay(3000);
    }
  } else // power off charger output
  {
    digitalWrite(IP5305_Key_pin, LOW);
    Serial1.println(">>>>>>>>>>>>>>> IP5305_Key_pin = LOW");
  }
}

void PowerOffChargerOut()
{

}

void PowerOffBoost()
{

}

void BlynkDelay(int num_of_delay)
{
  //  for (int cntr = 0; cntr < num_of_delay; cntr++)
  //  {
  //    delay(1);
  //    //loop();
  //    Blynk.run();
  //    timerSerialDatalog_1s.run();
  //    timerThingSpeakDatalog.run(); // Initiates BlynkTimer
  //
  //    ArduinoOTA.handle();
  //  }

  int end_time = millis() + num_of_delay;
  while (millis() < end_time)
  {
    if (Blynk.connected())
    {
      loop();
    }
    yield();
  }
}

void save_power_stat()
{
  EEPROM.write(0, power_stat);
  EEPROM.commit();
}

void read_power_stat()
{
  power_stat = EEPROM.read(0);
}

void Count_100msec()
{
  Counter_100msec++;
  //  Serial.print("Counter_1sec : ");
  //  Serial.println(Counter_1sec);
  charger_output_off();
  SudoHaltCmd();
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
    Serial1.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial1.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial1.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial1.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial1.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial1.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial1.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial1.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial1.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}
