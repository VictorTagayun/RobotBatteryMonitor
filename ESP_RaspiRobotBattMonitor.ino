/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial1

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <BlynkSimpleEsp8266.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include "ThingSpeak.h"
#include <EEPROM.h>
#include <SoftwareSerial.h>

ESP8266WiFiMulti WiFiMulti;

int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;
#define SECRET_CH_ID 650731     // replace 0000000 with your channel number
#define SECRET_WRITE_APIKEY "VYBJ5FSSJEV343N8"   // replace XYZ with your channel write API Key

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

char auth[] = "215df2de13a54c86b54cbb0fcbef1730";

WidgetTerminal terminal(V12);

int Charger_key_pin = 14;
int ChargerVout_pin  = 12;
int Vin_pin = 13;
int Rpi_Tx_pin = 15;
int Serial1_pin = 2;
int Boost_enable_pin  = 0;
int SoftwareSerialRpiCmd_pin = 3;
int Buzzer_pin = 1;
int Counter_100msec;

bool Charger_key_enable;
bool ChargerVout_enable;
bool Vin_enable;
bool Rpi_Tx_enable;
bool Boost_enable;

bool command_sent;
bool first_power_on = true;
bool Counter_1sec_stat;
bool charger_output_off_stat;
bool sudo_halt_stat;
bool serial_enable;
bool I2C_enable = true;
bool terminal_enable;

char power_stat;
float Batt_Voltage_ADC, Batt_Voltage_I2C, Batt_Percent_I2C;
int ChargingInput, LobattAlert;
int reset_cntr;

BlynkTimer timerSerialDatalog_3s, timerThingSpeakDatalog, timerCheckInputSignals, timerCheckCommandSent, timer_100msec;

SoftwareSerial RpiSerialCmd(SW_SERIAL_UNUSED_PIN, SoftwareSerialRpiCmd_pin); // RX, TX

void setup()
{
  // Check eeprom last saved power stat
  EEPROM.begin(2);

  // read_power_stat

  Wire.begin();

  Serial1.begin(115200);

  // raspi login
  RpiSerialCmd.begin(115200);
  RpiSerialCmd.println("pi");
  delay(1000);
  RpiSerialCmd.println("dwango");
  delay(1000);
  RpiSerialCmd.println("2778kulitlikotvj");

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

  pinMode(Charger_key_pin, OUTPUT);
  pinMode(ChargerVout_pin, INPUT);
  pinMode(Vin_pin, INPUT);
  pinMode(Rpi_Tx_pin, INPUT);
  pinMode(Boost_enable_pin, OUTPUT);
  pinMode(Buzzer_pin, OUTPUT);

  // check current raspi stat

  OTA();

  // Batt monitor reset
  write_mode(0x0);

  timerThingSpeakDatalog.setInterval(30000L, ThingSpeakDatalog);
  timerSerialDatalog_3s.setInterval(3000L, SerialDatalog_3s);
  timer_100msec.setInterval(100L, Count_100msec);

}

void loop()
{
  Blynk.run();
  timerSerialDatalog_3s.run();
  timerThingSpeakDatalog.run(); // Initiates BlynkTimer
  timer_100msec.run();

  ArduinoOTA.handle();

  // check Vin_enable = high and VoutCharger_pin = low, then is it not charging
}

float ReadAnalogBattLevel()
{
  int BattValueInt = 0;
  float BattValueFloat = 0;

  BattValueInt = analogRead(A0);
  BattValueFloat = (float(BattValueInt) / 1023) / 10 * (10 + 22 + 20);

  return BattValueFloat;
}

float i2c_read_battery_voltage() // add comm error
{
  float battery_voltage;
  char batt_lsb, batt_msb;
  int result;
  int error_cntr;

  Wire.beginTransmission(0x62);
  Wire.write(byte(0x02));
  Wire.endTransmission(false);
  result = Wire.requestFrom(0x62, 1, true);
  if (result != 1) {
    Serial1.print("batt_msb nok : ");
    Serial1.println(result, HEX);
    error_cntr++;
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
    error_cntr++;
  }
  batt_lsb = Wire.read(); // receive a byte as character

  battery_voltage += batt_lsb;
  battery_voltage = battery_voltage * 305 / 1000000;

  if (error_cntr == 0)
    return battery_voltage;
  else
    return 0;
}

float i2c_read_battery_percentage() // add comm error
{
  float battery_percentage;
  char percent_lsb, percent_msb;
  int result;
  int error_cntr;

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

  if (error_cntr == 0)
    return battery_percentage;
  else
    return 0;
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
  ThingSpeak.setField(4, Vin_enable);
  ThingSpeak.setField(5, ChargerVout_enable);
  ThingSpeak.setField(6, Boost_enable);

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
    Batt_Voltage_I2C = i2c_read_battery_voltage();
    Batt_Percent_I2C = i2c_read_battery_percentage();
    Serial1.println(">>>>>>>>>>>>> I2C enabled");
  } else
  {
    Serial1.println(">>>>>>>>>>>>> I2C disabled");
  }

  Charger_key_enable = digitalRead(Charger_key_pin);
  ChargerVout_enable = digitalRead(ChargerVout_pin);
  Vin_enable = digitalRead(Vin_pin);
  Rpi_Tx_enable = digitalRead(Rpi_Tx_pin);
  Boost_enable = digitalRead(Boost_enable_pin);

  Serial1.print("Battery Analog Voltage = ");
  Serial1.println(Batt_Voltage_ADC);
  Serial1.print("Battery Voltage        = ");
  Serial1.println(Batt_Voltage_I2C);
  Serial1.print("Battery Percentage     = ");
  Serial1.println(Batt_Percent_I2C);

  if (Charger_key_enable)
  {
    Serial1.println("Charger_key   HIGH");
  } else
  {
    Serial1.println("Charger_key   LOW");
  }

  if (ChargerVout_enable)
  {
    Serial1.println("ChargerVout   HIGH");
  } else
  {
    Serial1.println("ChargerVout   LOW");
  }

  if (Vin_enable)
  {
    Serial1.println("Vin           HIGH");
  } else
  {
    Serial1.println("Vin           LOW");
  }

  if (Rpi_Tx_enable)
  {
    Serial1.println("Rpi_Tx_enable HIGH");
  } else
  {
    Serial1.println("Rpi_Tx_enable LOW");
  }

  if (Boost_enable)
  {
    Serial1.println("Boost_enable  HIGH");
  } else
  {
    Serial1.println("Boost_enable  LOW");
  }

  Serial1.println("===============================");
}

BLYNK_WRITE(V1) // Boost Output Enable (toggle)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(Boost_enable_pin , HIGH);
    Serial1.println(">>>>>>>>>>>>>>> Boost_enable; HIGH");
  } else
  {
    digitalWrite(Boost_enable_pin, LOW);
    Serial1.println(">>>>>>>>>>>>>>> Boost_enable; LOW");
  }
}

BLYNK_WRITE(V2) // Charger key (toggle)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(Charger_key_pin, HIGH);
    Serial1.println(">>>>>>>>>>>>>>> Charger_key_pin HIGH");
  } else
  {
    digitalWrite(Charger_key_pin, LOW);
    Serial1.println(">>>>>>>>>>>>>>> Charger_key_pin LOW");
  }
}

BLYNK_WRITE(V3) // Charger key Output off (30sec low)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    digitalWrite(Charger_key_pin, LOW);
    Serial1.println(">>>>>>>>>>>>>>> Charger_key_pin (30sec LOW)");
    Counter_100msec = 0;
    charger_output_off_stat = true;
  }
}

void charger_output_off()
{
  if (charger_output_off_stat)
  {
    if (Counter_100msec == 300)
    {
      digitalWrite(Charger_key_pin, HIGH); // this code powers on again the charger output
      Serial1.println(">>>>>>>>>>>>>>> Charger_key_pin 30 secs HIGH");
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
    digitalWrite(Charger_key_pin, LOW);
    BlynkDelay(100);
    digitalWrite(Charger_key_pin, HIGH);
    Serial1.println(">>>>>>>>>>>>>>> Charger_key_pin (short)");
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
  digitalWrite(Charger_key_pin, LOW);
  BlynkDelay(3000);
  digitalWrite(Charger_key_pin, HIGH);
  Serial1.println(">>>>>>>>>>>>>>> Charger_key_pin (long)");
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
  digitalWrite(Charger_key_pin, LOW);
  BlynkDelay(100);
  digitalWrite(Charger_key_pin, HIGH);
  BlynkDelay(700);
  digitalWrite(Charger_key_pin, LOW);
  BlynkDelay(100);
  digitalWrite(Charger_key_pin, HIGH);
  Serial1.println(">>>>>>>>>>>>>>> Charger_key_pin (2 short)");
}

BLYNK_WRITE(V7) // Sudo Halt
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  // process received value
  if (pinValue)
  {
    Serial1.println(">>>>>>>>>>>>>>> Sudo Halt");
    RpiSerialCmd.println("sudo poweroff"); // did not restart
    RpiSerialCmd.println("sudo halt");
    // Serial.println("sudo shutdown"); // did not restart, but need to wait for 1minute to shutdown
    Counter_100msec = 0;
    sudo_halt_stat = true;
    I2C_enable = false;
  }
}

void SudoHaltCmd() // read rpitx then off boost enable pin
{
  if (sudo_halt_stat)
  {
    Rpi_Tx_enable = digitalRead(Rpi_Tx_pin);
    if (Rpi_Tx_enable) // change to timer
    {
      Counter_100msec = 0; // keep reset
      Serial1.println(">>>>>>>>>>>>>>> Rpi_Tx_pin_enable = HIGH"); // test if serial can power on raspi
    }  else
    {
      if (Counter_100msec == 100) // 10secs
      {
        digitalWrite(Boost_enable_pin, LOW);
        sudo_halt_stat = false;
        I2C_enable = true;
        Serial1.println(">>>>>>>>>>>>>>> Boost_enable_pin = LOW"); // test if serial can power on raspi
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
    RpiSerialCmd.println("pi"); // did not restart
    delay(1000);
    RpiSerialCmd.println("dwango");
    delay(1000);
    RpiSerialCmd.println("2778kulitlikotvj");
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
    ChargerVout_enable = digitalRead(ChargerVout_pin);
    while (ChargerVout_enable)
    {
      charger_key_2_short();
      BlynkDelay(3000);
    }
  } else // power off charger output
  {
    digitalWrite(Charger_key_pin, LOW);
    Serial1.println(">>>>>>>>>>>>>>> Charger_pin = LOW");
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
  power_stat = EEPROM.read(1);
}

void Count_100msec()
{
  Counter_100msec++;
  //  Serial.print("Counter_100msec : ");
  //  Serial.println(Counter_100msec);
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
