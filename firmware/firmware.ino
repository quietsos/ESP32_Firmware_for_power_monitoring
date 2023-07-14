#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 16, 2);


// ZMPT101B voltage sensor
const int voltageSensor1 = 34;
float voltage1 = 0;
const int voltageSensor2 = 35;
float voltage2 = 0;
const int voltageSensor3 = 32;
float voltage3 = 0;
const int voltageSensor4 = 22;
float voltage4 = 0;
const int voltageSensor5 = 19;
float voltage5 = 0;
const int voltageSensor6 = 21;
float voltage6 = 0;


// SCT013 30A/1V CTs current sensor
const int currentSensor = 36;           // Connect SCT013 output to ESP32 Analog Pin A0
const float Vref = 3.3;                 // ESP32 reference voltage
const float sensitivity = 30.0;         // SCT013 sensitivity (30A/1V)
const float supplyVoltage = 3.3;        // SCT013 supply voltage (usually 3.3V)
float current = 0;

// Power parameter

float power1 = 0;
float power2 = 0;
float power3 = 0;
float power4 = 0;
float power5 = 0;
float power6 = 0;


// relay pin number defines

const int relay1 = 4;          // D4 pin of ESP32
const int relay2 = 5;          // D5 pin of ESP32
const int relay3 = 18;         // D18 pin of ESP32
//const int relay4 = 19;         // D19 pin of ESP32


 // Keypad pin number define
const int key1 = 13;           // D13 pin of ESP32
const int key2 = 12;           // D12 pin of ESP32
const int key3 = 14;           // D14 pin of ESP32
const int key4 = 27;           // D27 pin of ESP32

// declaration variables for storing keypad button pressed value
int key1S = 0;
int key2S = 0;
int key3S = 0;
int key4S = 0;

// Replace with your network credentials
const char* ssid = "wifi name";
const char* password = "password";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_user = "your_MQTT_USER";
const char* mqtt_password = "your_MQTT_PASSWORD";
WiFiClient espClient;
PubSubClient client(espClient);



void setup() {

  Serial.begin(115200);

  // sim800l module setup
  Serial2.begin(115200);
  delay(3000);
  test_sim800_module();
//  send_SMS();

  
  
  pinModeDefine();   // pin mode define for each pin used.

  // wifi connection initialization
  initWiFi();

  // mqtt server setup using server ip/domain name and port
  client.setServer(mqtt_server, mqtt_port);
  // callback setup for receiving message from the mqtt server
  //  client.setCallback(callback);


  //  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Welcome.....");
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("R1 R2 R3R4 R5 R6");

}

void test_sim800_module()
{
  Serial2.println("AT");
  updateSerial();
  Serial.println();
  Serial2.println("AT+CSQ");
  updateSerial();
  Serial2.println("AT+CCID");
  updateSerial();
  Serial2.println("AT+CREG?");
  updateSerial();
  Serial2.println("ATI");
  updateSerial();
  Serial2.println("AT+CBC");
  updateSerial();
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}


void send_SMS()
{
  Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  Serial2.println("AT+CMGS=\"+919804049270\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  Serial2.print("Power 1 : "); //text content
  Serial2.println(power1);
  Serial2.print("Power 2 : "); //text content
  Serial2.println(power2);
  Serial2.print("Power 3 : "); //text content
  Serial2.println(power3);
  Serial2.print("Current : "); //text content
  Serial2.println(current);
  updateSerial();
  Serial2.write(26);
}



void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageRel;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageRel += (char)message[i];
  }
  Serial.println();

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    
    if(messageRel == "rel1"){
      digitalWrite(relay1, HIGH);
    }
     if(messageRel == "rel2"){
      digitalWrite(relay2, HIGH);
    }
     if(messageRel == "rel3"){ 
      digitalWrite(relay3, HIGH);
    }
  }
}


void pinModeDefine() {


  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
//  pinMode(relay4, OUTPUT);

  pinMode(key1, INPUT_PULLUP);// set pin as input
  pinMode(key2, INPUT_PULLUP);// set pin as input
  pinMode(key3, INPUT_PULLUP);// set pin as input
  pinMode(key4, INPUT_PULLUP);// set pin as input


  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, LOW);
//  digitalWrite(relay4, LOW);

}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println("Connected");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client",mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void readKeyPad() {

  key1S = digitalRead(key1);// read if key1 is pressed
  key2S = digitalRead(key2);// read if key1 is pressed
  key3S = digitalRead(key3);// read if key1 is pressed
  key4S = digitalRead(key4);// read if key1 is pressed
}

void relayControl() {

  if (!key1S) {
    digitalWrite(relay1, HIGH);

  }
  else {
    digitalWrite(relay1, LOW);
  }
  if (!key2S) {
    digitalWrite(relay2, HIGH);

  }
  else {
    digitalWrite(relay2, LOW);
  }

  if (!key3S) {
    digitalWrite(relay3, HIGH);

  }
  else {
    digitalWrite(relay3, LOW);
  }

  if (!key4S){
    send_SMS();
  }

 

}

void readVoltage1() {
  float voltValue = analogRead(voltageSensor1);
  voltage1 = (voltValue / 1024) * 225;
  Serial.print("Voltage 1: ");
  Serial.print(voltage1);
  Serial.println(" V");
  delay(5000);

}
void readVoltage2() {
  float voltValue = analogRead(voltageSensor2);
  voltage2 = (voltValue / 1024) * 225;
  Serial.print("Voltage 2: ");
  Serial.print(voltage2);
  Serial.println(" V");
  delay(5000);

}
void readVoltage3() {
  float voltValue = analogRead(voltageSensor3);
  voltage3 = (voltValue / 1024) * 225;
  Serial.print("Voltage 3: ");
  Serial.print(voltage3);
  Serial.println(" V");
  delay(5000);

}

void readVoltage4() {
  float voltValue = analogRead(voltageSensor4);
  voltage4 = (voltValue / 1024) * 225;
  Serial.print("Voltage 4: ");
  Serial.print(voltage4);
  Serial.println(" V");
  delay(5000);

}

void readVoltage5() {
  float voltValue = analogRead(voltageSensor5);
  voltage5 = (voltValue / 1024) * 225;
  Serial.print("Voltage 5: ");
  Serial.print(voltage5);
  Serial.println(" V");
  delay(5000);

}
void readVoltage6() {
  float voltValue = analogRead(voltageSensor6);
  voltage6 = (voltValue / 1024) * 225;
  Serial.print("Voltage 6: ");
  Serial.print(voltage6);
  Serial.println(" V");
  delay(5000);

}


void readCurrent() {
  int sensorValue = analogRead(currentSensor);  // Read the analog input from SCT013 sensor
  float Cvoltage = (sensorValue / 1024.0) * Vref;  // Convert the analog value to voltage
  current = (Cvoltage - (supplyVoltage / 2)) / ((supplyVoltage / 2) / sensitivity);  // Calculate the current using the SCT013 sensitivity and supply voltage
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println(" A");
  delay(1000);  // Wait for 1 second before taking another reading
}




void loop() {


  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  updateSerial();  // update serial for sim800l module

  readVoltage1();
  readVoltage2();
  readVoltage3();
  readVoltage4();
  readVoltage5();
  readVoltage6();
  readCurrent();
  readKeyPad();

  power1 = voltage1 * current;
  power2 = voltage2 * current;
  power3 = voltage3 * current;
  power4 = voltage4 * current;
  power5 = voltage5 * current;
  power6 = voltage6 * current;
  

  Serial.print("Power 1: ");
  Serial.println(power1);
  Serial.print("Power 2: ");
  Serial.println(power2);
  Serial.print("Power 3: ");
  Serial.println(power3);
  Serial.print("Power 4: ");
  Serial.println(power4);
  Serial.print("Power 5: ");
  Serial.println(power5);
  Serial.print("Power 6: ");
  Serial.println(power6);


  relayControl();


  lcd.setCursor(0, 1);
  lcd.print(power1);
  lcd.print(   power2);
  lcd.print(      power3);
  lcd.print(       power4);
  lcd.print(         power5);
  lcd.print(           power6);
 

  // mosquitto mqtt broker power value publish

  char pr1[8];
  dtostrf(power1, 6, 2, pr1);
  char pr2[8];
  dtostrf(power2, 6, 2, pr2);
  char pr3[8];
  dtostrf(power3, 6, 2, pr3);
  char pr4[8];
  dtostrf(power4, 6, 2, pr4);
  char pr5[8];
  dtostrf(power5, 6, 2, pr5);
  char pr6[8];
  dtostrf(power6, 6, 2, pr6);

  
  client.publish("power1", pr1);
  client.publish("power2", pr2);
  client.publish("power3", pr3);
  client.publish("power4", pr4);
  client.publish("power5", pr5);
  client.publish("power6", pr6);

  delay(1000);

}
