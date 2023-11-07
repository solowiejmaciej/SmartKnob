#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "AiEsp32RotaryEncoder.h"

const int oneWireBus = 4;
bool isRotoryButtonOn = false;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String currrentTimeStamp;

const char *ssid = "ssid";
const char *password = "password";
const char *mqtt_server_ip = "ip";
const int mqtt_server_port = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void blinkBlueLed();
void turnOnBlueLed();
void turnOffBlueLed();

float getTemperature();
void reconnectToMqtt();
static void sendTemperatureToServer();
void sendButtonPressToServer(bool isOn);
void sendRotoryValueToServer(int value);

void configure();
void configureTemperature();
void configureWiFi();
void configureMqtt();

#define ROTARY_ENCODER_A_PIN 23
#define ROTARY_ENCODER_B_PIN 22
#define ROTARY_ENCODER_BUTTON_PIN 21
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 1

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

void rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 1; // Soft debouncing
  if (isRotoryButtonOn)
  {
    sendButtonPressToServer(true);
    isRotoryButtonOn = false;
  }
  else
  {
    sendButtonPressToServer(false);
    isRotoryButtonOn = true;
  }
}

void rotary_loop()
{
  // dont print anything unless value changed
  if (rotaryEncoder.encoderChanged())
  {
    sendRotoryValueToServer(rotaryEncoder.readEncoder());
  }
  if (rotaryEncoder.isEncoderButtonClicked())
  {
    rotary_onButtonClick();
  }
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  configure();
  timeClient.begin();

  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 100, false);
  rotaryEncoder.setAcceleration(0);
}

void loop()
{
  rotary_loop();

  timeClient.forceUpdate();
}

void turnOnBlueLed()
{
  digitalWrite(2, HIGH); // Turn the LED off by making the voltage HIGH
}

void turnOffBlueLed()
{
  digitalWrite(2, LOW);
}

void blinkBlueLed()
{
  turnOnBlueLed();
  delay(2000);      // Wait for
  turnOffBlueLed(); // Turn the LED off by making sure
}

float getTemperature()
{
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

void configure()
{
  turnOnBlueLed();

  configureTemperature();
  configureWiFi();
  configureMqtt();

  turnOffBlueLed();
}

void configureTemperature()
{
  sensors.begin();
  pinMode(2, OUTPUT); // Initialize GPIO2 pin as an output
}

void configureWiFi()
{

  WiFi.mode(WIFI_STA); // Optional
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to wifi.");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100);
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void configureMqtt()
{
  mqttClient.setServer(mqtt_server_ip, mqtt_server_port);
  reconnectToMqtt();
}

void reconnectToMqtt()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.println("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str()))
    {
      Serial.println("Successfully connected to MQTT server");
    }
    else
    {
      Serial.println("failed, rc=");
      Serial.print(mqttClient.state());
      delay(5000);
    }
  }
}

static void sendTemperatureToServer()
{
  if (!mqttClient.connected())
  {
    reconnectToMqtt();
  }
  Serial.println("Reporting Temperature");
  StaticJsonDocument<120> doc;
  char output[120];
  doc["temp"] = getTemperature();
  doc["timeStamp"] = timeClient.getEpochTime();
  serializeJson(doc, output);
  mqttClient.publish("/home/sensors/temp", output);
}

void sendButtonPressToServer(bool isOn)
{
  if (!mqttClient.connected())
  {
    reconnectToMqtt();
  }
  Serial.println("Reporting button press");
  Serial.println(mqttClient.state());

  StaticJsonDocument<120> json;
  char buttonOutput[120];
  json["isOn"] = isOn;
  json["timeStamp"] = timeClient.getEpochTime();
  serializeJson(json, buttonOutput);
  mqttClient.publish("/home/sensors/button", buttonOutput);
}

void sendRotoryValueToServer(int value)
{
  if (!mqttClient.connected())
  {
    reconnectToMqtt();
  }
  Serial.println("Reporting rotary data");
  StaticJsonDocument<120> json;
  char buttonOutput[120];
  json["value"] = value;
  json["timeStamp"] = timeClient.getEpochTime();
  serializeJson(json, buttonOutput);
  mqttClient.publish("/home/sensors/rotory", buttonOutput);
}