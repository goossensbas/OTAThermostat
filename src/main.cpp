#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <PubSubClient.h>
#include "MHZ19.h"
#include <softwareserial.h>

#ifndef STASSID
#define STASSID "SSID"
#define STAPSK  "PASSWORD"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;
long lastMsg = 0;
char msg[50];
int value = 0;
unsigned long lastMillis = 0;
float temp = 0;
float humidity = 0;
int CO2 = 0;
unsigned long delayTime;
int counter = 0;

Adafruit_BME280 bme; // I2C
WiFiClient espClient;

IPAddress local_IP(192, 168, 0, 193);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

#define RX_PIN D5                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN D6                                         // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // create device to MH-Z19 serial



void callback(char* topic, byte* payload, unsigned int length) {
  // do something with the message

  String topicStr = topic; 
  String recv_payload = String(( char *) payload);

  Serial.println( "mqtt_callback - message arrived - topic [" + topicStr + 
                  "] payload [" + recv_payload + "]" );
}
PubSubClient client("192.168.0.212", 1883, callback, espClient);

//connect to MQTT broker with Last Will message configuration
void connect() {
  Serial.print("connecting...");
  while (!client.connect("espThermostat", "/home/room1/status", 1, 1, "offline")) {
    Serial.print(".");
    delay(1000);
  }
}


//connect to wifi and start OTA handler.
//OTA IP address is stored in the platformio.ini file.
void setup_wifi() {
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  delay(200);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
    // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
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

void setup() {
  Serial.begin(9600);
  setup_wifi();
  connect();
  Wire.begin(D3, D4);
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);
  mySerial.begin(BAUDRATE);
  myMHZ19.begin(mySerial);
  delay(100);
  myMHZ19.autoCalibration(true);                              // Turn auto calibration ON (OFF autoCalibration(false))

if (! bme.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        ESP.restart();
    }

}
void loop() {
  ArduinoOTA.handle();

if (!client.connected()) {
    connect();
}
  client.loop();
  delay(200);
if (counter == 1){
  temp = bme.readTemperature();							//read BME temperature
  humidity = bme.readHumidity();						//read BME humidity
  if (myMHZ19.getCO2() > 0) {							//to reduce zero values
    CO2 = myMHZ19.getCO2();								//read CO2
  };
  StaticJsonDocument<200> JSONbuffer;					//write to JSON
  JSONbuffer["device"] = "ESPBed1";
  JSONbuffer["temperature"] = temp;
  JSONbuffer["humidity"] = humidity;
  JSONbuffer["co2"] = CO2;

  char JSONmessageBuffer[200];
  serializeJson(JSONbuffer, JSONmessageBuffer);




  Serial.println("sending values over MQTT:");
if (client.publish("/home/bed1/hvac", JSONmessageBuffer) == true){
  Serial.println("Success sending message");
} else {
  Serial.println("Error sending message");
}
client.loop();
 Serial.println("-------------");
}
if (counter == 50){
client.publish("/home/bed1/status", "online");			//send status updates
}
delay(100);
counter = counter + 1;
if (counter == 100){
  counter = 0;
}
}