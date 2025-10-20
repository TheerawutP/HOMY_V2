#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <Arduino.h>

//MQTT libs
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <PubSubClient.h>     
#include <WiFiClientSecure.h>

//modbusRTU libs
#include <ModbusRTU.h>
#include "Hreg_setting.h"
#include "Hreg_display.h"
#include "AutoReset.h"
#include <esp_system.h>

ModbusRTU RTU_SLAVE;
#define PIN_RX2 16
#define PIN_TX2 17
#define PIN_EN  4
#define H_LIM 6
#define SLAVE_ID 125

//credential 
const char* ssid = "はなうた";
const char* password = "fff11111";
const char* mqtt_broker = "93cd9b390cc440cb911a0ee79126675c.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "Flink1";
const char* mqtt_password = "Fff11111";
const char* return_data_1 = "esp32/return_data_1";
const char* return_data_2 = "esp32/return_data_2";
const char* alive = "esp32/alive";

uint16_t m_data[5];
uint8_t publish_payload[2];
String lastTopic = "";
String lastPayload = "";


WiFiClientSecure wifiClient;                
PubSubClient mqttClient(wifiClient);


void setupMQTT() {
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(callback);
}

void endian(uint16_t data, uint8_t* payload){
  payload[0] = (data >> 8) & 0xFF;  // High byte
  payload[1] = data & 0xFF;         // Low byte
}


void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];
  lastTopic = topic;
  lastPayload = message;
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    //String clientId = "ESP32Client-0001";
    if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      Serial.print("Client ID: ");
      Serial.println(clientId);
      mqttClient.subscribe(return_data_1);
      mqttClient.subscribe(return_data_2);
      mqttClient.subscribe(alive);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup(){
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("[BOOT] Reset reason: %d\n", reason);
  Hreg_setting_call(RTU_SLAVE, SLAVE_ID, PIN_EN, PIN_RX2, PIN_TX2, H_LIM);
  AutoReset_init(1440);
  
  wifiClient.setInsecure();
  Serial.println(WiFi.localIP());
  setupMQTT();
}

void loop() {

  AutoReset_loop();
  RTU_SLAVE.task();

  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  RTU_SLAVE.Hreg(1,20);
  m_data[0] = RTU_SLAVE.Hreg(1);
  m_data[1] = RTU_SLAVE.Hreg(2);
  m_data[2] = RTU_SLAVE.Hreg(3);
  m_data[3] = RTU_SLAVE.Hreg(4);
  m_data[4] = RTU_SLAVE.Hreg(5);
  
  //Hreg[0] used for store alive bit

    if(lastTopic == "esp32/return_data_1"){     
      Serial.print("Responding to ");
      Serial.println(lastTopic);

      endian(m_data[0], publish_payload);
      Serial.print("HIGH BITS: ");
      Serial.println(publish_payload[0]);
      Serial.print("LOW BITS: ");
      Serial.println(publish_payload[1]);

      if (mqttClient.publish("esp32/return_data_1", publish_payload, 2, false)) {
        Serial.println("Publish SUCCESS");
      } else {
        Serial.println("Publish FAILED");
      }      
    }

    if(lastTopic == "esp32/return_data_2"){     
      Serial.print("Responding to ");
      Serial.println(lastTopic);

      endian(m_data[0], publish_payload);
      Serial.print("HIGH BITS: ");
      Serial.println(publish_payload[0]);
      Serial.print("LOW BITS: ");
      Serial.println(publish_payload[1]);

      if (mqttClient.publish("esp32/return_data_2", publish_payload, 2, false)) {
        Serial.println("Publish SUCCESS");
      } else {
        Serial.println("Publish FAILED");
      }      
    }

    if(lastTopic == "esp32/alive"){     
      Serial.print("Responding to ");
      Serial.println(lastTopic);
      uint16_t value = lastPayload.toInt();    // string to uint16_t
      RTU_SLAVE.Hreg(0,value);

    }
  lastTopic = "";
  lastPayload = "";
}


