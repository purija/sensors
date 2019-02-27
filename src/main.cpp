#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "MHZ.h"

// pin for pwm reading
#define CO2_IN D5

// pin for uart reading / tx and rx of mhz19b and wemos have to be crossed !
#define MH_Z19_RX D7  // D7
#define MH_Z19_TX D6  // D6

MHZ co2(MH_Z19_RX, MH_Z19_TX, CO2_IN, MHZ19B);

#define DHTPIN D1  // Pin connected to the DHT sensor.

#define DHTTYPE DHT22  // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

WiFiClient wifiClient;

void callback(char *topic, byte *payload, unsigned int length) {
  // handle message arrived
}

// use static ip or dnsmasq
PubSubClient client("<your_mqqt_host>", 1883, callback, wifiClient);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  WiFi.begin("<your_wifi_sid>", "<your_wifi_password");

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  dht.begin();
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.

  pinMode(CO2_IN, INPUT);
  delay(100);
  Serial.println("MHZ 19B");
  // enable debug to get addition information
  co2.setDebug(false);

  delayMS = sensor.min_delay / 1000;
}

void loop() {
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    char temp[16];
    snprintf(temp, sizeof(temp), "%.2f", event.temperature);

    if (client.connect("identy")) {
      client.publish("node1/temp", temp);

      Serial.println("mqtt published node1/temp");
      Serial.print(F("Temperature: "));
      Serial.print(temp);
      Serial.println(F("°C"));
    }
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  } else {
    char humidity[16];
    snprintf(humidity, sizeof(humidity), "%.2f", event.relative_humidity);

    if (client.connect("identy")) {
      client.publish("node1/humidity", humidity);

      Serial.println("mqtt published node1/humidity");
      Serial.print(F("Humidity: "));
      Serial.print(humidity);
      Serial.println(F("%"));
    }
  }

  if (co2.isPreHeating()) {
    Serial.println("MHZ19B - Preheating");
  }

  if (co2.isReady()) {
    int readPpmPmw = co2.readCO2PWM();
    int readPpmUart = co2.readCO2UART();
    uint8_t readLastTemp = co2.getLastTemperature();

    Serial.print("readPpmPmw ");
    Serial.println(readPpmPmw);
    Serial.print("readPpmUart ");
    Serial.println(readPpmUart);
    Serial.print("readLastTemp ");
    Serial.println(readLastTemp);

    char ppmPmw[16];
    char ppmUart[16];
    char lastTemp[8];

    snprintf(ppmPmw, sizeof(ppmPmw), "%d", readPpmPmw);
    snprintf(ppmUart, sizeof(ppmUart), "%d", readPpmUart);
    snprintf(lastTemp, sizeof(lastTemp), "%d", readLastTemp);

    if (client.connect("identy")) {
      client.publish("node1/co2ppmpmw", ppmPmw);
      Serial.println("mqtt published node1/co2ppmpmw");

      client.publish("node1/co2ppmuart", ppmUart);
      Serial.println("mqtt published node1/co2ppmuart");

      client.publish("node1/co2lasttemp", lastTemp);
      Serial.println("mqtt published node1/co2lasttemp");
    }
  }

  if (!client.connected()) {
    // reconnect
  }
  client.loop();
}
