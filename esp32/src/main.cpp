#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Pin definitions for SPI (if needed)
constexpr int BME_SCK = 13;
constexpr int BME_MISO = 12;
constexpr int BME_MOSI = 11;
constexpr int BME_CS = 10;

// Atmospheric pressure at sea level in hPa
constexpr float SEALEVEL_PRESSURE_HPA = 1013.25;

// I2C address of the BME280 sensor
constexpr uint8_t BME280_I2C_ADDRESS = 0x76;

// Delay between readings (milliseconds)
constexpr unsigned long DELAY_TIME_MS = 2000;

// WiFi credentials
const char* ssid = "<wifi-ssid>";
const char* password = "<wifi-password>";

// MQTT broker credentials
const char* mqtt_server = "192.168.178.50";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup_bme() {
  if (!bme.begin(BME280_I2C_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void setup() {
  Serial.begin(9600);
  setup_bme();
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publish data
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure();
  float altitude = bme.readAltitude(SEALEVEL_PRESSURE_HPA);

  client.publish("sensors/temperature", String(temperature).c_str(), true);
  client.publish("sensors/humidity", String(humidity).c_str(), true);
  client.publish("sensors/pressure", String(pressure).c_str(), true);
  client.publish("sensors/altitude", String(altitude).c_str(), true);

  Serial.println("Data published");

  delay(DELAY_TIME_MS);
}