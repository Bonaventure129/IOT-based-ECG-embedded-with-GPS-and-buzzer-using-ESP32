#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <TinyGPS++.h>

#define WIFISSID "xxxxxxxx" // Enter WifiSSID here
#define PASSWORD "xxxxxxxx" // Enter password here
#define TOKEN "xxxxxxxxx" // Ubidots' TOKEN
#define MQTT_CLIENT_NAME "mymqttclient" // MQTT client Name
#define DEVICE_LABEL "ECG_AND_GPSLOC0pu" // Ubidots device label
#define VARIABLE_LABEL1 "ECG_SENSOR" // ECG Variable label
#define VARIABLE_LABEL_LOCATION "location" // Location Variable label

#define SENSORPIN 33// Set the 34 as SENSORPIN
#define BUZZER_PIN 5

#define RXD2 16
#define TXD2 17

char mqttBroker[]  = "industrial.api.ubidots.com";
char payload[1000];
char topic[150];
// Space to store values to send
char str_ecg[10];
char str_lat[10];
char str_lng[10];
char str_millis[20];
double epochseconds = 0;
double epochmilliseconds = 0;
double current_millis = 0;
double current_millis_at_sensordata = 0;
double timestampp = 0;
int j = 0;

WiFiClient ubidots;
PubSubClient client(ubidots);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

TinyGPSPlus gps;
HardwareSerial neogps(1);

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  Serial.write(payload, length);
  Serial.println(topic);
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");

    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFISSID, PASSWORD);

// Begin serial communication with Neo6m GPS
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(SENSORPIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.println();
  Serial.print("Waiting for WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
  timeClient.update();
  epochseconds = timeClient.getEpochTime();
  epochmilliseconds = epochseconds * 1000;
  current_millis = millis();

 // neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  if (!client.connected()) {
    reconnect();
    j = 0;
  }

  timeClient.update();
  epochseconds = timeClient.getEpochTime();
  epochmilliseconds = epochseconds * 1000;
  current_millis = millis();

  sprintf(topic, "/v1.6/devices/%s", DEVICE_LABEL);
  sprintf(payload, "{\"%s\": ", VARIABLE_LABEL1); // ECG sensor data
  float ecg = analogRead(SENSORPIN);
  dtostrf(ecg, 4, 2, str_ecg);
  current_millis_at_sensordata = millis();
  timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
  dtostrf(timestampp, 10, 0, str_millis);
  sprintf(payload + strlen(payload), "{\"value\": %s, \"timestamp\": %s}", str_ecg, str_millis);

  // GPS data
  boolean newData = false;
  while (neogps.available()) {
    if (gps.encode(neogps.read())) {
      newData = true;
    }
  }

  if (newData) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    dtostrf(lat, 4, 6, str_lat);
    dtostrf(lng, 4, 6, str_lng);
    Serial.print("Latitude: ");
    Serial.println(lat, 6);
    Serial.print("Longitude: ");
    Serial.println(lng, 6);

    sprintf(payload + strlen(payload), ", \"%s\": {\"value\": 1, \"context\": {\"lat\": %s, \"lng\": %s}}", VARIABLE_LABEL_LOCATION, str_lat, str_lng);
  }

  sprintf(payload + strlen(payload), "}");
  Serial.println("Publishing data to Ubidots Cloud");
  client.publish(topic, payload);
  Serial.println(payload);

  if(ecg >= 1400 && ecg <= 2400){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);


  }else{
    digitalWrite(BUZZER_PIN, HIGH);

  }

  client.loop();
  delay(500);
}