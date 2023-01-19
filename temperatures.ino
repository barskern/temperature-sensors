// A temperature ESP01S sensor for house deployment

#define HOSTNAME "..."
#define WIFI_SSID "..."
#define WIFI_PSK "..."
#define MQTT_SERVER "..."
#define MQTT_PORT 1883

// Contains definitinos for the above variables
#include "config.h"

//#define DEBUG
//#define DHT_DEBUG

#define TOPIC_BASE "home/sensor/" HOSTNAME

#define DEFAULT_SAMPLE_INTERVAL 1000 * 10 // Ten seconds
#define NTP_SYNC_INTERVAL 1000 * 60 * 10 // Ten minutes
#define MINIMUM_LOOP_TIME 200
#define MAXIMUM_INTERVAL 1000 * 60 * 60 // One hour

//#define HTTP_UPDATER_SERVER

#define MAX_PAYLOAD_SIZE 20
#define MSG_BUFFER_SIZE 20

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <EEPROM.h>

#ifdef HTTP_UPDATER_SERVER
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#endif

#define DEBUG_PRINTER Serial

#ifdef DEBUG
#define DEBUG_INIT() { DEBUG_PRINTER.begin(115200); }
#define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
#define DEBUG_INIT()
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

// DHT22 data pin is connected to ESP8266 pin GPIO2 (TX)
DHT dht(2, DHT22);

WiFiClient wifi_client;
WiFiUDP ntpUDP;
NTPClient time_client(ntpUDP);

PubSubClient mqtt_client(wifi_client);

#ifdef HTTP_UPDATER_SERVER
ESP8266WebServer http_server(80);
ESP8266HTTPUpdateServer http_updater;
#endif

void connect_wifi() {
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PSK);

  DEBUG_PRINT("Connecting to ");
  DEBUG_PRINT(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    DEBUG_PRINT(".");
  }
  DEBUG_PRINTLN();

  DEBUG_PRINT("Connected, IP address: ");
  DEBUG_PRINTLN(WiFi.localIP());
}

void connect_mqtt() {
  // Loop until we're connected
  while (!mqtt_client.connected()) {
    DEBUG_PRINTLN("Connecting to MQTT broker...");

    if (mqtt_client.connect(HOSTNAME, nullptr, nullptr, TOPIC_BASE "/status", 1, true, "offline")) {
      DEBUG_PRINTLN("Connected to MQTT broker!");

      mqtt_client.publish(TOPIC_BASE "/status", "online", true);
      mqtt_client.subscribe(TOPIC_BASE "/set_sample_interval", 1);
    } else {
      DEBUG_PRINT("failed, rc=");
      DEBUG_PRINT(mqtt_client.state());
      DEBUG_PRINTLN(" try again in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

bool read_temperatures(float *h, float *t) {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h_ = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t_ = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h_) || isnan(t_)) {
    return false;
  }

  *h = h_;
  *t = t_;

  DEBUG_PRINT(F("Humidity: "));
  DEBUG_PRINT(h_);
  DEBUG_PRINT(F("%  Temperature: "));
  DEBUG_PRINT(t_);
  DEBUG_PRINT(F("°C "));

#ifdef DEBUG
  // Compute heat index in Celsius (isFahreheit = false)
  float hic_ = dht.computeHeatIndex(t_, h_, false);

  DEBUG_PRINT(F("°Heat index: "));
  DEBUG_PRINT(hic_);
  DEBUG_PRINT(F("°C "));
#endif
  DEBUG_PRINTLN();

  return true;
}

unsigned int sample_interval = DEFAULT_SAMPLE_INTERVAL;

void handle_mqtt_message(const char* topic, byte* payload, unsigned int length) {
  byte payload_buf[MAX_PAYLOAD_SIZE];
  char* payload_s = nullptr;

  DEBUG_PRINT("Processing MQTT packet with topic '");
  DEBUG_PRINT(topic);
  DEBUG_PRINTLN("'");
  if (strcmp(TOPIC_BASE "/set_sample_interval", topic) == 0) {
    // Ensure payload is a cstr and move to separat buffer
    if (payload[length - 1] == 0) {
      payload_s = (char*)payload;
    } else if (length + 1 < MAX_PAYLOAD_SIZE) {
      memcpy(&payload_buf, payload, length);
      payload_buf[length] = 0;
      payload_s = (char*)&payload_buf;
    } else {
      DEBUG_PRINTLN("Got too big of a payload in MQTT packet, ignoring");
      return;
    }

    int value = atoi(payload_s);
    if (value <= 0) {
      DEBUG_PRINTLN("Got invalid or empty packet");
      return;
    }

    if (value < MINIMUM_LOOP_TIME) {
      sample_interval = MINIMUM_LOOP_TIME;
    } else if (value > MAXIMUM_INTERVAL) {
      sample_interval = MAXIMUM_INTERVAL;
    } else {
      sample_interval = value;
    }
    DEBUG_PRINT("Updated sample interval to: ");
    DEBUG_PRINTLN(sample_interval);
    write_config();
  }
}

void read_config() {
  byte magic = EEPROM.read(0);

  // Config is only valid if the first value is the given magic number
  if (magic != 0xaf)
    return;

  DEBUG_PRINTLN("Found magic in EEPROM so reading config");

  unsigned int sample_interval_ = 0;
  sample_interval_ = EEPROM.get(1, sample_interval_);
  if (MINIMUM_LOOP_TIME < sample_interval_ && sample_interval_ < MAXIMUM_INTERVAL) {
    sample_interval = sample_interval_;
  }
}

void write_config() {
  EEPROM.write(0, 0xaf);
  EEPROM.put(1, sample_interval);
  EEPROM.commit();
  
  DEBUG_PRINTLN("Wrote config to EEPROM");
}

void setup() {
  DEBUG_INIT();
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Initializing...");

  EEPROM.begin(256);
  read_config();

  dht.begin();

  connect_wifi();

  DEBUG_PRINTLN("Starting NTP sync..");
  time_client.begin();

#ifdef HTTP_UPDATER_SERVER
  DEBUG_PRINTLN("Starting HTTP Updater server..");
  MDNS.begin(HOSTNAME);
  http_updater.setup(&http_server);
  http_server.begin();
  MDNS.addService("http", "tcp", 80);
  DEBUG_PRINT("Room for ");
  DEBUG_PRINT(ESP.getFreeSketchSpace());
  DEBUG_PRINTLN(" bytes for a binary file across network");
#endif

  mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt_client.setCallback(handle_mqtt_message);
  connect_mqtt();
}

unsigned long last_ntp_sync_time = 0;
unsigned long last_sample_time = 0;

void loop() {
  unsigned long loop_start = millis();

  if (WiFi.status() != WL_CONNECTED) {
    connect_wifi();
  }
  if (!mqtt_client.connected()) {
    connect_mqtt();
  }
  mqtt_client.loop();

  // Works even when overflow!
  unsigned long time_since_last_ntp_sync = loop_start - last_ntp_sync_time;
  if (last_ntp_sync_time == 0 || NTP_SYNC_INTERVAL <= time_since_last_ntp_sync) {
    time_client.update();
    last_ntp_sync_time = loop_start;
    
    DEBUG_PRINT("NTP synced to: ");
    DEBUG_PRINTLN(time_client.getFormattedTime());
  }

  unsigned long time_since_last_sample = loop_start - last_sample_time;
  if (last_sample_time == 0 || sample_interval <= time_since_last_sample) {

    // Consider doing cumulative average for each sent sample if signal gets noisy.
    float h = 0.0f, t = 0.0f;
    char msg[MSG_BUFFER_SIZE];
    if (!read_temperatures(&h, &t)) {
      DEBUG_PRINTLN("Unable to read temperatures from DHT sensor");
      mqtt_client.publish(TOPIC_BASE "/status", "sensor_error", false);
    } else {
      snprintf(msg, MSG_BUFFER_SIZE, "%.1f", t);
      mqtt_client.publish(TOPIC_BASE "/temperature", msg, true);
      snprintf(msg, MSG_BUFFER_SIZE, "%.1f", h);
      mqtt_client.publish(TOPIC_BASE "/humidity", msg, true);
      last_sample_time = loop_start;
    }
  }

#ifdef HTTP_UPDATER_SERVER
  http_server.handleClient();
  MDNS.update();
#endif

  unsigned long loop_time = millis() - loop_start;
  if (loop_time < MINIMUM_LOOP_TIME) {
    unsigned long delay_time = MINIMUM_LOOP_TIME - loop_time;
    delay(delay_time);
  }

  // Old way which would work for sleeping, though won't enable MQTT liveness
  // Works even when overflow!
  // unsigned long loop_time = millis() - loop_start;
  // if (loop_time < sample_interval) {
  //   unsigned long delay_time = sample_interval - loop_time;
  //   // TODO figure out sleeping instead of constant on here, but what happens to
  //   // WIFI then??
  //   delay(delay_time);
  // }
}
