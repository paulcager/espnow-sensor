#include <Arduino.h>

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <time.h>                       // time() ctime()
#include <sys/time.h>                   // struct timeval
#include <NTPClient.h>
#include <WiFiUdp.h>

#define MQTT_MAX_PACKET_SIZE 512
#include <PubSubClient.h>
#include <cQueue.h>
#include "secrets.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// The maximum size of an esp-now packet is 255 bytes. We will be adding some other fields to it.
// I could calculate the exact maximum size, but I can't be bothered.
#define MAX_ESP_PACKET 255
// Note that by default, the MQTT client limits messages to shorter than this.
#define MESSAGE_LEN MQTT_MAX_PACKET_SIZE

typedef struct {
    unsigned long arrival;
    uint32_t sequence;
    size_t len;
    uint8_t mac[6];
    uint8_t message[MAX_ESP_PACKET+1];
} inbound_t;

typedef struct {
    enum State {
        waiting_for_esp_now,
        waiting_for_wifi_connection,
        got_wifi_connection,
        idle_wifi,
    };

    unsigned long start_connect_time;
    unsigned long idle_until_time;
    volatile uint32_t sequence;
    uint32_t failed_mqqt_sends;
    volatile State state;

    Queue_t	q;
} state_t;

static state_t state = {};

static WiFiClient wifi_client;
static PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, wifi_client);
// Define NTP Client to get time
static WiFiUDP ntpUDP;
static NTPClient timeClient(ntpUDP);

bool connect_to_mqtt();
void send_to_mqtt(inbound_t *inbound_message);
void disconnect_from_mqtt();

void sanityCheck();

static void restart(const char *why) {
    Serial.println(why);
    Serial.flush();
    ESP.restart();
}

static timeval cbtime;			// time set in callback
static bool cbtime_set = false;
static void time_is_set(void) {
    gettimeofday(&cbtime, NULL);
    cbtime_set = true;
    Serial.println("------------------ settimeofday() was called ------------------");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    inbound_t inbound_message;

    inbound_message.sequence = state.sequence++;
    inbound_message.arrival = millis();
    inbound_message.len = len;
    memcpy((void *)inbound_message.mac, mac, 6);
    memcpy((void *)inbound_message.message, incomingData, len);
    inbound_message.message[len] = '\0';

    Serial.print("Got message: ");
    Serial.println(reinterpret_cast<const char *>(inbound_message.message));

    q_push(&state.q, &inbound_message);
}

static void start_espnow(){
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        restart("Error initializing ESP-NOW");
    }

    esp_now_register_recv_cb(OnDataRecv);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);

    // The Seeed S£s get quite hot. Reduce clock speed to keep cooler.
    setCpuFrequencyMhz(80);

    q_init(&state.q, sizeof(inbound_t), 3, FIFO, true);

    pinMode(LED_BUILTIN, OUTPUT);

    configTime(0, 0, "pool.ntp.org", "2.uk.pool.ntp.org", "3.europe.pool.ntp.org");

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_MODE_STA);
    sanityCheck();
    timeClient.begin();

    start_espnow();

    // Start by initiating a WiFi connection, to start updating timeClient.
    // Note that if the cvonnection attempt fails we will reboot (which is probably sensible).
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    state.start_connect_time = millis();
    state.state = state_t::waiting_for_wifi_connection;
}

void sanityCheck() {
#ifdef SANITY_CHECK
    // Sanity check. Validate we can connect to WiFi.
    delay(5000);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        Serial.print(".");
    }
    Serial.println();
    timeClient.begin();
    for (int i = 0; i < 100; i++) {
        timeClient.update();
        if (timeClient.isTimeSet()) {
            break;
        }

        delay(50);
    }
    Serial.print("Time: ");
    Serial.println(timeClient.getEpochTime());
    timeClient.end();
    WiFi.disconnect(false, true);
#endif
}

void loop() {
    static unsigned long display_time = 10000;
    static bool have_displayed = false;

    if (!have_displayed && millis() > display_time) {
        // Delay printing of startup messages until we've been up for a few seconds and
        // the serial connection is ready (especially for ESP32-S3).
        have_displayed = true;
        Serial.print("MAC: ");
        Serial.println(WiFi.macAddress());
        Serial.print("LED: ");
        Serial.println(LED_BUILTIN);
        Serial.print("CPU Freq: ");
        Serial.println(getCpuFrequencyMhz());
        Serial.flush();
    }

    mqttClient.loop();

    digitalWrite(LED_BUILTIN, state.state == state_t::waiting_for_esp_now ? HIGH : LOW);

    if (state.state == state_t::waiting_for_wifi_connection) {
        if (WiFi.status() == WL_CONNECTED) {
            state.state = state_t::got_wifi_connection;
        } else if (state.start_connect_time + 45000 < millis()) {
            restart("Waited 45s for WiFi - will discard and reboot");
        }

        return;
    }

    if (state.state == state_t::got_wifi_connection || state.state == state_t::idle_wifi) {
        timeClient.update();
    }

    if (state.state == state_t::got_wifi_connection) {
        if (connect_to_mqtt()) {
            inbound_t msg;
            while (q_pop(&state.q, &msg)) {
                send_to_mqtt(&msg);
            }
            disconnect_from_mqtt();
        } else {
            state.failed_mqqt_sends++;
        }

        state.idle_until_time = millis() + 50;
        state.state = state_t::idle_wifi;
        return;
    }

    if (state.state == state_t::idle_wifi && millis() >= state.idle_until_time) {
        WiFi.disconnect(false, true);

        start_espnow();

        state.state = state_t::waiting_for_esp_now;
        return;
    }

    if (!q_isEmpty(&state.q)) {
        //    uint8_t bssid[] = {0x10, 0xD7, 0xB0, 0x4A, 0x43, 0xDE};
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        state.start_connect_time = millis();
        state.state = state_t::waiting_for_wifi_connection;
        return;
    }
}

bool connect_to_mqtt(){
    if (!mqttClient.connect("esp-now", MQTT_USER, MQTT_PASSWORD)) {
        Serial.print("MQTT connection failed! Host=");
        Serial.print(MQTT_BROKER);
        Serial.print(":");
        Serial.print(MQTT_PORT);
        Serial.print(". State=");
        Serial.println(mqttClient.state());

        return false;
    }
    return true;
}

void disconnect_from_mqtt() {
    mqttClient.disconnect();
}

void send_to_mqtt(inbound_t *inbound_message) {
    static uint8_t output_buff[MESSAGE_LEN];
    static char mac_buff[19];
    memset(mac_buff, 0, sizeof mac_buff);
    for (int i = 0; i < 6; i++) {
        snprintf(mac_buff +(3*i), 4, "%02X:", inbound_message->mac[i]);
    }
    mac_buff[17] = '\0';

    unsigned long rtc = timeClient.getEpochTime();

    // TODO - should check mqttClient.getBufferSize() here.

    int n = snprintf(reinterpret_cast<char *>(output_buff), sizeof(output_buff) - 1,
                     "{ \"t\": %ld, \"seq\": %d, \"mac\": \"%s\", \"delay\": %d, \"mqtt_err\": %d, \"data\": %s  }\n",
                     rtc,
                     inbound_message->sequence,
                     mac_buff,
                     (int)(millis() - state.start_connect_time),
                     state.failed_mqqt_sends,
                     inbound_message->message);

    unsigned int output_len = n > sizeof(output_buff) ? sizeof(output_buff) : n;
    char topic_name[strlen(MQTT_TOPIC) + 1 + strlen(mac_buff) + 1];
    strcpy(topic_name, MQTT_TOPIC);
    strcat(topic_name, "/");
    strcat(topic_name, mac_buff);

    if (!mqttClient.publish(topic_name, output_buff, output_len, true)) {
        state.failed_mqqt_sends++;
        Serial.print("mqttClient.publish failed: state=");
        Serial.println(mqttClient.state());
    }

    unsigned long now = millis();
    Serial.print(now);
    Serial.print(' ');
    Serial.print(now - state.start_connect_time);
    Serial.print(' ');
    Serial.println(reinterpret_cast<const char *>(output_buff));
}


