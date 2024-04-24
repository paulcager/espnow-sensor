#include <Arduino.h>

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <PubSubClient.h>
#include <cQueue.h>
#include "secrets.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// The maximum size of an esp-now packet is 255 bytes. We will be adding some other fields to it.
// I could calculate the exact maximum size, but I can't be bothered.
#define MAX_ESP_PACKET 255
#define MESSAGE_LEN (MAX_ESP_PACKET + 256)

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
    volatile State state;

    Queue_t	q;
} state_t;

static state_t state;

static WiFiClient wifi_client;
static PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, wifi_client);

bool connect_to_mqtt();
void send_to_mqtt(inbound_t *inbound_message);
void disconnect_from_mqtt();

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    static uint32_t sequence = 0;

    inbound_t inbound_message;

    inbound_message.sequence = sequence++;
    inbound_message.arrival = millis();
    inbound_message.len = len;
    memcpy((void *)inbound_message.mac, mac, 6);
    memcpy((void *)inbound_message.message, incomingData, len);
    inbound_message.message[len] = '\0';

    q_push(&state.q, &inbound_message);
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);

    q_init(&state.q, sizeof(inbound_t), 2, FIFO, true);

    pinMode(LED_BUILTIN, OUTPUT);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_MODE_STA);

    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

//    uint8_t bssid[] = {0x10, 0xD7, 0xB0, 0x4A, 0x43, 0xDE};
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD , WIFI_CHANNEL, NULL, false);

    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
}

void loop() {
    mqttClient.loop();

    digitalWrite(LED_BUILTIN, state.state == state_t::waiting_for_esp_now ? HIGH : LOW);

    if (state.state == state_t::waiting_for_wifi_connection) {
        if (WiFi.status() == WL_CONNECTED) {
            state.state = state_t::got_wifi_connection;
        }

        return;
    }

    if (state.state == state_t::got_wifi_connection) {
        if (connect_to_mqtt()) {
            inbound_t msg;
            while (q_pop(&state.q, &msg)) {
                send_to_mqtt(&msg);
            }
            disconnect_from_mqtt();
        }

        state.idle_until_time = millis() + 1000;
        state.state = state_t::idle_wifi;
        return;
    }

    if (state.state == state_t::idle_wifi && millis() >= state.idle_until_time) {
        WiFi.disconnect(false, false);
        esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
        state.state = state_t::waiting_for_esp_now;
        return;
    }

    if (!q_isEmpty(&state.q)) {
        WiFi.reconnect();
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
    if (!connect_to_mqtt()) {
        return;
    }

    static char output_buff[MESSAGE_LEN];
    static char mac_buff[19];
    memset(mac_buff, 0, sizeof mac_buff);
    for (int i = 0; i < 6; i++) {
        snprintf(mac_buff +(3*i), 4, "%02X:", inbound_message->mac[i]);
    }
    mac_buff[17] = '\0';

    snprintf(output_buff, sizeof(output_buff) - 1,
             "{ \"sequence\": %d, \"mac\": \"%s\", \"delay\": %d, \"data\": %s  }\n",
             inbound_message->sequence,
             mac_buff,
             (int)(millis() - state.start_connect_time),
             inbound_message->message);

    char topic_name[strlen(MQTT_TOPIC) + 1 +strlen(mac_buff) + 1];
    strcpy(topic_name, MQTT_TOPIC);
    strcat(topic_name, "/");
    strcat(topic_name, mac_buff);

    if (!mqttClient.publish(topic_name, output_buff)) {
        Serial.println("mqttClient.publish failed");
    }

    unsigned long now = millis();
    Serial.print(now);
    Serial.print(' ');
    Serial.print(now - state.start_connect_time);
    Serial.print(' ');
    Serial.println(output_buff);
}


