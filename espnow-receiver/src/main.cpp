#ifdef EVO_ETH
#define ETH_PHY_TYPE ETH_PHY_DM9051
#define ETH_PHY_ADDR         1
#define ETH_PHY_CS           9
#define ETH_PHY_IRQ          8
#define ETH_PHY_RST          6
#define ETH_PHY_SPI_HOST    SPI2_HOST
#define ETH_PHY_SPI_SCK      7
#define ETH_PHY_SPI_MISO     3
#define ETH_PHY_SPI_MOSI    10
#else
#ifndef ETH_PHY_MDC
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#if CONFIG_IDF_TARGET_ESP32
#define ETH_PHY_ADDR  0
#define ETH_PHY_MDC   23
#define ETH_PHY_MDIO  18
#define ETH_PHY_POWER -1
#define ETH_CLK_MODE  ETH_CLOCK_GPIO0_IN
#elif CONFIG_IDF_TARGET_ESP32P4
#define ETH_PHY_ADDR  0
#define ETH_PHY_MDC   31
#define ETH_PHY_MDIO  52
#define ETH_PHY_POWER 51
#define ETH_CLK_MODE  EMAC_CLK_EXT_IN
#endif
#endif
#endif

#include <Arduino.h>

#include "ETH.h"

#include <esp_now.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <time.h>                       // time() ctime()
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <esp_http_server.h>


#include <PubSubClient.h>
#include <cQueue.h>

#include "../../secrets.h"

// The maximum size of an esp-now packet is 255 bytes. We will be adding some other fields to it.
// I could calculate the exact maximum size, but I can't be bothered.
#define MAX_ESP_PACKET 255
// Note that by default, the MQTT client limits messages to shorter than this.
#define MESSAGE_LEN 512

#define WIFI_WAIT_TIMEOUT_MILLIS 60000

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
    unsigned long last_send_time_millis;
    volatile uint32_t sequence;
    volatile uint32_t failed_mqtt_sends;
    volatile State state;

    Queue_t	q;
} state_t;

static state_t state = {};

static uint8_t chipId[8];


static WiFiClient wifi_client;
static PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, wifi_client);
// Define NTP Client to get time
static WiFiUDP ntpUDP;
static NTPClient timeClient(ntpUDP);

static bool ethAvailable = false;      // True if MAC/PHY exist; does not need cable plugged in.
static volatile bool ethConnected = false;

#define HOSTNAME_PREFIX "espnow-receiver-"
#define HOSTNAME_FORMAT (HOSTNAME_PREFIX "%02X%02X")
static char host[sizeof(HOSTNAME_PREFIX) + 4 + 1];

bool connect_to_mqtt();
void send_to_mqtt(inbound_t *inbound_message);
void disconnect_from_mqtt();

void sanityCheck();
static httpd_handle_t start_webserver();

static void led_task();

void send_queued_message();

static void restart(const char *why) {
    Serial.println(why);
    // Don't use flush in case it hangs.
    //Serial.flush();
    delay(50);
    ESP.restart();
}

// now() returns a string containing the current time in HH:MM:SS format. It is unreliable as
// it depends on the timeClient, and so should only be used for debugging.
const char *now() {
    static char time_buff[16];
    if (!timeClient.isTimeSet()) {
        sprintf(time_buff, "BOOT+%ld", millis() / 1000);
        return time_buff;
    }

    time_t now = (time_t)timeClient.getEpochTime();
    struct tm *t = gmtime(&now);
    strftime(time_buff, sizeof(time_buff), "%H:%M:%S", t);
    return time_buff;
}

static unsigned long last_event_time = 0;
void wifi_event(arduino_event_id_t event) {
    Serial.print(millis() - last_event_time);
    Serial.print(' ');
    Serial.println(WiFi.eventName(event));
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    }
    last_event_time = millis();
}

void onEvent(arduino_event_id_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            // The hostname must be set after the interface is started, but needs
            // to be set before DHCP, so set it from the event handler thread.

            Serial.print("ETH Started: host=");
            Serial.print(host);
            Serial.print(", ETH_MAC=");
            Serial.println(ETH.macAddress());
            ETH.setHostname(host);
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("ETH Connected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.print("ETH Got IP: ");
            Serial.println(ETH.localIP());
            ethConnected = true;
            break;
//        case ARDUINO_EVENT_ETH_LOST_IP:
//            Serial.println("ETH Lost IP");
//            ethConnected = false;
//            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("ETH Disconnected");
            ethConnected = false;
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("ETH Stopped");
            ethConnected = false;
            ethAvailable = false;
            break;
        default: break;
    }
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info* info, const uint8_t *incomingData, int len) {
    const uint8_t *mac = info->src_addr;
    inbound_t inbound_message;

    inbound_message.sequence = state.sequence++;
    inbound_message.arrival = millis();
    inbound_message.len = len;
    memcpy((void *)inbound_message.mac, mac, 6);
    memcpy((void *)inbound_message.message, incomingData, len);
    inbound_message.message[len] = '\0';

    Serial.print(now());
    Serial.print(" in  ");
    Serial.println(reinterpret_cast<const char *>(inbound_message.message));

    q_push(&state.q, &inbound_message);
}

static void start_espnow(){
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        restart("Error initializing ESP-NOW");
    }

    esp_now_register_recv_cb(&OnDataRecv);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

void setup() {
#ifndef ARDUINO_WT32_ETH01
    // The Seeed S3s get quite hot. Reduce clock speed to keep cooler.
    setCpuFrequencyMhz(80);
#else
    setCpuFrequencyMhz(80);
#endif

    enableLoopWDT();

    // Initialize Serial Monitor
    Serial.begin(115200);

    pinMode(2, OUTPUT);
    pinMode(5, OUTPUT);
    q_init(&state.q, sizeof(inbound_t), 3, FIFO, true);

#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
#endif

    esp_efuse_mac_get_default(chipId);
    snprintf(host, sizeof(host),  HOSTNAME_FORMAT, chipId[4], chipId[5]);

    mqttClient.setBufferSize(MESSAGE_LEN + MQTT_MAX_HEADER_SIZE + 2 + sizeof(MQTT_TOPIC) + 1 + 6);

    //WiFi.onEvent(wifi_event);
    WiFi.onEvent(onEvent);

    Serial.println("ETH.begin():");
    ethAvailable = ETH.begin();
    Serial.print("ETH.begin returned:");
    Serial.println(ethAvailable);

    // Set device as a Wi-Fi Station so it can receive espnpw
    WiFi.mode(WIFI_MODE_STA);
    // Uncomment the following to add debugging of wireless timings.
    // A rule of thumb is that STA_CONNECT takes about 2.5 seconds, STA_GOT_IP about 1.5.
    //sanityCheck();

    configTime(0, 0, "pool.ntp.org", "2.uk.pool.ntp.org", "3.europe.pool.ntp.org");
    timeClient.begin();

    start_espnow();
    start_webserver();

    state.state = state_t ::waiting_for_esp_now;
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
    timeClient.
    for (int i = 0; i < 100; i++) {
        timeClient.update();
        timeClient.
        if (timeClient.isTimeSet()) {
            timeClient.
            break;
        }

        delay(50);
    }
    Serial.print("Time: ");
    Serial.println(timeClient.getEpochTime());
    timeClient.
    timeClient.end();
    timeClient.
    WiFi.disconnect(false, true);
#endif
}

void loop() {
    static unsigned long display_time = 10000;
    static bool have_displayed = false;

    if (!have_displayed && millis() > display_time) {
        // Delay printing of startup messages until we've been up for a few seconds and
        // the serial connection is ready (especially for ESP32-S3). We may also have the timeClient
        // available by then (for wired connections).
        have_displayed = true;
        Serial.print("Starting up ");
        Serial.println(ESP.getChipModel());
        Serial.print("Host ");
        Serial.println(host);
        Serial.print("ethAvailable=");
        Serial.println(ethAvailable);
        Serial.print("STA MAC: ");
        Serial.println(WiFi.macAddress());
        Serial.print("ETH MAC: ");
        Serial.println(ETH.macAddress());
        Serial.print("CPU Freq: ");
        Serial.println(getCpuFrequencyMhz());
        Serial.print("Time: ");
        Serial.println(timeClient.getFormattedTime());
    }

    led_task();
    mqttClient.loop();

#ifdef LED_PIN
    digitalWrite(LED_PIN, state.state == state_t::waiting_for_esp_now ? HIGH : LOW);
#endif

    if (state.state == state_t::waiting_for_wifi_connection) {
        if (WiFi.status() == WL_CONNECTED) {
            state.state = state_t::got_wifi_connection;
            //Serial.printf("Got wifi %s after %ld millis\n", WiFi.localIP().toString().c_str(), millis() - state.start_connect_time);
        } else if (state.start_connect_time + WIFI_WAIT_TIMEOUT_MILLIS < millis()) {
            Serial.printf("%s: Waited too long for WiFi: WiFi.status() == %d\n", now(), WiFi.status());
            WiFi.printDiag(Serial);
            restart("Timed out waiting for WiFi - will discard and reboot");
        }

        return;
    }

    if (ethConnected || state.state == state_t::got_wifi_connection || state.state == state_t::idle_wifi) {
        timeClient.update();
        send_queued_message();
    }

    if (state.state == state_t::got_wifi_connection) {
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

    if (!q_isEmpty(&state.q) && !ethConnected) {
        // Note that attempting to avoid DHCP only saves about a second, so it is not attempted.
//        uint8_t bssid[] = {0x12, 0xD7, 0xB0, 0x4A, 0x43, 0xDE};
//        WiFi.config(
//                IPAddress(192, 168, 0, 100),
//                IPAddress(192, 168, 0, 1),
//                IPAddress(8, 8, 8, 8),
//                IPAddress(1, 1, 1, 1)
//                );
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, nullptr);
        state.start_connect_time = millis();
        last_event_time = millis();
        state.state = state_t::waiting_for_wifi_connection;
        return;
    }

    // A little delay to reduce power.
    delay(10);
}

// If there's a queued message make sure we are connected to MQTT, pop it and
// send to MQTT. Only send one message at a time, so that we don't starve the
// MQTT client loop function.
void send_queued_message() {
    if (q_isEmpty(&state.q)) {
        return;
    }

    if (connect_to_mqtt()) {
        inbound_t msg;
        if (q_pop(&state.q, &msg)) {
            send_to_mqtt(&msg);
        }
    } else {
        state.failed_mqtt_sends++;
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

#define MAC_FORMAT "%02X:%02X:%02X:%02X:%02X:%02X"
#define MAC_BYTES(MAC) (MAC[0]),(MAC[1]),(MAC[2]),(MAC[3]),(MAC[4]),(MAC[5])

void send_to_mqtt(inbound_t *inbound_message) {
    static uint8_t output_buff[MESSAGE_LEN];

    unsigned long rtc = timeClient.getEpochTime();

    static char in_mac[6*2 + 5 + 1];
    snprintf(in_mac, sizeof in_mac, MAC_FORMAT,MAC_BYTES(inbound_message->mac));
    String relay_mac = ethConnected ? ETH.macAddress() : WiFi.macAddress();
    int delay = ethConnected ? 0 : (int)(millis() - state.start_connect_time);


    int n = snprintf(reinterpret_cast<char *>(output_buff), sizeof(output_buff) - 1,
                     "{ \"t\": %ld, \"seq\": %d, \"mac\": \"%s\", \"relay\": \"%s\", \"delay\": %d, \"mqtt_err\": %d, \"data\": %s  }\n",
                     rtc,
                     inbound_message->sequence,
                     in_mac,
                     relay_mac.c_str(),
                     delay,
                     state.failed_mqtt_sends,
                     inbound_message->message);

    unsigned int output_len = n > sizeof(output_buff) ? sizeof(output_buff) : n;
    char topic_name[strlen(MQTT_TOPIC) + 1 + strlen(in_mac) + 1];
    strcpy(topic_name, MQTT_TOPIC);
    strcat(topic_name, "/");
    strcat(topic_name, in_mac);

    // TODO - should check mqttClient.getBufferSize() here.

    if (mqttClient.publish(topic_name, output_buff, output_len, true)) {
        state.last_send_time_millis = millis();
    } else {
        state.failed_mqtt_sends++;
        Serial.print("mqttClient.publish failed: state=");
        Serial.print(mqttClient.state());
        Serial.print(", output_len=");
        Serial.print(output_len);
        Serial.print(", buffer_size=");
        Serial.println(mqttClient.getBufferSize());
    }

    Serial.print(now());
    Serial.print(" out ");
    Serial.println(reinterpret_cast<const char *>(output_buff));
}


static esp_err_t metrics_get_handler(httpd_req_t *req) {
    // TODO - check buffer overflow
    static char metrics_buff[1024];

    httpd_resp_set_type(req, "text/plain");

    // TODO - counts for each sender.
    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_message_count_total counter\n"
             "espnow_receiver_message_count_total{receiver=\"%s\"} %d\n",
             host,
             state.sequence);
    httpd_resp_sendstr_chunk(req, metrics_buff);

    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_failed_mqtt_count_total counter\n"
             "espnow_receiver_failed_mqtt_count_total{receiver=\"%s\"} %d\n",
             host,
             state.failed_mqtt_sends);
    httpd_resp_sendstr_chunk(req, metrics_buff);

    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_uptime gauge\n"
             "espnow_receiver_uptime{receiver=\"%s\"} %f\n",
             host,
             (double)millis() / 1000.0);
    httpd_resp_sendstr_chunk(req, metrics_buff);

    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_message_age gauge\n"
             "espnow_receiver_message_age{receiver=\"%s\"} %f\n",
             host,
             ((double)(millis() - state.last_send_time_millis)) / 1000.0);
    httpd_resp_sendstr_chunk(req, metrics_buff);

    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_heap_free_bytes gauge\n"
             "espnow_receiver_heap_free_bytes{receiver=\"%s\"} %u\n",
             host,
             esp_get_free_heap_size());
    httpd_resp_sendstr_chunk(req, metrics_buff);

    // Indicate sent all chunks.
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static const httpd_uri_t metrics = {
        .uri       = "/metrics",
        .method    = HTTP_GET,
        .handler   = metrics_get_handler,
};

static httpd_handle_t start_webserver()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &metrics);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void led_task() {
    static unsigned long last_change = 0;
    static int state = 0;
    if (millis() < last_change + 1000) {
        return;
    }

    last_change = millis();
    digitalWrite(2, state);
    state = (state+1) & 0x01;
    digitalWrite(5, state);
}