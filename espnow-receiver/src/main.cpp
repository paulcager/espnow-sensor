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

#include "version.h"

#include "ETH.h"

#include <esp_now.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <NTPClient.h>
#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <esp_http_server.h>
#include "FS.h"
#include <LittleFS.h>
#include <cQueue.h>
#include "Update.h"
#include "esp_https_ota.h"

#include "../../secrets.h"

// The maximum size of an esp-now packet is 255 bytes. We will be adding some other fields to it.
// I could calculate the exact maximum size, but I can't be bothered.
#define MAX_ESP_PACKET 255
// Note that by default, the MQTT client limits messages to shorter than this.
#define MESSAGE_LEN 512

#define WIFI_WAIT_TIMEOUT_MILLIS 60000

#define MAC_FORMAT "%02X:%02X:%02X:%02X:%02X:%02X"
#define MAC_BYTES(MAC) (MAC[0]),(MAC[1]),(MAC[2]),(MAC[3]),(MAC[4]),(MAC[5])

typedef struct {
    unsigned long arrival;
    uint32_t sequence;
    size_t len;
    uint8_t src_mac[6];
    uint8_t dest_mac[6];
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
    volatile int rssi;
    volatile int noise_floor;
    volatile unsigned int channel;

    volatile State state;

    Queue_t	q;
} state_t;

static state_t state = {};


static WiFiClient wifi_client;
static PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, wifi_client);
// Define NTP Client to get time
static WiFiUDP ntpUDP;
static NTPClient timeClient(ntpUDP);
static void init_etag();

static bool ethAvailable = false;      // True if MAC/PHY exist; does not need cable plugged in.
static volatile bool ethConnected = false;

#define HOSTNAME_PREFIX "espnow-receiver-"
#define HOSTNAME_FORMAT (HOSTNAME_PREFIX "%02X%02X")
static char host[sizeof(HOSTNAME_PREFIX) + 4 + 1];
static uint8_t baseMac[6];
static uint8_t chipId[8];
static uint8_t etag[128];

bool connect_to_mqtt();
void send_to_mqtt(inbound_t *inbound_message);
void disconnect_from_mqtt();

void sanityCheck();
static httpd_handle_t start_webserver();
static void led_task();
static void send_queued_message();
static void check_updates();
static int httpGet(HTTPClient &http, char *url, const char *etag);

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
    const uint8_t *dest = info->des_addr;

    wifi_pkt_rx_ctrl_t *ctrl = info->rx_ctrl;
    state.channel = ctrl->channel;
    state.rssi = ctrl->rssi;
    state.noise_floor = ctrl->noise_floor;

    inbound_t inbound_message;

    inbound_message.sequence = state.sequence++;
    inbound_message.arrival = millis();
    inbound_message.len = len;
    memcpy((void *)inbound_message.src_mac, mac, 6);
    memcpy((void *)inbound_message.dest_mac, dest, 6);
    memcpy((void *)inbound_message.message, incomingData, len);
    inbound_message.message[len] = '\0';

    Serial.print(now());
    Serial.printf(" in for " MAC_FORMAT " : ", MAC_BYTES(dest));
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

#ifdef MULTICAST_ADDRESS
    static uint8_t multicast_mac_address[6] = MULTICAST_ADDRESS;
    static esp_now_peer_info_t multicast_peer;
    memset(&multicast_peer, 0, sizeof multicast_peer);
    memcpy(multicast_peer.peer_addr, multicast_mac_address, sizeof multicast_mac_address);
    multicast_peer.channel = WIFI_CHANNEL;
    esp_err_t ret = esp_now_add_peer(&multicast_peer);
    if (ret == ESP_OK || ret == ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("Registered multicast address: " MAC_FORMAT "\n",MAC_BYTES(multicast_mac_address));
    } else {
        Serial.printf("Failed to register multicast peer " MAC_FORMAT " ret=%d\n", MAC_BYTES(multicast_mac_address), ret);
    }
#endif
}

void setup() {
#ifndef ARDUINO_WT32_ETH01
    // The Seeed S3s get quite hot. Reduce clock speed to keep cooler.
    setCpuFrequencyMhz(80);
#else
    setCpuFrequencyMhz(80);
#endif

    // Initialize Serial Monitor
    Serial.begin(115200);
    Serial.println(APP_BANNER);

    q_init(&state.q, sizeof(inbound_t), 3, FIFO, true);

    esp_efuse_mac_get_default(chipId);
    snprintf(host, sizeof(host),  HOSTNAME_FORMAT, chipId[4], chipId[5]);

    // Read etag (if any) of last applied update.
    if (LittleFS.begin(true)) {
        init_etag();
    } else {
        Serial.println("LittleFS Mount Failed");
    }

    mqttClient.setBufferSize(MESSAGE_LEN + MQTT_MAX_HEADER_SIZE + 2 + sizeof(MQTT_TOPIC) + 1 + 6);

    WiFi.onEvent(onEvent);

    Serial.println("ETH.begin():");
    ethAvailable = ETH.begin();
    Serial.print("ETH.begin returned:");
    Serial.println(ethAvailable);

    // Set device as a Wi-Fi Station, so it can receive espnpw
    WiFi.mode(WIFI_MODE_STA);
    // Uncomment the following to add debugging of wireless timings.
    // A rule of thumb is that STA_CONNECT takes about 2.5 seconds, STA_GOT_IP about 1.5.
    //sanityCheck();

    (void)esp_wifi_get_mac(WIFI_IF_STA, baseMac);

    configTime(0, 0, "pool.ntp.org", "2.uk.pool.ntp.org", "3.europe.pool.ntp.org");
    timeClient.begin();

    MDNS.begin(host);

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

char *get_info() {
    static char *buff = NULL;
    static size_t buff_size;

    if (buff == NULL) {
        buff_size = 512;
        buff = static_cast<char *>(malloc(512));
        if (buff == NULL) {
            return "Out of Memory";
        }
    }

    // TODO - reallocate buff if too small? Not that it will be.
    snprintf(buff, buff_size,
                        "%s\n"
                        "Chip:               %s\r\n"
                        "ETAG:               %s\r\n"
                        "Uptime (millis):    %lu\r\n"
                        "Host:               %s\r\n"
                        "ethAvailable:       %d\r\n"
                        "BASE MAC:           " MAC_FORMAT "\r\n"
                        "ETH MAC:            %s\r\n"
                        "Channel:            %d\r\n"
                        "CPU Freq (Mhz):     %lu\r\n"
                        "Time:               %s\r\n"
                        "IP:                 %s\r\n",
                        APP_BANNER,
                        ESP.getChipModel(),
                        etag,
                        millis(),
                        host,
                        ethAvailable,
                        MAC_BYTES(baseMac),
                        ETH.macAddress().c_str(),
                        WIFI_CHANNEL,
                        getCpuFrequencyMhz(),
                        timeClient.getFormattedTime().c_str(),
                        ethConnected ? ETH.localIP().toString().c_str() : "None"
    );
    return buff;
}

void loop() {
    static bool have_displayed = false;
    static unsigned long display_time = 10000;
    static unsigned long next_check_updates = 0;

    if (!have_displayed && millis() > display_time) {
        // Delay printing of startup messages until we've been up for a few seconds and
        // the serial connection is ready (especially for ESP32-S3). We may also have the timeClient
        // available by then (for wired connections).
        have_displayed = true;
        Serial.print("Starting up ");
        Serial.println(get_info());
        return;
    }

    if (millis() >= next_check_updates && have_displayed && (WiFi.isConnected() || ethConnected)) {
        next_check_updates = millis() + (1000 * 60 * 60);
        check_updates();
        return;
    }

    led_task();
    mqttClient.loop();

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
static void send_queued_message() {
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
    if (!mqttClient.connect(host, MQTT_USER, MQTT_PASSWORD)) {
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

    unsigned long rtc = timeClient.getEpochTime();

    static char src_mac[6 * 2 + 5 + 1];
    snprintf(src_mac, sizeof src_mac, MAC_FORMAT, MAC_BYTES(inbound_message->src_mac));
    static char dest_mac[6 * 2 + 5 + 1];
    snprintf(dest_mac, sizeof src_mac, MAC_FORMAT, MAC_BYTES(inbound_message->dest_mac));
    String relay_mac = ethConnected ? ETH.macAddress() : WiFi.macAddress();
    int delay = ethConnected ? 0 : (int)(millis() - state.start_connect_time);


    int n = snprintf(reinterpret_cast<char *>(output_buff), sizeof(output_buff) - 1,
                     "{ \"t\": %ld, \"seq\": %lu, \"src_mac\": \"%s\", \"dest_mac\": \"%s\", \"relay\": \"%s\", \"delay\": %d, \"mqtt_err\": %lu, \"data\": %s  }\n",
                     rtc,
                     inbound_message->sequence,
                     src_mac,
                     dest_mac,
                     relay_mac.c_str(),
                     delay,
                     state.failed_mqtt_sends,
                     inbound_message->message);

    unsigned int output_len = n > sizeof(output_buff) ? sizeof(output_buff) : n;
    char topic_name[strlen(MQTT_TOPIC) + 1 + strlen(src_mac) + 1];
    strcpy(topic_name, MQTT_TOPIC);
    strcat(topic_name, "/");
    strcat(topic_name, src_mac);

    // TODO - should check mqttClient.getBufferSize() here.

    if (mqttClient.publish(topic_name, output_buff, output_len, true)) {
        state.last_send_time_millis = millis();
    } else {
        state.failed_mqtt_sends++;
        Serial.print("mqttClient.publish failed: state=");
        Serial.print(mqttClient.state());
        Serial.print(", write_error=");
        Serial.print(mqttClient.getWriteError());
        Serial.print(", output_len=");
        Serial.print(output_len);
        Serial.print(", buffer_size=");
        Serial.println(mqttClient.getBufferSize());
        return;
    }

    Serial.print(now());
    Serial.print(" out ");
    Serial.println(reinterpret_cast<const char *>(output_buff));
}


static esp_err_t metrics_get_handler(httpd_req_t *req) {
    // TODO - check buffer overflow
    static char metrics_buff[1024];
    char name[5];
    snprintf(name, sizeof name, "%02X%02X", baseMac[4], baseMac[5]);

    httpd_resp_set_type(req, "text/plain");

    // TODO - counts for each sender? Maybe.
    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_message_count_total counter\n"
             "espnow_receiver_message_count_total{receiver=\"%s\"} %d\n",
             name,
             state.sequence);
    httpd_resp_sendstr_chunk(req, metrics_buff);

    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_failed_mqtt_count_total counter\n"
             "espnow_receiver_failed_mqtt_count_total{receiver=\"%s\"} %d\n",
             name,
             state.failed_mqtt_sends);
    httpd_resp_sendstr_chunk(req, metrics_buff);

    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_uptime gauge\n"
             "espnow_receiver_uptime{receiver=\"%s\", chip=\"%s\"} %f\n",
             name,
             ESP.getChipModel(),
             (double)millis() / 1000.0);
    httpd_resp_sendstr_chunk(req, metrics_buff);

    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_message_age gauge\n"
             "espnow_receiver_message_age{receiver=\"%s\"} %f\n",
             name,
             ((double)(millis() - state.last_send_time_millis)) / 1000.0);
    httpd_resp_sendstr_chunk(req, metrics_buff);

    snprintf(metrics_buff, sizeof metrics_buff - 1,
             "# TYPE espnow_receiver_heap_free_bytes gauge\n"
             "espnow_receiver_heap_free_bytes{receiver=\"%s\"} %u\n",
             name,
             esp_get_free_heap_size());
    httpd_resp_sendstr_chunk(req, metrics_buff);

    // There's no sensible default for rssi etc. Make sure we don't send zeros before we get the first message.
    if (state.sequence > 0) {
        snprintf(metrics_buff, sizeof metrics_buff - 1,
                 "# TYPE espnow_receiver_rssi gauge\n"
                 "espnow_receiver_rssi{receiver=\"%s\"} %d\n",
                 name,
                 state.rssi);
        httpd_resp_sendstr_chunk(req, metrics_buff);

        snprintf(metrics_buff, sizeof metrics_buff - 1,
                 "# TYPE espnow_receiver_noise_floor gauge\n"
                 "espnow_receiver_noise_floor{receiver=\"%s\"} %d\n",
                 name,
                 state.noise_floor);
        httpd_resp_sendstr_chunk(req, metrics_buff);

        snprintf(metrics_buff, sizeof metrics_buff - 1,
                 "# TYPE espnow_receiver_channel gauge\n"
                 "espnow_receiver_channel{receiver=\"%s\"} %u\n",
                 name,
                 state.channel);
        httpd_resp_sendstr_chunk(req, metrics_buff);
    } else {
        uint8_t chan;
        esp_wifi_get_channel(&chan, NULL);

        snprintf(metrics_buff, sizeof metrics_buff - 1,
                 "# TYPE espnow_receiver_channel gauge\n"
                 "espnow_receiver_channel{receiver=\"%s\"} %u\n",
                 name,
                 chan);
        httpd_resp_sendstr_chunk(req, metrics_buff);
    }

    // Indicate no more chunks.
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static const httpd_uri_t metrics = {
        .uri       = "/metrics",
        .method    = HTTP_GET,
        .handler   = metrics_get_handler,
};

// /home/paul/.platformio/penv/bin/pio run -e evo-eth01 && curl -v  -H "Content-Type: application/octet-stream" --data-binary "@.pio/build/evo-eth01/firmware.bin" http://192.168.0.96/ota
static esp_err_t ota_upload_handler(httpd_req_t *req) {
    const size_t BUFF_SIZE = 4096;
    long max_image_size = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    uint8_t *buffer = (uint8_t *)(malloc(BUFF_SIZE));
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    size_t remaining = req->content_len;
    Serial.printf("Size of image: %ld\r\n", (long)remaining);
    if (!Update.begin(remaining)) {
        free(buffer);
        return ESP_FAIL;
    }

    int count = 0;
    while (remaining > 0) {
        int read = httpd_req_recv(req, (char *)buffer, BUFF_SIZE);
        //Serial.printf("Read: %ld\r\n", (long)read);

        if (read <= 0) {
            Update.abort();
            free(buffer);
            return read == 0 ? ESP_FAIL : read;
        }

        if (Update.write(buffer, read) != read) {
            Update.printError(Serial);
            Update.abort();
            free(buffer);
            return ESP_FAIL;
        }

        remaining -= read;
        Serial.print('.');
        count++;

        if ((count % 80) == 0) {
            Serial.println();
        }
//        Serial.printf("Remaining: %ld\r\n", (long)remaining);
    }

    free(buffer);
    Update.end(true);
    Serial.println();
    String msg("Upload MD5: ");
    msg.concat(Update.md5String());
    msg.concat("\r\nRebooting.....\r\n");
    httpd_resp_sendstr(req, msg.c_str());
    delay(500);
    ESP.restart();


    return ESP_OK;
}

static const httpd_uri_t ota = {
        .uri       = "/ota",
        .method    = HTTP_POST,
        .handler   = ota_upload_handler,
};

static esp_err_t reboot_handler(httpd_req_t *req) {
    httpd_resp_sendstr(req, "Rebooting...\r\n");
    delay(500);
    ESP.restart();
    return ESP_OK;
}

static const httpd_uri_t reboot = {
        .uri       = "/reboot",
        .method    = HTTP_POST,
        .handler   = reboot_handler,
};

static esp_err_t info_handler(httpd_req_t *req) {
    httpd_resp_sendstr(req, get_info());
    return ESP_OK;
}

static const httpd_uri_t info = {
        .uri       = "/info",
        .method    = HTTP_GET,
        .handler   = info_handler,
};

static httpd_handle_t start_webserver()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &metrics);
        httpd_register_uri_handler(server, &ota);
        httpd_register_uri_handler(server, &reboot);
        httpd_register_uri_handler(server, &info);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void led_task() {
#ifdef LED_PIN
    static bool initialised = false;
    if (!initialised) {
        pinMode(LED_PIN, OUTPUT);
    }

    digitalWrite(LED_PIN, state.state == state_t::waiting_for_esp_now ? HIGH : LOW);
#endif
}

static void init_etag() {
    memset(etag, '\0', sizeof etag);

    File file = LittleFS.open("/bin-etag", "r");
    if (file) {
        int etag_size = file.read(etag, sizeof etag);
        Serial.printf("Etag read(%d): %s\n", etag_size, etag);
    }
    file.close();
}

static void check_updates() {
    // Query for any newer firmware.
    char url[128];

    HTTPClient http;

    // First check for host-specific firmware.
    snprintf(url, sizeof url, "http://192.168.0.2:18080/%s", host);
    int ret = httpGet(http, url, reinterpret_cast<const char *>(etag));
    if (ret == 404) {
        // No host-specific. Try generic instead.
        http.end();

        // The URL we query is /appname/chipmodel. For ESP8266 I can't see any way to get the chip model.
        // We could also vary it depending on size of flash.
#ifdef ESP8266
        snprintf(url, sizeof url, "http://192.168.0.2:18080/%s/esp8266", APP);
#else
        snprintf(url, sizeof url, "http://192.168.0.2:18080/%s/%s", APP, ESP.getChipModel());
#endif

        ret = httpGet(http, url, reinterpret_cast<const char *>(etag));
    }

    String returned_etag = http.header("Etag");
    Serial.printf("Etag returned: %s\n", returned_etag.c_str());
    Serial.printf("Last Modified: %s\n", http.header("Last-Modified").c_str());
    Serial.printf("Content-Length: %d\n", http.getSize());

    if (ret != 200 || http.getSize() <= 0) {
        // Expected statuses:
        //  304 Not Modified
        //  404 Not Found
        // No matter if it is an expected or unexpected failure, the action is the
        // same: abandon updating. We also need to know the size of the file to
        // do the update (chunked encoding is not supported).

        http.end();
        return;
    }

    Serial.printf("Starting update from %s\n", url);
    WiFiClient stream = http.getStream();
    if (!Update.begin(http.getSize())) {
        Serial.println("Could not update\n");
        http.end();
        return;
    }

    size_t written = Update.writeStream(http.getStream());
    Serial.printf("Written %d, expect remaining to be zero: %d\n", written, Update.remaining());
    Update.end();
    Serial.printf("Update %s: status = %d\n", Update.hasError() ? "FAILED": "OK", Update.getError());

    http.end();

    if (Update.hasError()) {
        // No need to reboot.
        return;
    }

    if (!returned_etag.isEmpty()) {
        // We don't appear to be able to open the file as read-write, hence why we close and re-open.
        File file = LittleFS.open("/bin-etag", "w");
        size_t written = file.write((uint8_t *)returned_etag.c_str(), returned_etag.length());
        Serial.printf("Saved etag. written=%d out of %d for %s\n", written,returned_etag.length(), returned_etag.c_str());
        file.close();
    }

    LittleFS.end();
    Serial.printf("MD5: %s\n", Update.md5String().c_str());
    Serial.println("Restarting....");
    delay(250);
    ESP.restart();
}

static int httpGet(HTTPClient &http, char *url, const char *etag) {
    http.begin(wifi_client, url);

    // Adding a Host header won't work. See https://stackoverflow.com/a/68714543/683825. Also,
    // VM won't allow DNS responses that resolve to local addresses. WTF?
    // Instead, I have configured Caddy to serve firmware on an additional port: 18080
    // http.addHeader("Host", "firmware.paulcager.org");

    if (etag[0] != '\0') {
        http.addHeader("If-None-Match", etag);
    }

    // Configure http to save any interesting headers.
    const char *headerKeys[] = {"Etag", "Last-Modified"};
    http.collectHeaders(headerKeys, (sizeof(headerKeys)) / (sizeof(char *)));

    int ret = http.GET();
    Serial.printf("GET %s: %d\n", url, ret);
    return ret;
}
