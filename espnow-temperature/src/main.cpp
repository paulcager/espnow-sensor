#include <Arduino.h>

// Won't work: see https://www.dzombak.com/blog/2021/10/ESP8266-How-to-enable-debug-logging-for-Arduino-s-ESP8266HTTPClient-with-PlatformIO.html
//#define DEBUG_HTTPCLIENT(...) Serial.printf(__VA_ARGS__)
//#define DEBUG_WIFI_GENERIC(...) Serial.printf(__VA_ARGS__)

#include "version.h"

#ifdef ESP32
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#include <Update.h>
#include <HTTPClient.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "ESP8266HTTPClient.h"
#include <ESP8266httpUpdate.h>
#define esp_now_send_status_t uint8
#define ESP_NOW_SEND_SUCCESS 0
#define ERR_OK 0
#define ESP_OK 0
#endif

#include "LittleFS.h"
#define DHT_DEBUG true
#include <DHT.h>
#include <SHT2x.h>
#include "../../secrets.h"

#define PRINT_TIME(X) { Serial.print(X ": "); Serial.println(millis()); }

static DHT dht(4, DHT22);
static SHT2x sht;

#define RTC_MEMORY_ADDR 17

uint8_t receiver_macs[][6] = {
#ifdef MULTICAST_ADDRESS
        MULTICAST_ADDRESS,       // Multicast from secrets.h
#else
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // Broadcast.
#endif
      //{0x54, 0x32, 0x04, 0x47, 0xBD, 0x38}, // Second evo-eth01 (ESP32-C3)
      //{0x84, 0xfc, 0xe6, 0x78, 0x23, 0xec},       // seeed esp32-s3 number 2
      //{0x30, 0xc9, 0x22, 0xef, 0x4a, 0x14},       // WiFi MAC on ESP32 Ethernet (w headers)
      //{0x78, 0x42, 0x1c, 0x6a, 0x2d, 0x14},       // WiFi MAC on ESP32 Ethernet (w headers)
      //{0x30, 0xc9, 0x22, 0xef, 0x42, 0xc4},       // WiFi MAC on ESP32 Ethernet (directly soldered).
};
//uint8_t receiver_mac[6] = {0xdc, 0xa6, 0x32, 0xcc, 0xa6, 0x4d};     // pi wlan0
//uint8_t receiver_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#define NUM_RECEIVERS (sizeof(receiver_macs)/6)

#ifdef ESP32
static esp_now_peer_info_t peerInfo[NUM_RECEIVERS];
#endif


typedef struct {
    uint32_t sequence;
    int32_t restarts_before_update_required;   // Number of startups allowed before we should call check_updates.
    uint32_t send_success;
    uint32_t send_fail;
    unsigned long time_millis;
    uint32_t last_send_time_millis;
    uint8_t last_proxy_mac[6];
} payload_t;

static const int SUCCESS_SLEEP_SECONDS = 15 * 60;
static const int FAIL_SLEEP_SECONDS = 2 * 60;

#ifdef ESP32
RTC_DATA_ATTR
#endif
static payload_t payload;

static int receives_outstanding;
static int successful_sends;


static void send_message(bool data_unavailable, float humidity, float temperature);

static void set_wifi_channel(uint channel);

void check_updates() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    for (int i = 0; i < 40; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            break;
        }
        delay(500);
        Serial.print('.');
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("Could not connect to WiFi in order to check for updates: %d\n", WiFi.status());
        WiFi.disconnect();
        return;
    }

    Serial.printf("\nAssigned IP address: %s\n", WiFi.localIP().toString().c_str());

    // Read etag (if any) of last applied update.
#ifdef ESP8266
    LittleFSConfig fsconfig(true);
    LittleFS.setConfig(fsconfig);
    LittleFS.begin();
#else
    LittleFS.begin(true);
#endif
    File file = LittleFS.open("/bin-etag", "r");
    uint8_t etag[128];
    memset(etag, '\0', sizeof etag);
    int etag_size = file.read(etag, sizeof etag);
    Serial.printf("Etag read(%d): %s\n", etag_size, etag);
    file.close();

    // Query for any newer firmware.
    // The URL we query is /appname/chipmodel. For ESP8266 I can't see any way to get the chip model.
    // We could also vary it depending on size of flash.

    // TODO - allow us to specify different versions for different instances, based on ChipID.
    char url[128];
#ifdef ESP8266
    snprintf(url, sizeof url, "http://192.168.0.2:18080/%s/esp8266", APP);
#else
    snprintf(url, sizeof url, "http://192.168.0.2:18080/%s/%s", APP, ESP.getChipModel());
#endif

    WiFiClient c;
    HTTPClient http;
    http.begin(c, url);

    // Adding a Host header won't work. See https://stackoverflow.com/a/68714543/683825. Also,
    // VM won't allow DNS responses that resolve to local addresses. WTF?
    // Instead, I have configured Caddy to serve firmware on an additional port: 18080
    // http.addHeader("Host", "firmware.paulcager.org");

    if (etag_size > 0) {
        etag[etag_size] = '\0';
        http.addHeader("If-None-Match", (const char *)etag);
    }

    // Configure http to save any interesting headers.
    const char *headerKeys[] = {"Etag", "Last-Modified"};
    http.collectHeaders(headerKeys, (sizeof(headerKeys)) / (sizeof(char *)));

    int ret = http.GET();
    Serial.printf("GET %s: %d\n", url, ret);

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
        WiFi.disconnect();
        LittleFS.end();
        return;
    }

    WiFiClient stream = http.getStream();
    if (!Update.begin(http.getSize())) {
        Serial.println("Count not update\n");
        http.end();
        WiFi.disconnect();
        LittleFS.end();
        return;
    }

    size_t written = Update.writeStream(http.getStream());
    Serial.printf("Written %d, expect remaining to be zero: %d\n", written, Update.remaining());
    Update.end();
    Serial.printf("Update status = %d\n", Update.getError());

    http.end();
    WiFi.disconnect();

    if (Update.hasError()) {
        LittleFS.end();
        // No need to reboot.
        return;
    }

    if (!returned_etag.isEmpty() && !Update.hasError()) {
        // We don't appear to be able to open the file as read-write, hence why we close and re-open.
        File file = LittleFS.open("/bin-etag", "w");
        size_t written = file.write((uint8_t *)returned_etag.c_str(), returned_etag.length());
        Serial.printf("Saved etag. written=%d out of %d for %s\n", written,returned_etag.length(), returned_etag.c_str());
        file.close();
    }

    LittleFS.end();
    Serial.printf("MD5: %s\n", Update.md5String().c_str());
    delay(250);
    ESP.restart();
}

#ifdef ESP32
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#else
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
#endif
    Serial.printf("OnDataSent: %02X%02X%02X%02X%02X%02X - %d\r\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], status);

    receives_outstanding--;

    if (status == ESP_NOW_SEND_SUCCESS) {
        successful_sends++;
        memcpy(payload.last_proxy_mac, mac_addr, sizeof(payload.last_proxy_mac));
    }

    if (receives_outstanding > 0) {
        return;
    }

    if (successful_sends > 0) {
        payload.send_success++;
    } else {
        payload.send_fail++;
    }

    payload.last_send_time_millis = millis() - payload.time_millis;
    payload.sequence++;

#ifdef ESP8266
    ESP.rtcUserMemoryWrite(RTC_MEMORY_ADDR, (uint32_t *)&payload, sizeof(payload_t));
#endif

    uint64_t sleep_time_secs;
    if (successful_sends) {
        sleep_time_secs = SUCCESS_SLEEP_SECONDS;
    } else {
        // Bring the next time forwards, but delay by a random amount to prevent collisions.
        sleep_time_secs = random(FAIL_SLEEP_SECONDS/2, FAIL_SLEEP_SECONDS*3/2);
    }

    Serial.print("Status: "); Serial.print(status);
    Serial.print(", Up: "); Serial.print(millis());
    Serial.print(" millis. Sleeping ");
    Serial.print(sleep_time_secs); Serial.println(" secs");
    Serial.flush();
#ifdef LED_BUILTIN
    digitalWrite(LED_BUILTIN, LOW);
#endif

#ifdef ESP32
    esp_sleep_enable_timer_wakeup(sleep_time_secs * 1000L * 1000L);
    esp_deep_sleep_start();
#else
    // Instant does not wait for WiFi chip to idle.
    ESP.deepSleepInstant(sleep_time_secs * 1000000L);
#endif
}

void setup() {
    // Timing notes:
    //  It usually takes about 72ms from boot to get to start of setup (ESP12F).
    //  WiFi.mode(WIFI_STA); then takes about 100ms.
    //  read DHT takes about 10ms.
    //  send/ack espnow takes a couple of ms (or up to about 15ms if there is no receiver to ack).

#ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
#endif

    Serial.begin(115200);
    Serial.println();

#ifdef ESP8266
    uint32 reset_reason = ESP.getResetInfoPtr()->reason;
    bool deep_sleep_awake = reset_reason == REASON_DEEP_SLEEP_AWAKE;
    uint wakeup_cause = 0;
#else
    esp_reset_reason_t reset_reason = esp_reset_reason();
    bool deep_sleep_awake = reset_reason == ESP_RST_DEEPSLEEP;
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
#endif

    Serial.println(APP_BANNER);
    Serial.print("Reset reason: ");
    Serial.print(reset_reason);
    Serial.print(", cause: ");
    Serial.println(wakeup_cause);

    if (deep_sleep_awake) {
#ifdef ESP8266
        ESP.rtcUserMemoryRead(RTC_MEMORY_ADDR, (uint32_t *)&payload, sizeof(payload_t));
#endif
    } else {
        memset(&payload, 0, sizeof(payload_t));
    }

    if (payload.restarts_before_update_required <= 0) {
        check_updates();
        WiFi.disconnect();
        set_wifi_channel(WIFI_CHANNEL);
        payload.restarts_before_update_required = 16;
    } else {
        payload.restarts_before_update_required--;
    }

    dht.begin();

    Wire.begin();
    bool sht_available = sht.begin();

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    set_wifi_channel(WIFI_CHANNEL);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(OnDataSent);

#ifdef ESP32
    memcpy(peerInfo.peer_addr, receiver_mac, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
#else
    for (int i = 0; i < NUM_RECEIVERS; i++) {
        esp_now_add_peer(receiver_macs[i], ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
    }
#endif

    float h;
    float t;

    // On first power on the DHT might not be ready.
    // NB - this only applies to power-on; it should be OK after deep sleep.
    for (int i = 0; i < (deep_sleep_awake ? 2 : 10); i++) {
        h = dht.readHumidity(true);
        t = dht.readTemperature();

        if (!isnan(t) && !isnan(h)) {
            send_message(false, h, t);
            return;
        }

        if (sht_available) {
            if (sht.read()) {
                h = sht.getHumidity();
                t = sht.getTemperature();
                send_message(false, h, t);
                return;
            }
        }

#if defined(ESP8266)
        yield(); // Handle WiFi / reset software watchdog
#endif

        delay(20);
    }

    send_message(true, 0, 0);
}

static void set_wifi_channel(uint channel) {
#ifdef ESP32
    esp_wifi_set_channel(channel ,WIFI_SECOND_CHAN_NONE);
#else
    wifi_set_channel(channel);
#endif
}

void loop() {
    // Waiting for esp_now_send to be acked.
    delay(1000);
}

static void send_message(bool data_unavailable, float humidity, float temperature) {
    payload.time_millis = millis();

    static char buff[256];
    if (data_unavailable) {
        snprintf(buff, sizeof buff,
                 "{ "
                 "\"seq\": %lu, "
                 "\"upd_grace\": %ld, "
                 "\"vers\": %s, "
                 "\"run\": %lu, "
                 "\"success\": %lu, "
                 "\"fail\": %lu"
                 "}",
                 (unsigned long)payload.sequence,
                 (unsigned long)payload.restarts_before_update_required,
                 VERSION_COMMIT_HASH,
                 millis(),
                 (unsigned long)payload.send_success,
                 (unsigned long)payload.send_fail);
    } else {
        snprintf(buff, sizeof buff,
                 "{ "
                 "\"seq\": %lu, "
                 "\"upd_grace\": %ld, "
                 "\"run\": %lu, "
                 "\"success\": %lu, "
                 "\"fail\": %lu, "
                 "\"t\": %0.1f, "
                 "\"h\": %0.1f"
                 "}",
                 (unsigned long)payload.sequence,
                 (unsigned long)payload.restarts_before_update_required,
                 millis(),
                 (unsigned long)payload.send_success,
                 (unsigned long)payload.send_fail,
                 temperature,
                 humidity
        );
    }
    printf("buff %s\r\n", buff);
    receives_outstanding = NUM_RECEIVERS;
    successful_sends = 0;
    memset(payload.last_proxy_mac, 0xff, sizeof(payload.last_proxy_mac));

    size_t size = strlen(buff) + 1;
    for (int i = 0; i < NUM_RECEIVERS; i++) {
        uint8_t *mac_addr = receiver_macs[i];
        int ret = esp_now_send(mac_addr, (uint8_t *) buff, size);
        Serial.printf("esp_now_send: %02X%02X%02X%02X%02X%02X - %d\r\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], ret);

#if defined(ESP8266)
        yield(); // Handle WiFi / reset software watchdog
        // TODO TEMP just so we can see if they all get received.
        delay(1000);
#endif
    }
}