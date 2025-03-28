#include <Arduino.h>

#ifdef ESP32
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#define esp_now_send_status_t uint8
#define ESP_NOW_SEND_SUCCESS 0
#define ERR_OK 0
#define ESP_OK 0
#endif

#define DHT_DEBUG true
#include <DHT.h>

#define PRINT_TIME(X) { Serial.print(X ": "); Serial.println(millis()); }

static DHT dht(4, DHT22);

#define RTC_MEMORY_ADDR 17

#define NUM_RECEIVERS 2
uint8_t receiver_macs[NUM_RECEIVERS][6] = {
      //{0x30, 0xc9, 0x22, 0xef, 0x4a, 0x14},       // WiFi MAC on ESP32 Ethernet (w headers)
        {0x78, 0x42, 0x1c, 0x6a, 0x2d, 0x14},       // WiFi MAC on ESP32 Ethernet (w headers)
        {0x30, 0xc9, 0x22, 0xef, 0x42, 0xc4},       // WiFi MAC on ESP32 Ethernet (directly soldered).
      //{0x84, 0xfc, 0xe6, 0x78, 0x23, 0xec},       // seeed esp32-s3 number 2
};
//uint8_t receiver_mac[6] = {0x34, 0x94, 0x54, 0x24, 0x95, 0xc8};     // esp32-receiver
//uint8_t receiver_mac[6] = {0x30, 0x30, 0xf9, 0x18, 0x14, 0xf8};     // seeed esp32-s3 number 1
//uint8_t receiver_mac[6] = {0x84, 0xfc, 0xe6, 0x78, 0x23, 0xec};     // seeed esp32-s3 number 2
//uint8_t receiver_mac[6] = {0xdc, 0xa6, 0x32, 0xcc, 0xa6, 0x4d};     // pi wlan0
//uint8_t receiver_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
    uint32_t sequence;
    uint32_t send_success;
    uint32_t send_fail;
    unsigned long time_millis;
    uint32_t last_send_time_millis;
    uint8_t  last_proxy_mac[6];
} payload_t;

static const int SUCCESS_SLEEP_SECONDS = 15 * 60;
static const int FAIL_SLEEP_SECONDS = 1 * 60;
static payload_t payload;

static int receives_outstanding;
static int successful_sends;

static void send_message(float humidity, float temperature);

#ifdef ESP32
static esp_now_peer_info_t peerInfo;
#endif

static void print_payload() {

    Serial.print(millis());
    Serial.print("\t");
    Serial.print(payload.time_millis);
    Serial.print("\t");
    Serial.print(payload.sequence);
    Serial.print("\t");
    Serial.print(payload.send_success);
    Serial.print("\t");
    Serial.print(payload.send_fail);
    Serial.print("\t");
    Serial.print(millis() - payload.time_millis);
    Serial.println();

}

#ifdef ESP32
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#else
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
#endif
    if (false) print_payload();

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

    ESP.rtcUserMemoryWrite(RTC_MEMORY_ADDR, (uint32_t *)&payload, sizeof(payload_t));

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
    digitalWrite(LED_BUILTIN, LOW);

    // Instant does not wait for WiFi chip to idle.
    ESP.deepSleepInstant(sleep_time_secs * 1000000L);
}

void setup() {
    // Timing notes:
    //  It usually takes about 72ms from boot to get to start of setup (ESP12F).
    //  WiFi.mode(WIFI_STA); then takes about 100ms.
    //  read DHT takes about 10ms.
    //  send/ack espnow takes a couple of ms (or up to about 15ms if there is no receiver to ack).

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
//    delay(2000);
//    Serial.println("Yes, serial is working.");

    if (ESP.getResetInfoPtr()->reason == REASON_DEEP_SLEEP_AWAKE) {
        ESP.rtcUserMemoryRead(RTC_MEMORY_ADDR, (uint32_t *)&payload, sizeof(payload_t));
    } else {
        memset(&payload, 0, sizeof(payload_t));
    }

    dht.begin();

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

#ifdef ESP32
    esp_wifi_set_channel(10 ,WIFI_SECOND_CHAN_NONE);
#else
    wifi_set_channel(10);
#endif

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(OnDataSent);

#ifdef ESP32
    memcpy(peerInfo.peer_addr, receiver_mac, 6);
    peerInfo.channel = 10;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
#else
    for (int i = 0; i < NUM_RECEIVERS; i++) {
        esp_now_add_peer(receiver_macs[i], ESP_NOW_ROLE_SLAVE, 10, NULL, 0);
    }
#endif

    float h;
    float t;

    // On first power on the DHT might not be ready.
    // NB - this only applies to power-on; it should be OK after deep sleep.
    for (int i = 0; i < (ESP.getResetInfoPtr()->reason == REASON_DEEP_SLEEP_AWAKE ? 2 : 10); i++) {
        h = dht.readHumidity(true);
        t = dht.readTemperature();

        Serial.print("h=");
        Serial.print(h);
        Serial.print(", t=");
        Serial.println(t);

        if (!isnan(t) && !isnan(h)) {
            send_message(h, t);
            return;
        }

#if defined(ESP8266)
        yield(); // Handle WiFi / reset software watchdog
#endif

        delay(20);
    }

    // Could not read the sensor, so no message has been sent. Sleep and try later.
    ESP.deepSleepInstant(120 * 1000000L);
}

void loop() {
    // Waiting for esp_now_send to be acked.
    delay(1000);
}

static void send_message(float humidity, float temperature) {
    payload.time_millis = millis();

    static char buff[256];
    snprintf(buff, sizeof buff,
             "{ "
             "\"seq\": %d, "
             "\"run\": %lu, "
             "\"success\": %d, "
             "\"fail\": %d, "
             "\"t\": %0.1f, "
             "\"h\": %0.1f"
             "}",
             payload.sequence,
             millis(),
             payload.send_success,
             payload.send_fail,
             temperature,
             humidity
    );
    printf("buff %s\r\n", buff);
    receives_outstanding = NUM_RECEIVERS;
    successful_sends = 0;
    memset(payload.last_proxy_mac, 0xff, sizeof(payload.last_proxy_mac));

    int size = strlen(buff) + 1;
    for (int i = 0; i < NUM_RECEIVERS; i++) {
        esp_now_send(receiver_macs[i], (uint8_t *) buff, size);

#if defined(ESP8266)
        yield(); // Handle WiFi / reset software watchdog
#endif
    }
}