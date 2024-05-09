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

static DHT dht(4, DHT22);

//uint8_t receiver_mac[6] = {0x34, 0x94, 0x54, 0x24, 0x95, 0xc8};     // esp32-receiver
//uint8_t receiver_mac[6] = {0x30, 0x30, 0xf9, 0x18, 0x14, 0xf8};     // seeed esp32-s3 (bad?)
uint8_t receiver_mac[6] = {0x84, 0xfc, 0xe6, 0x78, 0x23, 0xec};     // seeed esp32-s3 number 2
//uint8_t receiver_mac[6] = {0xdc, 0xa6, 0x32, 0xcc, 0xa6, 0x4d};     // pi wlan0
//uint8_t receiver_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
    uint32_t sequence;
    uint32_t send_success;
    uint32_t send_fail;
    unsigned long time_millis;
    uint32_t last_send_time_millis;
} payload_t;

static payload_t payload;

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
    print_payload();

    payload.last_send_time_millis = millis() - payload.time_millis;
    payload.sequence++;

    if (status == ESP_NOW_SEND_SUCCESS) {
        payload.send_success++;
    }
    else {
        payload.send_fail++;
    }

    ESP.rtcUserMemoryWrite(17, (uint32_t *)&payload, sizeof(payload_t));

    uint32_t sleep_time_secs;
    if (status == ESP_NOW_SEND_SUCCESS) {
        sleep_time_secs = 15 * 60;
    } else {
        // Bring the next time forwards, but delay by a random amount to prevent collisions.
        sleep_time_secs = random(1 * 60, 2 * 60);
    }

    Serial.print("Up: "); Serial.print(millis()); Serial.print(" millis. Sleeping "); Serial.print(sleep_time_secs); Serial.println(" secs");
    Serial.flush();
    digitalWrite(LED_BUILTIN, LOW);

    // Instant does not wait for WiFi chip to idle.
    ESP.deepSleepInstant(sleep_time_secs * 1000000);
    Serial.println("Woops!");
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    Serial.println();

    if (ESP.getResetInfoPtr()->reason == REASON_DEEP_SLEEP_AWAKE) {
        ESP.rtcUserMemoryRead(17, (uint32_t *)&payload, sizeof(payload_t));
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
    esp_now_add_peer(receiver_mac, ESP_NOW_ROLE_SLAVE, 10, NULL, 0);
#endif

    float h;
    float t;

    // On first power on the DHT might not be ready).
    // NB - this only applies to power-on; it should be OK after deep sleep.
    for (int i = 0; i < (ESP.getResetInfoPtr()->reason == REASON_DEEP_SLEEP_AWAKE ? 2 : 10); i++) {
        h = dht.readHumidity(true);
        t = dht.readTemperature();

        Serial.print("h=");
        Serial.print(h);
        Serial.print(", t=");
        Serial.println(t);

        if (!isnan(t) && !isnan(h)) {
            break;
        }
        delay(20);
    }

    send_message(h, t);
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
             "\"success\": %d, "
             "\"fail\": %d, "
             "\"t\": %0.1f, "
             "\"h\": %0.1f "
             "}",
             payload.sequence,
             payload.send_success,
             payload.send_fail,
             temperature,
             humidity
    );
    esp_now_send(receiver_mac, (uint8_t *)buff, strlen(buff)+1);
}