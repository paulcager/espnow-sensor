#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

static void toggle_led();

// Define the broadcast MAC address
uint8_t broadcastAddress[] = {0x7d, 0xDF, 0xA1, 0x86, 0x96, 0xF8};

int counter = 0;

static const int wifi_channel = 10;

// Create a peer information structure
esp_now_peer_info_t peerInfo;

// Variable to store the send status
esp_now_send_status_t sendStatus;

// Callback function executed when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);
    Serial.println("ESP32 ESP-NOW Broadcast Pinger");
    Serial.print("LED_BUILTIN=");Serial.println(LED_BUILTIN);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    WiFi.channel(wifi_channel);
    esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);
    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return; // Stop execution if initialization fails
    }

    Serial.println("ESP-NOW Initialized Successfully");

    // Register the send callback function
    esp_now_register_send_cb(OnDataSent);

    // Configure peer information for broadcasting
    memcpy(peerInfo.peer_addr, broadcastAddress, 6); // Set peer MAC to broadcast address
    peerInfo.channel = wifi_channel;
    peerInfo.encrypt = false; // Disable encryption for this example

    // Add the broadcast peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
        return; // Stop execution if adding peer fails
    } else {
        Serial.println("Broadcast Peer Added Successfully");
    }

    toggle_led();
}

void loop() {
    static char buff[128];
    size_t len = snprintf(buff, sizeof(buff), "{\"seq\": %d}", counter++);

    // Send the message via ESP-NOW to the broadcast address
    esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)buff, len);

    // Optional: Check the immediate result of the send function (doesn't guarantee delivery)
    // The callback function (OnDataSent) provides more reliable status feedback
    if (result == ESP_OK) {
        // Serial.println("Send command success (waiting for callback status)");
    } else {
        Serial.print("Error sending data immediately: ");
        Serial.println(result); // Print ESP error code if send fails immediately
    }

    // Wait before sending the next ping
    delay(2000); // Send a ping every 2 seconds
    toggle_led();
}

static void toggle_led() {
    static int state = 0;
    static bool initialized = false;

    if (!initialized) {
        initialized = true;
        pinMode(LED_BUILTIN, OUTPUT);
    }

    state = (state+1) & 0x01;
    digitalWrite(LED_BUILTIN, state);
}