#include <Arduino.h>

#include "version.h"

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#include "../../secrets.h"

#ifdef RECEIVER
#include <WiFi.h>
#include <PubSubClient.h>
#include "ESPAsyncWebServer.h"

#define PROM_NAMESPACE "esp_sensors"
static void handle_http_metrics_client(AsyncWebServerRequest* request);

AsyncWebServer server(80);
#endif

#ifdef SAMD21
#include "ArduinoLowPower.h"
#include <RTCZero.h>
#include "nRF24L01.h"
RTCZero rtc;
#endif

#define CHAN 80 // Near the upper limit of channel 13

#if defined(XIAO_ESP32S3)
#define CE_PIN 4
#define CSN_PIN 5
#elif defined(ESP32S3)
#define CE_PIN 4
#define CSN_PIN 5
#elif defined(XIAO_ESP32C6)
#define CE_PIN 21
#define CSN_PIN 22
#define MOSI 18
#define MISO 20
#define SCK 19
#elif defined(ESP32)
#define CE_PIN 4
#define CSN_PIN 5
#elif ESP8266
#define CE_PIN 4
#define CSN_PIN 5
#else
#define CE_PIN D0
#define CSN_PIN D1
#endif

#ifndef BOARD_LED
#define BOARD_LED LED_BUILTIN
#endif



static void led_toggle();

RF24 radio(CE_PIN, CSN_PIN);

uint8_t address[6] = {0x6f, 0xca, 0x40, 0x07, 0x23, 0x08};
struct {
    uint16_t seq;
    uint16_t successes_total;
    uint16_t retries_total;
    uint16_t failures_total;
    uint32_t send_time_micros_total;
} payload;

#ifdef SAMD21
// For radio.printPrettyDetails
int printf(const char *format, ...) {
    size_t r;
    char buf[PRINTF_BUF];
    va_list ap;
    va_start(ap, format);
    r = vsnprintf(buf, sizeof(buf), format, ap);
    Serial.write(buf);
    va_end(ap);
    return r;
}
#endif

void setup() {
    Serial.begin(115200);
//    Serial0.begin(115200); Serial1.begin(115200); delay(2000);
//    Serial0.println("Serial0");
//    Serial1.println("Serial1");
    // some boards need to wait to ensure access to serial over USB
    for (int i = 0; i < 10; i++) {
        if (Serial) {
            break;
        }
        delay(500);
    }

    delay(500);

    Serial.println();
    Serial.print("MOSI: ");
    Serial.println(MOSI);
    Serial.print("MISO: ");
    Serial.println(MISO);
    Serial.print("SCK: ");
    Serial.println(SCK);
    Serial.print("SS: ");
    Serial.println(SS);    
    Serial.print("CE: ");
    Serial.println(CE_PIN);
    Serial.print("CSN: ");
    Serial.println(CSN_PIN);
    Serial.print("CHAN: ");
    Serial.println(CHAN);

    pinMode(CE_PIN, OUTPUT);
    pinMode(CSN_PIN, OUTPUT);

    pinMode(BOARD_LED, OUTPUT);

#ifdef ESP32
    SPI.begin(SCK, MISO, MOSI, SS);
#endif

    // initialize the transceiver on the SPI bus
    if (!radio.begin()) {
        Serial.print(millis());

        Serial.println(F(": radio hardware is not responding!!"));
        digitalWrite(BOARD_LED, LOW);
        delay(2000);
        digitalWrite(BOARD_LED, HIGH);
        while (1) {}  // hold in infinite loop
    }

    radio.setChannel(CHAN);
    radio.setPALevel(RF24_PA_HIGH);  // RF24_PA_MAX is default.

    radio.setPayloadSize(sizeof(payload));
    Serial.printf("CRC length: %d\n", radio.getCRCLength());

#ifdef SENDER
    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address);  // always uses pipe 0
    radio.stopListening();  // put radio in TX mode
#else
    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address);  // using pipe 1
    radio.startListening();  // put radio in RX mode

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        led_toggle();
        delay(250);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server.on("/metrics", HTTP_GET, handle_http_metrics_client);
    server.begin();
#endif

    radio.printPrettyDetails();
    Serial.println();
}

void loop() {
#ifdef SENDER
    radio.powerDown();

#ifdef SAMD21
    // For some reason, SAMD21 only seem to sleep for 47 millis at a time.
    unsigned long sleepEnd = millis() + 1000;
    for (long remaining = 1000; remaining > 0; remaining = (long)(sleepEnd - millis())) {
        unsigned long x = millis();
        LowPower.idle((unsigned long)remaining);
        unsigned long slept = millis() - x;
        Serial.print("LowPower.idle: tried to sleep ");
        Serial.print(remaining);
        Serial.print(" millis, slept ");
        Serial.println(slept);
    }
    radio.powerUp();
#else
    delay(1000);
#endif

    bool ok = false;
    unsigned long retry_delay = 8;
    unsigned long start = micros();
    uint16_t retries = 0;
    for (int i = 0; i < 3; i++) {
        ok = radio.write(&payload, sizeof(payload));
        retries += radio.getARC();
        if (ok) break;

        delay(retry_delay);
        retry_delay *= 2;
    }
    unsigned long end = micros();

    payload.seq++;
    payload.send_time_micros_total += (end - start);
    payload.retries_total += retries;
    if (ok) {
        led_toggle();
        payload.successes_total++;
        static char digits[] = ".:23456789abcdefghijklmnopqrstuvwxyz";
        Serial.print(retries >= sizeof(digits) - 1 ? 'X' : digits[retries]);
    } else {
        payload.failures_total++;
        Serial.print('X');
    }
    if ((payload.seq & 0x7f) == 0) {
        Serial.println();
    }

#else
    if (radio.available()) {
        radio.read(&payload, sizeof(payload));
        led_toggle();

        Serial.printf("%8lu %4d: %d %d %d %d\n",
                      millis(),
                      payload.seq, payload.successes_total, payload.retries_total,
                      payload.failures_total, payload.send_time_micros_total);
    }
#endif
}  // loop


static void led_toggle() {
    static uint8_t level = LOW;
    level = (level + 1) & 0x01;
    digitalWrite(BOARD_LED, level);
}

#ifdef RECEIVER
static void handle_http_metrics_client(AsyncWebServerRequest* request) {
    AsyncResponseStream *response = request->beginResponseStream("text/plain");
    response->printf(
            "# HELP " PROM_NAMESPACE "_seq receive sequence number.\n"
            "# TYPE " PROM_NAMESPACE "_seq counter\n"
            PROM_NAMESPACE "_seq %d\n",
            payload.seq
            );

    response->printf(
            "# HELP " PROM_NAMESPACE "_success_total Successes.\n"
            "# TYPE " PROM_NAMESPACE "_success_total counter\n"
            PROM_NAMESPACE "_success_total %d\n",
            payload.successes_total
            );

    response->printf(
            "# HELP " PROM_NAMESPACE "_retries_total Retries.\n"
            "# TYPE " PROM_NAMESPACE "_retries_total counter\n"
            PROM_NAMESPACE "_retries_total %d\n",
            payload.retries_total
            );

    response->printf(
            "# HELP " PROM_NAMESPACE "_failures_total failures.\n"
            "# TYPE " PROM_NAMESPACE "_failures_total counter\n"
            PROM_NAMESPACE "_failures_total %d\n",
            payload.failures_total
            );

    response->printf(
            "# HELP " PROM_NAMESPACE "_send_time_micros_total Send time.\n"
            "# TYPE " PROM_NAMESPACE "_send_time_micros_total counter\n"
            PROM_NAMESPACE "_send_time_micros_total %ld\n",
            (unsigned long)payload.send_time_micros_total
            );

    request->send(response);
}

#endif