#include <Arduino.h>
#include <PubSubClient.h>

// --- Konfigurasi Kamera ---
#include "esp_camera.h"
#include "board_config.h"
#include "camera_pins.h"

// --- Konfigurasi Power ---
#include "DFRobot_AXP313A.h"
DFRobot_AXP313A axp;

// --- Konfigurasi Display ---
#include <SPI.h>
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();
#include <TJpg_Decoder.h>

// ====================================================================
// --- Konfigurasi Keamanan ---
// true = Menggunakan SSL (Port 8884)
// false = Menggunakan Non-SSL (Port 1883)
// ====================================================================
#define USE_MQTT_SSL false
// ====================================================================

// --- Konfigurasi Wi-Fi ---
#include <WiFi.h>
const char *WIFI_SSID = "SUHARNO";
const char *WIFI_PASSWORD = "Suharno090970";

// --- Konfigurasi MQTT ---
const char *MQTT_USER = "mosyrf";
const char *MQTT_PASSWORD = "mosyrfMQTT";
const char *MQTT_SERVER = "broker.avisha.id";

// Topik
const char *MQTT_TOPIC_PUB = "mosyrf/camera"; // Upload Gambar Mentah
const char *MQTT_TOPIC_SUB = "mosyrf/result"; // Download Gambar Hasil FOMO
const char *MQTT_CLIENT_ID_BASE = "ESP32_Camera_Stream";

// Buffer MQTT (Harus cukup besar untuk gambar balik)
const int MQTT_BUFFER_SIZE = 1024 * 30; // 30KB

// --- Variables untuk Logika Tombol ---
const int btnDetect = 13; // Tombol Deteksi FOMO
const int btnToggle = 47; // Tombol Toggle LED
const int led1 = 15;
const int led2 = 16; // LED 2 di Pin 16
bool ledStatus = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// --- Buffer Gambar Masuk (Dari Python) ---
uint8_t *incomingImageBuffer = NULL;
size_t incomingImageLen = 0;
bool newResultAvailable = false;

#if USE_MQTT_SSL
#include <WiFiClientSecure.h>
#include <time.h>

const int MQTT_PORT = 8884;
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200; // WIB (UTC+7)
const int daylightOffset_sec = 0;

// --- SERTIFIKAT CA ---
const char *root_ca = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDtTCCAp2gAwIBAgIUOKMAqMXFCssUwLZtEx+8/dj+d/AwDQYJKoZIhvcNAQEL
BQAwajELMAkGA1UEBhMCSUQxEDAOBgNVBAgMB0pha2FydGExEDAOBgNVBAcMB0ph
a2FydGExDTALBgNVBAoMBEVNUVgxDTALBgNVBAsMBE1RVFQxGTAXBgNVBAMMEGJy
b2tlci5hdmlzaGEuaWQwHhcNMjUxMTAyMTEzODM2WhcNMjYxMTAyMTEzODM2WjBq
MQswCQYDVQQGEwJJRDEQMA4GA1UECAwHSmFrYXJ0YTEQMA4GAk
-----END CERTIFICATE-----
)EOF"; // (Dipotong untuk ringkas, pastikan pakai sertifikat lengkap Anda)

WiFiClientSecure espClient;

#else
// --- Jika SSL TIDAK AKTIF ---
const int MQTT_PORT = 1883;
WiFiClient espClient;

#endif

PubSubClient client(espClient);

// --- Deklarasi Fungsi ---
void cameraInit();
void connectWifi();
void connectMQTT();
void grabImage();
void buttonConfig();
void display_jpeg_to_tft(uint8_t *jpeg_buf, size_t jpeg_len);
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap);
void mqttCallback(char *topic, byte *payload, unsigned int length);

#if USE_MQTT_SSL
void setTime();
#endif

// ====================================================================
// --- SETUP ---
// ====================================================================
void setup()
{
    Serial.begin(115200);

    // Power Init
    while (axp.begin() != 0)
    {
        Serial.println("AXP313A init error.");
        delay(1000);
    }
    axp.enableCameraPower(axp.eOV2640);
    Serial.println("AXP313A Power OK.");

    // Setup Pin
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    pinMode(btnDetect, INPUT_PULLUP);
    pinMode(btnToggle, INPUT_PULLUP);

    connectWifi();

    // --- Setup SSL ---
#if USE_MQTT_SSL
    Serial.println("Mode SSL: Menyinkronkan waktu...");
    setTime();
    Serial.println("Mode SSL: Mengatur Sertifikat CA...");
    espClient.setCACert(root_ca);
#else
    Serial.println("Mode Non-SSL: Melewatkan pengaturan SSL.");
#endif

    // Setup MQTT Client
    // Alokasi memori buffer gambar masuk
    incomingImageBuffer = (uint8_t *)malloc(MQTT_BUFFER_SIZE);

    client.setBufferSize(MQTT_BUFFER_SIZE);
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback); // Callback Penting!

    cameraInit();

    // --- Inisialisasi TFT ---
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    TJpgDec.setJpgScale(1);
    TJpgDec.setSwapBytes(true);
    TJpgDec.setCallback(tft_output);
    Serial.println("TFT Display Ready.");
}

// ====================================================================
// --- LOOP ---
// ====================================================================
void loop()
{
    // 1. Handle Tombol Toggle LED (Pin 47)
    buttonConfig();

    // 2. Cek Koneksi MQTT
    if (!client.connected())
    {
        connectMQTT();
    }
    client.loop(); // Wajib dipanggil untuk menjalankan Callback

    // 3. Ambil Gambar
    grabImage();
}

// ====================================================================
// --- FUNGSI UTAMA ---
// ====================================================================

// Callback saat menerima data dari Python (Topik mosyrf/result)
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    if (String(topic) == MQTT_TOPIC_SUB)
    {
        if (incomingImageBuffer != NULL && length < MQTT_BUFFER_SIZE)
        {
            memcpy(incomingImageBuffer, payload, length);
            incomingImageLen = length;
            newResultAvailable = true;
            // Serial.printf("Terima Gambar Hasil: %d bytes\n", length);
        }
    }
}

void grabImage()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb || fb->format != PIXFORMAT_JPEG)
    {
        Serial.println("Gagal ambil frame");
        if (fb)
            esp_camera_fb_return(fb);
        return;
    }

    // --- 1. Kirim ke MQTT (Selalu kirim agar Python bisa proses) ---
    if (client.connected() && fb->len < MQTT_BUFFER_SIZE)
    {
        client.publish(MQTT_TOPIC_PUB, (const uint8_t *)fb->buf, fb->len);
    }

    // --- 2. Logika Display TFT ---
    // Jika Tombol 13 Ditekan (LOW) -> Tampilkan hasil olahan Python
    if (digitalRead(btnDetect) == LOW)
    {
        if (newResultAvailable && incomingImageLen > 0)
        {
            display_jpeg_to_tft(incomingImageBuffer, incomingImageLen);
            newResultAvailable = false; // Reset flag
        }
    }
    // Jika Tombol 13 Dilepas (HIGH) -> Tampilkan stream kamera lokal
    else
    {
        display_jpeg_to_tft(fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
}

void buttonConfig()
{
    // Logika Toggle LED dengan Debounce (Pin 47)
    int reading = digitalRead(btnToggle);
    if (reading == LOW && (millis() - lastDebounceTime) > debounceDelay)
    {
        ledStatus = !ledStatus;
        digitalWrite(led1, ledStatus);
        digitalWrite(led2, ledStatus);
        Serial.println(ledStatus ? "LED ON" : "LED OFF");
        lastDebounceTime = millis();
        while (digitalRead(btnToggle) == LOW)
            delay(5);
    }
}

// --- Helper Functions ---

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
{
    if (y >= tft.height())
        return 0;
    tft.pushImage(x, y, w, h, bitmap);
    return 1;
}

void display_jpeg_to_tft(uint8_t *jpeg_buf, size_t jpeg_len)
{
    if (!jpeg_buf || jpeg_len == 0)
        return;
    TJpgDec.drawJpg(0, 0, jpeg_buf, jpeg_len);
}

void cameraInit()
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 12000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_HVGA;
    config.jpeg_quality = 12; // Kualitas medium agar transfer cepat
    config.fb_count = 2;

    if (esp_camera_init(&config) != ESP_OK)
    {
        Serial.println("Camera Init Failed");
        ESP.restart();
    }
}

void connectWifi()
{
    Serial.print("WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");
}

void connectMQTT()
{
    while (!client.connected())
    {
        Serial.print("Connecting MQTT...");
        String clientId = String(MQTT_CLIENT_ID_BASE) + "-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD))
        {
            Serial.println("Connected!");
            // Subscribe ke topik hasil untuk menerima gambar balik
            client.subscribe(MQTT_TOPIC_SUB);
        }
        else
        {
            Serial.print("Failed, rc=");
            Serial.print(client.state());

#if USE_MQTT_SSL
            char err_buf[100];
            if (espClient.lastError(err_buf, 100) != 0)
            {
                Serial.print(" | SSL Err: ");
                Serial.println(err_buf);
            }
#endif
            Serial.println(" Retrying...");
            delay(3000);
        }
    }
}

#if USE_MQTT_SSL
void setTime()
{
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}
#endif