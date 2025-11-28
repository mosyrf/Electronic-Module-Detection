/* Includes ---------------------------------------------------------------- */
#include <Arduino.h>
#include <Electronics_Objects_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include "board_config.h"
#include "camera_pins.h"

#include "DFRobot_AXP313A.h"
DFRobot_AXP313A axp;

#include <SPI.h>
#include <TFT_eSPI.h>      // Hardware-specific library
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
#include <TJpg_Decoder.h>

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3
const int led1 = 15;
const int led2 = 9;

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; // points to the output of the capture

bool ledStatus = false;
const int inBoardButton = 47;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 12000000, // 12MHz for OV2640 Red Module
    // .xclk_freq_hz = 20000000, // 20MHz for BuiltinOV2640
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,   // QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 10, // 0-63 lower number means higher quality
    .fb_count = 1,      // if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void buttonConfig();
void display_detection_to_tft(uint8_t *rgb888_buf, uint32_t src_w, uint32_t src_h, ei_impulse_result_t *result);

/**
 * @brief      Arduino setup function
 */
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    digitalWrite(led1, ledStatus);
    digitalWrite(led2, ledStatus);

    pinMode(inBoardButton, INPUT_PULLUP);

    while (axp.begin() != 0)
    {
        Serial.println("init error");
        delay(1000);
    }
    axp.enableCameraPower(axp.eOV2640);

    // Inisialisasi TFT
    tft.init();
    tft.setRotation(1); // 0=PORTRAIT, 1=LANDSCAPE, 2=PORTRAIT_FLIP, 3=LANDSCAPE_FLIP
    tft.fillScreen(TFT_BLACK);
    tft.setSwapBytes(true);

    // comment out the below line to start inference immediately after upload
    // while (!Serial)
    //    ;

    Serial.println("Edge Impulse Inferencing Demo");

    if (ei_camera_init() == false)
    {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else
    {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
}

/**
 * @brief      Get data and run inferencing
 *
 * @param[in]  debug  Get debug info if true
 */
void loop()
{
    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK)
    {
        return;
    }

    snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if (snapshot_buf == nullptr)
    {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false)
    {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK)
    {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

    buttonConfig();
    display_detection_to_tft(snapshot_buf, (uint32_t)EI_CLASSIFIER_INPUT_WIDTH, (uint32_t)EI_CLASSIFIER_INPUT_HEIGHT, &result);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t i = 0; i < result.bounding_boxes_count; i++)
    {
        auto bb = result.bounding_boxes[i];
        if (bb.value == 0)
        {
            continue;
        }
        ei_printf("%s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n",
                  bb.label,
                  bb.value,
                  bb.x, bb.y,
                  bb.width,
                  bb.height);
    }
    if (!bb_found)
    {
        ei_printf("No objects found\n");
    }
#else
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
    {
        ei_printf("%s: %.5f\n", result.classification[i].label, result.classification[i].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly Score: %.3f\n", result.anomaly);
#endif

    free(snapshot_buf);
}

void buttonConfig()
{
    // Tombol terhubung ke INPUT_PULLUP, jadi status LOW saat tombol DITEKAN
    int reading = digitalRead(inBoardButton);

    // Cek apakah tombol sedang DITEKAN (LOW) dan debouncing sudah lewat
    if (reading == LOW && (millis() - lastDebounceTime) > debounceDelay)
    {
        // Ubah status LED (Toggle)
        ledStatus = !ledStatus;

        // Terapkan status baru ke kedua LED
        // Jika ledStatus true: LED MATI (HIGH). Jika ledStatus false: LED NYALA (LOW).
        digitalWrite(led1, ledStatus);
        digitalWrite(led2, ledStatus);

        Serial.print("Tombol Ditekan! LED Status: ");
        Serial.println(ledStatus ? "ON" : "OFF"); // Tampilkan status yang benar (kebalikan dari nilai digital)

        // Catat waktu terakhir penekanan
        lastDebounceTime = millis();

        // Tunggu hingga tombol dilepas sebelum kembali (menghindari tekan ganda)
        while (digitalRead(inBoardButton) == LOW)
        {
            delay(5);
        }
    }
}

/**
 * display_detection_to_tft
 *
 * - rgb888_buf : pointer ke buffer RGB888 source (r,g,b per pixel)
 * - src_w, src_h: ukuran buffer source
 * - result: pointer ke ei_impulse_result_t hasil inferensi (untuk bounding boxes)
 *
 * Fungsi: scale -> konversi RGB888 -> RGB565 -> push ke TFT -> gambar bounding box dan label.
 */
void display_detection_to_tft(uint8_t *rgb888_buf, uint32_t src_w, uint32_t src_h, ei_impulse_result_t *result)
{
    if (!rgb888_buf)
        return;

    const uint16_t tft_w = 480;
    const uint16_t tft_h = 320;

    // Alokasi buffer RGB565 (2 bytes per pixel)
    size_t buf_bytes = tft_w * tft_h * 2;
    uint16_t *buf565 = (uint16_t *)malloc(buf_bytes);
    if (buf565 == nullptr)
    {
        ei_printf("ERR: Failed to allocate TFT buffer (%u bytes)\n", (unsigned)buf_bytes);
        return;
    }

    // Jika source aspect berbeda, kita gunakan scaling (nearest neighbor).
    // Map pixel (x,y) in TFT ke source pixel di rgb888_buf:
    for (uint16_t y = 0; y < tft_h; y++)
    {
        // nearest source y
        uint32_t src_y = (uint32_t)(((uint32_t)y * src_h) / tft_h);
        if (src_y >= src_h)
            src_y = src_h - 1;
        for (uint16_t x = 0; x < tft_w; x++)
        {
            uint32_t src_x = (uint32_t)(((uint32_t)x * src_w) / tft_w);
            if (src_x >= src_w)
                src_x = src_w - 1;

            size_t src_idx = (src_y * src_w + src_x) * 3;
            uint8_t r = rgb888_buf[src_idx + 0];
            uint8_t g = rgb888_buf[src_idx + 1];
            uint8_t b = rgb888_buf[src_idx + 2];

            // Konversi RGB888 -> RGB565
            uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            buf565[y * tft_w + x] = c;
        }
    }

    // Push image ke TFT (fast)
    tft.pushImage(0, 0, tft_w, tft_h, buf565);

    // Gambar bounding boxes (jika ada)
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    // Warna dan teks
    uint16_t box_color = TFT_RED;
    uint16_t text_color = TFT_WHITE;
    uint16_t bg_color = TFT_BLACK;

    // Set text properties
    tft.setTextSize(1);
    tft.setTextFont(1);

    for (size_t i = 0; i < result->bounding_boxes_count; i++)
    {
        auto bb = result->bounding_boxes[i];
        if (bb.value == 0)
            continue;

        // Asumsi bb.x, bb.y, bb.width, bb.height berada pada skala model (src_w x src_h)
        // Jika model memakai ukuran lain, sesuaikan (kami gunakan src_w/src_h)
        float scale_x = (float)tft_w / (float)src_w;
        float scale_y = (float)tft_h / (float)src_h;

        int x = (int)round(bb.x * scale_x);
        int y = (int)round(bb.y * scale_y);
        int w = (int)round(bb.width * scale_x);
        int h = (int)round(bb.height * scale_y);

        // Boundary check
        if (x < 0)
            x = 0;
        if (y < 0)
            y = 0;
        if (x + w > tft_w)
            w = tft_w - x;
        if (y + h > tft_h)
            h = tft_h - y;

        // Gambar rectangle (outline)
        tft.drawRect(x, y, w, h, box_color);

        // Tulis label di atas box (background kecil)
        String label = String(bb.label) + " " + String((int)round(bb.value * 100)) + "%";
        int txt_x = x + 2;
        int txt_y = (y - 10 >= 0) ? (y - 10) : (y + 2);

        // Draw filled rect behind text for readability
        int txt_w = label.length() * 6 + 4; // approx width (font 1, size 1 => ~6px/char)
        int txt_h = 10;
        if (txt_x + txt_w > tft_w)
            txt_w = tft_w - txt_x;
        tft.fillRect(txt_x - 1, txt_y - 1, txt_w, txt_h, bg_color);
        tft.drawRect(txt_x - 1, txt_y - 1, txt_w, txt_h, box_color);

        // Draw string
        tft.setTextColor(text_color, bg_color);
        tft.setCursor(txt_x, txt_y);
        tft.print(label);
    }
#endif

    free(buf565);
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void)
{

    if (is_initialised)
        return true;

#if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
#endif

    // initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_vflip(s, 1);      // flip it back
        s->set_brightness(s, 1); // up the brightness just a bit
        s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void)
{

    // deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf)
{
    bool do_resize = false;

    if (!is_initialised)
    {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb)
    {
        ei_printf("Camera capture failed\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

    esp_camera_fb_return(fb);

    if (!converted)
    {
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS))
    {
        do_resize = true;
    }

    if (do_resize)
    {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0)
    {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif