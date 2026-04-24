#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "led_strip.h"
#include "VL53L0X.h"
#include "esp_timer.h"

// ── Pin Map ────────────────────────────────────
#define I2C_SDA_PIN (gpio_num_t)22
#define I2C_SCL_PIN (gpio_num_t)20
#define I2C_PWR_PIN (gpio_num_t)7
#define I2C_PULLUP_PIN (gpio_num_t)8

#define SENSOR_COUNT 7

const gpio_num_t XSHUT_PINS[SENSOR_COUNT] = {
    (gpio_num_t)25, // Sensor 0
    (gpio_num_t)14, // Sensor 1
    (gpio_num_t)32, // Sensor 2
    (gpio_num_t)15, // Sensor 3
    (gpio_num_t)27, // Sensor 4
    (gpio_num_t)12, // Sensor 5
    (gpio_num_t)13, // Sensor 6
};

const uint8_t SENSOR_ADDRS[SENSOR_COUNT] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36};

// ── LED Config ─────────────────────────────────
#define LED_STRIP_GPIO 5
#define LED_STRIP_COUNT 60
#define BIN_COUNT SENSOR_COUNT
#define LEDS_PER_BIN (LED_STRIP_COUNT / BIN_COUNT)

// ── Timing ─────────────────────────────────────
#define FRAME_MS 10  // 100fps — was 20ms/50fps, sensors are the bottleneck anyway
#define SENSOR_MS 20 // sensor poll rate — was 25ms

// ── Idle / Rainbow ─────────────────────────────
#define IDLE_FRAMES 500 // 5 seconds at 100fps
#define RAINBOW_SPEED 1.0f

// ── Distance Config ────────────────────────────
#define DIST_MIN_MM 50
#define DIST_MAX_MM 600
#define DIST_FAR_MM 800

// ── Smoothing ──────────────────────────────────
#define SMOOTH_FACTOR 0.25f // snappier than 0.15
#define NO_HAND_FRAMES 3    // ~30ms at 100fps — very fast off

// ── Stale reading timeout ──────────────────────
// If a sensor hasn't sent a fresh reading in this many ms, treat it as no hand
#define STALE_MS 80

// ── Shared State ───────────────────────────────
typedef struct
{
    int id;
    uint16_t distance;
} sensor_msg_t;

typedef struct
{
    uint16_t distance;
    int64_t last_update_us; // timestamp of last fresh reading
} bin_state_t;

static bin_state_t bin_states[BIN_COUNT];
static SemaphoreHandle_t bin_mutex;
static QueueHandle_t sensor_queue;
static led_strip_handle_t led_strip;
static VL53L0X *sensors[SENSOR_COUNT];

// ── HSV → RGB ──────────────────────────────────
static void hsv_to_rgb(float h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    float sf = s / 255.0f, vf = v / 255.0f;
    float hh = fmodf(h, 360.0f) / 60.0f;
    int i = (int)hh;
    float ff = hh - i;
    float p = vf * (1 - sf), q = vf * (1 - sf * ff), t = vf * (1 - sf * (1 - ff));
    float rf, gf, bf;
    switch (i)
    {
    case 0:
        rf = vf;
        gf = t;
        bf = p;
        break;
    case 1:
        rf = q;
        gf = vf;
        bf = p;
        break;
    case 2:
        rf = p;
        gf = vf;
        bf = t;
        break;
    case 3:
        rf = p;
        gf = q;
        bf = vf;
        break;
    case 4:
        rf = t;
        gf = p;
        bf = vf;
        break;
    default:
        rf = vf;
        gf = p;
        bf = q;
        break;
    }
    *r = (uint8_t)(rf * 255);
    *g = (uint8_t)(gf * 255);
    *b = (uint8_t)(bf * 255);
}

// ── Distance → hue + brightness ───────────────
//
// Instead of green/red brightness, each bin gets a hue based on
// distance. Close = warm (red/orange), far = cool (blue/cyan).
// The brightness still fades so distant hands are dimmer.
//
static void distance_to_color(uint16_t d, float *hue, float *brightness)
{
    if (d >= DIST_FAR_MM)
    {
        *hue = 0;
        *brightness = 0;
        return;
    }
    if (d <= DIST_MIN_MM)
    {
        *hue = 0;
        *brightness = 1.0f;
        return;
    }

    // Map distance to hue: 0mm=red(0°), 600mm=blue(240°)
    float t = (float)(d - DIST_MIN_MM) / (float)(DIST_MAX_MM - DIST_MIN_MM);
    *hue = t * 240.0f;

    // Brightness: full when close, fades to 30% at max distance
    *brightness = 1.0f - (t * 0.7f);
}

// ── LED Task — Core 1 ──────────────────────────
static void led_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    float smooth_brightness[SENSOR_COUNT] = {0};
    float smooth_hue[SENSOR_COUNT] = {0};
    int no_hand_counter[SENSOR_COUNT] = {0};

    int idle_frames = 0;
    bool rainbow_mode = false;
    float rainbow_offset = 0;
    float rainbow_brightness = 0;

    while (1)
    {
        // Snapshot state
        xSemaphoreTake(bin_mutex, portMAX_DELAY);
        uint16_t dist[SENSOR_COUNT];
        int64_t last_update[SENSOR_COUNT];
        for (int b = 0; b < SENSOR_COUNT; b++)
        {
            dist[b] = bin_states[b].distance;
            last_update[b] = bin_states[b].last_update_us;
        }
        xSemaphoreGive(bin_mutex);

        int64_t now_us = esp_timer_get_time();

        // A bin is "active" only if it has a hand AND the reading is fresh
        bool any_hand = false;
        bool bin_active[SENSOR_COUNT];
        for (int b = 0; b < SENSOR_COUNT; b++)
        {
            int64_t age_ms = (now_us - last_update[b]) / 1000;
            bin_active[b] = (dist[b] < DIST_FAR_MM) && (age_ms < STALE_MS);
            if (bin_active[b])
                any_hand = true;
        }

        // Idle counter
        if (any_hand)
        {
            idle_frames = 0;
            rainbow_mode = false;
        }
        else
        {
            if (++idle_frames >= IDLE_FRAMES)
                rainbow_mode = true;
        }

        // ── Rainbow idle mode ──────────────────
        if (rainbow_mode)
        {
            rainbow_brightness += (1.0f - rainbow_brightness) * 0.03f;
            rainbow_offset = fmodf(rainbow_offset + RAINBOW_SPEED, 360.0f);
            for (int i = 0; i < LED_STRIP_COUNT; i++)
            {
                float hue = fmodf(rainbow_offset + (360.0f / LED_STRIP_COUNT) * i, 360.0f);
                uint8_t r, g, b, v = (uint8_t)(rainbow_brightness * 180);
                hsv_to_rgb(hue, 255, v, &r, &g, &b);
                led_strip_set_pixel(led_strip, i, r, g, b);
            }
            for (int b = 0; b < SENSOR_COUNT; b++)
            {
                smooth_brightness[b] = 0;
                no_hand_counter[b] = 0;
            }
        }
        // ── Sensor mode ────────────────────────
        else
        {
            rainbow_brightness *= 0.85f;

            for (int b = 0; b < SENSOR_COUNT; b++)
            {
                float target_hue, target_brightness;

                if (bin_active[b])
                {
                    distance_to_color(dist[b], &target_hue, &target_brightness);
                    no_hand_counter[b] = 0;
                }
                else
                {
                    // Stale or no reading — ramp to off immediately
                    no_hand_counter[b]++;
                    target_hue = smooth_hue[b]; // hold hue while fading
                    target_brightness = 0.0f;
                }

                // Fast attack, fast decay (NO_HAND_FRAMES is now very small)
                float spd = (target_brightness > smooth_brightness[b])
                                ? SMOOTH_FACTOR * 2.0f
                                : SMOOTH_FACTOR * 3.0f; // decay faster than attack

                smooth_brightness[b] += (target_brightness - smooth_brightness[b]) * spd;
                smooth_hue[b] += (target_hue - smooth_hue[b]) * SMOOTH_FACTOR;

                if (smooth_brightness[b] < 0.01f)
                    smooth_brightness[b] = 0.0f;

                uint8_t v = (uint8_t)(smooth_brightness[b] * 200), r, g, bv;
                hsv_to_rgb(smooth_hue[b], 255, v, &r, &g, &bv);

                int start = b * LEDS_PER_BIN;
                int count = (b == SENSOR_COUNT - 1)
                                ? (LED_STRIP_COUNT - start)
                                : LEDS_PER_BIN;
                for (int j = 0; j < count; j++)
                    led_strip_set_pixel(led_strip, start + j, r, g, bv);
            }
        }

        led_strip_refresh(led_strip);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FRAME_MS));
    }
}

// ── Sensor Worker ──────────────────────────────
static void sensor_worker(void *pvParameters)
{
    int id = (int)pvParameters;
    sensor_msg_t msg = {.id = id, .distance = 0};
    while (1)
    {
        if (sensors[id]->read(&msg.distance))
            xQueueSend(sensor_queue, &msg, pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(SENSOR_MS));
    }
}

// ── Processor Task — Core 0 ────────────────────
static void processor_task(void *pvParameters)
{
    sensor_msg_t incoming;
    while (1)
    {
        if (xQueueReceive(sensor_queue, &incoming, portMAX_DELAY))
        {
            xSemaphoreTake(bin_mutex, portMAX_DELAY);
            bin_states[incoming.id].distance = incoming.distance;
            bin_states[incoming.id].last_update_us = esp_timer_get_time();
            xSemaphoreGive(bin_mutex);
            for (int i = 0; i < SENSOR_COUNT; i++)
                printf(">S%d:%u\n", i, bin_states[i].distance);
        }
    }
}

// ── LED Init ───────────────────────────────────
static void init_leds(void)
{
    led_strip_config_t sc = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = LED_STRIP_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {.invert_out = false},
    };
    led_strip_rmt_config_t rc = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {.with_dma = false},
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc, &rc, &led_strip));
}

// ── Entry Point ────────────────────────────────
extern "C" void app_main(void)
{
    gpio_reset_pin((gpio_num_t)2);
    gpio_set_direction((gpio_num_t)2, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)2, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n--- STAGE 1: LED TEST ---\n");
    init_leds();
    for (int i = 0; i < LED_STRIP_COUNT; i++)
        led_strip_set_pixel(led_strip, i, 50, 50, 50);
    led_strip_refresh(led_strip);
    printf("LED OK — white for 2 seconds\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
    for (int i = 0; i < LED_STRIP_COUNT; i++)
        led_strip_set_pixel(led_strip, i, 0, 0, 0);
    led_strip_refresh(led_strip);

    printf("\n--- STAGE 2: POWER RAILS ---\n");
    gpio_set_direction(I2C_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_PWR_PIN, 1);
    gpio_set_direction(I2C_PULLUP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_PULLUP_PIN, 1);
    printf("Power rails up\n");

    printf("\n--- STAGE 3: XSHUT ALL LOW ---\n");
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        gpio_set_direction(XSHUT_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(XSHUT_PINS[i], 0);
    }
    printf("All %d sensors held in reset\n", SENSOR_COUNT);
    vTaskDelay(pdMS_TO_TICKS(200));

    for (int i = 0; i < LED_STRIP_COUNT; i++)
        led_strip_set_pixel(led_strip, i, 0, 50, 0);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (int i = 0; i < LED_STRIP_COUNT; i++)
        led_strip_set_pixel(led_strip, i, 0, 0, 0);
    led_strip_refresh(led_strip);

    printf("\n--- STAGE 4: I2C INIT ---\n");
    VL53L0X temp_bus(I2C_NUM_0);
    temp_bus.i2cMasterInit(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    printf("I2C bus ready\n");

    printf("\n--- STAGE 5: SENSOR INIT ---\n");
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        printf("Waking sensor %d (GPIO %d)...\n", i, (int)XSHUT_PINS[i]);
        gpio_set_level(XSHUT_PINS[i], 1);
        vTaskDelay(pdMS_TO_TICKS(50));

        sensors[i] = new VL53L0X(I2C_NUM_0);
        if (sensors[i]->init())
        {
            sensors[i]->setDeviceAddress(SENSOR_ADDRS[i]);
            sensors[i]->setTimingBudget(20000); // 20ms — slightly faster than before
            printf("  Sensor %d OK at 0x%02x\n", i, SENSOR_ADDRS[i]);
            int start = i * LEDS_PER_BIN;
            int count = (i == SENSOR_COUNT - 1) ? (LED_STRIP_COUNT - start) : LEDS_PER_BIN;
            for (int j = 0; j < count; j++)
                led_strip_set_pixel(led_strip, start + j, 0, 0, 80);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
            for (int j = 0; j < count; j++)
                led_strip_set_pixel(led_strip, start + j, 0, 0, 0);
            led_strip_refresh(led_strip);
        }
        else
        {
            printf("  ERROR: Sensor %d FAILED — halting\n", i);
            int start = i * LEDS_PER_BIN;
            int count = (i == SENSOR_COUNT - 1) ? (LED_STRIP_COUNT - start) : LEDS_PER_BIN;
            while (1)
            {
                for (int j = 0; j < count; j++)
                    led_strip_set_pixel(led_strip, start + j, 80, 0, 0);
                led_strip_refresh(led_strip);
                vTaskDelay(pdMS_TO_TICKS(300));
                for (int j = 0; j < count; j++)
                    led_strip_set_pixel(led_strip, start + j, 0, 0, 0);
                led_strip_refresh(led_strip);
                vTaskDelay(pdMS_TO_TICKS(300));
            }
        }
    }

    printf("\n--- STAGE 6: STARTING TASKS ---\n");
    for (int i = 0; i < LED_STRIP_COUNT; i++)
        led_strip_set_pixel(led_strip, i, 0, 80, 0);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (int i = 0; i < LED_STRIP_COUNT; i++)
        led_strip_set_pixel(led_strip, i, 0, 0, 0);
    led_strip_refresh(led_strip);

    bin_mutex = xSemaphoreCreateMutex();
    memset(bin_states, 0, sizeof(bin_states));
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        bin_states[i].distance = DIST_FAR_MM;
        bin_states[i].last_update_us = 0; // will trigger stale immediately
    }

    sensor_queue = xQueueCreate(30, sizeof(sensor_msg_t));

    xTaskCreatePinnedToCore(led_task, "led_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(processor_task, "proc", 4096, NULL, 10, NULL, 0);
    for (int i = 0; i < SENSOR_COUNT; i++)
        xTaskCreate(sensor_worker, "work", 4096, (void *)i, 5, NULL);

    printf("All %d tasks running.\n", SENSOR_COUNT);
}