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

// ── Pin Map ────────────────────────────────────
#define I2C_SDA_PIN (gpio_num_t)22
#define I2C_SCL_PIN (gpio_num_t)20
#define I2C_PWR_PIN (gpio_num_t)7
#define I2C_PULLUP_PIN (gpio_num_t)8

const gpio_num_t XSHUT_PINS[3] = {(gpio_num_t)27, (gpio_num_t)12, (gpio_num_t)13};
const uint8_t SENSOR_ADDRS[3] = {0x30, 0x31, 0x32};

// ── LED Config ─────────────────────────────────
#define LED_STRIP_GPIO 5
#define LED_STRIP_COUNT 60
#define BIN_COUNT 3
#define LEDS_PER_BIN (LED_STRIP_COUNT / BIN_COUNT)
#define FRAME_MS 20 // 50fps

// At 50fps, 5 seconds = 250 frames with no hand across ALL bins
#define IDLE_FRAMES 250
#define RAINBOW_SPEED 1.5f // degrees per frame

// ── Distance Config ────────────────────────────
#define DIST_MIN_MM 50
#define DIST_MAX_MM 600
#define DIST_FAR_MM 800

// ── Smoothing ──────────────────────────────────
// How fast the displayed value chases the real reading.
// 0.0 = never moves, 1.0 = instant. 0.15 feels natural.
#define SMOOTH_FACTOR 0.15f

// How many frames with no hand before we consider the bin empty.
// At 50fps, 25 frames = 500ms timeout before fade-out begins.
#define NO_HAND_FRAMES 25

// ── Shared State ───────────────────────────────
typedef struct
{
    int id;
    uint16_t distance;
} sensor_msg_t;

typedef struct
{
    uint16_t distance;
} bin_state_t;

static bin_state_t bin_states[BIN_COUNT];
static SemaphoreHandle_t bin_mutex;
static QueueHandle_t sensor_queue;
static led_strip_handle_t led_strip;
static VL53L0X *sensors[3];

// ── HSV → RGB ──────────────────────────────────
static void hsv_to_rgb(float h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    float sf = s / 255.0f;
    float vf = v / 255.0f;
    float hh = fmodf(h, 360.0f) / 60.0f;
    int i = (int)hh;
    float ff = hh - i;
    float p = vf * (1.0f - sf);
    float q = vf * (1.0f - sf * ff);
    float t = vf * (1.0f - sf * (1.0f - ff));
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

// ── Distance → target brightness + hue ────────
static void distance_to_target(uint16_t dist_mm, float *brightness, float *hue)
{
    if (dist_mm >= DIST_FAR_MM)
    {
        *brightness = 0.0f;
        *hue = 0.0f;
        return;
    }
    if (dist_mm <= DIST_MIN_MM)
    {
        *brightness = 1.0f;
        *hue = 120.0f;
        return;
    }
    if (dist_mm <= DIST_MAX_MM)
    {
        float t = (float)(dist_mm - DIST_MIN_MM) / (float)(DIST_MAX_MM - DIST_MIN_MM);
        *brightness = 1.0f - (t * 0.85f);
        *hue = 120.0f;
        return;
    }
    // 600–800mm: shift to red and fade out
    float t = (float)(dist_mm - DIST_MAX_MM) / (float)(DIST_FAR_MM - DIST_MAX_MM);
    *brightness = (1.0f - t) * 0.15f;
    *hue = 0.0f;
}

// ── LED Task — Core 1 ──────────────────────────
static void led_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    float smooth_brightness[BIN_COUNT] = {0};
    float smooth_hue[BIN_COUNT] = {120.0f, 120.0f, 120.0f};
    int no_hand_counter[BIN_COUNT] = {0};

    int idle_frames = 0;
    bool rainbow_mode = false;
    float rainbow_offset = 0.0f;

    // Fade state for transitioning in/out of rainbow
    float rainbow_brightness = 0.0f;

    while (1)
    {
        xSemaphoreTake(bin_mutex, portMAX_DELAY);
        uint16_t dist[BIN_COUNT];
        for (int b = 0; b < BIN_COUNT; b++)
            dist[b] = bin_states[b].distance;
        xSemaphoreGive(bin_mutex);

        // Check if ANY bin has a hand
        bool any_hand = false;
        for (int b = 0; b < BIN_COUNT; b++)
        {
            if (dist[b] < DIST_FAR_MM)
            {
                any_hand = true;
                break;
            }
        }

        // Update idle counter
        if (any_hand)
        {
            idle_frames = 0;
            rainbow_mode = false;
        }
        else
        {
            idle_frames++;
            if (idle_frames >= IDLE_FRAMES)
                rainbow_mode = true;
        }

        // ── Rainbow Mode ───────────────────────
        if (rainbow_mode)
        {
            // Fade rainbow in smoothly
            rainbow_brightness += (1.0f - rainbow_brightness) * 0.03f;

            rainbow_offset = fmodf(rainbow_offset + RAINBOW_SPEED, 360.0f);

            for (int i = 0; i < LED_STRIP_COUNT; i++)
            {
                float hue = fmodf(rainbow_offset + (360.0f / LED_STRIP_COUNT) * i, 360.0f);
                uint8_t r, g, b;
                uint8_t v = (uint8_t)(rainbow_brightness * 180.0f);
                hsv_to_rgb(hue, 255, v, &r, &g, &b);
                led_strip_set_pixel(led_strip, i, r, g, b);
            }

            // Also reset smooth state so sensors snap back cleanly
            for (int b = 0; b < BIN_COUNT; b++)
            {
                smooth_brightness[b] = 0.0f;
                no_hand_counter[b] = 0;
            }
        }
        // ── Normal Sensor Mode ─────────────────
        else
        {
            // Fade rainbow out when returning to sensor mode
            rainbow_brightness *= 0.85f;

            for (int b = 0; b < BIN_COUNT; b++)
            {
                float target_brightness, target_hue;
                distance_to_target(dist[b], &target_brightness, &target_hue);

                if (dist[b] >= DIST_FAR_MM)
                    no_hand_counter[b]++;
                else
                    no_hand_counter[b] = 0;

                if (no_hand_counter[b] > NO_HAND_FRAMES)
                    target_brightness = 0.0f;

                float bright_speed = (target_brightness > smooth_brightness[b])
                                         ? SMOOTH_FACTOR * 2.0f
                                         : SMOOTH_FACTOR * 0.5f;

                smooth_brightness[b] += (target_brightness - smooth_brightness[b]) * bright_speed;
                smooth_hue[b] += (target_hue - smooth_hue[b]) * SMOOTH_FACTOR;

                if (smooth_brightness[b] < 0.005f)
                    smooth_brightness[b] = 0.0f;

                // Blend: if rainbow is still fading out, mix it with sensor color
                uint8_t v = (uint8_t)(smooth_brightness[b] * 200.0f);
                uint8_t r, g, bv;
                hsv_to_rgb(smooth_hue[b], 255, v, &r, &g, &bv);

                for (int j = 0; j < LEDS_PER_BIN; j++)
                    led_strip_set_pixel(led_strip, b * LEDS_PER_BIN + j, r, g, bv);
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
    sensor_msg_t msg;
    msg.id = id;

    while (1)
    {
        if (sensors[id]->read(&msg.distance))
        {
            xQueueSend(sensor_queue, &msg, pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

// ── Processor Task ─────────────────────────────
static void processor_task(void *pvParameters)
{
    sensor_msg_t incoming;

    while (1)
    {
        if (xQueueReceive(sensor_queue, &incoming, portMAX_DELAY))
        {
            xSemaphoreTake(bin_mutex, portMAX_DELAY);
            bin_states[incoming.id].distance = incoming.distance;
            xSemaphoreGive(bin_mutex);

            printf(">S0:%u\n", bin_states[0].distance);
            printf(">S1:%u\n", bin_states[1].distance);
            printf(">S2:%u\n", bin_states[2].distance);
        }
    }
}

// ── LED Init ───────────────────────────────────
static void init_leds(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = LED_STRIP_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {.invert_out = false},
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {.with_dma = false},
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}

// ── Entry Point ────────────────────────────────
extern "C" void app_main(void)
{
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
    for (int i = 0; i < 3; i++)
    {
        gpio_set_direction(XSHUT_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(XSHUT_PINS[i], 0);
    }
    printf("All sensors held in reset\n");
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
    for (int i = 0; i < 3; i++)
    {
        printf("Waking sensor %d...\n", i);
        gpio_set_level(XSHUT_PINS[i], 1);
        vTaskDelay(pdMS_TO_TICKS(50));

        sensors[i] = new VL53L0X(I2C_NUM_0);
        if (sensors[i]->init())
        {
            sensors[i]->setDeviceAddress(SENSOR_ADDRS[i]);
            sensors[i]->setTimingBudget(25000);
            printf("  Sensor %d OK at 0x%02x\n", i, SENSOR_ADDRS[i]);

            for (int j = 0; j < LEDS_PER_BIN; j++)
                led_strip_set_pixel(led_strip, i * LEDS_PER_BIN + j, 0, 0, 80);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
            for (int j = 0; j < LEDS_PER_BIN; j++)
                led_strip_set_pixel(led_strip, i * LEDS_PER_BIN + j, 0, 0, 0);
            led_strip_refresh(led_strip);
        }
        else
        {
            printf("  ERROR: Sensor %d FAILED — halting\n", i);
            while (1)
            {
                for (int j = 0; j < LEDS_PER_BIN; j++)
                    led_strip_set_pixel(led_strip, i * LEDS_PER_BIN + j, 80, 0, 0);
                led_strip_refresh(led_strip);
                vTaskDelay(pdMS_TO_TICKS(300));
                for (int j = 0; j < LEDS_PER_BIN; j++)
                    led_strip_set_pixel(led_strip, i * LEDS_PER_BIN + j, 0, 0, 0);
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
    for (int i = 0; i < BIN_COUNT; i++)
        bin_states[i].distance = DIST_FAR_MM;

    sensor_queue = xQueueCreate(20, sizeof(sensor_msg_t));

    xTaskCreatePinnedToCore(led_task, "led_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(processor_task, "proc", 4096, NULL, 10, NULL, 0);
    for (int i = 0; i < 3; i++)
        xTaskCreate(sensor_worker, "work", 4096, (void *)i, 5, NULL);

    printf("All tasks running.\n");
}