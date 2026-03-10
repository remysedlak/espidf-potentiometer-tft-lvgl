#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "esp_adc/adc_oneshot.h"
#include "lvgl.h"

// Display config
#define LCD_HOST          SPI2_HOST
#define LCD_H_RES         (240)
#define LCD_V_RES         (320)
#define LCD_BIT_PER_PIXEL (16)

#define PIN_LCD_CS        (GPIO_NUM_10)
#define PIN_LCD_PCLK      (GPIO_NUM_12)
#define PIN_LCD_MOSI      (GPIO_NUM_11)
#define PIN_LCD_DC        (GPIO_NUM_9)
#define PIN_LCD_RST       (GPIO_NUM_14)

// ADC config 
#define ADC_CHANNEL       ADC_CHANNEL_2

// LVGL config
#define LVGL_TICK_PERIOD_MS  (2)
#define LVGL_DRAW_BUF_LINES  (20)

static const char *TAG = "display-pot";

// Semaphore to signal when direct memory access transfer to display is complete
static SemaphoreHandle_t refresh_finish = NULL;

// LVGL v8 uses lv_display_t as a Display structure
static lv_disp_t *lv_disp = NULL;

// Label widget that shows the pot reading
static lv_obj_t *reading_label = NULL;

// Panel handle needs to be global so the flush callback can access it
static esp_lcd_panel_handle_t g_panel = NULL;

// DMA transfer complete callback
static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
        esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;
    xSemaphoreGiveFromISR(refresh_finish, &need_yield);
    return (need_yield == pdTRUE);
}

// LVGL flush callback (v8 API)
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    // Send rendered region to display via DMA
    // x2+1 and y2+1 because esp_lcd uses exclusive end coordinates
    esp_lcd_panel_draw_bitmap(g_panel,
        area->x1, area->y1,
        area->x2 + 1, area->y2 + 1,
        color_p
    );
    // Wait for DMA to finish before returning
    xSemaphoreTake(refresh_finish, portMAX_DELAY);
    // Tell LVGL this flush is done - it can render the next region
    lv_disp_flush_ready(drv);
}

// LVGL tick timer callback
static void lvgl_tick_cb(void *arg)
{
    // VGL needs a system tick to know elapsed time 
    // for animations and other tasks.

    // You need to call the lv_tick_inc(tick_period) function periodically and provide the call period in milliseconds. 
    // For example, lv_tick_inc(1) when calling every millisecond.
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// TFT Display + LVGL initialization
static void display_init(void)
{
    refresh_finish = xSemaphoreCreateBinary();

    // Step 1: Configure and init SPI bus
    const spi_bus_config_t bus_cfg = ILI9341_PANEL_BUS_SPI_CONFIG(
        PIN_LCD_PCLK, PIN_LCD_MOSI,
        LCD_H_RES * LVGL_DRAW_BUF_LINES * LCD_BIT_PER_PIXEL / 8
    );
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // Step 2: Configure and init Panel IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_cfg = ILI9341_PANEL_IO_SPI_CONFIG(
        PIN_LCD_CS, PIN_LCD_DC,
        on_color_trans_done, NULL
    );
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io_handle
    ));

    // Step 3: Configure and create ILI9341 panel driver
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_cfg, &g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(g_panel, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(g_panel, true));

    // Step 4: Initialize LVGL
    lv_init();

    // Allocate two pixel draw buffers in DMA-capable RAM
    // LVGL v8 uses lv_disp_draw_buf_t to manage these

    // Draw buffer(s) are simple array(s) used to render the screen content. 
    static lv_disp_draw_buf_t draw_buf; 
    size_t buf_size = LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color_t);
    lv_color_t *buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    
    // Tell LVGL about the two buffers 
    // it will alternate between them to stop tearing
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_H_RES * LVGL_DRAW_BUF_LINES);

    // LVGL v8 uses lv_disp_drv_t to register a display driver
    // You fill in the driver struct then call lv_disp_drv_register()
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res   = LCD_H_RES;
    disp_drv.ver_res   = LCD_V_RES;
    disp_drv.flush_cb  = lvgl_flush_cb;  // our callback that sends pixels to display
    disp_drv.draw_buf  = &draw_buf;
    lv_disp = lv_disp_drv_register(&disp_drv);

    // Step 5: Timer to drive LVGL's internal tick counter
    const esp_timer_create_args_t timer_args = {
        .callback = lvgl_tick_cb,
        .name     = "lvgl_tick"
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Display + LVGL initialized");
}

// Analog Digital Converter initialization
static adc_oneshot_unit_handle_t adc_init(void)
{
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .ulp_mode = ADC_ULP_MODE_DISABLE,
        .unit_id  = ADC_UNIT_1
    };
    adc_oneshot_unit_handle_t handle;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_CHANNEL, &chan_cfg));
    return handle;
}

// Potentiometer + display task (Core 1)
static void pot_display_task(void *pvParameters)
{
    adc_oneshot_unit_handle_t adc = adc_init();

    // Black background
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

    // Create label - lv_font_montserrat_14 is enabled by default in LVGL 8
    reading_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(reading_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(reading_label, &lv_font_montserrat_14, 0);
    lv_obj_align(reading_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(reading_label, "---");

    int old_reading = 0;
    int new_reading = 0;

    while (1) {
        adc_oneshot_read(adc, ADC_CHANNEL, &new_reading);

        if (old_reading != new_reading) {
            old_reading = new_reading;
            lv_label_set_text_fmt(reading_label, "%d", new_reading);
        }

        // Drive LVGL rendering - must be called regularly
        lv_timer_handler();

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

// ESP-IDF main entry point
void app_main(void)
{
    // initialize display once
    display_init();             

    // pin display/sensor task to 2nd core
    xTaskCreatePinnedToCore(
        pot_display_task,       // implemented task method
        "pot_display",          // name
        8192,                   // stack size/depth
        NULL,
        1,                      // priority
        NULL,
    1                           // core 1, core 0 reserved for audio
    );
}