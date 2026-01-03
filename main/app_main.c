#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"
#include "qrcode.h"

static const char *TAG = "SMART_SOCKET_C3_STABLE";

// --- CẤU HÌNH PIN ---
#define RELAY_PIN 8
#define RESET_BUTTON_PIN 9
#define CURR_ADC_CHAN ADC_CHANNEL_3

#define FIXED_VOLTAGE 220.0f
#define NOISE_THRESHOLD 0.18f
#define ADC_VREF_MV 3100.0f
#define ADC_MAX_RAW 4095.0f
#define CURR_RATIO 100.0f

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_EVENT = BIT0;
esp_mqtt_client_handle_t mqtt_client = NULL;
bool relay_on = false;
static bool ble_is_running = true;

// --- HÀM TẮT BLUETOOTH AN TOÀN ---
void safely_stop_ble()
{
    if (ble_is_running)
    {
        ESP_LOGW(TAG, "Đang đóng Bluetooth an toàn...");
        wifi_prov_mgr_stop_provisioning();
        // Không gọi deinit quá sớm nếu không cần thiết để tránh treo stack
        ble_is_running = false;
        ESP_LOGI(TAG, "Bluetooth đã được tắt.");
    }
}

// --- TASK QUẢN LÝ PROVISIONING (CHÍNH XÁC) ---
void provisioning_guard_task(void *pvParameters)
{
    // Đợi WiFi kết nối thành công HOẶC hết 60 giây
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_EVENT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(60000));

    if (bits & WIFI_CONNECTED_EVENT)
    {
        // KỊCH BẢN 1: Kết nối WiFi thành công
        ESP_LOGI(TAG, "WiFi đã kết nối. Đợi 10s để hệ thống ổn định...");
        vTaskDelay(pdMS_TO_TICKS(10000)); // Grace period 10 giây
        safely_stop_ble();
    }
    else
    {
        // KỊCH BẢN 2: Quá 60s không làm gì
        ESP_LOGW(TAG, "Hết thời gian chờ 60s mà không có cấu hình mới.");
        safely_stop_ble();
    }
    vTaskDelete(NULL);
}

// --- CÁC HÀM XỬ LÝ KHÁC (GIỮ NGUYÊN) ---
void check_reset_button_task(void *pvParameters)
{
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << RESET_BUTTON_PIN), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&io_conf);
    int hold_time = 0;
    while (1)
    {
        if (gpio_get_level(RESET_BUTTON_PIN) == 0)
        {
            if (++hold_time >= 50)
            {
                nvs_flash_erase();
                esp_restart();
            }
        }
        else
            hold_time = 0;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void current_monitor_task(void *pvParameters)
{
    adc_oneshot_unit_handle_t adc_h;
    adc_oneshot_unit_init_cfg_t u_cfg = {.unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&u_cfg, &adc_h);
    adc_oneshot_chan_cfg_t c_cfg = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12};
    adc_oneshot_config_channel(adc_h, CURR_ADC_CHAN, &c_cfg);
    while (1)
    {
        uint64_t sum_sq_i = 0;
        int samples = 1000, i_off = 0;
        for (int k = 0; k < 100; k++)
        {
            int ri;
            adc_oneshot_read(adc_h, CURR_ADC_CHAN, &ri);
            i_off += ri;
            esp_rom_delay_us(50);
        }
        i_off /= 100;
        for (int i = 0; i < samples; i++)
        {
            int ri;
            adc_oneshot_read(adc_h, CURR_ADC_CHAN, &ri);
            int32_t di = ri - i_off;
            sum_sq_i += (uint64_t)(di * di);
            esp_rom_delay_us(50);
        }
        float i_rms = (sqrtf((float)sum_sq_i / samples) * (ADC_VREF_MV / ADC_MAX_RAW)) / CURR_RATIO;
        float f_curr = (relay_on && i_rms > NOISE_THRESHOLD) ? (i_rms - 0.05f) : 0;
        ESP_LOGI(TAG, "I: %.3fA | P: %.1fW | Relay: %d", f_curr, f_curr * FIXED_VOLTAGE, relay_on);
        if (mqtt_client)
        {
            char payload[128];
            snprintf(payload, sizeof(payload), "{\"curr\":%.3f,\"pwr\":%.1f,\"relay\":%d}", f_curr, f_curr * FIXED_VOLTAGE, relay_on);
            esp_mqtt_client_publish(mqtt_client, "home/c3/status", payload, 0, 1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    if (event_id == MQTT_EVENT_CONNECTED)
        esp_mqtt_client_subscribe(event->client, "home/c3/led", 0);
    else if (event_id == MQTT_EVENT_DATA)
    {
        if (strncmp(event->data, "1", event->data_len) == 0)
        {
            gpio_set_level(RELAY_PIN, 1);
            relay_on = true;
        }
        else if (strncmp(event->data, "0", event->data_len) == 0)
        {
            gpio_set_level(RELAY_PIN, 0);
            relay_on = false;
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_event_group = xEventGroupCreate();

    gpio_reset_pin(RELAY_PIN);
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_PIN, 0);

    esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL);

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_prov_mgr_config_t prov_config = {.scheme = wifi_prov_scheme_ble, .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM};
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_config));

    char service_name[] = "PROV_C3_SMART_HANG";
    const char *pop = "12345678_HANG";
    char qr_payload[150];
    snprintf(qr_payload, sizeof(qr_payload), "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"ble\"}", service_name, pop);
    printf("\n--- QR CODE ---\n");
    esp_qrcode_config_t qrcfg = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&qrcfg, qr_payload);

    wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1, pop, service_name, NULL);

    xTaskCreate(check_reset_button_task, "reset_task", 2048, NULL, 10, NULL);
    xTaskCreate(current_monitor_task, "curr_task", 4096, NULL, 5, NULL);
    xTaskCreate(provisioning_guard_task, "prov_guard", 3072, NULL, 5, NULL); // Task quản lý Bluetooth mới

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, portMAX_DELAY);
    const esp_mqtt_client_config_t mqtt_cfg = {.broker.address.uri = "mqtt://phuongnamdts.com:4783", .credentials.username = "baonammqtt", .credentials.authentication.password = "mqtt@d1git"};
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}