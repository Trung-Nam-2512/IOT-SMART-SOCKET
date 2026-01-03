idf_component_register(
    SRCS "app_main_copy.c" "app_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES 
        nvs_flash        # Thêm cái này để hết lỗi nvs_flash.h
        esp_wifi         # Cần cho WiFi
        esp_event        # Cần cho Event Loop
        esp_netif        # Cần cho TCP/IP stack
        wifi_provisioning 
        qrcode 
        mqtt 
        esp_adc
        dht              # Component cảm biến độ ẩm của bạn
        driver
)