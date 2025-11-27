idf_component_register(
    SRCS "microgreen_test18.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        driver
        esp_driver_gpio
        esp_http_client
        esp-tls
        esp_wifi
        nvs_flash
        esp_netif
        esp_event
        wifi_provisioning
        protocomm
        esp_timer
)
