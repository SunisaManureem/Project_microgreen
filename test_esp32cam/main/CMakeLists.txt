idf_component_register(
    SRCS "test_esp32cam.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_http_server
        wifi_provisioning
)
