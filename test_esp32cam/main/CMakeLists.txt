idf_component_register(
  SRCS "test_esp32cam.c"
  INCLUDE_DIRS "."
  REQUIRES esp32-camera esp_http_server esp_wifi esp_event esp_netif nvs_flash
  PRIV_REQUIRES esp_timer
)
