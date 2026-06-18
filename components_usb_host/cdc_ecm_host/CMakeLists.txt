idf_component_register(SRCS "cdc_ecm_host.c" "cdc_host_descriptor_parsing.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES usb esp_event esp_netif esp_eth
                       )
