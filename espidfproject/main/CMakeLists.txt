idf_component_register(SRCS "xiao_wakeword.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES audio_pipeline audio_stream esp-tls esp_http_client
                    esp-adf-libs audio_sal esp_peripherals fatfs
)