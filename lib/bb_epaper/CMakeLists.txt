idf_component_register(
    SRCS "src/bb_epaper.cpp"
         "src/bb_ep.inl"
         "src/Group5.cpp"
         "src/bb_ep_gfx.inl"
         "src/g5dec.inl"
    INCLUDE_DIRS "src"
    REQUIRES driver esp_timer
)
