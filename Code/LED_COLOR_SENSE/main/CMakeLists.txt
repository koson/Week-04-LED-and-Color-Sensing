idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc driver)

# ล็อกการเชื่อมโยงกับไลบรารี Math ของ C compiler
component_compile_options(-lm)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
