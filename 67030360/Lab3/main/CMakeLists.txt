idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc driver)

#  link ตัวเชื่อมคอมไพเลอร์เข้ากับ Library ฟังก์ชันคณิตศาสตร์สถิติ <math.h>
component_compile_options(-lm)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
