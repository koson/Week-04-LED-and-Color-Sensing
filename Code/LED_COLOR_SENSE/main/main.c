#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ส่วนควบคุมสำหรับขา Digital I/O แบบสากลในระบบเวอร์ชัน 5
#include "driver/gpio_filter.h" 
#include "rom/gpio.h"

static const char *TAG = "RGB_ACTIVE_SENSING";

// -----------------------------------------------------------------
// CONFIGURATION: กำหนดขา GPIO และ ADC
// -----------------------------------------------------------------
// ขาควบคุม LED RGB ภาคส่ง (ปรับเปลี่ยนขาได้ตามความเหมาะสมของบอร์ด ESP32-C6)
#define TX_LED_R_GPIO        GPIO_NUM_4
#define TX_LED_G_GPIO        GPIO_NUM_5
#define TX_LED_B_GPIO        GPIO_NUM_6

// ขาอ่านสัญญาณ ADC ภาครับ (ใช้ ADC1 CH2 ตรงกับขา GPIO2 ของ ESP32-C6)
#define RX_ADC_UNIT          ADC_UNIT_1
#define RX_ADC_CHANNEL       ADC_CHANNEL_2
#define V_REF                3300  // แรงดันอ้างอิง 3.3V (3300 mV)

#define NUM_SAMPLES          50    // จำนวนตัวอย่างที่ต้องการสุ่มอ่านต่อหนึ่งรอบสี

// โครงสร้างตัวแปรระบบ Calibrate ของ ESP-IDF
static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_cali = false;

// -----------------------------------------------------------------
// ฟังก์ชันตั้งค่าระบบฮาร์ดแวร์
// -----------------------------------------------------------------
void init_hardware(adc_oneshot_unit_handle_t *adc_handle)
{
    // 1. ตั้งค่าขา GPIO สำหรับ LED RGB ภาคส่ง เป็น Output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TX_LED_R_GPIO) | (1ULL << TX_LED_G_GPIO) | (1ULL << TX_LED_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // ดับไฟ LED ทุกสีเริ่มต้น (กรณีต่อแบบ Common Cathode จ่าย LOW = ดับ)
    gpio_set_level(TX_LED_R_GPIO, 1);
    gpio_set_level(TX_LED_G_GPIO, 1);
    gpio_set_level(TX_LED_B_GPIO, 1);

    // 2. ตั้งค่า ADC Unit 1
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = RX_ADC_UNIT,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, adc_handle));

    // 3. ตั้งค่า ADC Channel (ความละเอียด 12 บิต, ลดทอน 12dB เพื่อรับแรงดันช่วง 0 - 3.3V)
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, RX_ADC_CHANNEL, &chan_config));

    // 4. เปิดใช้งานระบบปรับเทียบแรงดัน (eFuse Hardware Calibration Line Fitting)
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = RX_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        do_cali = true;
    }
#endif
}

int compare_ints(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

// -----------------------------------------------------------------
// ฟังก์ชันอ่านค่าดิบ กรองสัญญาณรบกวน แล้วค่อยแปลงเป็นมิลลิโวลต์ตอนท้าย
// -----------------------------------------------------------------
void process_color_sensing(adc_oneshot_unit_handle_t adc_handle, const char *color_name)
{
    int raw_samples[NUM_SAMPLES];
    
    // หน่วงเวลาหลังเปิดไฟ 2s พอ เพื่อให้ไฟฟ้าเข้าสู่ Steady State 
    vTaskDelay(pdMS_TO_TICKS(2000)); 

    // 1. อ่านค่าดิจิทัลดิบ (Raw 0-4095) เก็บลงอาร์เรย์อย่างรวดเร็ว (ห่างกันแซมเปิ้ลละ 20ms)
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int raw_value = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, RX_ADC_CHANNEL, &raw_value));
        raw_samples[i] = raw_value;
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }

    // 2. จัดเรียงลำดับระดับบิตดิบจากน้อยไปมาก
    qsort(raw_samples, NUM_SAMPLES, sizeof(int), compare_ints);

    // 3. ทำ Trimmed กรอง Outliers ออก (ตัดหัวท้ายฝั่งละ 10%)
    int trim_count = NUM_SAMPLES * 0.10; 
    int valid_count = NUM_SAMPLES - (2 * trim_count);
    
    double raw_sum = 0.0;
    for (int i = trim_count; i < NUM_SAMPLES - trim_count; i++) {
        raw_sum += raw_samples[i];
    }

    // 4. หาค่าเฉลี่ยระดับบิตดิบ (Mean Raw)
    double mean_raw = raw_sum / valid_count;

    // 5. คำนวณส่วนเบี่ยงเบนมาตรฐาน (SD) ในรูปแบบระดับบิตดิบ
    double variance_sum = 0.0;
    for (int i = trim_count; i < NUM_SAMPLES - trim_count; i++) {
        variance_sum += pow((raw_samples[i] - mean_raw), 2);
    }
    double sd_raw = sqrt(variance_sum / (valid_count - 1));

    // 6. แปลงผลลัพธ์สุดท้ายจากระดับบิตดิบ (Bit scale) ให้เป็นแรงดันทางกายภาพ (mV)
    int final_voltage_mv = 0;
    int sd_voltage_mv = 0;

    if (do_cali) {
        // แปลงค่าเฉลี่ยดิบเป็นมิลลิโวลต์
        adc_cali_raw_to_voltage(adc_cali_handle, (int)mean_raw, &final_voltage_mv);
        // แปลงระยะกระจายดิบ (SD) เป็นมิลลิโวลต์เชิงเส้น
        adc_cali_raw_to_voltage(adc_cali_handle, (int)sd_raw, &sd_voltage_mv);
        
        // ชดเชยค่าจุดตัดแกน (Offset) สำหรับค่า SD ที่ได้จากการแปลง
        int zero_voltage_offset = 0;
        adc_cali_raw_to_voltage(adc_cali_handle, 0, &zero_voltage_offset);
        sd_voltage_mv = abs(sd_voltage_mv - zero_voltage_offset);
    } else {
        // สูตรสำรองกรณีฮาร์ดแวร์ Calibrate ไม่ทำงาน
        final_voltage_mv = ((int)mean_raw * V_REF) / 4095;
        sd_voltage_mv = ((int)sd_raw * V_REF) / 4095;
    }

    // ป้องกันกรณีแสงมืดสนิทแล้ว ADC แกว่งที่เลข 0-5 บิต ไม่ให้แสดงผลติดลบหรือเพี้ยน
    if (mean_raw <= 2.0) {
        final_voltage_mv = 0;
        sd_voltage_mv = 0;
    }

    // 7. พิมพ์ผลออกทาง Serial Port ตามฟอร์แมตที่ใบงานกำหนด
    printf("Color %s, n = %d (filtered), mean = %d.00, sd = %d.00\n", 
           color_name, valid_count, final_voltage_mv, sd_voltage_mv);
}

// -----------------------------------------------------------------
// Main App Loop
// -----------------------------------------------------------------
void app_main(void)
{
    adc_oneshot_unit_handle_t adc1_handle;
    init_hardware(&adc1_handle);

    ESP_LOGI(TAG, "Active Sensing System Initialized successfully.");
    printf("==============================================================\n");

    while (1) {
        // --- PHASE 1: เปิดเฉพาะแสงสีแดง (R) ---
        gpio_set_level(TX_LED_R_GPIO, 0);
        gpio_set_level(TX_LED_G_GPIO, 1);
        gpio_set_level(TX_LED_B_GPIO, 1);
        process_color_sensing(adc1_handle, "R");

        // --- PHASE 2: เปิดเฉพาะแสงสีเขียว (G) ---
        gpio_set_level(TX_LED_R_GPIO, 1);
        gpio_set_level(TX_LED_G_GPIO, 0);
        gpio_set_level(TX_LED_B_GPIO, 1);
        process_color_sensing(adc1_handle, "G");

        // --- PHASE 3: เปิดเฉพาะแสงสีน้ำเงิน (B) ---
        gpio_set_level(TX_LED_R_GPIO, 1);
        gpio_set_level(TX_LED_G_GPIO, 1);
        gpio_set_level(TX_LED_B_GPIO, 0);
        process_color_sensing(adc1_handle, "B");

        // ดับไฟทั้งหมดและหน่วงเวลา 1.5 วินาที ก่อนเริ่มต้นการสแกนรอบใหม่
        gpio_set_level(TX_LED_R_GPIO, 1);
        gpio_set_level(TX_LED_G_GPIO, 1);
        gpio_set_level(TX_LED_B_GPIO, 1);
        printf("--------------------------------------------------------------\n");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}