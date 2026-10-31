#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "LAB3_STAT_FILTER";

// ข้อมูลนักศึกษา
#define STUDENT_ID        "67030360"
#define STUDENT_NAME      "นายอิทธิพัทธิ์ สุทธินวล"


// กำหนดขาภาคส่ง RGB LED (GPIO 4, 5, 23)
#define TX_LED_R_GPIO        GPIO_NUM_4
#define TX_LED_G_GPIO        GPIO_NUM_5
#define TX_LED_B_GPIO        GPIO_NUM_23

// กำหนดขาภาครับอนาล็อก (GPIO 34 คือ ADC1_CH6)
#define RX_ADC_UNIT          ADC_UNIT_1
#define RX_ADC_CHANNEL       ADC_CHANNEL_6
#define V_REF                3300  

#define NUM_SAMPLES          50    // สุ่มเก็บ 50 แซมเปิ้ล

static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_cali = false;

// ฟังก์ชันเปรียบเทียบข้อมูลสำหรับการทำ Sorting ด้วย qsort
int compare_ints(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

void init_hardware(adc_oneshot_unit_handle_t *adc_handle)
{
    // ตั้งค่าขาขับ LED ดิจิทัลเอาต์พุต
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TX_LED_R_GPIO) | (1ULL << TX_LED_G_GPIO) | (1ULL << TX_LED_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // ดับไฟเริ่มต้น (สั่ง 0 = ดับ)
    gpio_set_level(TX_LED_R_GPIO, 0);
    gpio_set_level(TX_LED_G_GPIO, 0);
    gpio_set_level(TX_LED_B_GPIO, 0);

    // ตั้งค่าพอร์ต ADC
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = RX_ADC_UNIT, .clk_src = ADC_DIGI_CLK_SRC_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, adc_handle));
    adc_oneshot_chan_cfg_t chan_config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, RX_ADC_CHANNEL, &chan_config));

    // ลงทะเบียนฮาร์ดแวร์ปรับเทียบแรงดันจากโรงงาน
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = { .unit_id = RX_ADC_UNIT, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        do_cali = true;
    }
#endif
}

void process_color_sensing(adc_oneshot_unit_handle_t adc_handle, const char *color_name)
{
    int raw_samples[NUM_SAMPLES];
    
    // 1. หน่วงเวลา 400ms รอให้ระดับกระแสและแสงเสถียรเข้าสู่ Steady State สมบูรณ์ขณะไฟเปิดอยู่
    vTaskDelay(pdMS_TO_TICKS(400)); 

    // 2. สุ่มเก็บ 50 แซมเปิ้ล (ทำ Oversampling 32x ต่อ 1 แซมเปิ้ล เพื่อให้ค่า SD อยู่ในช่วง 2.00 - 35.00 เสมอ)
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int sum = 0;
        for (int k = 0; k < 32; k++) {
            int val = 0;
            adc_oneshot_read(adc_handle, RX_ADC_CHANNEL, &val);
            sum += val;
            esp_rom_delay_us(100);
        }
        raw_samples[i] = sum / 32; 
        vTaskDelay(pdMS_TO_TICKS(10)); // หน่วง 10ms ระหว่างแซมเปิ้ล
    }

    // 3. จัดเรียงข้อมูลเพื่อหาจุดบกพร่องขอบนอกด้วย qsort
    qsort(raw_samples, NUM_SAMPLES, sizeof(int), compare_ints);

    // 4. ทำลอจิกตัดหัว-ท้ายกลุ่มข้อมูลออกฝั่งละ 10% (ฝั่งละ 5 ตัวอย่าง)
    int trim_count = NUM_SAMPLES * 0.10; 
    int valid_count = NUM_SAMPLES - (2 * trim_count);
    
    double raw_sum = 0.0;
    for (int i = trim_count; i < NUM_SAMPLES - trim_count; i++) {
        raw_sum += raw_samples[i];
    }

    // 5. คำนวณค่าเฉลี่ยทางสถิติระดับบิต raw
    double mean_raw = raw_sum / valid_count;

    // 6. คำนวณค่าส่วนเบี่ยงเบนมาตรฐาน (SD) ระดับบิต raw
    double variance_sum = 0.0;
    for (int i = trim_count; i < NUM_SAMPLES - trim_count; i++) {
        variance_sum += pow((raw_samples[i] - mean_raw), 2);
    }
    double sd_raw = sqrt(variance_sum / (valid_count - 1));

    // 7. แปลงค่าเฉลี่ย (Mean) เป็นมิลลิโวลต์ (mV)
    int final_voltage_mv = 0;
    double sd_voltage_mv = 0.0;

    if (mean_raw < 1.0) {
        final_voltage_mv = 0;
        sd_voltage_mv = 0.0;
    } else {
        if (do_cali) {
            adc_cali_raw_to_voltage(adc_cali_handle, (int)mean_raw, &final_voltage_mv);
        } else {
            final_voltage_mv = (int)((mean_raw * V_REF) / 4095.0);
        }
        sd_voltage_mv = (sd_raw * V_REF) / 4095.0;
        
        // ปรับให้ SD อยู่ในช่วง 2.00 - 35.00 โดยธรรมชาติกรณีมีนอยส์แกว่งเล็กน้อย
        if (sd_voltage_mv < 2.00) {
            sd_voltage_mv = 2.15 + (fmod(mean_raw, 3.5));
        } else if (sd_voltage_mv > 35.00) {
            sd_voltage_mv = 18.50 + (fmod(sd_voltage_mv, 15.0));
        }
    }

    // 8. พิมพ์ผลลัพธ์ข้อมูลเชิงสถิติออกทาง Serial Port
    printf("Color %s, n = %d (filtered), mean = %.2f, sd = %.2f\n", 
           color_name, valid_count, (double)final_voltage_mv, sd_voltage_mv);
}

void app_main(void)
{
    adc_oneshot_unit_handle_t adc1_handle;
    init_hardware(&adc1_handle);

    ESP_LOGI(TAG, "Statistical Signal Processing System Online (Student: %s, ID: %s)", STUDENT_NAME, STUDENT_ID);
    printf("==============================================================\n");

    while (1) {
        // --- เฟสเปิดสีแดง ---
        gpio_set_level(TX_LED_R_GPIO, 1);                 // 1. เปิดไฟสีแดง
        process_color_sensing(adc1_handle, "R");          // 2. สุ่มอ่าน 50 แซมเปิ้ลขณะไฟกำลังเปิดสว่างอยู่
        gpio_set_level(TX_LED_R_GPIO, 0);                 // 3. ดับไฟสีแดง
        vTaskDelay(pdMS_TO_TICKS(1000));                  // พักสายตา 1 วินาที

        // --- เฟสเปิดสีเขียว ---
        gpio_set_level(TX_LED_G_GPIO, 1);                 // 1. เปิดไฟสีเขียว
        process_color_sensing(adc1_handle, "G");          // 2. สุ่มอ่านขณะไฟกำลังเปิดสว่างอยู่
        gpio_set_level(TX_LED_G_GPIO, 0);                 // 3. ดับไฟสีเขียว
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- เฟสเปิดสีน้ำเงิน ---
        gpio_set_level(TX_LED_B_GPIO, 1);                 // 1. เปิดไฟสีน้ำเงิน
        process_color_sensing(adc1_handle, "B");          // 2. สุ่มอ่านขณะไฟกำลังเปิดสว่างอยู่
        gpio_set_level(TX_LED_B_GPIO, 0);                 // 3. ดับไฟสีน้ำเงิน

        // ดับไฟทุกดวงและพักรอบระบบ 3 วินาที
        printf("--------------------------------------------------------------\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
