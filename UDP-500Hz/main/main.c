#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_pm.h"

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "sfud.h"

#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* WIFI 宏定义 */
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

bool flag_exp = false;      //实验模式标志
bool flag_test = false;     //测试模式标志
bool flag_export = false;   //数据导出模式标志
bool flag_erease = false;   //擦除flash标志

bool flag_exp_kill = false;     //测试模式自杀标志
bool flag_test_kill = false;     //测试模式自杀标志
bool flag_adc_kill = false;     //adc自杀标志

// UART CONFIG
#define ECHO_TEST_TXD 17
#define ECHO_TEST_RXD 16
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      2
#define ECHO_UART_BAUD_RATE     921600
#define ECHO_TASK_STACK_SIZE    8192
// #define CONFIG_DELAY_LONG_TIME        1000
#define CONFIG_DELAY_SHORT_TIME       100

#define BUF_SIZE (1024)

//WIFI CONFIG
static int sock = -1;

//ADC1 Channels
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_6  //IO34-NET3
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_7  //IO35-ANA_VOLT

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

static int adc_raw[2];
static int voltage[2];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
// static void example_adc_calibration_deinit(adc_cali_handle_t handle);

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1;

//-------------ADC1 Config---------------//
adc_oneshot_chan_cfg_t config;

//-------------ADC1 Calibration Init---------------//
adc_cali_handle_t adc1_cali_chan0_handle;
adc_cali_handle_t adc1_cali_chan1_handle;
bool do_calibration1_chan0;
bool do_calibration1_chan1;

//IMU
static uint8_t go_to_config[] = {0xfa, 0xff, 0x30, 0x00, 0xd1};

/*
0x40, 0x20, Acceleration     0x00, 0x64 100
0x40, 0x40, AccelerationHR   0x01, 0xF4 500

0x80, 0x20, Rate of Turn     0x00, 0x64 100
0x80, 0x40, Rate of TurnHR   0x01, 0xF4 500

0xC0, 0x20, Magnetic Field   0x00, 0x64 100
*/
// static uint8_t  config[] = {0xfa, 0xff, 0xc0, 0x0C, 0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64, 0X29};
static uint8_t config1[] = {0xfa, 0xff, 0xc0, 0x0c, 0x40, 0x40, 0x01, 0xF4, 0x80, 0x40, 0x01, 0xF4, 0x20, 0x30, 0x01, 0x64, 0x56}; // AccelerationHR 500Hz RateOfTurnHR 500Hz Euler Angles 100Hz
static uint8_t config2[] = {0xfa, 0xff, 0xc0, 0x0c, 0x40, 0x40, 0x01, 0xF4, 0x80, 0x40, 0x01, 0xF4, 0x20, 0x30, 0x01, 0x64, 0x56}; // AccelerationHR 500Hz RateOfTurnHR 500Hz Euler Angles 100Hz

// static uint8_t config2[] = {0xfa, 0xff, 0xc0, 0x04, 0x20, 0x30, 0x00, 0x64, 0x89}; 

uint8_t baud_rate[] = {0xfa, 0xff, 0x18, 0x01, 0x80 , 0x68}; // 921600

static uint8_t go_to_measure[] = {0xfa, 0xff, 0x10, 0x00, 0xF1};

// #define IMU_SOURCE_DATA_LEN 50

//flash int  块-扇区-页
#define FLASH_BLOCK_NUMBER                  256     //整个flash的块数量
#define FLASH_BLOCK_BYTES_SIZE              65536   //每个块有多少字节
#define FLASH_TOTAL_SECTOR_NUMBER           4096    //整个flash的扇区数量 4096
#define FLASH_SECTOR_NUMBER                 16      //每个块有多少扇区
#define FLASH_SECTOR_BYTES_SIZE             4096    //每个扇区的字节大小
#define FLASH_TOTAL_PAGE_NUMBER             65536   //整个flash的页数量
#define FLASH_PAGE_NEMBER                   16      //每一个扇区包含多少页
#define FLASH_PAGE_BYTE_SIZE                256     //每一页的字节大小  

#define FLASH_DATA_TEST_MODE_PACKAGE_SIZE       15   //测试模式数据包大小
#define FLASH_DATA_EXP_MODE_SEND_PACKAGE_SIZE   18   //实验模式发送至上位机的数据包大小
#define FLASH_DATA_EXP_MODE_WRITE_PACKAGE_SIZE  256  //实验模式写入flash的数据包大小
#define ADC_PACKAGE_SIZE                        10    //ADC数据包大小

#define FLASH_DATA_PACKAGE_IMU_START    8            //数据包中IMU数据的起始地址
#define FLASH_DATA_PACKAGE_IMU_LEN      12           //每读一次IMU数据的数据大小
#define IMU_DATA_NUMBER                 14           //每个数据包包含几次读取的IMU数据
#define IMU_DARA_NUMBER_NEW             20           //IMU数据包内容的大小   

struct sockaddr_in dest_addr;

//实验模式
static const char* mode_exp_start =    "0105000001d8cc";
static const char* mode_exp_start_ack ="010501000048cc";
static const char* mode_exp_stop =     "0105000100189c";
static const char* mode_exp_stop_ack = "010500100014cc";

//测试模式
static const char* mode_test_start =    "010500000298cd";
static const char* mode_test_start_ack ="0105020000b8cc";
static const char* mode_test_stop =     "0105000200186c";
static const char* mode_test_stop_ack = "010500200000cc";

//数据导出模式
static const char* mode_data_export_start =       "0105000003590d";
static const char* mode_data_export_start_ack =   "0105030000e90c";
// static const char* mode_data_export_stop =        "010500030019fc";
static const char* mode_data_export_stop_ack =    "01050030000d0c";
static const char* mode_data_export_receive_ack = "01010101010101";

//建立连接
static const char* mode_connect_start =     "010500000418cf";
static const char* mode_connect_start_ack = "010504000058cd";
static const char* mode_connect_stop =      "01050004001bcc";
static const char* mode_connect_stop_ack =  "010500400028cc";

//falsh擦除模式
static const char* mode_flash_erease_start =    "0105000005d90f";
static const char* mode_flash_erease_start_ack ="0105050000090d";
// static const char* mode_flash_erease_stop =     "01050005001a5c";
static const char* mode_flash_erease_stop_ack = "0105005000250c";

//数据接收应答
static const char * data_receive_ack =  "10101010101010";

//连接检测
static const char* connect_test  = "0105000006990e";
static const char* connect_test_ack = "0105060000f90d";

//RUN
TaskHandle_t expTaskHandle = NULL;
TaskHandle_t testTaskHandle = NULL;
TaskHandle_t exportdataTaskHandle = NULL;
TaskHandle_t ereaseflashTaskHandle = NULL;
TaskHandle_t ereaseallflashTaskHandle = NULL;
TaskHandle_t stopTaskHandle = NULL;
TaskHandle_t adcTaskHandle = NULL;

esp_pm_lock_handle_t power_lock; // 电源管理锁句柄

static uint8_t data[BUF_SIZE];

void udp_client_task(void *pvParameters);

// static int s_retry_num = 0;
TaskHandle_t udp_client_task_handle = NULL;

void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    static const char *TAG = "wifi station";

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        if (udp_client_task_handle != NULL) 
        {
            vTaskDelete(udp_client_task_handle);
            udp_client_task_handle = NULL;
            ESP_LOGI(TAG, "udp_client_task deleted !");
            vTaskDelete(adcTaskHandle);
            adcTaskHandle = NULL;
            ESP_LOGI(TAG, "adc_task deleted !");
        }

        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 8, &udp_client_task_handle);
        ESP_LOGI(TAG, "udp_client_task created !");
    }
}

void wifi_init_sta(void)
{
    static const char *TAG = "wifi station";

    s_wifi_event_group = xEventGroupCreate(); //创建事件组

    //1. Wi-Fi/LwIP 初始化阶段
    ESP_ERROR_CHECK(esp_netif_init());                //创建一个 LwIP 核心任务，并初始化 LwIP

    ESP_ERROR_CHECK(esp_event_loop_create_default()); //创建一个系统事件任务，并初始化应用程序事件的回调函数
    esp_netif_create_default_wifi_sta();              //创建有 TCP/IP 堆栈的默认网络接口实例绑定 station

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));             //创建 Wi-Fi 驱动程序任务，并初始化 Wi-Fi 驱动程序

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    //2. Wi-Fi 配置阶段                                                 
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

    //3. Wi-Fi 启动阶段
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void adc_task(void *arg)
{
    //ADC_CHANNEL_6  IO34-NET3
    //ADC_CHANNEL_7  IO35-ANA_VOLT = 1/2 VCC_BAT
    static const char *TAG = "adc_task";
    uint8_t adc_data_package[ADC_PACKAGE_SIZE] = {0};
    adc_data_package[0] = 0xFA; //数据包起始字节
    adc_data_package[1] = 0xAF; //数据包起始字节
    while (1)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0]));
        if (do_calibration1_chan0) 
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0], &voltage[0]));
            // ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV， 0x%x", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0], voltage[0]);
        }

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[1]));
        if (do_calibration1_chan1) 
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[1], &voltage[1]));
            // ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV， 0x%x", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, voltage[1], voltage[1]);
        }

        adc_data_package[2] = voltage[0] & 0xFF;
        adc_data_package[3] = (voltage[0] >> 8) & 0xFF;
        adc_data_package[4] = voltage[1] & 0xFF;
        adc_data_package[5] = (voltage[1] >> 8) & 0xFF;

        adc_data_package[6] = adc_raw[0] & 0xFF;
        adc_data_package[7] = (adc_raw[0] >> 8) & 0xFF;
        adc_data_package[8] = adc_raw[1] & 0xFF;
        adc_data_package[9] = (adc_raw[1] >> 8) & 0xFF;

        int err = sendto(sock, adc_data_package, sizeof(adc_data_package), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        } 

        vTaskDelay(pdMS_TO_TICKS(15000));

        if (flag_adc_kill) //测试模式自杀标志
        {
            vTaskDelete(NULL);         //删除任务
        }
    }
}

/*  
test_task:
    测试模式

    0x20, 0x30, Euler Angles     0x00, 0x64 100
    0x40, 0x20, Acceleration     0x00, 0x64 100
    0x40, 0x40, AccelerationHR   0x01, 0xF4 500

    0x80, 0x20, Rate of Turn     0x00, 0x64 100
    0x80, 0x40, Rate of TurnHR   0x01, 0xF4 500

    0xC0, 0x20, Magnetic Field   0x00, 0x64 100

    测试模式数据包

    new:
        0 byte  0xFA
        1 byte  0xFF
        2 byte  0x01 acc    0x02 rot    0x03 euler
        3   3-6  7-10 11-14             4 acc_x + 4 acc_y + 4 acc_z | 4 rot_x + 4 rot_y + 4 rot_z | 4 Roll  + 4 Pitch  + 4 Yaw 
    
*/
static void test_task(void *arg)
{
    static const char *TAG = "test_task";

    int intr_alloc_flags = 0;
    uint8_t test_mode_data_package[FLASH_DATA_TEST_MODE_PACKAGE_SIZE] = {0}; // 用于存储测试模式数据包

    uart_config_t uart_config = 
    {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,


        
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_measure, 5);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_measure, 5);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_measure, 5);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    test_mode_data_package[0] = 0xFA; //数据包起始字节
    test_mode_data_package[1] = 0xFF; //数据包起始字节
    uart_flush(ECHO_UART_PORT_NUM);
    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, IMU_DARA_NUMBER_NEW, 0 / portTICK_PERIOD_MS);
        if(len != 0)
        {
            if (data[0] == 0xFA && data[1]== 0xff && data[2] == 0x36)
            {
                //判断数据类型做好标志
                if (data[4] == 0x40 && data[5] == 0x40)
                {
                    test_mode_data_package[2] = 0x01;  //AccelerationHR
                }
                else if (data[4] == 0x80 && data[5] == 0x40)
                {
                    test_mode_data_package[2] = 0x02;  //Rate of TurnHR
                }
                else if (data[4] == 0x20 && data[5] == 0x30)
                {
                    test_mode_data_package[2] = 0x03;  // Euler Angles
                }
                
                for(int i = 0; i < FLASH_DATA_PACKAGE_IMU_LEN ; i++)
                {
                    test_mode_data_package[3 + i] = data[7 + i];
                }
                int err = sendto(sock, test_mode_data_package, sizeof(test_mode_data_package), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }   
            }
        }


        if (flag_test_kill) //测试模式自杀标志
        {
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_config, 5);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_config, 5);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_config, 5);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            flag_test_kill = false; //清除标志
            uart_driver_delete(ECHO_UART_PORT_NUM);
            sendto(sock, mode_test_stop_ack, strlen(mode_test_stop_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            vTaskDelete(NULL);                //删除任务
        }
    }
}

/*  
exp_task:
    实验模式:

    实验模式数据包 (18byte)
    new:
        0 byte  0xAA
        1 byte  0x01 acc    0x02 rot    0x03 euler
        2  2-5  6-9 10-13     4 acc_x + 4 acc_y + 4 acc_z | 4 rot_x + 4 rot_y + 4 rot_z | 4 Roll  + 4 Pitch  + 4 Yaw 
        14 15 16 17 number 
        .
        .
        .(共计14次)
        .
        .
        234 byte 0xAA
        235 byte 0x01 acc    0x02 rot    0x03 euler
        236 236-239 240-243 244-247   4 acc_x + 4 acc_y + 4 acc_z | 4 rot_x + 4 rot_y + 4 rot_z | 4 Roll  + 4 Pitch  + 4 Yaw
        248 249 250 251 number
        252 253 254 255 reserve
*/
static void exp_task(void *arg)
{
    static const char *TAG = "exp_task";
    const sfud_flash *flash = sfud_get_device_table() + 0;
    int intr_alloc_flags = 0;
    uint32_t flash_write_addr = 0;              //flash 写入起始地址
    uint8_t exp_mode_data_send_package[FLASH_DATA_EXP_MODE_SEND_PACKAGE_SIZE] = {0};   //用于实验模式发送至上位机的数据包
    uint8_t exp_mode_data_write_package[FLASH_DATA_EXP_MODE_WRITE_PACKAGE_SIZE] = {0}; //用于实验模式写入flash的数据包
    uint8_t number = 0;                          //用于实验模式数据包的计数
    uint32_t acc_number = 0;                     //加速度数据计数
    uint32_t rot_number = 0;                     //角速度数据计数
    uint32_t euler_number = 0;                   //欧拉角数据计数

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART pins
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_measure, 5);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_measure, 5);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_measure, 5);
    vTaskDelay(100 / portTICK_PERIOD_MS);


    exp_mode_data_send_package[0] = 0xAA;  //数据包起始字节
    // uart_flush(ECHO_UART_PORT_NUM);
    // esp_pm_lock_acquire(power_lock);
    while(1)
    {
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, IMU_DARA_NUMBER_NEW, 0 / portTICK_PERIOD_MS);
        if (len != 0)
        {            
            if (data[0] == 0xFA && data[1] == 0xff && data[2] == 0x36)
            {
                //判断数据类型做好标志
                if (data[4] == 0x40 && data[5] == 0x40)
                {
                    exp_mode_data_send_package[1] = 0x01;  //AccelerationHR
                    exp_mode_data_send_package[14] = acc_number >> 24; 
                    exp_mode_data_send_package[15] = acc_number >> 16; 
                    exp_mode_data_send_package[16] = acc_number >> 8 ; 
                    exp_mode_data_send_package[17] = acc_number ; 
                    acc_number += 1;
                }
                else if (data[4] == 0x80 && data[5] == 0x40)
                {
                    exp_mode_data_send_package[1] = 0x02;  //Rate of TurnHR
                    exp_mode_data_send_package[14] = rot_number >> 24; 
                    exp_mode_data_send_package[15] = rot_number >> 16; 
                    exp_mode_data_send_package[16] = rot_number >> 8 ; 
                    exp_mode_data_send_package[17] = rot_number ; 
                    rot_number += 1;
                }
                else if (data[4] == 0x20 && data[5] == 0x30)
                {
                    exp_mode_data_send_package[1] = 0x03;  //Euler Angles
                    exp_mode_data_send_package[14] = euler_number >> 24; 
                    exp_mode_data_send_package[15] = euler_number >> 16; 
                    exp_mode_data_send_package[16] = euler_number >> 8 ; 
                    exp_mode_data_send_package[17] = euler_number ; 
                    euler_number += 1;
                }
                
                for(int i = 0; i < FLASH_DATA_PACKAGE_IMU_LEN ; i++)
                {
                    exp_mode_data_send_package[2 + i] = data[7 + i];
                }
                // 发送至上位机
                int err = sendto(sock, exp_mode_data_send_package, sizeof(exp_mode_data_send_package), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                } 

                //将发送数据包的内容写入存储数据包
                for(int j = 0; j < FLASH_DATA_EXP_MODE_SEND_PACKAGE_SIZE ; j++)
                {
                    exp_mode_data_write_package[(number * FLASH_DATA_EXP_MODE_SEND_PACKAGE_SIZE) + j] = exp_mode_data_send_package[j];
                }
                number += 1; 
                if (number  == IMU_DATA_NUMBER)
                {
                    number = 0;

                    //写一页数据
                    sfud_write(flash, flash_write_addr, FLASH_DATA_EXP_MODE_WRITE_PACKAGE_SIZE, exp_mode_data_write_package); 
                    flash_write_addr += FLASH_PAGE_BYTE_SIZE;   //每次写完一页数据地址加256
                    if (flash_write_addr / FLASH_PAGE_BYTE_SIZE > FLASH_TOTAL_PAGE_NUMBER) 
                    {
                        uart_driver_delete(ECHO_UART_PORT_NUM);
                        flag_exp = false;
                        sendto(sock, mode_exp_stop_ack, strlen(mode_exp_stop_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                        ESP_LOGI(TAG, "Stop IMU exp");     
                        vTaskDelete(NULL);
                    }
                }
            }
        } 
        if (flag_exp_kill)  //实验模式自杀标志
        {
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_config, 5);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_config, 5);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_config, 5);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            flag_exp_kill = false;            //清除标志
            uart_driver_delete(ECHO_UART_PORT_NUM);
            sendto(sock, mode_exp_stop_ack, strlen(mode_exp_stop_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            // esp_pm_lock_release(power_lock);
            vTaskDelete(NULL);                //删除任务
        }
    }
}

/*  
flash_erease_task:
    擦除flash模式
*/
void flash_erease_task(void *pvParameters) 
{
    static const char *TAG = "flash_erease_task";
    const sfud_flash *flash = sfud_get_device_table() + 0;
    uint32_t addr = 0;                      //起始擦除地址
    uint32_t size = FLASH_BLOCK_BYTES_SIZE; //每次擦除大小
    uint16_t read_size = 1; //每次读取大小
    uint8_t loop_counter = 0;               //新增计数器变量
    uint8_t read_buf[2] = {0};

    vTaskDelay(100 / portTICK_PERIOD_MS);

    for (int16_t i = 0; i < FLASH_BLOCK_NUMBER; i++)
    {
        sfud_read(flash, addr, read_size, read_buf);
        // ESP_LOGI(TAG, "%u, %u", read_buf[0], read_buf[1]);
        if(read_buf[0] == 0xAA)          //如果有数据
        {
            sfud_erase(flash, addr, size);
            addr += size;                       //更新下一个擦除地址
            // if (result != SFUD_SUCCESS) 
            // {
            //     ESP_LOGW(TAG, "Erase the %s flash data failed at block %d.", flash->name, i);
            //     return;
            // }
            // else                                //擦除成功打印调试信息
            // {
            //     ESP_LOGI(TAG, "Erase the %s flash data finish. Start from %d Block, 0x%08lu, size is %lu.",
            //             flash->name, i, addr - size, size); 
            // }            
        } 
        else
        {
            ESP_LOGI(TAG, "No data in block %d, skip erase.", i);
            break;                              //如果没有数据则跳出循环   
        }
        
        loop_counter++;                     //每次循环计数器加 1
        if (loop_counter >= 10)             //擦除一个块需要237ms, 40个就是9.48s
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            loop_counter = 0;               //重置计数器
        }
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    sendto(sock, mode_flash_erease_stop_ack, strlen(mode_flash_erease_stop_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    ESP_LOGI(TAG, "Erease flash end!");    
    vTaskDelete(NULL);
}

/*  
flash_all_erease_task:
    擦除全部flash模式
*/
void flash_all_erease_task(void *pvParameters) 
{
    static const char *TAG = "flash_all_erease_task";
    sfud_err result = SFUD_SUCCESS;
    const sfud_flash *flash = sfud_get_device_table() + 0;
    uint32_t addr = 0;                      //起始擦除地址
    uint32_t size = FLASH_BLOCK_BYTES_SIZE; //每次擦除大小
    // uint16_t read_size = 2; //每次读取大小
    uint8_t loop_counter = 0;               //新增计数器变量

    vTaskDelay(100 / portTICK_PERIOD_MS);

    for (int16_t i = 0; i < FLASH_BLOCK_NUMBER; i++)
    {
        result = sfud_erase(flash, addr, size);
        addr += size;                       //更新下一个擦除地址
        if (result != SFUD_SUCCESS) 
        {
            ESP_LOGW(TAG, "Erase the %s flash data failed at block %d.", flash->name, i);
            return;
        }
        else                                //擦除成功打印调试信息
        {
            // ESP_LOGI(TAG, "Erase the %s flash data finish. Start from %d Block, 0x%08lu, size is %lu.",
            //         flash->name, i, addr - size, size); 
        }            
        
        loop_counter++;                     //每次循环计数器加 1
        if (loop_counter >= 10)             //擦除一个块需要237ms, 40个就是9.48s
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            loop_counter = 0;               //重置计数器
        }
    }
    
    // sendto(sock, mode_flash_erease_stop_ack, strlen(mode_flash_erease_stop_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    ESP_LOGI(TAG, "Erease flash end!");    
    vTaskDelete(NULL);
}

/*  
export_data_task:
    数据导出模式
*/
static void export_data_task(void *arg)
{
    static const char *TAG = "export_data_task";
    // sfud_err result = SFUD_SUCCESS;
    const sfud_flash *flash = sfud_get_device_table() + 0;
    uint32_t addr = 0;
    size_t size = FLASH_PAGE_BYTE_SIZE; //每次读取一页数据
    uint8_t read_buf[FLASH_PAGE_BYTE_SIZE] = {0};
    // char rx_buffer[30];
    // struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    // socklen_t socklen = sizeof(source_addr);
    uint8_t loop_counter = 0;               //新增计数器变量

    vTaskDelay(100 / portTICK_PERIOD_MS);

    while (1)
    {
        sfud_read(flash, addr, size, read_buf);
        // if (result == SFUD_SUCCESS)
        // {
        //     ESP_LOGI(TAG, "Read the %s flash data success. Page 0x%05lX, Start from 0x%08lu, size is %u.",
        //         flash->name, addr/FLASH_PAGE_BYTE_SIZE, addr, FLASH_PAGE_BYTE_SIZE);
        // } 


        if (read_buf[0] == 0xAA)//如果此页有数据
        {
            while(1)
            {
                // memset(rx_buffer, 0, sizeof(rx_buffer));
                sendto(sock, read_buf, sizeof(read_buf), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                uint32_t ulNotifiedValue = ulTaskNotifyTake(pdTRUE, 200 / portTICK_PERIOD_MS);
                if (ulNotifiedValue != 0)
                {
                    addr += FLASH_PAGE_BYTE_SIZE;                                       //每次读取一页数据地址加256
                    break;
                }
            }

            if (addr / FLASH_PAGE_BYTE_SIZE > FLASH_TOTAL_PAGE_NUMBER) 
            {
                break;
            }

            loop_counter++;                     //每次循环计数器加 1
            if (loop_counter >= 20)             
            {
                vTaskDelay(20 / portTICK_PERIOD_MS);
                loop_counter = 0;               //重置计数器
            }
        }
        else
        {
            break; 
        }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    flag_export = false;
    sendto(sock, mode_data_export_stop_ack, strlen(mode_data_export_stop_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    ESP_LOGI(TAG, "Export flash data end!");   
    vTaskDelete(NULL);
}

/*  
stop_connect_task:
    停止连接任务：
        停止连接任务用于断开连接后，将系统状态恢复至重新连接模式
*/
static void stop_connect_task(void *arg)
{
    static const char *TAG = "stop_connect_task";

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    sendto(sock, mode_connect_stop_ack, strlen(mode_connect_stop_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    vTaskDelete(adcTaskHandle);
    adcTaskHandle = NULL;
    ESP_LOGI(TAG, "adc_task deleted !");

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 8, &udp_client_task_handle);
    ESP_LOGI(TAG, "udp_client_task created !");
    vTaskDelete(NULL);
}

/*  
void udp_client_task:
    UDP数据监听任务：
*/
void udp_client_task(void *pvParameters)
{
    static const char *TAG = "udp_client_task";

    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1)
    {
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 2;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
        while (1)
        {
            int err = sendto(sock, mode_connect_start, strlen(mode_connect_start), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d !", errno);
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                // ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                // ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, mode_connect_start_ack, len-1) == 0)
                {
                    // ESP_LOGI(TAG, "Start connect success!");   
                    break; // 成功接收到应答，跳出循环
                }
                else
                {
                    ESP_LOGI(TAG, "Start connect fail!");   
                }
            }
        }

        // Set timeout
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        xTaskCreate(adc_task, "adc_task", ECHO_TASK_STACK_SIZE, NULL, 7, &adcTaskHandle);
        ESP_LOGI(TAG, "Start ADC task");
        while (1)
        {
            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                // ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                // ESP_LOGI(TAG, "%s", rx_buffer);

                if (strncmp(rx_buffer, mode_test_start, len-1) == 0)            //启动测试模式
                {
                    flag_test = true;
                    flag_exp = false;
                    flag_export = false;
                    flag_erease = false;

                    flag_test_kill = false;                     //清除测试模式杀死标志

                    xTaskCreate(test_task, "test_task", ECHO_TASK_STACK_SIZE, NULL, 7, &testTaskHandle);
                    sendto(sock, mode_test_start_ack, strlen(mode_test_start_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    ESP_LOGI(TAG, "Start IMU test");
                }
                else if (strncmp(rx_buffer, mode_test_stop, len-1) == 0)        //停止测试模式
                {
                    flag_test = false;                          //清除标志
                    flag_test_kill = true;                      //设置测试模式杀死标志
                }
                else if (strncmp(rx_buffer, mode_exp_start, len-1) == 0)        //启动实验模式
                {
                    flag_exp = true;
                    flag_test = false;
                    flag_export = false;
                    flag_erease = false;

                    xTaskCreate(exp_task, "exp_task", ECHO_TASK_STACK_SIZE, NULL, 7, &expTaskHandle);
                    sendto(sock, mode_exp_start_ack, strlen(mode_exp_start_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    ESP_LOGI(TAG, "Start IMU exp");    
                }
                else if (strncmp(rx_buffer, mode_exp_stop, len-1) == 0)         //停止实验模式
                {
                    flag_exp = false;
                    flag_exp_kill = true;                      //设置实验模式杀死标志
                }
                else if (strncmp(rx_buffer, mode_flash_erease_start, len-1) == 0) //启动擦除模式
                {
                    flag_exp = false;
                    flag_test = false;
                    flag_export = false;
                    flag_erease = true;

                    xTaskCreate(flash_erease_task, "flash_erease_task", ECHO_TASK_STACK_SIZE, NULL, 7, &ereaseflashTaskHandle);
                    sendto(sock, mode_flash_erease_start_ack, strlen(mode_flash_erease_start_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    ESP_LOGI(TAG, "Start erease flash ");    
                }
                else if (strncmp(rx_buffer, mode_data_export_start, len-1) == 0) //启动数据导出模式
                {
                    flag_exp = false;
                    flag_test = false;
                    flag_export = true;
                    flag_erease = false;

                    xTaskCreate(export_data_task, "export_data_task", ECHO_TASK_STACK_SIZE, NULL, 9, &exportdataTaskHandle);
                    sendto(sock, mode_data_export_start_ack, strlen(mode_data_export_start_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    ESP_LOGI(TAG, "Start export flash data");    
                }
                else if (strncmp(rx_buffer, mode_data_export_receive_ack, len-1) == 0) //发送的数据上位机接收到了
                {
                    xTaskNotifyGive(exportdataTaskHandle);
                }
                else if (strncmp(rx_buffer, mode_connect_stop, len-1) == 0)        //停止连接
                {
                    xTaskCreate(stop_connect_task, "stop_connect_task", ECHO_TASK_STACK_SIZE, NULL, 7, &stopTaskHandle);
                    ESP_LOGI(TAG, "Stop connect task created !");
                    vTaskDelete(NULL);
                    ESP_LOGI(TAG, "UDP client task delete!");
                }
                else if (strncmp(rx_buffer, connect_test, len-1) == 0)     
                {
                    sendto(sock, connect_test_ack, strlen(connect_test_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    // ESP_LOGI(TAG, "connect_test !");
                }
                else//未知消息
                {
                    sendto(sock, data_receive_ack, strlen(data_receive_ack), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                }
            }
        }
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


void app_main(void)
{
    static const char *TAG = "app_main";
    int intr_alloc_flags = 0;

    if (sfud_init() == SFUD_SUCCESS) 
    {
        ESP_LOGI(TAG, "IMU config !");

        // Configure UART parameters
        uart_config_t uart_config = {
            .baud_rate = ECHO_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

        // Configure UART pins
        ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, intr_alloc_flags));
        ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
        
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_config, 5);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_config, 5);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        // uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) config2, sizeof(config2));
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_measure, 5);
        // vTaskDelay(100 / portTICK_PERIOD_MS);
        // uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) go_to_measure, 5);
        // vTaskDelay(100 / portTICK_PERIOD_MS);

        uart_driver_delete(ECHO_UART_PORT_NUM);                            
        
        // Initialize NVS                                           
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
          ESP_ERROR_CHECK(nvs_flash_erase());
          ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

        //-------------ADC1 Init---------------//
        init_config1.unit_id = ADC_UNIT_1;
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
        //-------------ADC1 Config---------------//
        config.atten = EXAMPLE_ADC_ATTEN;
        config.bitwidth = ADC_BITWIDTH_DEFAULT;

        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

        //-------------ADC1 Calibration Init---------------//
        adc1_cali_chan0_handle = NULL;
                adc1_cali_chan1_handle = NULL;
        do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
        do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);

        // xTaskCreate(flash_all_erease_task, "flash_all_erease_task", ECHO_TASK_STACK_SIZE, NULL, 7, &ereaseallflashTaskHandle);

        esp_pm_config_esp32_t pm_config = {
            .max_freq_mhz = 240,         // 最高频率，纽扣电池建议不超过 80
            .min_freq_mhz = 20,         // 最低频率 (XTAL 频率)
            .light_sleep_enable = true  // 开启自动轻度睡眠！
        };
        ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
        esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "ImportantMode", &power_lock);
        wifi_init_sta();
    }
    else
    {
        ESP_LOGE(TAG, "SFUD initialization failed");
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    char *TAG = "ADC";
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) 
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = 
        {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) 
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}