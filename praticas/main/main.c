/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

////////////////////////////////////////////////////
#define GPIO_INPUT_IO_21    21
#define GPIO_INPUT_IO_22    22
#define GPIO_INPUT_IO_23    23
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_21) | (1ULL<<GPIO_INPUT_IO_22) | (1ULL<<GPIO_INPUT_IO_23)) 
////////////////////////////////////////////////////
#define GPIO_OUTPUT_IO_2    2
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_2))
////////////////////////////////////////////////////
#define ESP_INTR_FLAG_DEFAULT 0
static QueueHandle_t gpio_evt_queue = NULL;
// Variáveis para debounce
static int tickAnterior = 0;
#define TEMPO_DEBOUNCE 250

static const char* TAG = "PRATICA1";
static const char* TAG2 = "PRATICA2";

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void pratica02(void* arg)
{
    uint32_t io_num;
    char acendeLed = 0;
    int tickAtual;
    for (;;) {          
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            tickAtual = xTaskGetTickCount();
            if (io_num==23)
            {
                // Converte milissegundos para ticks
                int debounce_ticks = pdMS_TO_TICKS(TEMPO_DEBOUNCE);

                // Se a diferença de tempo for menor que o tempo de debounce, ignora o evento
                if (tickAtual - tickAnterior < debounce_ticks) {
                    continue;
                }
                
                // Atualiza o tempo do último clique
                tickAnterior = tickAtual;
                ESP_LOGI(TAG2,"Acendeu");
                acendeLed = acendeLed ^ 1;
                gpio_set_level(GPIO_OUTPUT_IO_2, acendeLed);
            
            }
            else
            {
                acendeLed = io_num%2;
                gpio_set_level(GPIO_OUTPUT_IO_2, acendeLed);
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG,"Ola, mundo!"); // O esp-idf gerou esse comando sozinho
    //esp_log_level_set(TAG,ESP_LOG_WARN);
    /* Print chip information */
    esp_chip_info_t chip_info;  // Criacao do struct para informacoes
    uint32_t flash_size;        // Variavel para tamanho do flash
    esp_chip_info(&chip_info);  // Funcao que preenche o struct com as informacoes do chip

    /*
    CONFIG_IDF_TARGET = Modelo
    chip_info.cores = Nª de nucleos
    (chip_info.features & CHIP_FEATURE_WIFI_BGN) Verifica se constante wifi existe no struct e realiza operacao tenaria
    Se sim sai wifi, se nao a string eh nula
    (chip_info.features & CHIP_FEATURE_BT) Mesma coisa verificando se existe capacidades de bluetooth classic
    (chip_info.features & CHIP_FEATURE_BLE) Mesma coisa verificando se existe capacide de bluetooth low energy
    (chip_info.features & CHIP_FEATURE_IEEE802154) Mesma coisa verificando se possui suporte zigbee
    */
    ESP_LOGI(TAG,"Esse é o %s chip com %d CPU nucleo(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    ESP_LOGI(TAG,"%d",chip_info.revision);  // chip_info.revision = 100
    unsigned major_rev = chip_info.revision / 100;  // Revisao de Chip, provavelmente no formato XXX
    unsigned minor_rev = chip_info.revision % 100;  // Revisao de Chip, chip_info.revision = 100, portanto zero
    ESP_LOGI(TAG,"Versao do Esp v%d.%d, ", major_rev, minor_rev);

    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) // Verifica se eh possivel obter
    {
        ESP_LOGE(TAG,"Nao foi possivel obter valor da flash");
        return;
    }

    ESP_LOGE(TAG,"Testando Msg de Erro");

    ESP_LOGI(TAG,"%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "interna" : "externa");
    ESP_LOGI(TAG,"Minimo espaco livre no Heap: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 2; i >= 0; i--) {
        ESP_LOGI(TAG,"Testando vTask durante %d seconds...\n",i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    fflush(stdout);
    
    ////////////////////////////////////////////////////
    //ENTRADAS DOS BOTOES
    gpio_config_t io_conf = {};
    //interrupcao borda de descida
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // Habilita Pull Up
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //SAIDA LED, VOU REINICIAR POR SEGURANCA
    //io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // CONFIGURA QUEU E TASK

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(pratica02, "pratica02", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_21, gpio_isr_handler, (void*) GPIO_INPUT_IO_21);
    gpio_isr_handler_add(GPIO_INPUT_IO_22, gpio_isr_handler, (void*) GPIO_INPUT_IO_22);
    gpio_isr_handler_add(GPIO_INPUT_IO_23, gpio_isr_handler, (void*) GPIO_INPUT_IO_23);

    while (1)
    {
        ESP_LOGW(TAG,"Travou");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
