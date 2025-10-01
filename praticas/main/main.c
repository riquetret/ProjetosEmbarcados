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
#include "driver/gptimer.h"

//////////////////////////////////////////////////// PRATICA 2
#define GPIO_INPUT_IO_21    21  //Botao 0
#define GPIO_INPUT_IO_22    22  //Botao 1
#define GPIO_INPUT_IO_23    23  //Botao 2
//1ULL é uma palavra de 64bits com o primeiro bit em 1, depois eh feito o deslocamento para configuracao correta
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_21) | (1ULL<<GPIO_INPUT_IO_22) | (1ULL<<GPIO_INPUT_IO_23)) 

#define GPIO_OUTPUT_IO_2    2   //LED DA PLACA
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_2))

#define ESP_INTR_FLAG_DEFAULT 0 //Flag da interrupcao de IO
static QueueHandle_t gpio_evt_queue = NULL;

// Variáveis para debounce PRATICA 2
static unsigned int tickAnterior = 0;    //Tick da ultima mudanca do led quando pressiona Botao 2 (GPIO23)
#define TEMPO_DEBOUNCE 250  //250ms de debounce

//////////////////////////////////////////////////// PRATICA 3
typedef struct {
    uint64_t contAtualTimer;
    uint64_t valorAlarme;
} filaTimer1;

static QueueHandle_t timer1Queue = NULL;
static gptimer_handle_t gptimer1 = NULL;

//////////////////////////////////////////////////// TAGS
static const char* TAG = "Pratica-1";
static const char* TAG2 = "Pratica-2";
static const char* TAG3 = "Pratica-3";

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    /*  Gpio_num recebe o pino que sofreu interrupcao, 
        o argumento eh o endereco "GPIO_INPUT_IO_XX"(21,22,23) 
        que eh convertido para um inteiro */
    uint32_t gpio_num = (uint32_t) arg; 
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void configuraIO_pratica02(){
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
    memset(&io_conf, 0, sizeof(io_conf)); // Reseta todo o struct para zero
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
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_21, gpio_isr_handler, (void*) GPIO_INPUT_IO_21); //Atribue a interrupcao para funcao gpio_isr_handler
    gpio_isr_handler_add(GPIO_INPUT_IO_22, gpio_isr_handler, (void*) GPIO_INPUT_IO_22);
    gpio_isr_handler_add(GPIO_INPUT_IO_23, gpio_isr_handler, (void*) GPIO_INPUT_IO_23);
}

static void pratica02(void* arg)
{
    configuraIO_pratica02();
    char mensagens[][8] = {"Apagou","Acendeu","Cambiou"};   //Se GPIO22 mostra apaga, se GPIO21 mostra acende, se GPIO23 cambia led
    char mensagen2[] = "LED";
    uint32_t io_num;    // Qual GPIO foi acionado
    char acendeLed = 0; // Variavel que representa o nivel logica do led
    unsigned int tickAtual;  
    // Converte milissegundos para ticks
    unsigned short int debounce_ticks = pdMS_TO_TICKS(TEMPO_DEBOUNCE);
    for (;;) {          
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {    //Coloca task bloqueada enquanto nao chegar evento GPIO
            tickAtual = xTaskGetTickCount();    //Captura tempo atual
            if (io_num==23)
            {
                // Se a diferença de tempo for menor que o tempo de debounce, ignora o evento
                if (tickAtual - tickAnterior < debounce_ticks) {
                    continue;
                }
                
                // Atualiza o tempo do último clique
                tickAnterior = tickAtual;
                ESP_LOGI(TAG2,"%s %s",mensagens[2],mensagen2);
                acendeLed = acendeLed ^ 1;  // Troca Nivel Logico
                gpio_set_level(GPIO_OUTPUT_IO_2, acendeLed);
            }
            else if (io_num>20 && io_num<23)
            {
                acendeLed = io_num&1;   // Se GPIO21 io_num=21 (Botao 0) temos resultado 1, se GPIO22 (Botao 1) temos resultado 0
                ESP_LOGI(TAG2,"%s %s",mensagens[(size_t)acendeLed],mensagen2);
                gpio_set_level(GPIO_OUTPUT_IO_2, acendeLed);
            }
        }
    }
}

static bool IRAM_ATTR Timer1Interrupcao(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    QueueHandle_t queue = (QueueHandle_t)user_data;
    filaTimer1 retornoFila = {
        .contAtualTimer = edata->count_value,
        .valorAlarme = edata->alarm_value
    };
    xQueueSendFromISR(queue, &retornoFila, NULL);
    return 1;
}

void AtualizaHoras(unsigned short int* ptr){
    *(ptr+2) = *(ptr+2) + 1;    //Soma 1 Segundo
    if (*(ptr+2)==60)
    {
        *(ptr+2) = 0; // Reseta Segundos
        *(ptr+1) = *(ptr+1) + 1; // Avanca Minutos
    }
    if(*(ptr+1)==60){
        *(ptr+1) = 0; // Reseta Minutos
        *(ptr) = *(ptr) + 1; // Avanca Horas
    }
    if(*(ptr)==24){
        *(ptr) = 0; // Reseta Horas
    }
}

static void pratica03(void* arg){
    static unsigned short int horas[] = {23,59,57};
    unsigned short int contagem = 0;
    filaTimer1 filaDados = {0,0};
    timer1Queue = xQueueCreate(1, sizeof(filaTimer1));

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, //80MHz
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1E6, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer1));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = Timer1Interrupcao,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer1, &cbs, timer1Queue));

    ESP_LOGI(TAG3, "Habilita timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer1));

    ESP_LOGI(TAG3, "Timer comecou, configure o alarme");
    gptimer_alarm_config_t alarm_config1 = {
        .reload_count = 0,
        .alarm_count = 100E3, // periodo = 100ms
        .flags.auto_reload_on_alarm = false,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer1, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer1));

    while(1) 
    {
        if (xQueueReceive(timer1Queue, &filaDados, portMAX_DELAY)) {
            //ESP_LOGI(TAG3, "Recebeu: Tempo Atual = %llu, Alarme = %llu",filaDados.contAtualTimer, filaDados.valorAlarme);
            alarm_config1.alarm_count=filaDados.valorAlarme+100E3;
            ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer1, &alarm_config1));
            contagem++;
        }
        if (contagem>=10)
        {
            contagem=0;
            AtualizaHoras(horas);
            ESP_LOGI(TAG3, "Relogio: %02d:%02d:%02d", horas[0], horas[1], horas[2]);
        }
    }
}

void TamanhoVariaveis(){
    // A função sizeof() retorna o tamanho em bytes
    ESP_LOGI(TAG, "Tamanho dos Tipos de Dados em C (em bytes e bits):");
    
    // Tipos de dados básicos
    ESP_LOGI(TAG, "char: %2zu bytes (%3zu bits)", sizeof(char), sizeof(char) * 8);
    ESP_LOGI(TAG, "signed char: %2zu bytes (%3zu bits)", sizeof(signed char), sizeof(signed char) * 8);
    ESP_LOGI(TAG, "unsigned char: %2zu bytes (%3zu bits)", sizeof(unsigned char), sizeof(unsigned char) * 8);
    
    ESP_LOGI(TAG, "bool: %2zu bytes (%3zu bits)", sizeof(bool), sizeof(bool) * 8);

    // Tipos de dados inteiros
    ESP_LOGI(TAG, "short int: %2zu bytes (%3zu bits)", sizeof(short int), sizeof(short int) * 8);
    ESP_LOGI(TAG, "unsigned short int: %2zu bytes (%3zu bits)", sizeof(unsigned short int), sizeof(unsigned short int) * 8);
    
    ESP_LOGI(TAG, "int: %2zu bytes (%3zu bits)", sizeof(int), sizeof(int) * 8);
    ESP_LOGI(TAG, "unsigned int: %2zu bytes (%3zu bits)", sizeof(unsigned int), sizeof(unsigned int) * 8);
    
    ESP_LOGI(TAG, "long int: %2zu bytes (%3zu bits)", sizeof(long int), sizeof(long int) * 8);
    ESP_LOGI(TAG, "unsigned long int: %2zu bytes (%3zu bits)", sizeof(unsigned long int), sizeof(unsigned long int) * 8);
    
    ESP_LOGI(TAG, "long long int: %2zu bytes (%3zu bits)", sizeof(long long int), sizeof(long long int) * 8);
    ESP_LOGI(TAG, "unsigned long long int: %2zu bytes (%3zu bits)", sizeof(unsigned long long int), sizeof(unsigned long long int) * 8);
    
    // Tipos de dados de ponto flutuante
    ESP_LOGI(TAG, "float: %2zu bytes (%3zu bits)", sizeof(float), sizeof(float) * 8);
    ESP_LOGI(TAG, "double: %2zu bytes (%3zu bits)", sizeof(double), sizeof(double) * 8);
    ESP_LOGI(TAG, "long double: %2zu bytes (%3zu bits)", sizeof(long double), sizeof(long double) * 8);
}

void app_main(void)
{
    ESP_LOGI(TAG,"Ola, mundo!"); // O esp-idf gerou esse comando sozinho
    TamanhoVariaveis();
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

    //Inicia task da pratica 2
    xTaskCreate(pratica02, "pratica02", 2048, NULL, 10, NULL);
    xTaskCreate(pratica03, "pratica03", 2048, NULL, 9, NULL);

    while (1)
    {
        ESP_LOGI(TAG,"Task 1 a cada 15 segundos");
        vTaskDelay(15E3 / portTICK_PERIOD_MS);
    }
}
