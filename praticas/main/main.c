/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"


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

//////////////////////////////////////////////////// PRATICA 4
static SemaphoreHandle_t semaphore_pwm = NULL;
// Dentro da seção PRATICA 4 (ou global)
static QueueHandle_t botao_pratica2_pratica4_queue = NULL; 
//////////////////////////////////////////////////// PRATICA 5
static SemaphoreHandle_t semaphore_adc = NULL;
static QueueHandle_t lidoADC_to_timer = NULL;
typedef struct {
    int bruto;
    int tensao;
} filaADC;
//////////////////////////////////////////////////// TAGS
static const char* TAG = "Pratica-1";
static const char* TAG2 = "Pratica-2";
static const char* TAG3 = "Pratica-3";
static const char* TAG4 = "Pratica-4";
static const char* TAG5 = "Pratica-5";

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

            // Se a diferença de tempo for menor que o tempo de debounce, ignora o evento
            if (tickAtual - tickAnterior < debounce_ticks) {
                continue;
            }

            tickAnterior = tickAtual;

            if (io_num==23)
            {
                // Atualiza o tempo do último clique
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

            // NOVO: Envia o número do pino para a task pratica04
            if (botao_pratica2_pratica4_queue != NULL) {
                // Envia o número do pino (io_num) para a Queue da pratica04
                // Sem timeout (0) pois já estamos dentro de um loop de evento
                if (xQueueSend(botao_pratica2_pratica4_queue, &io_num, 0) != pdPASS) {
                    ESP_LOGW(TAG2, "Falha ao enviar o botão %lu para a Pratica 4", io_num);
                }
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
    xSemaphoreGiveFromISR(semaphore_pwm,NULL); //Função na TASK Timer para sincronizar com a task PWM
    xSemaphoreGiveFromISR(semaphore_adc,NULL); //Função na TASK Timer para sincronizar com a task ADC
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
    unsigned short int horas[] = {0,0,0};
    unsigned short int contagem = 0;
    filaTimer1 filaDados = {0,0};
    filaADC dadosADC;
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
            ESP_LOGI(TAG3, "Relogio: %02d:%02d:%02d, Cont = %llu, Alarme = %llu",horas[0], horas[1], horas[2],filaDados.contAtualTimer, filaDados.valorAlarme);
            if (xQueueReceive(lidoADC_to_timer, &dadosADC, 0)){
                ESP_LOGI(TAG5, "ADC, RAW: %d, Tensao: %d",dadosADC.bruto,dadosADC.tensao);
            }
        }
    }
}

void configuraPWM_pratica04(){
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = 13,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 5E3,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 16,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    /////////////////////////////////////////////////////////////////////////////////////////
    // Prepare and then apply the LEDC PWM timer configuration

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = 33;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void pratica04(void* arg){
    uint32_t io_num_recebido;
    uint32_t dutyPWM = 4096;
    char automatico = 0;
    char mudouDutyPWM = 1;
    
    configuraPWM_pratica04();
    semaphore_pwm = xSemaphoreCreateBinary();

    botao_pratica2_pratica4_queue = xQueueCreate(1, sizeof(uint32_t)); 
    if (botao_pratica2_pratica4_queue == NULL) {
        ESP_LOGE(TAG4, "Falha ao criar a queue para o botão!");
    }

    for (;;) {
        if (xQueueReceive(botao_pratica2_pratica4_queue, &io_num_recebido, 10 / portTICK_PERIOD_MS)) {
            ESP_LOGI(TAG4, "Botão acionado recebido: GPIO%lu", io_num_recebido);
            // Sua lógica para checar o botão na pratica04 vai aqui:
            if (io_num_recebido == GPIO_INPUT_IO_23) {
                automatico=1;
            } else if (io_num_recebido == GPIO_INPUT_IO_22) {
                automatico=0;
                dutyPWM = dutyPWM - 1023;
                if(dutyPWM>8191) dutyPWM = 8191;
            } else if (io_num_recebido == GPIO_INPUT_IO_21) {
                automatico=0;
                dutyPWM = dutyPWM + 1024;
                if(dutyPWM>8191) dutyPWM = 0;
            }
            ESP_LOGI(TAG4,"PWM Duty= %" PRIu32 "",dutyPWM);
            mudouDutyPWM = 1;
        }
        if (automatico)
        {
            if(xSemaphoreTake(semaphore_pwm, portMAX_DELAY))
            {
                dutyPWM = dutyPWM + 128;
                if(dutyPWM>8191) dutyPWM = 0;
            };
            mudouDutyPWM = 1;
        }
        if(mudouDutyPWM)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyPWM);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyPWM);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            mudouDutyPWM=0;
        }
    }
}

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
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

static void pratica05(void* arg){
    filaADC filaDados = {0,0};
    lidoADC_to_timer = xQueueCreate(1, sizeof(filaADC)); 
    if (lidoADC_to_timer == NULL) {
        ESP_LOGE(TAG5, "Falha ao criar a queue para o ADC!");
    }
    semaphore_adc = xSemaphoreCreateBinary();
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_3, ADC_ATTEN_DB_12, &adc1_cali_chan0_handle);

    while (1) {
        if(xSemaphoreTake(semaphore_adc, portMAX_DELAY)){
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &filaDados.bruto));
            if(do_calibration1_chan0)ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, filaDados.bruto, &filaDados.tensao));
            if (xQueueOverwrite(lidoADC_to_timer, &filaDados) != pdPASS) {
                ESP_LOGE(TAG5, "Falha crítica ao sobrescrever o ADC na fila");
            }
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
    xTaskCreate(pratica04, "pratica04", 2048, NULL, 8, NULL);
    xTaskCreate(pratica05, "pratica05", 2048, NULL, 7, NULL);

    while (1)
    {
        ESP_LOGI(TAG,"Task 1 a cada 15 segundos");
        vTaskDelay(15E3 / portTICK_PERIOD_MS);
    }
}
