/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "esp_log.h"             // Necessário para ESP_LOGI, ESP_LOGE, etc.
#include "driver/ledc.h"
#include "driver/gptimer.h"      // Necessário para o Timer Geral (GPTimer)
#include "esp_adc/adc_oneshot.h" // Necessário para o ADC (API nova)
#include "esp_adc/adc_cali.h"    // Necessário para calibração do ADC
#include "esp_adc/adc_cali_scheme.h" // Esquemas de calibração
#include "freertos/semphr.h"     // Necessário para o SemaphoreHandle_t e xSemaphore...

static const char* TAG = "PRINCIPAL";
static const char* TAG2 = "CONFIG TIMER, PWM, ADC";
static const char* TAG3 = "BUCK";

SemaphoreHandle_t semaphore_adc;

typedef struct {
    uint64_t contAtualTimer;
    uint64_t valorAlarme;
} filaTimer1;

typedef struct {
    int bruto;
    int tensao;
} filaADC;

typedef struct {
    gptimer_handle_t gptimer;
    QueueHandle_t timerQueue;
    adc_oneshot_unit_handle_t adcHandle;
    adc_cali_handle_t adcCaliHandle;
    bool doCalibration;
} buck_params_t;


static bool IRAM_ATTR Timer1Interrupcao(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    QueueHandle_t queue = (QueueHandle_t)user_data;
    filaTimer1 retornoFila = {
        .contAtualTimer = edata->count_value,
        .valorAlarme = edata->alarm_value
    };
    xQueueSendFromISR(queue, &retornoFila, NULL);
    xSemaphoreGiveFromISR(semaphore_adc,NULL); //Função na TASK Timer para sincronizar com a task ADC
    return 1;
}

void configuraPWM_pratica(){
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = 10,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50E3,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 32,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    /////////////////////////////////////////////////////////////////////////////////////////
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

static void buck(void *pvParameters) {
    semaphore_adc = xSemaphoreCreateBinary();
    buck_params_t *params = (buck_params_t *) pvParameters;
    filaADC filaDadosADC = {0,0};
    
    // Estados do controlador
    static float x_prev = 0.0f;
    static float y_prev = 0.0f;
    float duty_float = 0.0f;

    // 1. Resetar o timer do PWM para garantir que ele comece do ZERO agora
    ledc_timer_rst(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);

    // Configuração inicial: Alarme em 1 (meio ciclo de 50kHz se o timer está a 100kHz)
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1, 
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(params->gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(params->gptimer));

    ESP_LOGI(TAG3, "Timer comecou, alarme configurado");

    bool primeira_execucao = true;

    while (1) {
        if (xSemaphoreTake(semaphore_adc, portMAX_DELAY)) {
            
            // Se for a primeira vez, agora mudamos o alarme para 2 (frequência de 50kHz)
            // Assim, as próximas interrupções ocorrem sempre a cada 2 ticks do timer (20us)
            if (primeira_execucao) {
                alarm_config.alarm_count = 2;
                gptimer_set_alarm_action(params->gptimer, &alarm_config);
                primeira_execucao = false;
            }

            // 1. Leitura rápida do ADC
            //ESP_ERROR_CHECK(adc_oneshot_read(params->adcHandle, ADC_CHANNEL_3, &filaDadosADC.bruto));
            
            float x_n = 0.0f;
            if (params->doCalibration) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(params->adcCaliHandle, filaDadosADC.bruto, &filaDadosADC.tensao));
                x_n = 12-(float)filaDadosADC.tensao / 1000.0f;
            }

            // 2. Equação de Diferenças
            duty_float = (0.001241f * x_n) + (0.001241f * x_prev) + y_prev;

            // 3. Saturação (0 a 0.95)
            if (duty_float > 0.95f) duty_float = 0.95f;
            if (duty_float < 0.0f)  duty_float = 0.0f;
            duty_float = 0.1;

            // 4. Atualização imediata do hardware
            uint32_t duty_hardware = (uint32_t)(duty_float * 1023.0f);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_hardware);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

            // 5. Atualiza histórico
            x_prev = x_n;
            y_prev = duty_float;
        }
    }
}

static void configTimer_e_PWM(){
    // Configuracao Timer
    gptimer_handle_t gptimer1 = NULL;
    QueueHandle_t timer1Queue = NULL;
    timer1Queue = xQueueCreate(1, sizeof(filaTimer1));

    filaTimer1 filaDados = {0,0};

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, //80MHz
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 100E3,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer1));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = Timer1Interrupcao,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer1, &cbs, timer1Queue));

    ESP_LOGI(TAG2, "Habilita timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer1)); // Habilita Timer
    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Configura PWM
    configuraPWM_pratica();
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Configura ADC
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
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    buck_params_t *params = malloc(sizeof(buck_params_t));
    params->gptimer = gptimer1;
    params->timerQueue = timer1Queue;
    params->adcHandle = adc1_handle;
    params->adcCaliHandle = adc1_cali_chan0_handle;
    params->doCalibration = do_calibration1_chan0;

    xTaskCreate(buck, "buck", 2048, params, 9, NULL);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG,"Ola, mundo!"); // O esp-idf gerou esse comando sozinho
    //TamanhoVariaveis();
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

    //uint32_t resolution = ledc_find_suitable_duty_resolution(80E6, 50E3);

    fflush(stdout);
    
    ////////////////////////////////////////////////////

    //Inicia task da pratica 2
    xTaskCreate(configTimer_e_PWM, "configTimer_e_PWM", 2048, NULL, 10, NULL);

    while (1)
    {
        ESP_LOGI(TAG,"Task 1 a cada 15 segundos");
        vTaskDelay(15000 / portTICK_PERIOD_MS);
    }
}
