#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_task_wdt.h" // Adicione este include

static const char* TAG = "SISTEMA_BUCK";

// Variáveis Globais de Hardware
SemaphoreHandle_t semaphore_adc;
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration;

// --- FUNÇÕES DE CONFIGURAÇÃO ---

static bool iniciar_calibracao_adc(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
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

void configurar_adc() {
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config));
    do_calibration = iniciar_calibracao_adc(ADC_UNIT_1, ADC_CHANNEL_5, ADC_ATTEN_DB_12, &adc1_cali_handle);
}

void configurar_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50000, 
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 32,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(semaphore_adc, &high_task_awoken);
    // xSemaphoreGiveFromISR(semaphore_adc, NULL);
    return (high_task_awoken == pdTRUE);
    // return 1;
}

void configurar_timer(gptimer_handle_t *timer_ret) {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1E6, // 1us por tick
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, timer_ret));

    gptimer_event_callbacks_t cbs = { .on_alarm = timer_callback };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*timer_ret, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(*timer_ret));
}

// --- MAIN LOOP ---

void desativar_watchdog_total() {
    // 1. Desinscreve a tarefa atual (main)
    esp_task_wdt_delete(NULL);
    
    // 2. Tenta desinicializar o periférico TWDT completamente
    // Isso fará com que nenhuma tarefa em nenhum núcleo seja monitorada
    esp_err_t err = esp_task_wdt_deinit();
    
    if (err == ESP_OK) {
        ESP_LOGI("WDT", "Watchdog de tarefas desativado com sucesso.");
    } else {
        ESP_LOGE("WDT", "Erro ao desativar: tarefas ainda inscritas (IDLE?).");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Sistema Buck Converter...");
    desativar_watchdog_total();

    // 1. Inicializar Sincronização
    semaphore_adc = xSemaphoreCreateBinary();

    // 2. Configurar Periféricos
    configurar_pwm();
    configurar_adc();
    gptimer_handle_t gptimer = NULL;
    configurar_timer(&gptimer);

    // 3. Configurar Alarme do Timer (50kHz de amostragem)
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 20, // 
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    
    // 4. Iniciar Hardware
    ledc_timer_rst(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    // Variáveis de Controle
    float x_prev = 0.0f;
    float y_prev = 0.0f;
    // float setpoint = 9.0f-0.77f;
    float setpoint = 10.0f;
    int valor_bruto = 0;
    int tensao_mv = 0;

    ESP_LOGI(TAG, "Loop de controle iniciado no app_main.");
    // Desativa o watchdog para a tarefa atual (main)

    while (1) {
        // Espera o sinal do Timer (Interrupção)
         if (xSemaphoreTake(semaphore_adc,  portMAX_DELAY)) {
            
            // A. Leitura do ADC (Raw -> Voltage)
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &valor_bruto));
        
            float x_n = 0.0f;
            if (do_calibration) {
                // ESP_LOGI(TAG,"ENTROU DO CALIBRATION");
                adc_cali_raw_to_voltage(adc1_cali_handle, valor_bruto, &tensao_mv);
                x_n = setpoint - ((float)tensao_mv / 1000.0f) * 4.921568627;
                // ESP_LOGI(TAG,"%d",tensao_mv);
            } 

            // // ESP_LOGI(TAG,"%d",tensao_mv);
            // // else {
            //     x_n = setpoint - ((float)valor_bruto * 3.3f / 4095.0f) * 4.921568627;
            // // }

            // // B. Equação de Diferenças (Controlador)
            float duty_float = (0.001241f * x_n) + (0.001241f * x_prev) + y_prev;

            // C. Saturação e Segurança
            if (duty_float >= 0.9f) duty_float = 0.9f;
            if (duty_float < 0.0f)  duty_float = 0.0f;

            // float duty_float;
            // Para teste fixo como solicitado:
            //duty_float = 0.2; 

            // D. Atualiza Hardware
            uint32_t duty_hardware = (uint32_t)(duty_float * 1023.0f);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_hardware);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

            // E. Histórico
            x_prev = x_n;
            y_prev = duty_float;
        }
        
        // O Watchdog não vai disparar aqui porque a cada ciclo de 50kHz, 
        // o xSemaphoreTake permite que o sistema processe outras tarefas 
        // se o tempo de execução for curto o suficiente.
    }
}