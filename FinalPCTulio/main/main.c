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

//Bibliotecas MQTT
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "esp_task_wdt.h" // Adicione este include

static const char* TAG = "SISTEMA_BUCK";
static const char* TAG7 = "MQTT";


// Variáveis Globais de Hardware
SemaphoreHandle_t semaphore_adc;
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration;
static esp_mqtt_client_handle_t client; // Variável global para o arquivo

typedef struct {
    float tensaoAtual;
    float dutyAtual;
} filaEnviaP7;

typedef struct {
    float setPointDesejado;
    char requisaTensaoBool;
} filaRecebeP7;

static QueueHandle_t filaFromP7Queue = NULL;
static QueueHandle_t filaToP7Queue = NULL;

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

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG7, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG7, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            // ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            // msg_id = esp_mqtt_client_publish(client, "teste", "data_3: 4.0 data_2:8.0", 0, 1, 0);
            // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "VoutEnvia", 1);
            ESP_LOGI(TAG7, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "ReqDados", 1);
            ESP_LOGI(TAG7, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
            // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG7, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG7, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "teste", "11", 0, 0, 0);
            ESP_LOGI(TAG7, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG7, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG7, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            {
                filaRecebeP7 filaDados = {0,0};
                // 1. Defina buffers com tamanho suficiente
                char topic_str[12]; // O maximo que vou receber é tópicos com 9 caracteres, estou colocando um excesso por segurança
                char data_str[6];   // Vou receber um float em formato de string no meu MQTT aparece com apenas uma casa decimal

                // 2. Use snprintf para formatar e salvar na variável
                snprintf(topic_str, sizeof(topic_str), "%.*s", event->topic_len, event->topic);
                snprintf(data_str, sizeof(data_str), "%.*s", event->data_len, event->data);
                ESP_LOGI(TAG7,"%s",topic_str);
                ESP_LOGI(TAG7,"%s",data_str);

                filaDados.requisaTensaoBool = 1;
                if (strcmp(topic_str, "VoutEnvia") == 0) {
                    filaDados.setPointDesejado = strtof(data_str, NULL);
                    ESP_LOGI(TAG7,"Valor Float=%f",strtof(data_str, NULL));
                    if (xQueueOverwrite(filaFromP7Queue, &filaDados) != pdPASS) { 
                        ESP_LOGW(TAG, "Falha ao sobrescrever na fila");
                    }
                }
                else if (strcmp(topic_str, "ReqDados") == 0) {
                    msg_id = esp_mqtt_client_publish(client, "ReqDados", "0", 2, 1, 0); // Tamanho 2, QOS 1, RETAIN 0
                    filaDados.setPointDesejado = -1;
                    if (xQueueOverwrite(filaFromP7Queue, &filaDados) != pdPASS) { 
                        ESP_LOGW(TAG, "Falha ao sobrescrever na fila");
                    }
                }
                break;
            } // Fim do bloco case
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG7, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG7, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG7, "Other event id:%d", event->event_id);
            break;
        }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://g3device:g3device@node02.myqtthub.com:1883",
        .credentials.client_id =  "g3device",
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    ESP_LOGI(TAG7,"INICIANDO MQTT");
}

static void pratica07(void* arg){
    ESP_LOGI(TAG7, "[APP] Iniciando MQTT..");
    ESP_LOGI(TAG7, "[APP] Memoria Livre: %" PRIu32 " bytes", esp_get_free_heap_size());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

    filaEnviaP7 filaDados = {0,0};
    char buffer_envio[10]; // Buffer auxiliar para converter o número em texto

    while(1){
        if (xQueueReceive(filaToP7Queue, &filaDados, portMAX_DELAY)){
            // 1. Envia a Tensão normalmente
            snprintf(buffer_envio, sizeof(buffer_envio), "%2.1f", filaDados.tensaoAtual);
            esp_mqtt_client_publish(client, "VoutAtual", buffer_envio, 0, 1, 0);

            // 2. Converte o Duty de (0.0 - 1.0) para (0.0 - 100.0)
            // Multiplicamos por 100.0 antes de formatar a string
            snprintf(buffer_envio, sizeof(buffer_envio), "%2.1f", filaDados.dutyAtual * 100.0);
            esp_mqtt_client_publish(client, "DutyAtual", buffer_envio, 0, 1, 0);
        }
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
    float setpoint = 1.0f;
    float x_n = 0;
    int tensao_mv = 0;
    int valor_bruto = 0;
    float tensaoLidaComGanho = 0;

    ESP_LOGI(TAG, "Loop de controle iniciado no app_main.");
    // Desativa o watchdog para a tarefa atual (main)

    // Crie as filas PRIMEIRO
    filaFromP7Queue = xQueueCreate(1, sizeof(filaRecebeP7));
    filaToP7Queue = xQueueCreate(1, sizeof(filaEnviaP7));

    // Depois crie a task
    xTaskCreate(pratica07, "pratica07", 4096, NULL, 10, NULL);

    filaRecebeP7 filaDados = {1,0};
    filaEnviaP7 filaDados2 = {0,0};

    while (1) {
        // Espera o sinal do Timer (Interrupção)
        if(xQueueReceive(filaFromP7Queue, &filaDados, 0)){
            ESP_LOGI(TAG,"RECEBEU DA FILA");
            ESP_LOGI(TAG,"%f",filaDados.setPointDesejado);
            ESP_LOGI(TAG, "%d", (int)filaDados.requisaTensaoBool);
        }

        if (xSemaphoreTake(semaphore_adc,  portMAX_DELAY)) {

            if(filaDados.setPointDesejado>0 && filaDados.requisaTensaoBool){
                setpoint = filaDados.setPointDesejado;
                filaDados.setPointDesejado = -1; // Bloqueia re-processamento
            }

            if (do_calibration) {
                // ESP_LOGI(TAG,"ENTROU DO CALIBRATION");
                ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &valor_bruto));
                adc_cali_raw_to_voltage(adc1_cali_handle, valor_bruto, &tensao_mv);
                tensaoLidaComGanho = ((float)tensao_mv / 1000.0f) * 4.921568627;
                x_n = setpoint - tensaoLidaComGanho;
            } 

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

            if(filaDados.requisaTensaoBool){ 
                filaDados2.dutyAtual = duty_float;
                filaDados2.tensaoAtual = tensaoLidaComGanho;
                if (xQueueOverwrite(filaToP7Queue, &filaDados2) != pdPASS) { 
                    ESP_LOGW(TAG, "Falha ao sobrescrever na fila");
                }
                filaDados.requisaTensaoBool = 0;
            }
        }
        
        // O Watchdog não vai disparar aqui porque a cada ciclo de 50kHz, 
        // o xSemaphoreTake permite que o sistema processe outras tarefas 
        // se o tempo de execução for curto o suficiente.
    }
}