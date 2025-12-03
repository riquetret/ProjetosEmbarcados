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

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_lcd_panel_vendor.h"

#include "mqtt_client.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"


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
//////////////////////////////////////////////////// PRATICA 6
static QueueHandle_t horaAtual = NULL; 
typedef struct {
    esp_lcd_panel_io_handle_t handleDeIo;
    esp_lcd_panel_handle_t handleDePainel;
} configuracaoDisplay;
#define I2C_BUS_PORT  0
static SemaphoreHandle_t semaphore_display = NULL;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           19
#define EXAMPLE_PIN_NUM_SCL           18
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

//////////////////////////////////////////////////// TAGS
static const char* TAG = "Pratica-1";
static const char* TAG2 = "Pratica-2";
static const char* TAG3 = "Pratica-3";
static const char* TAG4 = "Pratica-4";
static const char* TAG5 = "Pratica-5";
static const char* TAG6 = "Pratica-6";
static const char* TAG7 = "Pratica-7";

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
    horaAtual = xQueueCreate(1, sizeof(horas));

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
            if (xQueuePeek(lidoADC_to_timer, &dadosADC, 0)){
                ESP_LOGI(TAG5, "ADC, RAW: %d, Tensao: %d",dadosADC.bruto,dadosADC.tensao);
            }
            if (xQueueOverwrite(horaAtual, horas) != pdPASS) { 
                 ESP_LOGW(TAG3, "Falha ao sobrescrever horas");
            }
            xSemaphoreGive(semaphore_display); //Manda Display Atualizar
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
                dutyPWM = dutyPWM - 1024;
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

configuracaoDisplay pratica06Configura(){
    ESP_LOGI(TAG6, "Inicializa I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG6, "Instalacao Io de painel"); // Enderecos i2c, clock, n bits para comandar
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

        ESP_LOGI(TAG6, "Instalacao SSD1306 painel driver"); // Gerenciamento de funcoes
        esp_lcd_panel_handle_t panel_handle = NULL;
        esp_lcd_panel_dev_config_t panel_config = {
            .bits_per_pixel = 1,
            .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
        };
        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = EXAMPLE_LCD_V_RES,
        };
        panel_config.vendor_config = &ssd1306_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    configuracaoDisplay objeto1;
    objeto1.handleDeIo = io_handle;
    objeto1.handleDePainel = panel_handle;
    return objeto1;
}

static void pratica06(void* arg){
    configuracaoDisplay objeto2= pratica06Configura();
    filaADC dadosADC;
    unsigned short int horas[] = {0,0,0};
    char mensagem[39];
    semaphore_display = xSemaphoreCreateBinary();
    
    // CORREÇÃO LVGL: Criação única de objetos
    lv_obj_t *scr=NULL;
    lv_obj_t *label=NULL;

    ESP_LOGI(TAG6, "Inicializa LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = objeto2.handleDeIo,
        .panel_handle = objeto2.handleDePainel,
        // CORREÇÃO: Buffer Size (128*64 / 8 = 1024 bytes para 1bpp)
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES, 
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
    lv_disp_set_rotation(disp, LV_DISP_ROT_180);

    // CRIAÇÃO DO LABEL (APENAS UMA VEZ)
    if (lvgl_port_lock(0)) {
        scr = lv_disp_get_scr_act(disp);
        label = lv_label_create(scr);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(label, disp->driver->hor_res);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
        lv_label_set_text(label, "INICIANDO..."); // Texto inicial
        lvgl_port_unlock();
    }
    
    while(1)
    {
        if(xSemaphoreTake(semaphore_display, portMAX_DELAY)){
            // 1. Tenta PEGAR os dados mais recentes (não bloqueante)
            xQueuePeek(lidoADC_to_timer, &dadosADC, 0);
            xQueuePeek(horaAtual, &horas, 0); // Use PEEK, a Task 3 já enviou os dados com Overwrite

            // 2. Formata a nova mensagem
            // Usando snprintf para evitar estouro de buffer
            snprintf(mensagem, sizeof(mensagem), "Tensao: %dmV\nRelogio: %02d:%02d:%02d",
                    dadosADC.tensao, horas[0], horas[1], horas[2]);

            // 3. Atualiza o LVGL (apenas o texto)
            if (lvgl_port_lock(0)) {
                lv_label_set_text(label, mensagem); // Apenas atualiza o texto do label existente
                lvgl_port_unlock();
            }
        }
    }
}


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
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
            // msg_id = esp_mqtt_client_publish(client, "teste", "data_3", 0, 1, 0);
            // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "teste", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
            // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            // break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            {
                ESP_LOGI(TAG, "MQTT_EVENT_DATA");
                printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
                printf("DATA=%.*s\r\n", event->data_len, event->data);

                // 1. Defina um buffer de tamanho suficiente
                // Adicionamos +1 para o terminador nulo '\0'
                size_t data_len = event->data_len;
                char *data_str = (char *)malloc(data_len + 1);

                if (data_str == NULL) {
                    ESP_LOGE(TAG, "Falha na alocação de memória.");
                    break;
                }

                // 2. Copie o payload da mensagem para o buffer, garantindo o terminador nulo
                // strncpy é mais seguro, mas precisa de manuseio cuidadoso.
                // Usamos memcpy e adicionamos o terminador manualmente.
                memcpy(data_str, event->data, data_len);
                data_str[data_len] = '\0'; // Garante o terminador nulo

                // 3. Converta a string para um número inteiro longo (long int)
                char *endptr;
                long valor_inteiro_long = strtol(data_str, &endptr, 10); // Base 10 (decimal)
                int valor_inteiro = (int)valor_inteiro_long; // Converte para int (se cabível)

                // 4. Verifique a conversão e imprima o resultado
                if (endptr == data_str || *endptr != '\0') {
                    ESP_LOGE(TAG, "Conversão falhou: '%s' não é um número inteiro válido.", data_str);
                } else {
                    ESP_LOGI(TAG, "Valor Inteiro Recebido: %d", valor_inteiro);

                    // AQUI você pode usar 'valor_inteiro' para sua lógica, por exemplo:
                    // if (valor_inteiro > 100) { ... }
                }

                // 5. Libere a memória alocada
                free(data_str);

                break;
            } // Fim do bloco case
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
        }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://g3device:g3device@node02.myqtthub.com:1883",
        .credentials.client_id =  "g3device",
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
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
    while(1){
        ESP_LOGI(TAG7,"Task 7 a cada 5segundos");
        vTaskDelay(5E3 / portTICK_PERIOD_MS);
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
    xTaskCreate(pratica06, "pratica06", 4096, NULL, 6, NULL);
    xTaskCreate(pratica07, "pratica07", 4096, NULL, 5, NULL);

    while (1)
    {
        ESP_LOGI(TAG,"Task 1 a cada 15 segundos");
        vTaskDelay(15E3 / portTICK_PERIOD_MS);
    }
}
