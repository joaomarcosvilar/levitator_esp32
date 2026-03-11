#include "comon.h"
#include "height_sensor.h"
#include "power_controller.h"

#include "string.h"

#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" 

#define TAG                         "MAIN"

#define UART_PORT_NUM               UART_NUM_0
#define UART_BAUD_RATE              (115200)
#define UART_BUFFER_SIZE            (256)

#define COMMAND_GET_VALUE           "s"
#define COMMAND_SET_VALUE           "p"

#define UART_QUEUE_LEN              (5)
#define LEVITADOR_QUEUE_LEN         (5)

#define MAIN_TASK_NAME              ("MAIN TASK")
#define MAIN_TASK_STACK_SIZE        (2 * 1024)
#define MAIN_TASK_PRIOR             (2)

#define SERIAL_TASK_NAME            ("SERIAL TASK")
#define SERIAL_TASK_STACK_SIZE      (3 * 1024)
#define SERIAL_TASK_PRIOR           (3)

static QueueHandle_t uart_queue = NULL;
static QueueHandle_t levitador_queue = NULL;
static TaskHandle_t serial_task_h = NULL;
static TaskHandle_t levitador_task_h = NULL;


typedef enum
{
    CMD_GET = 0,
    CMD_SET
} app_command_e;

typedef struct
{
    app_command_e command;
    uint16_t value;
} app_event_s;

// Tratamento do que recebeu do SERIAL
esp_err_t app_serial_comand_process(char *read_serial)
{
    app_event_s data = {};

    if (read_serial[0] == 'p') 
    {
        int duty_recebido = 0;
        if (sscanf(read_serial, "p%d", &duty_recebido) == 1) 
        {
            //ESP_LOGI(TAG, "Comando PWM recebido! Valor: %d", duty_recebido);
            data.command = CMD_SET;
            data.value = duty_recebido;
        }
    }
    else if (strcmp(read_serial, COMMAND_GET_VALUE) == 0)
    {
        //ESP_LOGI(TAG, "Comando GET recebido. Enviando status...");
        data.command = CMD_GET;
    }

    if(levitador_queue)
    {
        xQueueSend(levitador_queue, &data, pdMS_TO_TICKS(10));
        return ESP_OK;
    }
    else 
    {
        return ESP_OK;
    }
}

// Task de monitoramento e resgate do SERIAL
void app_serial_task(void *args)
{
    uart_event_t event;
    uint8_t* data_temp = (uint8_t*) malloc(UART_BUFFER_SIZE);
    char line_accumulator[UART_BUFFER_SIZE];
    int line_pos = 0;

    while (1)
    {
        if(xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
            if(event.type == UART_DATA)
            {
                int read_len = uart_read_bytes(UART_PORT_NUM, data_temp, event.size, pdMS_TO_TICKS(10));
                
                for (int i = 0; i < read_len; i++) {
                    uint8_t c = data_temp[i];

                    if (c == '\n' || c == '\r') {
                        if (line_pos > 0) {
                            line_accumulator[line_pos] = '\0';
                            
                            //ESP_LOGI(TAG, "Linha completa detectada: %s", line_accumulator);
                            
                            app_serial_comand_process(line_accumulator);

                            line_pos = 0;
                        }
                    } 
                    else {
                        if (line_pos < (UART_BUFFER_SIZE - 1)) {
                            line_accumulator[line_pos++] = c;
                        } else {
                            line_pos = 0;
                        }
                    }
                }
            }
            // Opcional: Tratar UART_FIFO_OVF aqui para limpar line_pos se houver erro
        }
    }
    free(data_temp);
    vTaskDelete(NULL);
}

void levitador_task(void *args)
{
    app_event_s data = {};
    uint16_t value = 0;
    while(1)
    {
        if(xQueueReceive(levitador_queue, &data, pdMS_TO_TICKS(10)))
        {
            switch (data.command)
            {
                case CMD_GET:
                {
                    value = height_sensor_get();
                    printf("%d\n", value);
                    value = 0;
                    break;
                }
                case CMD_SET:
                {
                    power_contr_set(data.value);
                    break;
                }
            }
        }
    }
}

esp_err_t app_setup(void)
{
    // Inicialização da comunicação serial
    uart_config_t uart_config = 
    {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(UART_PORT_NUM, &uart_config);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG,"Failed to config uart (E: %s)", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(  UART_PORT_NUM, 
                                UART_BUFFER_SIZE, 
                                UART_BUFFER_SIZE, 
                                UART_QUEUE_LEN, 
                                &uart_queue , 
                                0);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG,"Failed to install uart (E: %s)", esp_err_to_name(ret));
        return ret;
    }

    ret = height_sensor_init();
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG,"Failed to init height sensor (E: %s)", esp_err_to_name(ret));
        return ret;
    }
    
    ret = power_contr_init();
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG,"Failed to init power controller (E: %s)", esp_err_to_name(ret));
        return ret;
    }

    levitador_queue = xQueueCreate(LEVITADOR_QUEUE_LEN, sizeof(app_event_s));
    if(!levitador_queue)
    {
        ESP_LOGE(TAG, "Failed to create queue");
        return ESP_FAIL;
    }

    BaseType_t Retx = xTaskCreate(app_serial_task,
                                  SERIAL_TASK_NAME,
                                  SERIAL_TASK_STACK_SIZE,
                                  NULL,
                                  SERIAL_TASK_PRIOR,
                                  &serial_task_h);
    if (Retx != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to create serial task");
        return ESP_FAIL;
    }

    Retx = xTaskCreate( levitador_task,
                        MAIN_TASK_NAME,
                        MAIN_TASK_STACK_SIZE,
                        NULL,
                        MAIN_TASK_PRIOR,
                        &levitador_task_h);
    if (Retx != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to create main task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret = app_setup();
    if(ret != ESP_OK)
    {
        return;
    }    
}
