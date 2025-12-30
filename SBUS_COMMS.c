#include <stdio.h>
#include "include/SBUS_COMMS.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_log.h"
#include "freertos/queue.h"

#define TAG "SBUS_COMMS"

static uint8_t uartPort;
extern QueueHandle_t queueNewSBUS;  // TODO: traer por parametro en el init

uint8_t rawToPercent(uint16_t value){
    return ((value - CHANNEL_MIN) *100.00) / (CHANNEL_MAX-CHANNEL_MIN);
}

int8_t rawToSwith(uint16_t value){
    if(value > (SW_POS_1_VALUE - 5) && value < (SW_POS_1_VALUE + 5)){
        return SW_POS_1;
    }
    else if(value > (SW_POS_2_VALUE - 5) && value < (SW_POS_2_VALUE + 5)){
        return SW_POS_2;
    }
    else if(value > (SW_POS_3_VALUE - 5) && value < (SW_POS_3_VALUE + 5)){
        return SW_POS_3;
    }
    return -1;
}

static void receiveTask(void *pvParameters){

    uint8_t data[24],length = 0,lostPacketsCont = 0;
    channels_control_t newControl;
    int16_t channels[17];

    while(1){      
        uart_get_buffered_data_len(uartPort, (size_t*)&length);          // obtengo datos para leer
        
        if(length > 0){    
            length = uart_read_bytes(uartPort,data,length,0);

            // esp_log_buffer_hex(TAG, &data, length);
            if(data[0] == 0x0F && length == 25 && data[length] == 0x00){
                channels[0]  = (uint16_t) ((data[1] | data[2] <<8) & 0x07FF);
                channels[1]  = ((data[2]>>3 |data[3]<<5)  & 0x07FF);
                channels[2]  = ((data[3]>>6 |data[4]<<2 |data[5]<<10)  & 0x07FF);
                channels[3]  = ((data[5]>>1 |data[6]<<7) & 0x07FF);
                channels[4]  = ((data[6]>>4 |data[7]<<4) & 0x07FF);
                channels[5]  = ((data[7]>>7 |data[8]<<1 |data[9]<<9)   & 0x07FF);
                channels[6]  = ((data[9]>>2 |data[10]<<6) & 0x07FF);
                channels[7]  = ((data[10]>>5|data[11]<<3) & 0x07FF);
                channels[8]  = ((data[12]   |data[13]<<8) & 0x07FF);
                channels[9]  = ((data[13]>>3|data[14]<<5)  & 0x07FF);
                channels[10] = ((data[14]>>6|data[15]<<2|data[16]<<10) & 0x07FF);
                channels[11] = ((data[16]>>1|data[17]<<7) & 0x07FF);
                channels[12] = ((data[17]>>4|data[18]<<4) & 0x07FF);
                channels[13] = ((data[18]>>7|data[19]<<1|data[20]<<9)  & 0x07FF);
                channels[14] = ((data[20]>>2|data[21]<<6) & 0x07FF);
                channels[15] = ((data[21]>>5|data[22]<<3) & 0x07FF);
                channels[16] = ((data[23]));

                // printf("1: %d,2: %d,3: %d,4: %d,5: %d,6: %d,7: %d\n",rawToPercent(channels[0]),rawToPercent(channels[1]),rawToPercent(channels[2]),rawToPercent(channels[3]),rawToPercent(channels[4]),rawToSwith(channels[5]),rawToSwith(channels[6]));
                newControl.throttle = rawToPercent(channels[THROTTLE_CHANNEL]);
                newControl.aileron = rawToPercent(channels[AILERON_CHANNEL]);
                newControl.rudder = rawToPercent(channels[RUDDER_CHANNEL]);
                newControl.elevator = rawToPercent(channels[ELEVATOR_CHANNEL]);
                newControl.p1 = rawToPercent(channels[P1_CHANNEL]);
                newControl.s1 = rawToSwith(channels[S1_CHANNEL]);
                newControl.s2 = rawToSwith(channels[S2_CHANNEL]);
                newControl.err = 0;
                lostPacketsCont = 0;
                xQueueSend(queueNewSBUS,&newControl,0);
            }
            else{
                ESP_LOGE(TAG,"RESYNC");
                uart_flush(uartPort);
            }
        }
        lostPacketsCont++;
        if (lostPacketsCont > 10) {
            newControl.err = ERR_CONN_LOST;
            xQueueSend(queueNewSBUS,&newControl,0);
        }
        vTaskDelay(pdMS_TO_TICKS(14));
    }
    vTaskDelete(NULL);
}

void sbusInit(uint8_t _uartPort,uint8_t _txPin,uint8_t _rxPin){

    uartPort = _uartPort;
    uart_config_t uart_config={
        .baud_rate = 100000,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_EVEN,
        .rx_flow_ctrl_thresh = 112,
        .stop_bits = UART_STOP_BITS_2,
        .source_clk = UART_SCLK_DEFAULT,

    };
    uart_param_config(uartPort,&uart_config);
    uart_set_pin(uartPort,_txPin,_rxPin,-1,-1);
    uart_set_line_inverse(uartPort,UART_SIGNAL_RXD_INV);

    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    uart_driver_install(uartPort, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);
    xTaskCreate(&receiveTask,"read sbus task", 4096, NULL, 3 , NULL);
}
