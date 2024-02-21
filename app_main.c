/* SPI Slave example, sender (uses SPI master driver)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "led_strip.h"
/*
SPI sender (master) example.

This example is supposed to work together with the SPI receiver. It uses the standard SPI pins (MISO, MOSI, SCLK, CS) to
transmit data over in a full-duplex fashion, that is, while the master puts data on the MOSI pin, the slave puts its own
data on the MISO pin.

This example uses one extra pin: GPIO_HANDSHAKE is used as a handshake pin. The slave makes this pin high as soon as it is
ready to receive/send data. This code connects this line to a GPIO interrupt which gives the rdySem semaphore. The main
task waits for this semaphore to be given before queueing a transmission.
*/

/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#define GPIO_HANDSHAKE 18
#define GPIO_MOSI 15
#define GPIO_MISO 13
#define GPIO_SCLK 12
#define GPIO_CS 23
#define LED_OUTPUT 3
#define INT_INPUT 10

#define SENDER_HOST SPI2_HOST

#define LED_STRIP_BLINK_GPIO  6
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS 8
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static const char *TAG = "example";
bool Handshake_Flag = false;
bool Int_Input_Flag = false;

/*===================================== FUNCOES ====================================*/
led_strip_handle_t configure_led(void);
/*
This ISR is called when the handshake line goes high.
*/
static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{
    Handshake_Flag = true;
    //gpio_set_level(LED_OUTPUT, (uint32_t)Handshake_Flag);
}

static void gpio_input_isr_handler(void* arg)
{
    gpio_set_level(LED_OUTPUT, (uint32_t)Int_Input_Flag);
    if(Int_Input_Flag) Int_Input_Flag = false;
    else if(!Int_Input_Flag) Int_Input_Flag = true;

}

//Main application
void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t handle;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=3000000,
        .duty_cycle_pos=128,        //50% duty cycle
        .mode=0,
        .spics_io_num=GPIO_CS,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    /*========= CONFIGURAÇÃO DE GPIOS ==========*/
    /*CONFIGURAÇÃO GPIO HANDSHAKE*/
    gpio_config_t io_conf={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en= -1,
        .pull_down_en= 1,
        .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    };
    
    /*CONFIGURAÇÃO GPIO LED*/
    gpio_config_t io_conf_led_output={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1<<LED_OUTPUT)
    };
    /*CONFIGURAÇÃO GPIO INT*/
    gpio_config_t io_conf_int={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .pin_bit_mask=(1<<INT_INPUT)
    };


    /*================== LED STRIP ==================*/
    led_strip_handle_t led_strip =  configure_led();

    uint8_t send_start_buf[4] = {10,11,12,13};
    uint8_t recv_start_buf[4] = {0};

    uint8_t sendbuf[512] = {0};
    uint8_t recvbuf[512] = {0};

    spi_transaction_t transaction1;
    memset(&transaction1, 0, sizeof(transaction1));
    transaction1.length = sizeof(send_start_buf) * 8;
    transaction1.rx_buffer = recv_start_buf;
    transaction1.tx_buffer = send_start_buf;

    spi_transaction_t transaction2;
    memset(&transaction2, 0, sizeof(transaction2));
    transaction2.length = (LED_STRIP_LED_NUMBERS * 4) * 8;
    transaction2.rx_buffer = recvbuf;
    transaction2.tx_buffer = sendbuf;
    gpio_set_level(LED_OUTPUT, (uint32_t)Handshake_Flag);
    //Create the semaphore

    uint8_t Transaction_Number = 0;   

    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_config(&io_conf_led_output);
    gpio_config(&io_conf_int);

    gpio_install_isr_service(0);
    
    gpio_set_intr_type(GPIO_HANDSHAKE, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

    gpio_set_intr_type(INT_INPUT, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(INT_INPUT, gpio_input_isr_handler, NULL);

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret=spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(SENDER_HOST, &devcfg, &handle);
    assert(ret==ESP_OK);

    //Assume the slave is ready for the first transmission: if the slave started up before us, we will not detect
    //positive edge on the handshake line.

    for(int i = 0; i < LED_STRIP_LED_NUMBERS; i ++)  led_strip_set_pixel_rgbw(led_strip, i, 0,0,100,0); 
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    ESP_LOGI(TAG, "LED ON!");
    
    Handshake_Flag = false;
    
    while(1) {
        
        
        //Wait for slave to be ready for next byte before sending
        //sprintf(sendbuf, "ABCDEFGH");
        //ret=spi_device_transmit(handle, &transaction2);
        //printf("Received: %s\n", recvbuf);
        //sprintf(send_start_buf,"HIJ");
        if(Handshake_Flag && Transaction_Number == 0) 
        {
            send_start_buf[0] = 255;
            send_start_buf[1] = 255;
            send_start_buf[2] = 255;
            send_start_buf[3] = 255;
            ret = spi_device_transmit(handle, &transaction1);
            for(int i  = 0; i < 4; i ++ ) printf("Re: %d", recv_start_buf[i]);
            if((recv_start_buf[0] == 128)  && (recv_start_buf[1] == 128) && (recv_start_buf[2] == 128) && (recv_start_buf[3] == 128)) Transaction_Number = 1;
            Handshake_Flag = false;

        }else if(Handshake_Flag && Transaction_Number == 1)
        {
            for(int i  = 0; i < (LED_STRIP_LED_NUMBERS * 4); i ++ ) sendbuf[i] = 64;

            ret = spi_device_transmit(handle, &transaction2);

            if((recvbuf[0] == 128)  && (recvbuf[1] == 128) && (recvbuf[2] == 128) && (recvbuf[3] == 128)) Transaction_Number = 0;

            for(int i  = 0; i < (LED_STRIP_LED_NUMBERS * 4); i ++ ) printf("R: %d", recvbuf[i]);
            printf("\n\n");
            Handshake_Flag = false;
            for(int i = 0; i <LED_STRIP_LED_NUMBERS; i ++)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(led_strip, i, recvbuf[0 + (4 * i)], recvbuf[3 + (4 * i)], recvbuf[1 + (4 * i)], recvbuf[2 + (4 * i)])); //sendbuf[1 + (4 * i)]  RWGB
            }
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        } 
      
       
        
       /*
        
        ret=spi_device_transmit(handle, &transaction2);
        printf("Received: %s\n", recvbuf);
        for(int i = 0; i < LED_STRIP_LED_NUMBERS; i ++) ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(led_strip, i, 0,0, green, 0));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        green += 10;
        if(green > 240) green = 10;
       */
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    //Never reached.
    ret=spi_bus_remove_device(handle);
    assert(ret==ESP_OK);
}


led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRBW, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {

        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3

    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}
