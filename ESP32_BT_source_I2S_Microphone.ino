/*
  This example code is in the Public Domain (or CC0 licensed, at your option.)
  Unless required by applicable law or agreed to in writing, this
  software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
  CONDITIONS OF ANY KIND, either express or implied.

  Arduino port of https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/a2dp_source
  origin URL: https://github.com/dgm3333/Arduino_A2DP_Source

  After connection with A2DP sink is established, the example performs the following running loop 1-2-3-4-1:
    1. audio transmission starts and lasts for a while
    2. audio transmission stops
    3. disconnect with target device
    4. reconnect to target device
  The example implements an event loop triggered by a periodic "heart beat" timer and events from Bluetooth protocol stack callback functions.

  For current stage, the supported audio codec in ESP32 A2DP is SBC (SubBand Coding).
  SBC specification is in Appendix B (page 50) of the document A2DP_Spec_V1_0 (can be found with search engine, although the original is behind the Bluetooth firewall)

  SBC audio stream is encoded from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data.
  Other SBC configurations can be supported but there is a need for additional modifications to the protocol stack.
*/

#include "Arduino.h"
#include "a2dp_source.h"
#include <driver/i2s.h>

#define RX_I2S_DIN    33    // connect with I2S microphone
#define RX_I2S_BCLK   12
#define RX_I2S_LRC    14

char BT_SINK_NAME[]   = "Manhattan-165327"; // sink devicename
//char BT_SINK_NAME[]   = "ESP32-SPEAKER"; // sink devicename
char BT_SINK_PIN[]    = "1234";             // sink pincode
char BT_DEVICE_NAME[] = "ESP_A2DP_SRC";     // source devicename

const i2s_port_t I2S_PORT_RX = I2S_NUM_1;
const uint32_t   sample_rate = 44100;
const uint16_t   buf_len     = 1024;
size_t bytes_written = 0;
char readBuff1[buf_len];
uint16_t buff1Size;
char readBuff2[buf_len];
uint16_t buff2Size;
uint8_t buffNr;
uint8_t gain = 14;   // reduce volume -> increase gain

enum : uint8_t {BUFF_NONE, BUFF_ONE, BUFF_TWO};

//---------------------------------------------------------------------------------------------------------------------
void i2s_install(){
    /* RX: I2S_NUM_1 */
        i2s_config_t i2s_config_rx = {
        .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_RX), // Only TX
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,    // Only 8-bit DAC support
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // 2-channels
        .communication_format = (i2s_comm_format_t) I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,        // Interrupt level 1
        .dma_buf_count = 16,                            // number of buffers, 128 max.
        .dma_buf_len = 256,                          // size of each buffer
        .use_apll             = false,
        .tx_desc_auto_clear   = true,   // new in V1.0.1
        .fixed_mclk           = I2S_PIN_NO_CHANGE,
        };

        i2s_pin_config_t pin_config_rx = {
            .bck_io_num   = RX_I2S_BCLK,
            .ws_io_num    = RX_I2S_LRC,
            .data_out_num = I2S_PIN_NO_CHANGE,
            .data_in_num  = RX_I2S_DIN
        };

        i2s_driver_install(I2S_PORT_RX, &i2s_config_rx, 0, NULL);
        i2s_set_pin(I2S_PORT_RX, &pin_config_rx);
}
//---------------------------------------------------------------------------------------------------------------------
void setup(){
    i2s_install();
    buffNr = BUFF_ONE;
    log_i("free heap %i", esp_get_free_heap_size());
    a2dp_source_init(BT_SINK_NAME, BT_SINK_PIN);
    log_i("free heap %i", esp_get_free_heap_size());
}
//---------------------------------------------------------------------------------------------------------------------
void loop() {

//  char *buf_ptr_read1  = readBuff1 + 4; // connect L/R with ground
    char *buf_ptr_read1  = readBuff1;     // connect L/R with VDD
    char *buf_ptr_write1 = readBuff1;
//  char *buf_ptr_read2  = readBuff2 + 4; // connect L/R with ground
    char *buf_ptr_read2  = readBuff2;     // connect L/R with VDD
    char *buf_ptr_write2 = readBuff2;

    if(buffNr == BUFF_ONE) {
//        log_i("BUFF_ONE");
        size_t bytes_read = 0;
        while(bytes_read == 0) {
            i2s_read(I2S_PORT_RX, readBuff1, buf_len, &bytes_read, portMAX_DELAY);
        }
        uint32_t samples_read = bytes_read / 2 / (I2S_BITS_PER_SAMPLE_32BIT / 8);

        //  convert 2x 32 bit stereo -> 1 x 16 bit mono
        for(int i = 0; i < samples_read; i++) {

            // left channel
            int32_t sample = (buf_ptr_read1[3] << 24) + (buf_ptr_read1[2] << 16) + (buf_ptr_read1[1] << 8) + buf_ptr_read1[4];
            sample = sample >> gain;
            buf_ptr_write1[0] = sample & 0x00FF;
            buf_ptr_write1[1] = (sample >>8) & 0x00FF;

            // right channel
            buf_ptr_write1[2] = buf_ptr_write1[0]; // mid
            buf_ptr_write1[3] = buf_ptr_write1[1]; // high

            buf_ptr_write1 += 2 * (I2S_BITS_PER_SAMPLE_16BIT / 8);
            buf_ptr_read1 += 2 * (I2S_BITS_PER_SAMPLE_32BIT / 8);

            buff1Size = samples_read * 2 * (I2S_BITS_PER_SAMPLE_16BIT / 8);
        }
        buffNr = BUFF_NONE;
    }

    if(buffNr == BUFF_TWO) {
//        log_i("BUFF_TWO");
        size_t bytes_read = 0;
        while(bytes_read == 0) {
            i2s_read(I2S_PORT_RX, readBuff2, buf_len, &bytes_read, portMAX_DELAY);
        }

        uint32_t samples_read = bytes_read / 2 / (I2S_BITS_PER_SAMPLE_32BIT / 8);

        //  convert 2x 32 bit stereo -> 1 x 16 bit mono
        for(int i = 0; i < samples_read; i++) {

            // left channel
            int32_t sample = (buf_ptr_read2[3] << 24) + (buf_ptr_read2[2] << 16) + (buf_ptr_read2[1] << 8) + buf_ptr_read2[4];
            sample = sample >> gain;
            buf_ptr_write2[0] = sample & 0x00FF;
            buf_ptr_write2[1] = (sample >>8) & 0x00FF;

            // right channel
            buf_ptr_write2[2] = buf_ptr_write2[0]; // mid
            buf_ptr_write2[3] = buf_ptr_write2[1]; // high

            buf_ptr_write2 += 2 * (I2S_BITS_PER_SAMPLE_16BIT / 8);
            buf_ptr_read2 += 2 * (I2S_BITS_PER_SAMPLE_32BIT / 8);

            buff2Size = samples_read * 2 * (I2S_BITS_PER_SAMPLE_16BIT / 8);
        }
        buffNr = BUFF_NONE;
    }
}
//---------------------------------------------------------------------------------------------------------------------
int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len) // BT data event
{
    static uint8_t bnr = 1;
    if (len < 0 || data == NULL) {
        return 0;
    }
    if(bnr==1){
        if(!buff1Size) return 0;
//        log_i("buff1Size %i", buff1Size);
        memcpy(data, readBuff1, buff1Size);
        bnr=2;
        buffNr = BUFF_TWO;
        return buff1Size;
    }

    if(bnr==2){
        if(!buff2Size) return 0;
//        log_i("buff2Size %i", buff2Size);
        memcpy(data, readBuff2, buff2Size);
        bnr=1;
        buffNr = BUFF_ONE;
        return buff2Size;
    }
    return 0;
}




