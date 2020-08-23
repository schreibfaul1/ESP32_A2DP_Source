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
#include "SD.h"
#include "FS.h"

// Digital I/O used
#define SD_CS          5
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18

String BT_SINK        = "Manhattan-165327"; // BT loudspeaker
String BT_DEVICE_NAME = "ESP_A2DP_SRC";     // source devicename

File            audiofile;    // @suppress("Abstract class cannot be instantiated")

uint32_t        sampleRate;
uint32_t        bitRate;
uint8_t         channels;
uint8_t         bitsPerSample=16;

//---------------------------------------------------------------------------------------------------------------------
bool parseWAV(fs::FS &fs, String path){
    char chbuf[256];
    audiofile=fs.open(path.c_str());
    String afn = (String)audiofile.name();  //audioFileName

    if(afn.endsWith(".wav")) {
        audiofile.readBytes(chbuf, 4); // read RIFF tag
        if ((chbuf[0] != 'R') || (chbuf[1] != 'I') || (chbuf[2] != 'F') || (chbuf[3] != 'F')){
            Serial.println("file has no RIFF tag");
            audiofile.seek(0);
            return false;
        }

        audiofile.readBytes(chbuf, 4); // read chunkSize (datalen)
        uint32_t cs = (uint32_t)(chbuf[0] + (chbuf[1] <<8) + (chbuf[2] <<16) + (chbuf[3] <<24) - 8);

        audiofile.readBytes(chbuf, 4); /* read wav-format */ chbuf[5] = 0;
        if ((chbuf[0] != 'W') || (chbuf[1] != 'A') || (chbuf[2] != 'V') || (chbuf[3] != 'E')){
            Serial.println("format tag is not WAVE");
            audiofile.seek(0);
            return false;
        }

        while(true){ // skip wave chunks, seek for fmt element
            audiofile.readBytes(chbuf, 4); /* read wav-format */
            if ((chbuf[0] == 'f') && (chbuf[1] == 'm') && (chbuf[2] == 't')){
                //if(audio_info) audio_info("format tag found");
                break;
            }
        }

        audiofile.readBytes(chbuf, 4); // fmt chunksize
        cs = (uint32_t) (chbuf[0] + (chbuf[1] <<8));
        if(cs>40) return false; //something is wrong
        uint8_t bts=cs-16; // bytes to skip if fmt chunk is >16
        audiofile.readBytes(chbuf, 16);
        uint16_t fc  = (uint16_t)(chbuf[0]  + (chbuf[1] <<8));  // Format code
        uint16_t nic = (uint16_t)(chbuf[2]  + (chbuf[3] <<8));  // Number of interleaved channels
        uint32_t sr  = (uint32_t)(chbuf[4]  + (chbuf[5] <<8) + (chbuf[6]  <<16) + (chbuf[7]  <<24)); // Smpling rate
        uint32_t dr  = (uint32_t)(chbuf[8]  + (chbuf[9] <<8) + (chbuf[10] <<16) + (chbuf[11] <<24)); // Data rate
        uint16_t dbs = (uint16_t)(chbuf[12] + (chbuf[13] <<8));  // Data block size
        uint16_t bps = (uint16_t)(chbuf[14] + (chbuf[15] <<8));  // Bits per sample
        Serial.printf("FormatCode=%u\n", fc);
        Serial.printf("Channel=%u\n", nic);
        Serial.printf("SampleRate=%u\n", sr);
        Serial.printf("DataRate=%u\n", dr);
        Serial.printf("DataBlockSize=%u\n", dbs);
        Serial.printf("BitsPerSample=%u\n", bps);


        if(fc != 1){
            Serial.println("format code is not 1 (PCM)");
            return false;
        }

        if(nic != 1 && nic != 2){
            Serial.print("number of channels must be 1 or 2");
            return false;
        }

        if(bps != 8 && bps !=16){
            Serial.println("bits per sample must be 8 or 16");
            return false;
        }
        bitsPerSample=bps;
        channels = nic;
        sampleRate = sr;
        bitRate = nic * sr * bps;
        Serial.printf("BitRate=%u\n", bitRate);

        audiofile.readBytes(chbuf, bts); // skip to data
        uint32_t s = audiofile.position();
        //here can be extra info, seek for data;
        while(true){
            audiofile.seek(s);
            audiofile.readBytes(chbuf, 4); /* read header signature */
            if ((chbuf[0] == 'd') && (chbuf[1] == 'a') && (chbuf[2] == 't') && (chbuf[3] == 'a')) break;
            s++;
        }

        audiofile.readBytes(chbuf, 4); // read chunkSize (datalen)
        cs = chbuf[0] + (chbuf[1] <<8) + (chbuf[2] <<16) + (chbuf[3] <<24) - 44;
        sprintf(chbuf, "DataLength=%u\n", cs);
        Serial.print(chbuf);
        return true;
    }
    return false;
}

//---------------------------------------------------------------------------------------------------------------------
void setup(){

    esp_err_t res;

    Serial.begin(115200);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    SD.begin(SD_CS);
    parseWAV(SD, "/cola.wav");

    do{  // release controller memory
        res= esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    }while(res==0);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT(); //controller init

    res = esp_bt_controller_init(&bt_cfg);
    if(res != ESP_OK){log_e("initialize controller failed %d", res); return;}

    res = esp_bt_controller_enable(ESP_BT_MODE_BTDM); // enable BT
    if(res != ESP_OK){log_e("enable controller failed %d", res); return;}

    res = esp_bluedroid_init(); // init bluedroid
    if(res != ESP_OK){log_e("initialize bluedroid failed"); return;}

    res = esp_bluedroid_enable(); // activate bluedroid
    if(res != ESP_OK){log_e("enable bluedroid failed"); return;}

    perform_wipe_security_db(); // delete pair devices
    bt_app_task_start_up(); // create application task
    /* Bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

    // Set default parameters for Legacy Pairing, use variable pin, input pin code when pairing
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}
//---------------------------------------------------------------------------------------------------------------------
void loop() {}
//---------------------------------------------------------------------------------------------------------------------

int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len) // BT data event
{
    if (len < 0 || data == NULL) {
        return 0;
    }
    return audiofile.read(data, len);
}





