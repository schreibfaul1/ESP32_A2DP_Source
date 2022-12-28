/*
 * a2dp_source.h
 *
 *  Created on: 23.08.2020
 *      Author: wolle
 *
 *  updated on: 28.12.2021
 *
 *  use Arduino Version >= 2.0.4
 *
 */

#ifndef A2DP_SOURCE_H_
#define A2DP_SOURCE_H_

#include "Arduino.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#define BT_APP_HEART_BEAT_EVT             (0xff00)
#define BT_APP_SIG_WORK_DISPATCH          (0x01)

extern __attribute__((weak)) int32_t bt_data(uint8_t *data, int32_t len, uint32_t* sampleRate);
extern __attribute__((weak)) void bt_info(const char* info);
#define BT_INFO(...) {char buff[512]; sprintf(buff,__VA_ARGS__); if(bt_info) bt_info(buff);}

// event for handler "bt_av_hdl_stack_up
enum {
    BT_APP_EVT_STACK_UP = 0,
};

// A2DP global state
enum {
    APP_AV_STATE_IDLE = 0,
    APP_AV_STATE_DISCOVERING = 1,
    APP_AV_STATE_DISCOVERED = 2,
    APP_AV_STATE_UNCONNECTED = 3,
    APP_AV_STATE_CONNECTING = 4,
    APP_AV_STATE_CONNECTED = 5,
    APP_AV_STATE_DISCONNECTING = 6,
};

// sub states of APP_AV_STATE_CONNECTED
enum {
    APP_AV_MEDIA_STATE_IDLE = 0,
    APP_AV_MEDIA_STATE_STARTING = 1,
    APP_AV_MEDIA_STATE_STARTED = 2,
    APP_AV_MEDIA_STATE_STOPPING = 3,
};

typedef void (* bt_app_cb_t) (uint16_t event, void *param);

/* message to be sent */
typedef struct {
    uint16_t             sig;      /*!< signal to bt_app_task */
    uint16_t             event;    /*!< message event id */
    bt_app_cb_t          cb;       /*!< context switch callback */
    void                 *param;   /*!< parameter area needs to be last */
} bt_app_msg_t;

typedef void (* bt_app_copy_cb_t) (bt_app_msg_t *msg, void *p_dest, void *p_src);


bool bt_app_work_dispatch(uint16_t event, void *p_params, int param_len);
void a2dp_source_stop(void);
char *bda2str(esp_bd_addr_t bda, char *str, size_t size);
bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len);
void filter_inquiry_scan_result(esp_bt_gap_cb_param_t *param);
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len);
void bt_app_av_sm_hdlr(uint16_t event, void *param);
void bt_app_av_state_unconnected(uint16_t event, void *param);
void bt_app_av_state_connecting(uint16_t event, void *param);
void bt_app_av_media_proc(uint16_t event, void *param);
void bt_app_av_state_connected(uint16_t event, void *param);
void bt_app_av_state_disconnecting(uint16_t event, void *param);
int  get_APP_AV_STATE();
bool a2dp_source_init(String deviceName, String pinCode);
void bt_loop();
void print_status();
#endif /* A2DP_SOURCE_H_ */
