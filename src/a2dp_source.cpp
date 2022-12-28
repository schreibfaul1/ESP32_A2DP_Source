/*
 * a2dp_source.c
 *
 *  Created on: 27.08.2020
 *      Author: wolle
 *
 *  updated on: 28.12.2021
 *
 *  use Arduino Version >= 2.0.4
 *
 */

#include <Arduino.h>
#include "a2dp_source.h"

extern String BT_DEVICE_NAME;

static esp_bd_addr_t     s_peer_bda           = {0};
static char              s_peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static int               m_a2d_state          = APP_AV_STATE_IDLE;
static int               m_media_state        = APP_AV_MEDIA_STATE_IDLE;
static String            s_BT_sink_name       = "";
static esp_bt_pin_code_t s_pin_code           = "";
static int               s_pin_code_length    = 0;
static TimerHandle_t     s_tmr;
static bool              m_bt_enabled        = false; // blue tooth app
static bool              m_hb_enabled        = false; // heart beat
static bt_app_msg_t      m_msg;

//---------------------------------------------------------------------------------------------------------------------
bool bt_app_work_dispatch(uint16_t event, void *p_params, int param_len){
    memset(&m_msg, 0, sizeof(bt_app_msg_t));
    m_msg.sig = BT_APP_SIG_WORK_DISPATCH;
    m_msg.event = event;
    m_msg.cb = bt_app_av_sm_hdlr;

    if (p_params && param_len > 0) {
        if ((m_msg.param = malloc(param_len)) != NULL) {
            memcpy(m_msg.param, p_params, param_len);
        }
    }
    return false;
}
//---------------------------------------------------------------------------------------------------------------------
void bt_loop(){
    static uint32_t timer = 0;
    if(m_bt_enabled){
        if(m_msg.sig > 0){
            switch (m_msg.sig) {
                case BT_APP_SIG_WORK_DISPATCH:
                    vTaskDelay(50); // is absolutely necessary
                    bt_app_av_sm_hdlr(m_msg.event, m_msg.param);
                    break;
                default:
                    log_e("unhandled sig: %d",m_msg.sig);
                    break;
                }
                if (m_msg.param) {free(m_msg.param);}
                m_msg.sig = 0;
        }
        else if(m_hb_enabled){
            if(timer < millis()){
                timer = millis() + 1000;
                bt_app_work_dispatch(BT_APP_HEART_BEAT_EVT, NULL, 0);
            }
        }
    }
}
//---------------------------------------------------------------------------------------------------------------------
void a2dp_source_stop(void){
    m_bt_enabled = false;
}
//---------------------------------------------------------------------------------------------------------------------
char *bda2str(esp_bd_addr_t bda, char *str, size_t size){
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}
//---------------------------------------------------------------------------------------------------------------------
bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len){ // extended inquiry response
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}
//---------------------------------------------------------------------------------------------------------------------
void filter_inquiry_scan_result(esp_bt_gap_cb_param_t *param){
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *eir = NULL;
    esp_bt_gap_dev_prop_t *p;

    char* sd = bda2str(param->disc_res.bda, bda_str, 18);
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            // NB enumeration is listed in
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            eir = (uint8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
        default:
            break;
        }
    }
    if (eir) get_name_from_eir(eir, (uint8_t*)s_peer_bdname, NULL); else s_peer_bdname[0] = 0;

    char* buf; buf = (char*)malloc(strlen(sd) + strlen(s_peer_bdname) + 70);
    if(buf){
        sprintf(buf, "Scanned device: %s  --Class of Device: 0x%x, --RSSI %d, --%s", sd, cod, rssi, s_peer_bdname);
        if(bt_info) bt_info(buf); else log_i("%s", buf);
        if(buf) free(buf);
    }

    /* search for device with MAJOR service class as "rendering" in COD */
    if (!esp_bt_gap_is_valid_cod(cod) ||
            !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING)) {
        log_e("cod is not valid");
        return;
    }

    /* search for device named  BT_SINK in its extended inqury response */
    if (eir) {
        get_name_from_eir(eir, (uint8_t*)s_peer_bdname, NULL);
        if (strcmp((char *)s_peer_bdname,  s_BT_sink_name.c_str()) != 0) {
            log_i("s_peer_bdname %s, s_BT_sink_name %s", s_peer_bdname, s_BT_sink_name.c_str());
            return;
        }

        BT_INFO("Found a target device, address %s, name %s", bda_str, s_peer_bdname);
        m_a2d_state = APP_AV_STATE_DISCOVERED;
        memcpy(s_peer_bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
        esp_bt_gap_cancel_discovery();
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        if(m_media_state == APP_AV_MEDIA_STATE_STARTED) break; // no longer require discovery results
        filter_inquiry_scan_result(param);
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {

        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            if (m_a2d_state == APP_AV_STATE_DISCOVERED) {
                m_a2d_state = APP_AV_STATE_CONNECTING;
                BT_INFO("Device discovery stopped: a2dp connecting to peer: %s", s_peer_bdname);
                esp_a2d_source_connect(s_peer_bda);
            } else {
                // not discovered, continue to discover
                // log_i("Device discovery failed, continue to discover...");
                esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            }
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            //log_i("Discovery started.");
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVCS_EVT:
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            log_i("authentication success: %s", param->auth_cmpl.device_name);
        } else {
            log_e("authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        log_i("ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            log_i("Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            log_i("Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            log_i("Input pin code: %s", s_pin_code);
            esp_bt_gap_pin_reply(param->pin_req.bda, true, s_pin_code_length, s_pin_code);
        }
        break;
    }
    case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
        log_i("ESP_BT_GAP_CONFIG_EIR_DATA_EVT: stat:%d num:%d", param->config_eir_data.stat, param->config_eir_data.eir_type_num);
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        log_i("ESP_BT_GAP_MODE_CHG_EVT: mode: %d", param->mode_chg.mode);
        break;


    default: {
        log_e("unhandled event: %d", event);
        break;
    }
    }
    return;
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param){
    bt_app_work_dispatch(event, param, sizeof(esp_a2d_cb_param_t));
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_av_sm_hdlr(uint16_t event, void *param){
    switch (m_a2d_state) {
    case APP_AV_STATE_DISCOVERING:
        print_status();
        break;
    case APP_AV_STATE_DISCOVERED:
        print_status();
        break;
    case APP_AV_STATE_UNCONNECTED:
        print_status();
        bt_app_av_state_unconnected(event, param);
        break;
    case APP_AV_STATE_CONNECTING:
        print_status();
        bt_app_av_state_connecting(event, param);
        break;
    case APP_AV_STATE_CONNECTED:
        print_status();
        bt_app_av_state_connected(event, param);
        break;
    case APP_AV_STATE_DISCONNECTING:
        print_status();
        bt_app_av_state_disconnecting(event, param);
        break;
    default:
        log_e("invalid state %d", m_a2d_state);
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_av_state_unconnected(uint16_t event, void *param){
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        log_d("ESP_A2D_AUDIO_CFG_EVT");
        break;
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        break;
    case BT_APP_HEART_BEAT_EVT: {
        uint8_t *p = s_peer_bda;
        BT_INFO("Heartbeat Event: a2dp most recent peer connection: %s @ %02x:%02x:%02x:%02x:%02x:%02x",
                 s_peer_bdname, p[0], p[1], p[2], p[3], p[4], p[5]);
        esp_a2d_source_connect(s_peer_bda);
        m_a2d_state = APP_AV_STATE_CONNECTING;
        break;
    }
    default:
        log_e("unhandled evt %d", event);
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_av_state_connecting(uint16_t event, void *param){
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            m_a2d_state =  APP_AV_STATE_CONNECTED;
            m_media_state = APP_AV_MEDIA_STATE_IDLE;
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            m_a2d_state =  APP_AV_STATE_UNCONNECTED;
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
            m_a2d_state =  APP_AV_STATE_CONNECTING;
        }
        print_status();
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
        log_d("ESP_A2D_AUDIO_STATE_EVT");
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        log_d("ESP_A2D_AUDIO_CFG_EVT");
        break;
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        log_d("ESP_A2D_MEDIA_CTRL_ACK_EVT");
        break;
    case BT_APP_HEART_BEAT_EVT:
        a2d = (esp_a2d_cb_param_t *)(param);
        if(a2d) if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED)  log_i("a2dp connected");

        m_a2d_state =  APP_AV_STATE_CONNECTED;
        m_media_state = APP_AV_MEDIA_STATE_IDLE;
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
//        if (++s_connecting_intv >= 2) {
//            m_a2d_state = APP_AV_STATE_UNCONNECTED;
//            s_connecting_intv = 0;
//        }
        break;
    default:
        log_e("unhandled evt %d\n", event);
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_av_media_proc(uint16_t event, void *param){
    esp_a2d_cb_param_t *a2d = NULL;
    a2d = (esp_a2d_cb_param_t *)(param);
    switch (m_media_state) {
        case APP_AV_MEDIA_STATE_IDLE: {
            if (event == BT_APP_HEART_BEAT_EVT) {
                //log_i("a2dp media ready checking ...");
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
            } else if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
                if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY &&
                        a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                    //log_i("a2dp media ready, starting ...");
                    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
                    m_media_state = APP_AV_MEDIA_STATE_STARTING;
                }
            }
            break;
        }
        case APP_AV_MEDIA_STATE_STARTING: {
            if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
                a2d = (esp_a2d_cb_param_t *)(param);
                if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START &&
                        a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                    log_i("a2dp media start successfully.");
                    //s_intv_cnt = 0;
                    m_media_state = APP_AV_MEDIA_STATE_STARTED;
                } else {
                    // not started succesfully, transfer to idle state
                    log_e("a2dp media start failed.\n");
                    m_media_state = APP_AV_MEDIA_STATE_IDLE;
                }
            }
            break;
        }
        case APP_AV_MEDIA_STATE_STARTED: {
            if (event == BT_APP_HEART_BEAT_EVT) {
//                if (++s_intv_cnt >= 2) { // was 10
//                    log_i("a2dp media stopping...");
//                    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
//                    m_media_state = APP_AV_MEDIA_STATE_STOPPING;
//                    s_intv_cnt = 0;
//                }
            }
            break;
        }
        case APP_AV_MEDIA_STATE_STOPPING: {
            if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
                a2d = (esp_a2d_cb_param_t *)(param);
                if (a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_STOP &&
                        a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                    m_media_state = APP_AV_MEDIA_STATE_IDLE;
                    esp_a2d_source_disconnect(s_peer_bda);
                    m_a2d_state = APP_AV_STATE_DISCONNECTING;
                } else {
                    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
                }
            }
            break;
        }
    }
    print_status();
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_av_state_connected(uint16_t event, void *param){
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT: {
            a2d = (esp_a2d_cb_param_t *)(param);
            if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                // log_i("a2dp disconnected");
                m_a2d_state = APP_AV_STATE_UNCONNECTED;
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            }
            break;
        }
        case ESP_A2D_AUDIO_STATE_EVT: {
            a2d = (esp_a2d_cb_param_t *)(param);
            if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
                //s_pkt_cnt = 0;
            }
            break;
        }
        case ESP_A2D_AUDIO_CFG_EVT:
            log_d("ESP_A2D_AUDIO_CFG_EVT");
            // not suppposed to occur for A2DP source
            break;
        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
            bt_app_av_media_proc(event, param);
            break;
        case BT_APP_HEART_BEAT_EVT: {
            bt_app_av_media_proc(event, param);
            break;
        }
        default:
            log_e("unhandled evt %d", event);
            break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_av_state_disconnecting(uint16_t event, void *param){
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(param);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            log_i("a2dp disconnected");
            m_a2d_state =  APP_AV_STATE_UNCONNECTED;
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    case BT_APP_HEART_BEAT_EVT:
        break;
    default:
        log_e("unhandled evt %d", event);
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
int get_APP_AV_STATE(){
    return m_a2d_state;
}
//---------------------------------------------------------------------------------------------------------------------
bool a2dp_source_init(String deviceName, String pinCode){
    print_status();
    esp_err_t res;
    s_BT_sink_name = deviceName;
    int s_pin_code_length = pinCode.length();
    for(int i=0; i<s_pin_code_length; i++)
        s_pin_code[i] = pinCode[i];

    do{  // release controller memory
        res= esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    }while(res==0);


    if(!btStart()) {log_e("Failed to initialize controller"); return false;}
    else log_i("controller initialized");

    res = esp_bluedroid_init(); // init bluedroid
    if(res != ESP_OK){log_e("initialize bluedroid failed"); return false;}

    res = esp_bluedroid_enable(); // activate bluedroid
    if(res != ESP_OK){log_e("enable bluedroid failed"); return false;}
    m_bt_enabled = true;

    /* Bluetooth device name, connection mode and profile set up */
    esp_bt_dev_set_device_name(BT_DEVICE_NAME.c_str());                         /* set up device name */
    esp_bt_gap_register_callback(bt_app_gap_cb);                                /* register GAP callback function */
    esp_a2d_register_callback(&bt_app_a2d_cb);                                  /* initialize A2DP source */
    esp_a2d_source_register_data_callback(bt_app_a2d_data_cb);
    esp_a2d_source_init();
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);  /* set discoverable and connectable mode */
    m_a2d_state = APP_AV_STATE_DISCOVERING;                                     /* start device discovery */
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    m_hb_enabled = true;

    // Set default parameters for Legacy Pairing, use variable pin, input pin code when pairing
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    return true;
}
void print_status(){
    static int old_a2d_state = -1;
    static int old_media_state = -1;
    char   ad2_state[7][14]= {"idle", "discovering", "discovered", "unconnected", "connecting", "connected", "disconnecting"};
    char media_state[7][ 9]= {"idle", "starting", "started", "stopping"};

    if(m_a2d_state != old_a2d_state){
        if(old_a2d_state != -1) BT_INFO("bt_state: %s --> %s", ad2_state[old_a2d_state],ad2_state[m_a2d_state]);
        old_a2d_state = m_a2d_state;
    }
    if(m_media_state != old_media_state){
        if(old_media_state != -1) BT_INFO("media_state: %s --> %s", media_state[old_media_state],media_state[m_media_state]);
        old_media_state = m_media_state;
    }
}
//---------------------------------------------------------------------------------------------------------------------
int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len){ // BT data event
    uint32_t dataLength = 0;
    uint32_t sampleRate = 0;
    if(bt_data) dataLength = bt_data(data, len, &sampleRate);
    static uint8_t i = 0;
    if(sampleRate != 44100) log_e("SampleRate must be 44100");
    // todo scale sampleRate
    return dataLength;
}
