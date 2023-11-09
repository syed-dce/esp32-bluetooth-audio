// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD


#include "BluetoothA2DPSink.h"

/**
 * Some data that must be avaliable for C calls
 */
// to support static callback functions
BluetoothA2DPSink* actualBluetoothA2DPSink;

// Forward declarations for C Callback functions for ESP32 Framework
extern "C" void app_task_handler_2(void *arg);
extern "C" void audio_data_callback_2(const uint8_t *data, uint32_t len);
extern "C" void app_a2d_callback_2(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
extern "C" void app_rc_ct_callback_2(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

/**
 * Constructor
 */
BluetoothA2DPSink::BluetoothA2DPSink() {
	actualBluetoothA2DPSink = this;
	output = NULL;
}

BluetoothA2DPSink::~BluetoothA2DPSink() {
    app_task_shut_down();
    
    ESP_LOGI(BT_AV_TAG,"disable bluetooth");
    if (esp_bluedroid_disable() != ESP_OK){
        ESP_LOGE(BT_AV_TAG,"Failed to disable bluetooth");
    }
    
    ESP_LOGI(BT_AV_TAG,"deinit bluetooth");
    if (esp_bluedroid_deinit() != ESP_OK){
        ESP_LOGE(BT_AV_TAG,"Failed to deinit bluetooth");
    }
    
    ESP_LOGI(BT_AV_TAG,"esp_bt_controller_deinit");
	if (esp_bt_controller_deinit()!= ESP_OK){
    	ESP_LOGE(BT_AV_TAG,"esp_bt_controller_deinit failed");
	}
	
    ESP_LOGI(BT_AV_TAG,"esp_bt_controller_mem_release");
    if (esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT)!= ESP_OK){
	    ESP_LOGE(BT_AV_TAG,"esp_bt_controller_mem_release failed");
    }

}

void BluetoothA2DPSink::set_on_data_received(void (*callBack)()){
  this->data_received = callBack;
}

/**
 * Main function to start the Bluetooth Processing
 */
void BluetoothA2DPSink::start(char* name, SPDIFOut *output)
{
	ESP_LOGD(BT_AV_TAG, "%s", __func__);
    //store parameters
    if (name) {
      this->bt_name = name;
    }
    ESP_LOGI(BT_AV_TAG,"Device name will be set to '%s'",this->bt_name);
	this->output = output;
	output->SetBitsPerSample(32);
    output->SetChannels(2);

    if (!output->begin()){
		ESP_LOGE(BT_AV_TAG,"SPDIF output begin error!");
	}
	
    // setup bluetooth
    init_bluetooth();
    
    // create application task 
    app_task_start_up();

    //Lambda for callback
    auto av_hdl_stack_evt_2 = [](uint16_t event, void *p_param) {
        ESP_LOGD(BT_AV_TAG, "%s", __func__);
        if (actualBluetoothA2DPSink) {
            actualBluetoothA2DPSink->av_hdl_stack_evt(event,p_param);
        }
    };

    // Bluetooth device name, connection mode and profile set up 
    app_work_dispatch(av_hdl_stack_evt_2, BT_APP_EVT_STACK_UP, NULL, 0);
}

esp_a2d_audio_state_t BluetoothA2DPSink::get_audio_state() {
  return audio_state;
}

esp_a2d_mct_t BluetoothA2DPSink::get_audio_type() {
  return audio_type;
}


int BluetoothA2DPSink::init_bluetooth()
{
  ESP_LOGD(BT_AV_TAG, "%s", __func__);
  if (!btStart()) {
    ESP_LOGE(BT_AV_TAG,"Failed to initialize controller");
    return false;
  }
  ESP_LOGI(BT_AV_TAG,"controller initialized");
 
  if (esp_bluedroid_init() != ESP_OK) {
    ESP_LOGE(BT_AV_TAG,"Failed to initialize bluedroid");
    return false;
  }
  ESP_LOGI(BT_AV_TAG,"bluedroid initialized");
 
  if (esp_bluedroid_enable() != ESP_OK) {
    ESP_LOGE(BT_AV_TAG,"Failed to enable bluedroid");
    return false;
  }
  ESP_LOGI(BT_AV_TAG,"bluedroid enabled");
 
}

bool BluetoothA2DPSink::app_work_dispatch(app_callback_t p_cback, uint16_t event, void *p_params, int param_len)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s event 0x%x, param len %d", __func__, event, param_len);
    
    app_msg_t msg;
    memset(&msg, 0, sizeof(app_msg_t));

    msg.sig = APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            return app_send_msg(&msg);
        }
    }

    return false;
}

void BluetoothA2DPSink::app_work_dispatched(app_msg_t *msg)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}


bool BluetoothA2DPSink::app_send_msg(app_msg_t *msg)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    if (msg == NULL) {
        return false;
    }

    if (xQueueSend(app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}


void BluetoothA2DPSink::app_task_handler()
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    app_msg_t msg;
    for (;;) {
        if (pdTRUE == xQueueReceive(app_task_queue, &msg, (portTickType)portMAX_DELAY)) {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, sig 0x%x, 0x%x", __func__, msg.sig, msg.event);
            switch (msg.sig) {
            case APP_SIG_WORK_DISPATCH:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, APP_SIG_WORK_DISPATCH sig: %d", __func__, msg.sig);
                app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled sig: %d", __func__, msg.sig);
                break;
            } // switch (msg.sig)

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}

void BluetoothA2DPSink::app_task_start_up(void)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    app_task_queue = xQueueCreate(10, sizeof(app_msg_t));
    xTaskCreate(app_task_handler_2, "BtAppT", 2048, NULL, configMAX_PRIORITIES - 3, &app_task_handle);
    return;
}

void  BluetoothA2DPSink::app_task_shut_down(void)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    if (app_task_handle) {
        vTaskDelete(app_task_handle);
        app_task_handle = NULL;
    }
    if (app_task_queue) {
        vQueueDelete(app_task_queue);
        app_task_queue = NULL;
    }
}


void  BluetoothA2DPSink::app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);
    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;

    rc->meta_rsp.attr_text = attr_text;
}

void  BluetoothA2DPSink::app_rc_ct_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);

    // lambda for callback
    auto av_hdl_avrc_evt_2 = [](uint16_t event, void *p_param){
        ESP_LOGD(BT_AV_TAG, "%s", __func__);
        if (actualBluetoothA2DPSink) {
            actualBluetoothA2DPSink->av_hdl_avrc_evt(event,p_param);    
        }
    };

    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        ESP_LOGD(BT_AV_TAG, "%s ESP_AVRC_CT_METADATA_RSP_EVT", __func__);
        app_alloc_meta_buffer(param);
        app_work_dispatch(av_hdl_avrc_evt_2, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        ESP_LOGD(BT_AV_TAG, "%s ESP_AVRC_CT_CONNECTION_STATE_EVT", __func__);
        app_work_dispatch(av_hdl_avrc_evt_2, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
        ESP_LOGD(BT_AV_TAG, "%s ESP_AVRC_CT_PASSTHROUGH_RSP_EVT", __func__);
        app_work_dispatch(av_hdl_avrc_evt_2, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        ESP_LOGD(BT_AV_TAG, "%s ESP_AVRC_CT_CHANGE_NOTIFY_EVT", __func__);
        app_work_dispatch(av_hdl_avrc_evt_2, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        ESP_LOGD(BT_AV_TAG, "%s ESP_AVRC_CT_REMOTE_FEATURES_EVT", __func__);
        app_work_dispatch(av_hdl_avrc_evt_2, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

void  BluetoothA2DPSink::av_hdl_a2d_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_CONNECTION_STATE_EVT", __func__);
        a2d = (esp_a2d_cb_param_t *)(p_param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        ESP_LOGI(BT_AV_TAG, "A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
             m_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_STATE_EVT", __func__);
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP audio state: %s", m_a2d_audio_state_str[a2d->audio_stat.state]);
        m_audio_state = a2d->audio_stat.state;
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            m_pkt_cnt = 0;
        }
        break;
    }
    case ESP_A2D_AUDIO_CFG_EVT: {
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_CFG_EVT", __func__);
        esp_a2d_cb_param_t *esp_a2d_callback_param = (esp_a2d_cb_param_t *)(p_param);
        audio_type = esp_a2d_callback_param->audio_cfg.mcc.type;
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "a2dp audio_cfg_cb , codec type %d", a2d->audio_cfg.mcc.type);
        // for now only SBC stream is supported
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
			ESP_LOGI(BT_AV_TAG,"Sample rate: 16000");
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6)) {
				ESP_LOGD(BT_AV_TAG,"Sample rate: 32000");
            } else if (oct0 & (0x01 << 5)) {
				ESP_LOGD(BT_AV_TAG,"Sample rate: 44100");
            } else if (oct0 & (0x01 << 4)) {
				ESP_LOGD(BT_AV_TAG,"Sample rate: 48000");
            }

            ESP_LOGD(BT_AV_TAG, "configure audio player %x-%x-%x-%x\n",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
        }
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}

void  BluetoothA2DPSink::av_new_track()
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    //Register notifications and request metadata
    esp_avrc_ct_send_metadata_cmd(0, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_GENRE);
    esp_avrc_ct_send_register_notification_cmd(1, ESP_AVRC_RN_TRACK_CHANGE, 0);
}

void  BluetoothA2DPSink::av_notify_evt_handler(uint8_t event_id, uint32_t event_parameter)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    switch (event_id) {
    case ESP_AVRC_RN_TRACK_CHANGE:
        ESP_LOGD(BT_AV_TAG, "%s ESP_AVRC_RN_TRACK_CHANGE %d", __func__, event_id);
        av_new_track();
        break;
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event_id);
        break;
    }
}

void  BluetoothA2DPSink::av_hdl_avrc_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_AV_TAG, "AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (rc->conn_stat.connected) {
            av_new_track();
        }
        break;
    }
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        ESP_LOGI(BT_AV_TAG, "AVRC passthrough rsp: key_code 0x%x, key_state %d", rc->psth_rsp.key_code, rc->psth_rsp.key_state);
        break;
    }
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        ESP_LOGI(BT_AV_TAG, "AVRC metadata rsp: attribute id 0x%x, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
        free(rc->meta_rsp.attr_text);
        break;
    }
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        ESP_LOGI(BT_AV_TAG, "AVRC event notification: %d, param: %d", rc->change_ntf.event_id, rc->change_ntf.event_parameter);
        av_notify_evt_handler(rc->change_ntf.event_id, rc->change_ntf.event_parameter);
        break;
    }
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_AV_TAG, "AVRC remote features %x", rc->rmt_feats.feat_mask);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}


void  BluetoothA2DPSink::av_hdl_stack_evt(uint16_t event, void *p_param)
{
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        ESP_LOGD(BT_AV_TAG, "%s av_hdl_stack_evt %s", __func__, "BT_APP_EVT_STACK_UP");
        /* set up device name */
        esp_bt_dev_set_device_name(bt_name);

        /* initialize A2DP sink */
        esp_a2d_register_callback(app_a2d_callback_2);
        esp_a2d_sink_register_data_callback(audio_data_callback_2);
        esp_a2d_sink_init();

        /* initialize AVRCP controller */
        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(app_rc_ct_callback_2);

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}


/* callback for A2DP sink */
void  BluetoothA2DPSink::app_a2d_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    // lambda for callback
    auto av_hdl_a2d_evt_2=[](uint16_t event, void *p_param){
        ESP_LOGD(BT_AV_TAG, "%s", __func__);
        if (actualBluetoothA2DPSink) {
            actualBluetoothA2DPSink->av_hdl_a2d_evt(event,p_param);  
        }
    };

    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_CONNECTION_STATE_EVT", __func__);
        app_work_dispatch(av_hdl_a2d_evt_2, event, param, sizeof(esp_a2d_cb_param_t));
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_STATE_EVT", __func__);
        audio_state = param->audio_stat.state;
        app_work_dispatch(av_hdl_a2d_evt_2,event, param, sizeof(esp_a2d_cb_param_t));
        break;
    case ESP_A2D_AUDIO_CFG_EVT: {
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_CFG_EVT", __func__);
        app_work_dispatch(av_hdl_a2d_evt_2, event, param, sizeof(esp_a2d_cb_param_t));
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}

void  BluetoothA2DPSink::audio_data_callback(const uint8_t *data, uint32_t len) {
	uint16_t *data16 = (uint16_t *) data;
	for (int j=0;j<len/2;j+=2) {
		sample[0] = data16[j];
		sample[1] = data16[j+1];
		output->ConsumeSample(sample);
	}
}

void BluetoothA2DPSink::sendCommand(BT_CMND cmnd){
	switch(cmnd){
		case PLAY:
			esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_PRESSED);
			break;
			
		case PAUSE:
			esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PAUSE, ESP_AVRC_PT_CMD_STATE_PRESSED);
			break;
			
		case STOP:
			esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_STOP, ESP_AVRC_PT_CMD_STATE_PRESSED);
			break;
			
		case NEXT:
			esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_FORWARD, ESP_AVRC_PT_CMD_STATE_PRESSED);
			break;
			
		case PREV:
			esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_BACKWARD, ESP_AVRC_PT_CMD_STATE_PRESSED);
			break;
		
		default:
			break;
	}
}

/**
 * C Callback Functions needed for the ESP32 API
 */
extern "C" void app_task_handler_2(void *arg) {
  ESP_LOGD(BT_AV_TAG, "%s", __func__);
  if (actualBluetoothA2DPSink)
    actualBluetoothA2DPSink->app_task_handler();
}

extern "C" void audio_data_callback_2(const uint8_t *data, uint32_t len) {
  ESP_LOGD(BT_AV_TAG, "%s", __func__);
  if (actualBluetoothA2DPSink)
    actualBluetoothA2DPSink->audio_data_callback(data,len);
}

extern "C" void app_a2d_callback_2(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param){
  ESP_LOGD(BT_AV_TAG, "%s", __func__);
  if (actualBluetoothA2DPSink)
    actualBluetoothA2DPSink->app_a2d_callback(event, param);
}

extern "C" void app_rc_ct_callback_2(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param){
  ESP_LOGD(BT_AV_TAG, "%s", __func__);
  if (actualBluetoothA2DPSink)
    actualBluetoothA2DPSink->app_rc_ct_callback(event, param);
}
