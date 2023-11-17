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

#ifndef __A2DP_SINK_H__
#define __A2DP_SINK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "driver/i2s.h"
#include "esp_avrc_api.h"

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#include "esp32-hal-bt.h"
#endif

#define APP_CORE_TAG  "BT_APP_CORE"
#define APP_SIG_WORK_DISPATCH (0x01)

#define AUTOCONNECT_TRY_NUM (5)

/**
 * @brief     handler for the dispatched work
 */
typedef void (* app_callback_t) (uint16_t event, void *param);


/* message to be sent */
typedef struct {
    uint16_t             sig;      /*!< signal to app_task */
    uint16_t             event;    /*!< message event id */
    app_callback_t          cb;       /*!< context switch callback */
    void                 *param;   /*!< parameter area needs to be last */
} app_msg_t;

/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};



/**
 * Bluethooth Sink - We iniitialize and start the Bluetooth A2DP Sink. 
 * The example https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/a2dp_sink
 * was refactered into a C++ class 
 */

class BluetoothA2DPSink {
  public: 
    BluetoothA2DPSink();
    ~BluetoothA2DPSink();
    virtual void set_pin_config(i2s_pin_config_t pin_config);
    virtual void set_i2s_port(i2s_port_t i2s_num);
    virtual void set_i2s_config(i2s_config_t i2s_config);
    virtual void set_avrc_metadata_callback(void (*callback)(uint8_t, const uint8_t*)) {
      this->avrc_metadata_callback = callback;
    }

    
    virtual void start(char* name);
    virtual esp_a2d_audio_state_t get_audio_state();
    virtual esp_a2d_mct_t get_audio_type();
    virtual void set_stream_reader(void (*callBack)(const uint8_t*, uint32_t));
    virtual void set_on_data_received(void (*callBack)());
    virtual void play();
    virtual void pause();
    virtual void stop();
    virtual void next();
    virtual void previous();



    /**
     * Wrappbed methods called from callbacks
     */
    virtual void app_a2d_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
    virtual void app_rc_ct_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
    virtual void app_task_handler();
    // Callback for music stream 
    virtual void audio_data_callback(const uint8_t *data, uint32_t len);
    // av event handler
    virtual void av_hdl_stack_evt(uint16_t event, void *p_param);
    // a2dp event handler 
    virtual void av_hdl_a2d_evt(uint16_t event, void *p_param);
    // avrc event handler 
    virtual void av_hdl_avrc_evt(uint16_t event, void *p_param);
	
	void connectToLastDevice();
    
  protected:
    // protected data
    xQueueHandle app_task_queue;
    xTaskHandle app_task_handle;
    i2s_config_t i2s_config;
    i2s_pin_config_t pin_config;    
    char * bt_name;
    uint32_t m_pkt_cnt = 0;
    esp_a2d_audio_state_t m_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
    const char *m_a2d_conn_state_str[4] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
    const char *m_a2d_audio_state_str[3] = {"Suspended", "Stopped", "Started"};
    esp_a2d_audio_state_t audio_state;
    esp_a2d_mct_t audio_type;
    void (*data_received)() = NULL;
    void (*stream_reader)(const uint8_t*, uint32_t) = NULL;
	  esp_bd_addr_t lastBda = {NULL};
    void (*avrc_metadata_callback)(uint8_t, const uint8_t*) = NULL;


    // protected methods
    virtual int init_bluetooth();
    virtual void app_task_start_up(void);
    virtual void app_task_shut_down(void);
    virtual bool app_send_msg(app_msg_t *msg);
    virtual bool app_work_dispatch(app_callback_t p_cback, uint16_t event, void *p_params, int param_len);
    virtual void app_work_dispatched(app_msg_t *msg);
    virtual void app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
    virtual void av_new_track();
    virtual void av_notify_evt_handler(uint8_t event_id, uint32_t event_parameter);
    
    virtual void init_nvs();
    virtual void getLastBda();
    virtual void setLastBda(esp_bd_addr_t bda, size_t size);
    // execute AVRC command
    virtual void executeAVRCCommand(int cmd);

};


#ifdef __cplusplus
}
#endif


#endif
