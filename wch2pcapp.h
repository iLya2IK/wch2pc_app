// WC HTTP2 Application Template Interface
//
// Copyright 2023 Medvedkov Ilya
//
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef WCH2PC_APP_H
#define WCH2PC_APP_H

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include <cJSON.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_event_loop.h>
#include <nvs_flash.h>

#include <http2_protoclient.h>

/* Build-in modes in state-machina */
#define WIFI_CONNECTED_BIT   BIT0
#define HOST_CONNECTED_BIT   BIT1
#define AUTHORIZED_BIT       BIT2
#define MODE_SETIME          BIT3
// autorization step. is device need to authorize
#define MODE_AUTH            BIT4
// get new messages from host step
#define MODE_RECIEVE_MSG     BIT5
// send new messages to host
#define MODE_SEND_MSG        BIT6

#define MODE_ALL             0xfffffe

/* Application configuration layer */

typedef void (* h2pca_on_notify) ();
typedef void (* h2pca_oninitconfig) (cJSON ** json_cfg);
typedef void (* h2pca_onreadnvs) (nvs_handle handle);
typedef void (* h2pca_ondisconnect) (int32_t reason);
typedef void (* h2pca_onerror) (int h2pcerrorcode);
typedef void (* h2pca_onauthorized) (const char * ssid);

typedef uint32_t h2pca_state;

typedef uint32_t h2pca_task_id;
/* Async event callback for task.
 * @param id            [input] ID of the task
 * @praram user_data    [input] user data, associated with the task
 */
typedef void (* h2pca_task_cb) (h2pca_task_id id, void * user_data);
/* Sync event callback for task.
 * @param id            [input] ID of the task
 * @param cur_state     [input] current global state for application
 * @praram user_data    [input] user data, associated with the task
 * @param restart_period  [output] set to non-zero if you need to restart the task
 *                         with the new period (in ms)
 */
typedef void (* h2pca_sync_task_cb) (h2pca_task_id id,
                                     h2pca_state cur_state,
                                     void * user_data,
                                     uint32_t * restart_period);

typedef struct h2pca_task_t
{
    /* TAG name of the task for logging */
    const char * TAG;

    /* Uniq ID value to send as parameter in callbacks */
    h2pca_task_id ID;

    /* the value of timeout period */
    uint32_t period;

    /* the requiried bitmask to fire the on_time event
     * if ((req_bitmask & h2pca_locked_GET_STATES()) != 0)
     *    on_time(ID);
     */
    h2pca_state req_bitmask;

    /* bit mask to apply to the global state value on async callback
     * h2pca_state |= bitmask
     * set to non-zero
     *  if you need to fire the associated sync event in the main loop
     * set to zero
     *  if you do not need to fire a sync event in the main loop
     */
    h2pca_state apply_bitmask;

    /* Callback. Operations on task (async) */
    h2pca_task_cb on_time;

    /* Callback. Sync event in main loop when h2pca_state&apply_bitmask != 0 */
    h2pca_sync_task_cb on_sync;

    /* user data, associated with task */
    void * user_data;

} h2pca_task;


typedef struct h2pca_tasks_t
{
    int32_t cnt;
    h2pca_task ** tasks;
} h2pca_tasks;

typedef struct h2pca_ble_config_t
{
    int count;
    const char ** ids;
    const uint8_t * cfgs;
} h2pca_ble_config;


typedef struct h2pca_config_t {
    const char * LOG_TAG;

    /* Mode value to send as param in h2pc_initialize */
    int h2pcmode;

    /* Change this field to add meta data for device */
    cJSON * device_meta_data;

    /* BLE config */
    h2pca_ble_config ble_cfg;

    /* Tasks to run with the application */
    h2pca_tasks tasks;

    uint32_t main_loop_period;
    uint32_t send_msgs_period;
    uint32_t recv_msgs_period;

    int32_t inmsgs_proceed_chunk;

    /* wifi callbacks */
    h2pca_on_notify         on_wifi_init;
    h2pca_on_notify         on_wifi_con;
    h2pca_on_notify         on_wifi_dis;

    /* wc protocol callbacks */
    h2pca_on_notify         on_connect;
    h2pca_onauthorized      on_auth;
    h2pc_cb_next_msg        on_next_inmsg;
    h2pca_onerror           on_error;
    h2pca_on_notify         on_disconnect;

    /* main loop callbacks */
    h2pca_onreadnvs         on_read_nvs;
    h2pca_oninitconfig      on_init_cfg;
    h2pca_on_notify         on_ble_cfg_start;
    h2pca_on_notify         on_ble_cfg_finished;
    h2pca_on_notify         on_begin_loop;
    h2pca_on_notify         on_begin_step;
    h2pca_on_notify         on_before_inmsgs;
    h2pca_on_notify         on_after_inmsgs;
    h2pca_on_notify         on_finish_step;
    h2pca_on_notify         on_finish_loop;

} h2pca_config;


typedef struct h2pca_status_t
{
    h2pca_config * cfg;

    char mac_str[13];
    char device_name[32];
    char device_char[9];

    /* FreeRTOS event group of client state */
    EventGroupHandle_t client_state;

    nvs_handle nvs_h;

    int wifi_connect_errors;
    int connect_errors;

    esp_timer_handle_t * sys_handles;
    esp_timer_handle_t * user_handles;
} h2pca_status;


/* Application tasks layer */

/* Init the new h2pc task
 * @param TAG [input] TAG name of the task for logging
 * @param ID  [input] Uniq ID value to send as parameter in callbacks
 * @param user_data [input] The user data, associated with the task
 * @param error  [output] if not null - here stored the last error
 *                  till initialization
 * @return The pointer to the initialized task or NULL if error
 */
h2pca_task * h2pca_init_task(const char * TAG, h2pca_task_id ID, void * user_data, esp_err_t * error);

/* Destroy the task. if the task attached to an task pool -
 * no need to remove it - this task pool be removed in
 * h2pca_done_task_pool method internaly
 * @param tsks [input] initilized task pool
 * @return the last error code
 *         ESP_ERR_INVALID_ARG - \a tsk param is NULL or malformed
 */
esp_err_t h2pca_done_task(h2pca_task * tsk);

/* Init the new h2pc task pool
 * @param error  [output] if not null - here stored the last error
 *                  till initialization
 * @return The pointer to the initialized task pool or NULL if error
 */
h2pca_tasks * h2pca_init_task_pool(esp_err_t * error);

/* Add new initialized task to the task pool
 * @param error  [output] if not null - here stored the last error
 *                  till initialization
 * @return the last error code
 *         ESP_ERR_INVALID_ARG - param(s) is NULL or malformed
 *         ESP_ERR_NO_MEM - not enought memory avaible
 */
esp_err_t h2pca_task_pool_add_task(h2pca_tasks * pool, h2pca_task * tsk);

/* Destroy the tasks pool. if the task pool attached to an app -
 * no need to remove it - this task pool will be removed in
 * h2pca_done method internaly
 * @param tsks [input] initilized task pool
 * @return the last error code
 *         ESP_ERR_INVALID_ARG - \a tsks param is NULL or malformed
 */
esp_err_t h2pca_done_task_pool(h2pca_tasks * tsks);

/* Application lifecircle layer */

/* Init fields in configuration structure with
 * default values
 * @param cfg  [input/output] given config to reset
 * @return the last error code
 *         ESP_ERR_INVALID_ARG - \a cfg param is NULL or malformed
 */
esp_err_t h2pca_init_cfg(h2pca_config * cfg);

/* Init the new h2pc application with the given config
 * @param cfg  [input] given config
 * @param error  [output] if not null - here stored the last error
 *                  till initialization
 * @return The status of initializes app or NULL if error
 */
h2pca_status * h2pca_init(h2pca_config * cfg, esp_err_t* error);

esp_err_t h2pca_ble_config_init(h2pca_ble_config * cfg, int count, const char ** cfg_field, uint8_t * cfg_id);
esp_err_t h2pca_ble_config_init_standard(h2pca_ble_config * cfg);

/* Create the application task and start
 * @param heap_sz  [input] the heap size for task
 */
void h2pca_start(uint32_t heap_sz);

/* Main loop. In case application task is created by user
 */
void h2pca_loop();

esp_err_t h2pca_done();

/* Get current application status
 * @return reference to app
 */
h2pca_status * h2pca_get_status();

// bit operations with state mask
/* thread-safe get states route */
h2pca_state h2pca_locked_GET_STATES();
/* thread-safe check state route */
bool h2pca_locked_CHK_STATE(h2pca_state astate);
/* thread-safe set state route */
void h2pca_locked_SET_STATE(h2pca_state astate);
/* thread-safe clear state route */
void h2pca_locked_CLR_STATE(h2pca_state astate);
/* thread-safe clear all states route */
void h2pca_locked_CLR_ALL_STATES();


#endif // WCH2PC_APP_H