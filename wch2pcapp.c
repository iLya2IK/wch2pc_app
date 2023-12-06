// WC HTTP2 Application Template Implementation
//
// Copyright 2023 Medvedkov Ilya
//
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "wch2pcapp.h"

#include <sys/time.h>
#include "lwip/apps/sntp.h"
#include "esp_wifi.h"
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"

#ifdef CONFIG_WC_USE_IO_STREAMS
#include <wcframe.h>
#endif
#include <wcprotocol.h>
#include <ble_config.h>
#include <errno.h>

/* wifi config */
#define APP_WIFI_SSID CONFIG_WIFI_SSID
#define APP_WIFI_PASS CONFIG_WIFI_PASSWORD

/* HTTP2 HOST config */
// The HTTP/2 server to connect to
#define HTTP2_SERVER_URI   CONFIG_SERVER_URI
// The user's name
#define HTTP2_SERVER_NAME  CONFIG_SERVER_NAME
// The user's password
#define HTTP2_SERVER_PASS  CONFIG_SERVER_PASS

#define DEVICE_CONFIG   "device_config"
#define MAIN_TASK_NAME  "main_task"

#define DEFAULT_HEAP_SIZE (1024 * 12)

#define SEND_MSG_TIMER_DELTA                    1000000
#define GET_MSG_TIMER_DELTA                     4000000
#define MAIN_TASK_LOOP_DELAY                    200
#define STD_MSGS_CHUNK_SZ                       16

#define MAX_SYS_TASKS                           3
#define SYS_TASK_SEND                           0
#define SYS_TASK_RECV                           1

static h2pca_status app = { 0 };

/* JSON-RPC device metadata */
/* device's write char to identify */
static const char * JSON_BLE_CHAR         =  "ble_char";

static void __set_error(esp_err_t * error, esp_err_t erv) {
    if (error != NULL)
        *error = erv;
}

h2pca_task * h2pca_init_task(const char * TAG, h2pca_task_id ID, void * user_data, esp_err_t * error) {
    h2pca_task * tsk = (h2pca_task *)malloc(sizeof(h2pca_task));

    if (tsk == NULL) {
        __set_error(error, ESP_ERR_NO_MEM);
        return NULL;
    }
    memset(tsk, 0, sizeof(h2pca_task));
    tsk->ID = ID;
    tsk->TAG = TAG;
    tsk->user_data = user_data;

    return tsk;
}

esp_err_t h2pca_done_task(h2pca_task * tsk) {
    if (tsk == NULL) return ESP_ERR_INVALID_ARG;

    free(tsk);

    return ESP_OK;
}

h2pca_tasks * h2pca_init_task_pool(esp_err_t * error) {
    h2pca_tasks * pool = (h2pca_tasks*)malloc(sizeof(h2pca_tasks));

    if (pool == NULL) {
        __set_error(error, ESP_ERR_NO_MEM);
        return NULL;
    }

    pool->cnt = 0;
    pool->tasks = NULL;

    return pool;
}

esp_err_t h2pca_task_pool_add_task(h2pca_tasks * pool, h2pca_task * tsk) {
    if (pool == NULL) return ESP_ERR_INVALID_ARG;
    if (tsk == NULL) return ESP_ERR_INVALID_ARG;

    if (pool->tasks == NULL) {
        pool->tasks = (h2pca_task **) malloc(sizeof(h2pca_tasks*));
    } else {
        pool->tasks = (h2pca_task **) realloc(pool->tasks, sizeof(h2pca_tasks*) * (pool->cnt+1));
    }

    if (pool->tasks == NULL) return ESP_ERR_NO_MEM;

    pool->tasks[pool->cnt] = tsk;
    pool->cnt++;

    return ESP_OK;
}

esp_err_t h2pca_done_task_pool(h2pca_tasks * tsks) {
    if (tsks == NULL) return ESP_ERR_INVALID_ARG;

    if (tsks->tasks != NULL) {
        for (int i = 0; i < tsks->cnt; ++i) {
            h2pca_done_task(tsks->tasks[i]);
        }
        free(tsks->tasks);
        tsks->tasks = NULL;
    }
    tsks->cnt = 0;

    return ESP_OK;
}

esp_err_t h2pca_init_cfg(h2pca_config * cfg) {
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;

    memset(cfg, 0x00, sizeof(h2pca_config));

    cfg->recv_msgs_period = GET_MSG_TIMER_DELTA;
    cfg->send_msgs_period = SEND_MSG_TIMER_DELTA;
    cfg->main_loop_period = MAIN_TASK_LOOP_DELAY;

    cfg->inmsgs_proceed_chunk = STD_MSGS_CHUNK_SZ;

    cfg->h2pcmode = H2PC_MODE_MESSAGING;

    h2pca_ble_config_init_standard(&(cfg->ble_cfg));

    return ESP_OK;
}

h2pca_status * h2pca_init(h2pca_config * cfg, esp_err_t* error) {
    if (cfg == NULL) {
        __set_error(error, ESP_ERR_INVALID_ARG);
        return NULL;
    }

    memset(&app, 0, sizeof(h2pca_status));

    app.cfg = cfg;
    app.client_state = xEventGroupCreate();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            __set_error(error, err);
            h2pca_done();
            return NULL;
        }
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        __set_error(error, err);
        h2pca_done();
        return NULL;
    }

    /* generate mac address and device metadata */
    uint8_t sta_mac[6];
    err = esp_efuse_mac_get_default(sta_mac);
    if (err != ESP_OK) {
        __set_error(error, err);
        h2pca_done();
        return NULL;
    }

    for (int i = 0; i < 6; i++) {
        app.mac_str[i<<1] = UPPER_XDIGITS[(sta_mac[i] >> 4) & 0x0f];
        app.mac_str[(i<<1) + 1] = UPPER_XDIGITS[(sta_mac[i] & 0x0f)];
    }

    app.mac_str[12] = 0;

    app.device_char[4] = UPPER_XDIGITS[(uint8_t)((CONFIG_WC_DEVICE_CHAR1_UUID >> 12) & 0x000f)];
    app.device_char[5] = UPPER_XDIGITS[(uint8_t)((CONFIG_WC_DEVICE_CHAR1_UUID >> 8) & 0x000f)];
    app.device_char[6] = UPPER_XDIGITS[(uint8_t)((CONFIG_WC_DEVICE_CHAR1_UUID >> 4) & 0x000f)];
    app.device_char[7] = UPPER_XDIGITS[(uint8_t)((CONFIG_WC_DEVICE_CHAR1_UUID) & 0x000f)];

    if (app.cfg->device_meta_data == NULL)
        app.cfg->device_meta_data = cJSON_CreateObject();
    cJSON_AddItemToObject(app.cfg->device_meta_data, JSON_BLE_CHAR, cJSON_CreateStringReference(app.device_char));

    return &app;
}

esp_err_t h2pca_ble_config_init(h2pca_ble_config * cfg, int count, const char ** cfg_field, uint8_t * cfg_id) {
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;


    cfg->count = count;
    cfg->ids = cfg_field;
    cfg->cfgs = cfg_id;

    return ESP_OK;
}

esp_err_t h2pca_ble_config_init_standard(h2pca_ble_config * cfg) {
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;

    cfg->count = get_ble_std_config_count();
    cfg->ids = get_ble_std_config_idstr();
    cfg->cfgs = get_ble_std_config_idkey();

    return ESP_OK;
}

#define EXEC_CB(cb, ...) if (app.cfg->cb != NULL) \
                                app.cfg->cb(__VA_ARGS__);



/* forward decrlarations */
void finalize_app();

static void set_time(void)
{
    struct timeval tv = {
        .tv_sec = 1509449941,
    };
    struct timezone tz;
    memset(&tz, 0, sizeof(tz));
    settimeofday(&tv, &tz);

    /* Start SNTP service */
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_init();
}

static void __consume_protocol_error() {
    int err = h2pc_get_last_error();

    if (err != REST_RESULT_OK)
        EXEC_CB(on_error, err);

    if (err == REST_ERR_NO_SUCH_SESSION) {
        h2pca_locked_CLR_ALL_STATES();
        h2pca_locked_SET_STATE(MODE_AUTH);
    }
}

/* disconnect from host. reset all states */
static void __disconnect_host() {
    if (h2pca_locked_CHK_STATE(HOST_CONNECTED_BIT))
        h2pc_disconnect_http2();
    else
        h2pc_reset_buffers();
    h2pca_locked_CLR_ALL_STATES();

    EXEC_CB(on_disconnect);
}

/* connect to host */
static void __connect_to_http2() {
    __disconnect_host();

    char * addr;
    if (WC_CFG_VALUES != NULL)
        addr = get_cfg_value(CFG_HOST_NAME);
    else
        addr = HTTP2_SERVER_URI;

    if (h2pc_connect_to_http2(addr)) {
        app.connect_errors = 0;
        h2pca_locked_SET_STATE(HOST_CONNECTED_BIT | MODE_AUTH);
    } else
        app.connect_errors++;
}

static void __send_authorize() {
    ESP_LOGI(app.cfg->LOG_TAG, "Trying to authorize");

    const char * _name;
    const char * _pwrd;
    const char * _device;

    if (WC_CFG_VALUES != NULL) {
        _name =  get_cfg_value(CFG_USER_NAME);
        _pwrd =  get_cfg_value(CFG_USER_PASSWORD);
        _device = get_cfg_value(CFG_DEVICE_NAME);
    }
    else {
        _name =  HTTP2_SERVER_NAME;
        _pwrd =  HTTP2_SERVER_PASS;
        _device =  app.mac_str;
    }

    int res = h2pc_req_authorize_sync(_name, _pwrd, _device, app.cfg->device_meta_data, false);

    if (res == ESP_OK) {
        h2pca_locked_CLR_STATE(MODE_AUTH);
        h2pca_locked_SET_STATE(AUTHORIZED_BIT | MODE_RECIEVE_MSG);
        strcpy(app.device_name, _device);
        ESP_LOGI(app.cfg->LOG_TAG, "hash=%s", h2pc_get_sid());
    }
    else
    if (res == H2PC_ERR_PROTOCOL)
        __consume_protocol_error();
    else
        __disconnect_host();
}


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(app.cfg->LOG_TAG, "SYSTEM_EVENT_STA_START");
        ESP_ERROR_CHECK(esp_wifi_connect());
        EXEC_CB(on_wifi_init);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(app.cfg->LOG_TAG, "SYSTEM_EVENT_STA_GOT_IP");
        ESP_LOGI(app.cfg->LOG_TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        h2pca_locked_SET_STATE(WIFI_CONNECTED_BIT);
        h2pca_locked_SET_STATE(MODE_SETIME);
        app.wifi_connect_errors = 0;

        EXEC_CB(on_wifi_con);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(app.cfg->LOG_TAG, "SYSTEM_EVENT_STA_DISCONNECTED");

        app.wifi_connect_errors++;

        sntp_stop();

        EXEC_CB(on_wifi_dis);

        if (h2pca_locked_CHK_STATE(HOST_CONNECTED_BIT)) h2pc_disconnect_http2();
        h2pca_locked_CLR_ALL_STATES();

        h2pca_locked_CLR_STATE(WIFI_CONNECTED_BIT);
        h2pc_reset_buffers();
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi()
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config;
    memset(&wifi_config, 0x00, sizeof(wifi_config_t));

    char * value = get_cfg_value(CFG_SSID_NAME);
    if (value != NULL) {
        strcpy((char *) &(wifi_config.sta.ssid[0]), value);
        ESP_LOGD(app.cfg->LOG_TAG, "SSID setted from json config");
    } else {
        strcpy((char *) &(wifi_config.sta.ssid[0]), APP_WIFI_SSID);
        ESP_LOGD(app.cfg->LOG_TAG, "SSID setted from flash config");
    }
    value = get_cfg_value(CFG_SSID_PASSWORD);
    if (value != NULL) {
        strcpy((char *) &(wifi_config.sta.password[0]), value);
        ESP_LOGD(app.cfg->LOG_TAG, "Password setted from json config");
    } else {
        strcpy((char *) &(wifi_config.sta.password[0]), APP_WIFI_PASS);
        ESP_LOGD(app.cfg->LOG_TAG, "Password setted from flash config");
    }

    ESP_LOGI(app.cfg->LOG_TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}


void __msgs_get_cb(void* arg)
{
    ESP_LOGD(app.cfg->LOG_TAG, "Recieve msgs fired");
    bool isempty = h2pc_im_locked_waiting();
    if (isempty) {
        if (h2pca_locked_CHK_STATE(HOST_CONNECTED_BIT))
            h2pca_locked_SET_STATE(MODE_RECIEVE_MSG);
    }
}

void __msgs_send_cb(void* arg)
{
    ESP_LOGD(app.cfg->LOG_TAG, "Send msgs fired");

    bool isnempty = h2pc_om_locked_waiting();
    if (isnempty) {
        if (h2pca_locked_CHK_STATE(HOST_CONNECTED_BIT))
            h2pca_locked_SET_STATE(MODE_SEND_MSG);
    }
}

void __user_task_cb(void* arg)
{
    h2pca_task * tsk = (h2pca_task *)arg;
    ESP_LOGD(tsk->TAG, "User task fired");

    if (h2pca_locked_CHK_STATE(tsk->req_bitmask)) {

        if (tsk->on_time)
            tsk->on_time(tsk->ID, tsk->user_data);
        else
            h2pca_locked_SET_STATE(tsk->apply_bitmask);

    }
}

static void __check_h2pc_errors() {
    if (h2pca_locked_CHK_STATE(WIFI_CONNECTED_BIT|HOST_CONNECTED_BIT)) {
        if (h2pc_get_connected()) {
            if (h2pc_get_protocol_errors_cnt() > 0) {
                int err = h2pc_get_last_error();

                if (err != REST_RESULT_OK)
                    EXEC_CB(on_error, err);

                if (err == REST_ERR_NO_SUCH_SESSION) {
                    h2pca_locked_CLR_STATE(AUTHORIZED_BIT);
                    h2pca_locked_SET_STATE(MODE_AUTH);
                } else
                    __disconnect_host();
            }
        } else {
            __disconnect_host();
        }
    }
}

bool __std_on_incoming_msg(const cJSON * src, const cJSON * kind, const cJSON * iparams, const cJSON * msg_id) {
    // do nothing
    return true;
}

static void __recieve_msgs() {
    int res = h2pc_req_get_msgs_sync();
    if (res == ESP_OK)
        h2pca_locked_CLR_STATE(MODE_RECIEVE_MSG);
}

static void __send_msgs() {
    int res = h2pc_req_send_msgs_sync();
    if (res == ESP_OK)
        h2pca_locked_CLR_STATE(MODE_SEND_MSG);
}

void __finalize_app()
{
    __disconnect_host();

    h2pc_finalize();
}

static void __main_task(void *args)
{
    esp_err_t err;
    cJSON * loc_cfg = NULL;

    err = nvs_open(DEVICE_CONFIG, NVS_READWRITE, &(app.nvs_h));
    if (err == ESP_OK) {
        size_t required_size;
        err = nvs_get_str(app.nvs_h, DEVICE_CONFIG, NULL, &required_size);
        if (err == ESP_OK) {
            char * cfg_str = malloc(required_size);
            if (cfg_str == NULL)
                ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
            nvs_get_str(app.nvs_h, DEVICE_CONFIG, cfg_str, &required_size);
            loc_cfg = cJSON_Parse(cfg_str);
            free(cfg_str);
            ESP_LOGD(DEVICE_CONFIG, "JSON cfg founded");
            #ifdef LOG_DEBUG
            esp_log_buffer_char(DEVICE_CONFIG, cfg_str, strlen(cfg_str));
            #endif
        }
        EXEC_CB(on_read_nvs, app.nvs_h);
    }

    if (loc_cfg == NULL) {
        loc_cfg = cJSON_CreateArray();
        if (loc_cfg == NULL)
            ESP_ERROR_CHECK(ESP_ERR_NO_MEM);

        cJSON * cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_DEVICE_NAME), app.mac_str);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_USER_NAME), HTTP2_SERVER_NAME);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_USER_PASSWORD), HTTP2_SERVER_PASS);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_HOST_NAME), HTTP2_SERVER_URI);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_SSID_NAME), APP_WIFI_SSID);
        cJSON_AddItemToArray(loc_cfg, cfg_item);
        cfg_item = cJSON_CreateObject();
        cJSON_AddStringToObject(cfg_item, get_cfg_id(CFG_SSID_PASSWORD), APP_WIFI_PASS);
        cJSON_AddItemToArray(loc_cfg, cfg_item);

        EXEC_CB(on_init_cfg, &loc_cfg);
    }

    if (loc_cfg == NULL) {
        ESP_LOGE(app.cfg->LOG_TAG, "No config found");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
    }

    set_ble_config_params(app.cfg->ble_cfg.count, app.cfg->ble_cfg.ids, app.cfg->ble_cfg.cfgs);

    EXEC_CB(on_ble_cfg_start);

    error_t ret = initialize_ble(loc_cfg);
    cJSON_Delete(loc_cfg);
    if (ret == OK) {
        start_ble_config_round();
        while ( ble_config_proceed() ) {
            vTaskDelay(1000);
        }
        stop_ble_config_round();

        if (WC_CFG_VALUES != NULL) {
            char * cfg_str = cJSON_PrintUnformatted(WC_CFG_VALUES);
            nvs_set_str(app.nvs_h, DEVICE_CONFIG, cfg_str);
            nvs_commit(app.nvs_h);

            #ifdef LOG_DEBUG
            esp_log_buffer_char(JSON_CFG, cfg_str, strlen(cfg_str));
            #endif

            cJSON_free(cfg_str);
        }
    }
    nvs_close(app.nvs_h);

    EXEC_CB(on_ble_cfg_finished);

    ESP_ERROR_CHECK(h2pc_initialize(app.cfg->h2pcmode));
    initialise_wifi();

    /* init system timers */
    esp_timer_create_args_t timer_args;

    app.sys_handles = malloc(sizeof(esp_timer_handle_t) * MAX_SYS_TASKS);

    timer_args.callback = &__msgs_get_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &(app.sys_handles[SYS_TASK_RECV])));

    timer_args.callback = &__msgs_send_cb;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &(app.sys_handles[SYS_TASK_SEND])));

    /* start system timers */
    esp_timer_start_periodic(app.sys_handles[SYS_TASK_RECV], app.cfg->recv_msgs_period);
    esp_timer_start_periodic(app.sys_handles[SYS_TASK_SEND], app.cfg->send_msgs_period);

    /* init user timers */

    int user_tasks_cnt = app.cfg->tasks.cnt;

    if (user_tasks_cnt > 0) {

        app.user_handles = (esp_timer_handle_t*) malloc(sizeof(esp_timer_handle_t) * user_tasks_cnt);

        if (app.user_handles == NULL)
            ESP_ERROR_CHECK(ESP_ERR_NO_MEM);

        for (int i = 0; i < user_tasks_cnt; ++i) {
            h2pca_task * tsk = app.cfg->tasks.tasks[i];

            timer_args.callback = &__user_task_cb;
            timer_args.arg = tsk;

            ESP_ERROR_CHECK(esp_timer_create(&timer_args, &(app.user_handles[i])));

            esp_timer_start_periodic(app.user_handles[i], tsk->period);
        }
    }


    EXEC_CB(on_begin_loop);

    int connectDelay = 0;
    int wifiDisconnectedTime = 0;

    while (1)
    {
        // ESP_LOGI(WC_TAG, "New step. states: %d", locked_GET_STATES());

        EXEC_CB(on_begin_step);

        if (h2pca_locked_CHK_STATE(WIFI_CONNECTED_BIT)) {

            wifiDisconnectedTime = 0;

            if (h2pca_locked_CHK_STATE(MODE_SETIME)) {
                /* Set current time: proper system time is required for TLS based
                 * certificate verification.
                 */
                set_time();
                h2pca_locked_CLR_STATE(MODE_SETIME);
            }


            if (h2pca_locked_CHK_STATE(HOST_CONNECTED_BIT)) {

                /* authorize the device on server */
                if (h2pca_locked_CHK_STATE(MODE_AUTH)) {
                    __send_authorize();
                    __check_h2pc_errors();
                }
                /* gathering incoming msgs from server */
                if (h2pca_locked_CHK_STATE(MODE_RECIEVE_MSG)) {
                    esp_timer_stop(app.sys_handles[SYS_TASK_RECV]);
                    __recieve_msgs();
                    __check_h2pc_errors();
                    esp_timer_start_periodic(app.sys_handles[SYS_TASK_RECV], app.cfg->recv_msgs_period);
                }
                /* proceed incoming messages */
                EXEC_CB(on_before_inmsgs);
                if (app.cfg->on_next_inmsg)
                    h2pc_im_proceed(app.cfg->on_next_inmsg, app.cfg->inmsgs_proceed_chunk);
                else
                    h2pc_im_proceed(&__std_on_incoming_msg, app.cfg->inmsgs_proceed_chunk);
                EXEC_CB(on_after_inmsgs);

                /* send outgoing messages */
                if (h2pca_locked_CHK_STATE(MODE_SEND_MSG)) {
                    esp_timer_stop(app.sys_handles[SYS_TASK_SEND]);
                    __send_msgs();
                    __check_h2pc_errors();
                    esp_timer_start_periodic(app.sys_handles[SYS_TASK_SEND], app.cfg->send_msgs_period);
                }

            } else {

                connectDelay -= app.cfg->main_loop_period;

                if (connectDelay <= 0) {

                    __connect_to_http2();

                    if (app.connect_errors) {
                        switch (app.connect_errors)
                        {
                        case 11:
                            connectDelay = 300 * configTICK_RATE_HZ; // 5 minutes
                            break;
                        case 12:
                            ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE); // drop to deep reload if no connection to host over 15 minutes
                            break;
                        default:
                            connectDelay = app.connect_errors * 10 * configTICK_RATE_HZ;
                            break;
                        }
                    } else
                        connectDelay = 0;

                }

            }

        } else {
            wifiDisconnectedTime += app.cfg->main_loop_period;

            if (wifiDisconnectedTime > 900000)
                ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE); // drop to deep reload if no connection to AP over 15 minutes

            connectDelay -= app.cfg->main_loop_period;

            if ((connectDelay <= 0) && (app.wifi_connect_errors)) {

                app.wifi_connect_errors = 0;
                ESP_ERROR_CHECK(esp_wifi_connect());

                connectDelay = 30 * configTICK_RATE_HZ; // 30 sec timeout between two wifi connection attempts
            }
        }

        for (int i = 0; i < user_tasks_cnt; ++i) {
            h2pca_task * tsk = app.cfg->tasks.tasks[i];

            if (h2pca_locked_CHK_STATE((tsk->apply_bitmask | tsk->req_bitmask))) {
                uint32_t p = tsk->period;

                if (tsk->on_sync) {
                    tsk->on_sync(tsk->ID, h2pca_locked_GET_STATES(), tsk->user_data, &p);
                    if (p != tsk->period) {
                        esp_timer_stop(app.user_handles[i]);
                        tsk->period = p;
                        esp_timer_start_periodic(app.user_handles[i], p);
                    }
                }

            }
        }

        EXEC_CB(on_finish_step);

        vTaskDelay(app.cfg->main_loop_period);
    }

    EXEC_CB(on_finish_loop);

    __finalize_app();

    vTaskDelete(NULL);
}

void h2pca_start(uint32_t heap_sz) {
    if (heap_sz == 0) {
        heap_sz = DEFAULT_HEAP_SIZE;
    }

    xTaskCreate(&__main_task, MAIN_TASK_NAME, heap_sz, NULL, 5, NULL);
}

void h2pca_loop() {
    __main_task(NULL);
}

esp_err_t h2pca_done() {
    for (int i = 0; i < MAX_SYS_TASKS; ++i) {
        if (app.sys_handles[i] != 0) {
            esp_timer_stop(app.sys_handles[i]);
            esp_timer_delete(app.sys_handles[i]);
        }
    }
    for (int i = 0; i < app.cfg->tasks.cnt; ++i) {
        if (app.user_handles[i] != 0) {
            esp_timer_stop(app.user_handles[i]);
            esp_timer_delete(app.user_handles[i]);
        }
    }

    if (app.sys_handles != NULL) free(app.sys_handles);
    if (app.user_handles != NULL) free(app.user_handles);
    h2pca_done_task_pool(&(app.cfg->tasks));

    app.sys_handles = NULL;
    app.user_handles = NULL;

    if (app.cfg->device_meta_data != NULL) free(app.cfg->device_meta_data);

    return ESP_OK;
}

h2pca_status * h2pca_get_status() {
    return &app;
}

h2pca_state h2pca_locked_GET_STATES() {
    return xEventGroupGetBits(app.client_state);
}

bool h2pca_locked_CHK_STATE(h2pca_state astate) {
    bool val = ((h2pca_locked_GET_STATES() & astate) == astate);
    return val;
}

void h2pca_locked_SET_STATE(h2pca_state astate) {
    xEventGroupClearBits(app.client_state, astate);
}

void h2pca_locked_CLR_STATE(h2pca_state astate) {
    xEventGroupSetBits(app.client_state, astate);
}

void h2pca_locked_CLR_ALL_STATES() {
    xEventGroupClearBits(app.client_state, MODE_ALL);
}