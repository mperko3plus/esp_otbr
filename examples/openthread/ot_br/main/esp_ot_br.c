/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Border Router Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_openthread.h"
#include "esp_openthread_border_router.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_cli_extension.h"
#include "esp_ot_config.h"
#include "esp_ot_wifi_cmd.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "openthread/error.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"

#include "esp_system.h"
#include "cJSON.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#define EXAMPLE_ESP_WIFI_SSID "Mperko"
#define EXAMPLE_ESP_WIFI_PASS "mperko12"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define REMOTE_IP_ADDR "192.168.53.231"
#define REMOTE_PORT 5002
#define TAG "esp_ot_br"
static int s_retry_num = 0;
uint8_t socket_fd;
static EventGroupHandle_t s_wifi_event_group;

typedef enum
{
    EVENT_ON,
    EVENT_OFF,
    READ_LED,
    EVENT_UNKNOWN,
    ADD_DEVICE,
    SCAN
} EventType;


EventType get_event_type(const char *event)
{
    if (strcmp(event, "on") == 0)
    {
        return EVENT_ON;
    }
    else if (strcmp(event, "off") == 0)
    {
        return EVENT_OFF;
    }
    else if (strcmp(event, "scan") == 0)
    {
        return SCAN;
    }
    else if (strcmp(event, "get") == 0)
    {
        return READ_LED;
    }
    else if (strcmp(event, "add") == 0)
    {
        return ADD_DEVICE;
    }
    else
    {
        return EVENT_UNKNOWN;
    }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif != NULL)
    {
        ESP_LOGI(TAG, "WiFi already initialized by OTBR.");
        return;
    }

    ESP_LOGI(TAG, "Initializing WiFi...");
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}



/* Function to connect to a remote server */
/* Function to connect to a remote server */
int connect_to_server(const char *ip, int port)
{
    struct sockaddr_in dest_addr;
    int socket_fd;

    socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (socket_fd == -1)
    {
        ESP_LOGE(TAG, "Failed to create socket");
        return -1;
    }

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &dest_addr.sin_addr);

    if (connect(socket_fd, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0)
    {
        ESP_LOGE(TAG, "Socket connect failed");
        close(socket_fd);
        return -1;
    }

    ESP_LOGI(TAG, "Successfully connected to server");
    return socket_fd;
}

/* Function to send a JSON message over a connected socket */
void send_json_init_message(int socket_fd, const char *type, const char *content)
{
    // Create a JSON object
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to create cJSON object");
        return;
    }

    // Add data to the JSON object
    cJSON_AddStringToObject(root, "event", type);
    cJSON_AddStringToObject(root, "serial", content);

    // Convert JSON object to string
    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string == NULL)
    {
        ESP_LOGE(TAG, "Failed to print cJSON object to string");
        cJSON_Delete(root);
        return;
    }

    // Send JSON string to the server
    if (send(socket_fd, json_string, strlen(json_string), 0) < 0)
    {
        ESP_LOGE(TAG, "Failed to send JSON message");
    }
    else
    {
        ESP_LOGI(TAG, "JSON message sent successfully: %s", json_string);
    }

    // Free JSON object and string
    cJSON_Delete(root);
    free(json_string);
}

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif



#if CONFIG_EXTERNAL_COEX_ENABLE
static void ot_br_external_coexist_init(void)
{
    esp_external_coex_gpio_set_t gpio_pin = ESP_OPENTHREAD_DEFAULT_EXTERNAL_COEX_CONFIG();
    esp_external_coex_set_work_mode(EXTERNAL_COEX_LEADER_ROLE);
    ESP_ERROR_CHECK(esp_enable_extern_coex_gpio_pin(CONFIG_EXTERNAL_COEX_WIRE_TYPE, gpio_pin));
}
#endif /* CONFIG_EXTERNAL_COEX_ENABLE */

static void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *openthread_netif = esp_netif_new(&cfg);
    assert(openthread_netif != NULL);

    // Initialize the OpenThread stack
    ESP_ERROR_CHECK(esp_openthread_init(&config));

    // Initialize border routing features
    esp_openthread_lock_acquire(portMAX_DELAY);
#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
    ESP_ERROR_CHECK(esp_openthread_state_indicator_init(esp_openthread_get_instance()));
#endif
    ESP_ERROR_CHECK(esp_netif_attach(openthread_netif, esp_openthread_netif_glue_init(&config)));

    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
    esp_openthread_cli_init();

#if CONFIG_OPENTHREAD_BR_AUTO_START
    ESP_ERROR_CHECK(esp_openthread_border_router_init());
    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
#endif // CONFIG_OPENTHREAD_BR_AUTO_START

    esp_cli_custom_command_init();
    esp_openthread_lock_release();

    // Run the main loop
    esp_openthread_cli_create_task();
    esp_openthread_launch_mainloop();

    // Clean up
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);
    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}

void receive_json_messages(void *param)
{
    int socket_fd = (int)param;
    char buffer[1024];
    int len;

    while (1)
    {
        memset(buffer, 0, sizeof(buffer));
        len = recv(socket_fd, buffer, sizeof(buffer) - 1, 0);

        if (len < 0)
        {
            ESP_LOGE(TAG, "Failed to receive message");
            break;
        }
        else if (len == 0)
        {
            ESP_LOGI(TAG, "Connection closed by server");
            break;
        }

        ESP_LOGI(TAG, "Received message: %s", buffer);

        // Parse the JSON message
        cJSON *json = cJSON_Parse(buffer);
        if (json == NULL)
        {
            ESP_LOGE(TAG, "Failed to parse JSON message");
            continue;
        }

        // Handle events based on the event type
        cJSON *event = cJSON_GetObjectItem(json, "event");
        cJSON *cluster = cJSON_GetObjectItem(json, "cluster");
      

        if (cJSON_IsString(event) && cJSON_IsString(cluster))
        {
            EventType event_type = get_event_type(event->valuestring);
            switch (event_type)
            {

            default:
                ESP_LOGI(TAG, "Received unrecognized event: %s", event->valuestring);
                break;
            }
        }
        else if (cJSON_IsString(event))
        {
            EventType event_type = get_event_type(event->valuestring);
            switch (event_type)
            {

            default:
                ESP_LOGI(TAG, "Received unrecognized event: %s", event->valuestring);
                break;
            }
        }
        else
        {
            ESP_LOGE(TAG, "Invalid JSON format or missing required fields");
        }

        cJSON_Delete(json);
    }

    // Close the socket when done
    close(socket_fd);
    vTaskDelete(NULL);
}


void app_main(void)
{
    // Used eventfds:
    // * netif
    // * task queue
    // * border router
    esp_vfs_eventfd_config_t eventfd_config = {
#if CONFIG_OPENTHREAD_RADIO_NATIVE || CONFIG_OPENTHREAD_RADIO_SPINEL_SPI
        // * radio driver (A native radio device needs a eventfd for radio driver.)
        // * SpiSpinelInterface (The Spi Spinel Interface needs a eventfd.)
        // The above will not exist at the same time.
        .max_fds = 4,
#else
        .max_fds = 3,
#endif
    };
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_EXAMPLE_CONNECT_WIFI
#if CONFIG_OPENTHREAD_BR_AUTO_START
    ESP_ERROR_CHECK(example_connect());
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE && CONFIG_OPENTHREAD_RADIO_NATIVE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_coex_wifi_i154_enable());
#else
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if CONFIG_EXTERNAL_COEX_ENABLE
    ot_br_external_coexist_init();
#endif // CONFIG_EXTERNAL_COEX_ENABLE

#endif
    esp_openthread_set_backbone_netif(get_example_netif());
#else
    esp_ot_wifi_netif_init();
    esp_openthread_set_backbone_netif(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"));
#endif // CONFIG_OPENTHREAD_BR_AUTO_START
#elif CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(example_connect());
    esp_openthread_set_backbone_netif(get_example_netif());
#else
    ESP_LOGE(TAG, "ESP-Openthread has not set backbone netif");
#endif // CONFIG_EXAMPLE_CONNECT_WIFI

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp-ot-br"));
    xTaskCreate(ot_task_worker, "ot_br_main", 20480, xTaskGetCurrentTaskHandle(), 5, NULL);

    wifi_init_sta();

    // socket_fd = connect_to_server(REMOTE_IP_ADDR, REMOTE_PORT);
    // if (socket_fd != -1)
    // {
    //     // Send a JSON message
    //     send_json_init_message(socket_fd, "init", "213124");
    //     // Close the socket after sending the message
    //     // xTaskCreate(&receive_json_messages, "receive_json_messages", 4096, (void *)socket_fd, 5, NULL);
    // }
}
