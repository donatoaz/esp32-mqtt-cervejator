/* ************************************************************************* *
 * esp32-mqtt-client-c
 * 
 * A simple MQTT client running on an ESP-WROOM-32 modules. Connects to the
 * configured wireless network and communicates with the configured MQTT
 * broker. 
 * 
 * Author:  Jesse Braham <jesse@beta7.io>
 * Date:    February, 2017
 * Version: 2
 * 
 * Changelog:
 * ==========
 * v2:
 *  - an update to esp-idf seemingly broke things. non-volatile storage is now
 *    explicitly initialized on startup.
 * 
 * ************************************************************************* */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_mqtt.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "ds18b20.h"

// Define the GPIO pin (will be used shortly) and the wireless network's SSID
// and passphrase. To configure these values, run 'make menuconfig'.
#define SENSOR_GPIO CONFIG_SENSOR_GPIO
//#define SENSOR_WRITE_KEY CONFIG_SENSOR_WRITE_KEY
#define SENSOR_WRITE_KEY "sensor/RgQCqJStIMDwd2SQ_Onktg"
#define ACTUATOR_WRITE_KEY "actuator/USy53ARrqXqu8cr-jcYvAQ"
//#define ACTUATOR_GPIO CONFIG_ACTUATOR_GPIO
#define ACTUATOR_GPIO 5
//#define WIFI_SSID  CONFIG_WIFI_SSID
//#define WIFI_PASS  CONFIG_WIFI_PASS
#define WIFI_SSID  "FactaTI"
#define WIFI_PASS  "F@ct@_T!"

// Define the MQTT Broker's hostname, port, username and passphrase. To
// configure these values, run 'make menuconfig'.
// #define MQTT_HOST CONFIG_MQTT_BROKER
#define MQTT_HOST "35.231.216.221" 
#define MQTT_PORT CONFIG_MQTT_PORT
#define MQTT_USER CONFIG_MQTT_USER
#define MQTT_PASS CONFIG_MQTT_PASS


static EventGroupHandle_t wifi_event_group = NULL;
static TaskHandle_t task = NULL;

const int CONNECTED_BIT = BIT0;

static const char* TAG = "MQTT-ESP32";


/* ************************************************************************* *
 * Initialize the TCP/IP stack and set the wi-fi default configuration and
 * operating mode.
 * ************************************************************************* */
void
initialize_wifi(void)
{
    // Disable wifi driver logging
    esp_log_level_set("wifi", ESP_LOG_NONE);

    tcpip_adapter_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

/* ************************************************************************* *
 * Connect to the wireless network defined via menuconfig, using the supplied
 * passphrase.
 * ************************************************************************* */
void
wifi_connect(void)
{
    wifi_config_t cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK( esp_wifi_disconnect() );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg) );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

/* ************************************************************************* *
 * Initialize non-volatile storage. If there are no pages free, erase the
 * contents of the flash memory, and attempt to initialize the storage again.
 * ************************************************************************* */
void
initialize_nvs(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );
}

/* ************************************************************************* *
 * Configure the IOMUX register for pad BLINK_GPIO. Some pads are muxed to
 * GPIO on reset, but some default to other functions and will need to be
 * switched to GPIO if selected. Set the GPIO direction as 'Output', and set
 * the initial state of the pin to HIGH (1), as our LED is driven in an
 * active-low configuration.
 * ************************************************************************* */
void
initialize_gpio(void)
{
  // setting actuator output
  gpio_pad_select_gpio(ACTUATOR_GPIO);
  gpio_set_direction(ACTUATOR_GPIO, GPIO_MODE_OUTPUT);

  ds18b20_init(14);
}


/* ************************************************************************* *
 * Main event loop handler. On system start, attempt to connect to the
 * defined wireless network. If an IP address is acquired, set the 'Connected'
 * bit for the Event Group, and start the MQTT client. On disconnect, stop the
 * MQTT client and reset the 'Connected' bit.
 * ************************************************************************* */
static
esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "System started, going to connect to wifi.");
        wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "Got an IP address, going to connect to MQTT broker. %s : %d, user: %s, pass: %s", 
            MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS);
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        esp_mqtt_start(MQTT_HOST, MQTT_PORT, "esp-mqtt", "", "");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "WiFi connection lost, stopping MQTTT and going to attempt a reconnect.");
        esp_mqtt_stop();
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }

    return ESP_OK;
}


/* ************************************************************************* *
 * The MQTT processing task. In an infinite loop, pushlish the defined payload
 * to the defined channel, and wait 1000ms (1 second).
 * ************************************************************************* */
static void
process(void *p)
{
    char payload[10] = {0};
    float value = 0;
    
    for (;;) {
        value = ds18b20_get_temp();
        int ret = snprintf(payload, sizeof payload, "%f", value);

        if (ret < 0) {
            ESP_LOGE(TAG, "Conversion of float to char failed!, %f", value);
            return; // EXIT_FAILURE;
        }
        if (ret >= sizeof payload) {
            ESP_LOGE(TAG, "Conversion of float to char failed because buffer is too small!, %f", value);
            return;
        }

        esp_mqtt_publish(SENSOR_WRITE_KEY, (void *)payload, 10, 0, false);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

/* ************************************************************************* *
 * The MQTT Status callback function. If the status is 'Connected', subscribe
 * to the defined MQTT channel, and create a task to run the 'process' function
 * implemented above. On disconnect, destroy the task.
 * ************************************************************************* */
static void
mqtt_status_cb(esp_mqtt_status_t status)
{
    switch (status)
    {
    case ESP_MQTT_STATUS_CONNECTED:
        ESP_LOGI(TAG, "Cool, we got a CONNECT ACK from broker!");
        esp_mqtt_subscribe(ACTUATOR_WRITE_KEY, 0);
        xTaskCreatePinnedToCore(process, "process", 8192, NULL, 10, &task, 1);
        break;
    case ESP_MQTT_STATUS_DISCONNECTED:
        ESP_LOGI(TAG, "Shoot, MQTT disconnected! Stopping publishing task.");
        vTaskDelete(task);
        break;
    }
}

/* ************************************************************************* *
 * The MQTT Message callback function. When a message is received, print a
 * message containing the topic, payload, and the length of the payload to the
 * terminal.
 * ************************************************************************* */
static void
mqtt_message_cb(const char *topic, uint8_t *payload, size_t len)
{
    printf("incoming\t%s:%s (%d)\n", topic, payload, (int)len);
    if (strcmp((char *) payload,"on") == 0) {
      ESP_LOGI(TAG, "Setting pin %d to high", ACTUATOR_GPIO);
      gpio_set_level(ACTUATOR_GPIO, 1);
    } else if (strcmp((char *) payload, "off") == 0) {
      ESP_LOGI(TAG, "Setting pin %d to low", ACTUATOR_GPIO);
      gpio_set_level(ACTUATOR_GPIO, 0);
    } else {
      ESP_LOGI(TAG, "Received an unknown actuator command");
    } 

}


/* ************************************************************************* *
 * Main entry point. Start the event loop, assigning the above create Event
 * Handler. Create the wi-fi's Event Group prior to initializing the wi-fi
 * radio. Finally, initialize the MQTT client, specifying the status and
 * message callback functions.
 * ************************************************************************* */
void
app_main(void)
{
    initialize_nvs();
    initialize_gpio();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_event_group = xEventGroupCreate();
    initialize_wifi();

    esp_mqtt_init(mqtt_status_cb, mqtt_message_cb, 256, 2000);
}
