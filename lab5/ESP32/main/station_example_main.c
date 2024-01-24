/* WiFi station Example

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

//dht11
#include "dht11.h"
//oled
#include "ssd1306.h"
#include "font8x8_basic.h"
//thư viện cần để dùng mqtt
#include "mqtt_client.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
uint32_t MQTT_CONNEECTED = 0;   //cờ MQTT, nếu thành công thì giá trị 1
int humi = 0;   // HUMIDITY VARIABLE
int temp = 0;	//CONVERTED TEMPERATURE VARIABLE
int soil = 0;	// SOIL MOISTURE VARIABLE
char humi_text[15];	//CHAR ARRAY STORING HUMIDITY TO DISPLAY
char temp_text[15];	//CHAR ARRAY STORING TEMPERATURE TO DISPLAY
char data_topic[5];
SSD1306_t dev;
/* The examples use WiFi configuration that you can set via project configuration menu

 If you'd rather not, just change the below entries to strings with
 the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
 */
#define EXAMPLE_ESP_WIFI_SSID      "Pea_0010"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10 // cố gắng connect 10 lần

#define tag "SSD1306"
//define tên của topic
#define MQTT_TOPIC "/test"
#define TEMP_TOPIC "Temp"
#define HUMI_TOPIC "Humi"
//URL của mqtt broker
#define CONFIG_BROKER_URL "mqtt://mqtt.flespi.io"

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
void read_sensor_task();
void oled_task();
void init_oled();
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;   //biến static retry count

static void event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data) {
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
		ESP_LOGI(TAG, "Trying to connect with Wi-Fi\n");
	} else if (event_base == WIFI_EVENT
			&& event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG, "connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

void wifi_init_sta(void) {
	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(
			esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
	ESP_ERROR_CHECK(
			esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

	wifi_config_t wifi_config = { .sta = { .ssid = EXAMPLE_ESP_WIFI_SSID,
			.password = EXAMPLE_ESP_WIFI_PASS,
			/* Setting a password implies station will connect to all security modes including WEP/WPA.
			 * However these modes are deprecated and not advisable to be used. Incase your Access point
			 * doesn't support WPA2, these mode can be enabled by commenting below line */
			.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
			.sae_pwe_h2e = WPA3_SAE_PWE_BOTH, }, };
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(TAG, "wifi_init_sta finished.");

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
	 * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
	WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
	pdFALSE,
	pdFALSE,
	portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
	 * happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
				EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
				EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
	}

	/* The event will not be processed after unregister */
	ESP_ERROR_CHECK(
			esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
					instance_got_ip));
	ESP_ERROR_CHECK(
			esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
	vEventGroupDelete(s_wifi_event_group);
}

//hàm xử lí của mqtt
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
		int32_t event_id, void *event_data) {
	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base,
			event_id);
	esp_mqtt_event_handle_t event = event_data;
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	switch ((esp_mqtt_event_id_t) event_id) {
	case MQTT_EVENT_CONNECTED:
		MQTT_CONNEECTED = 1;
		ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
		msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC, 0);
		//esp_mqtt_client_subscribe(client, TEMP_TOPIC, 0);
		ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

		break;
	case MQTT_EVENT_DISCONNECTED:
		MQTT_CONNEECTED = 0;
		ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
		break;

	case MQTT_EVENT_SUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
//        printf("TOPIC = %.*s\r\n", event->topic_len, event->topic);
//        printf("CURRDATA = %.*s\r\n", event->data_len, event->data);
		break;
	case MQTT_EVENT_UNSUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_PUBLISHED:
		ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_DATA:
		ESP_LOGI(TAG, "MQTT_EVENT_DATA");
		printf("TOPIC = %.*s\r\n", event->topic_len, event->topic);
		printf("DATA = %.*s\r\n", event->data_len, event->data);
		break;
	case MQTT_EVENT_ERROR:
		ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
		break;
	default:
		ESP_LOGI(TAG, "Other event id:%d", event->event_id);
		break;
	}
}

esp_mqtt_client_handle_t client = NULL;

static void mqtt_app_start(void) {
	ESP_LOGI(TAG, "STARTING MQTT");
	esp_mqtt_client_config_t mqttConfig = { .uri = CONFIG_BROKER_URL, .host =
			"mqtt.flespi.io", .port = 1883, .username =
			"uaU1Kk7lPi29aQr0zvooVRHsgEW4VMadKU5k20wH8mH82WWph9C2Q9iWC4OlEOoe",
			.client_id = "hello", };

	client = esp_mqtt_client_init(&mqttConfig);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,
			client);
	esp_mqtt_client_start(client);
}

void Publisher_Task(void *params) {
//	int count = 1;
//	char DATA[100];
	while (true) {
		read_sensor_task();
		oled_task();
		if (MQTT_CONNEECTED && humi!=-1) {
			//sprintf(DATA, "Hello World %d", count);
			esp_mqtt_client_publish(client, MQTT_TOPIC, data_topic, 0, 0, 0);
			//count++;
			//esp_mqtt_client_publish(client, TEMP_TOPIC, temp_text, 0, 0, 0);
			vTaskDelay(10000 / portTICK_PERIOD_MS);
		}
	}
}

void app_main(void) {
	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	wifi_init_sta();
	mqtt_app_start();
	init_oled();
	xTaskCreate(Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL);
}

void read_sensor_task() {
	// INITIATE DHT
	DHT11_init(GPIO_NUM_26);	//DATA PIN

	humi = DHT11_read().humidity;
	temp = DHT11_read().temperature;
	//CONVERTING VALUE INTO TEXT
	sprintf(data_topic, "%d %d", temp, humi);
	sprintf(humi_text, "   Humi: %d %%", humi);
	sprintf(temp_text, "   Temp: %d 'C", temp);
//	while (1) {
//
//		// GETTING HUMIDITY VALUE FROM DHT
//		humi = DHT11_read().humidity;
//		temp = DHT11_read().temperature;
//		//CONVERTING VALUE INTO TEXT
//		sprintf(humi_text, "Humi: %d %%", humi);
//		sprintf(temp_text, "Temp: %d 'C", temp);
//
//		//vTaskDelay(10000 / portTICK_PERIOD_MS);
//
////		// TIMING FOR READING SENSOR
////		vTaskDelayUntil(&xLastWakeTime, 2000 / portTICK_RATE_MS);
//	}
}

void oled_task() {
	// INITIATING OLED
	char text1[] = "   CE232.N21.1";
	char text2[] = "   Nhom 2";
	if (humi != -1) {
		ssd1306_display_text(&dev, 0, text1, sizeof(text1), false);
		ssd1306_display_text(&dev, 1, text2, sizeof(text2), false);
		ssd1306_display_text(&dev, 3, temp_text, sizeof(temp_text), false);
		ssd1306_display_text(&dev, 4, humi_text, sizeof(humi_text), false);
	}

//	while (1) {
//		// WAIT FOR NOTIFICATION
//		xTaskGenericNotifyWait(0, 0x8000, 0x8000, NULL, portMAX_DELAY);
//		// OLED DISPLAY
//		//ssd1306_clear_screen(&dev, false);
//		ssd1306_display_text(&dev, 0, humi_text, sizeof(humi_text), false);
//		ssd1306_display_text(&dev, 1, temp_text, sizeof(temp_text), false);
//
//
//	}
}

void init_oled() {

	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
	ssd1306_init(&dev, 128, 64);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
}
