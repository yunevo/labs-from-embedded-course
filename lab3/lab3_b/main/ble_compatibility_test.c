/*
 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */

/********************************************************************************
 *
 * This file is for gatt server. It can send adv data, and get connected by client.
 *
 *********************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "ble_compatibility_test.h"
#include "esp_gatt_common_api.h"


#include "driver/i2c.h"
#include "esp_log.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

#define SDA_PIN 4
#define SCL_PIN 5
#define MASTER_PORT 0
#define OLED_FREQ 100000
#define OLED_ADDRESS 0x3C
#define WRITE_BIT 0
#define READ_BIT 1
#define ACK_EN 1

#define TAG "OLED"

#define OLED_CMD_SET_HORI_ADDR_MODE     0x00		
#define OLED_CMD_SET_VERT_ADDR_MODE     0x01    
#define OLED_CMD_SET_PAGE_ADDR_MODE     0x02    
#define OLED_CMD_SET_COLUMN_RANGE       0x21    

char text[8];
int count = 0;
#if DEBUG_ON
#define EXAMPLE_DEBUG ESP_LOGI
#else
#define EXAMPLE_DEBUG( tag, format, ... )
#endif

#define EXAMPLE_TAG "BLE_COMP"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "BLE_COMP_TEST"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the gatt client write or prepare write,
 *  the data length must be less than GATTS_EXAMPLE_CHAR_VAL_LEN_MAX.
 */
#define GATTS_EXAMPLE_CHAR_VAL_LEN_MAX 500
#define LONG_CHAR_VAL_LEN           500
#define SHORT_CHAR_VAL_LEN          10
#define GATTS_NOTIFY_FIRST_PACKET_LEN_MAX 20

#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)


// set up ssd1306

esp_err_t i2c_master_init() {
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = SCL_PIN,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 100000,
	};
	esp_err_t err = i2c_param_config(MASTER_PORT, &conf);
	if (err != ESP_OK) {
		return err;
	}
	return i2c_driver_install(MASTER_PORT, conf.mode, 0, 0, 0);
}

void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(TAG, "OLED configured successfully");
	}
	else {
		ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}

void write_text(int page, int column, char* data, int len) {
	int cur_page = page;
	int cur_col = column;
	if (cur_page > 7)
		return;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, OLED_ADDRESS << 1 | WRITE_BIT, ACK_EN);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, ACK_EN);

	i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, ACK_EN);
	i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDR_MODE, ACK_EN);

	i2c_master_write_byte(cmd, 0x0F & column, ACK_EN); //lower byte for start column
	i2c_master_write_byte(cmd, 0x10 | column >> 4, ACK_EN); //higher byte for start column

	i2c_master_write_byte(cmd, 0xB0 + page, ACK_EN);

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(MASTER_PORT, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);


	for (int i = 0; i < len; i++) {
		if (cur_col > 120) {
			cur_col = 0;
			cur_page++;
			if (cur_page > 7)
				return;
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, OLED_ADDRESS << 1 | WRITE_BIT, ACK_EN);
			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, ACK_EN);

			i2c_master_write_byte(cmd, 0x0F & cur_col, ACK_EN); //lower byte for start column
			i2c_master_write_byte(cmd, 0x10 | cur_col >> 4, ACK_EN);

			i2c_master_write_byte(cmd, 0xB0 + cur_page, ACK_EN);

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(MASTER_PORT, cmd, 1000 / portTICK_RATE_MS);
			i2c_cmd_link_delete(cmd);
		}

		if (data[i] == '\n') {
			cur_col = 127;
			continue;
		}

		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, OLED_ADDRESS << 1 | WRITE_BIT, ACK_EN);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, ACK_EN);

		i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)data[i]], 8, ACK_EN);

		i2c_master_stop(cmd);
		i2c_master_cmd_begin(MASTER_PORT, cmd, 1000 / portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd);
		cur_col += 8;
	}
}

void task_ssd1306_display_clear() {
	i2c_cmd_handle_t cmd;

	uint8_t clear[128];
	for (uint8_t i = 0; i < 128; i++) {
		clear[i] = 0;
	}
	for (uint8_t i = 0; i < 8; i++) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, 0xB0 | i, true);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
		i2c_master_write(cmd, clear, 128, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}
}


static uint8_t adv_config_done = 0;

uint16_t gatt_db_handle_table[HRS_IDX_NB];

typedef struct {
	uint8_t *prepare_buf;
	int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
/* flags */
0x02, 0x01, 0x06,
/* tx power*/
0x02, 0x0a, 0xeb,
/* service uuid */
0x03, 0x03, 0xFF, 0x00,
/* device name */
0x07, 0x09, 'N', 'H', 'O', 'M', '_', '2' };
static uint8_t raw_scan_rsp_data[] = { //thông tin bổ sung thêm khi nhấn vào xem
		/* flags */
		0x02, 0x01, 0x06,
		/* tx power */
		0x02, 0x0a, 0xeb,
		/* service uuid */
		0x03, 0x03, 0xFF, 0x00,
		//thông tin name bổ sung
				0x0A, 0x09, 'N', 'H', 'O', 'M', '_', '2', '_', 'N', 'E' };

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 16,
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = { .adv_int_min = 0x40, .adv_int_max =
		0x40, .adv_type = ADV_TYPE_IND, .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
		.channel_map = ADV_CHNL_ALL, .adv_filter_policy =
				ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, };

struct gatts_profile_inst {
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
	uint16_t char_handle;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;
	uint16_t descr_handle;
	esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
		esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
		[PROFILE_APP_IDX] = { .gatts_cb = gatts_profile_event_handler,
				.gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		}, };

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST = 0x00FF;
static const uint16_t CHAR_1_SHORT_WR = 0xFF01;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;

static const uint16_t character_user_description =
		ESP_GATT_UUID_CHAR_DESCRIPTION;

static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE
		| ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char1_name[] = "Please input ID of each team member:";

static const uint8_t char_value[4] = { 0x11, 0x22, 0x33, 0x44 };

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {
// Service Declaration
		[IDX_SVC] =
		{ { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
				(uint8_t*) &primary_service_uuid, ESP_GATT_PERM_READ,
				sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST),
				(uint8_t*) &GATTS_SERVICE_UUID_TEST } },

		/* Characteristic Declaration */
		[IDX_CHAR_A] =
		{ { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
				(uint8_t*) &character_declaration_uuid, ESP_GATT_PERM_READ,
				CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
				(uint8_t*) &char_prop_read_write } },

		/* Characteristic Value */
		[IDX_CHAR_VAL_A
				] =
				{ { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
						(uint8_t*) &CHAR_1_SHORT_WR, ESP_GATT_PERM_READ
								| ESP_GATT_PERM_WRITE,
						SHORT_CHAR_VAL_LEN, sizeof(char_value),
						(uint8_t*) char_value } },

		[IDX_CHAR_CFG_A ] =
		{ { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
				(uint8_t*) &character_user_description,
				ESP_GATT_PERM_READ, sizeof(char1_name), sizeof(char1_name),
				(uint8_t*) char1_name } }

};

static void show_bonded_devices(void) {
	int dev_num = esp_ble_get_bond_device_num();

	esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t*) malloc(
			sizeof(esp_ble_bond_dev_t) * dev_num);
	esp_ble_get_bond_device_list(&dev_num, dev_list);
	EXAMPLE_DEBUG(EXAMPLE_TAG, "Bonded devices number : %d\n", dev_num);

	EXAMPLE_DEBUG(EXAMPLE_TAG, "Bonded devices list : %d\n", dev_num);
	for (int i = 0; i < dev_num; i++) {
#if DEBUG_ON
        esp_log_buffer_hex(EXAMPLE_TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
        #endif
	}

	free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void) {
	int dev_num = esp_ble_get_bond_device_num();

	esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t*) malloc(
			sizeof(esp_ble_bond_dev_t) * dev_num);
	esp_ble_get_bond_device_list(&dev_num, dev_list);
	for (int i = 0; i < dev_num; i++) {
		esp_ble_remove_bond_device(dev_list[i].bd_addr);
	}

	free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
		esp_ble_gap_cb_param_t *param) {
	switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
	case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
		adv_config_done &= (~ADV_CONFIG_FLAG);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
		adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
#else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		/* advertising start complete event to indicate advertising start successfully or failed */
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(EXAMPLE_TAG, "advertising start failed");
		} else {
			ESP_LOGI(EXAMPLE_TAG,
					"(0) ***** advertising start successfully ***** \n");
		}
		break;
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(EXAMPLE_TAG, "Advertising stop failed");
		} else {
			ESP_LOGI(EXAMPLE_TAG, "Stop adv successfully\n");
		}
		break;
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		EXAMPLE_DEBUG(EXAMPLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
				param->update_conn_params.status,
				param->update_conn_params.min_int,
				param->update_conn_params.max_int,
				param->update_conn_params.conn_int,
				param->update_conn_params.latency,
				param->update_conn_params.timeout);
		break;
	case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */
		EXAMPLE_DEBUG(EXAMPLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
		//esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);
		break;

	case ESP_GAP_BLE_NC_REQ_EVT:
		/* The app will receive this event when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
		 show the passkey number to the user to confirm it with the number displayed by peer device. */
		ESP_LOGI(EXAMPLE_TAG,
				"ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d",
				param->ble_security.key_notif.passkey);
		break;
	case ESP_GAP_BLE_SEC_REQ_EVT:
		/* send the positive(true) security response to the peer device to accept the security request.
		 If not accept the security request, should send the security response with negative(false) accept value*/
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		break;
	case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: ///the app will receive this evt when the IO has Output capability and the peer device IO has Input capability.
		///show the passkey number to the user to input it in the peer device.
		ESP_LOGI(EXAMPLE_TAG, "The passkey notify number:%d",
				param->ble_security.key_notif.passkey);
		break;
	case ESP_GAP_BLE_KEY_EVT:
		//shows the ble key info share with peer device to the user.
		EXAMPLE_DEBUG(EXAMPLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
		break;
	case ESP_GAP_BLE_AUTH_CMPL_EVT: {
		esp_bd_addr_t bd_addr;
		memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr,
				sizeof(esp_bd_addr_t));
		EXAMPLE_DEBUG(EXAMPLE_TAG, "remote BD_ADDR: %08x%04x",
				(bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
				(bd_addr[4] << 8) + bd_addr[5]);EXAMPLE_DEBUG(EXAMPLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
		if (param->ble_security.auth_cmpl.success) {
			ESP_LOGI(EXAMPLE_TAG,
					"(1) ***** pair status = success ***** \n Please input ID of each team member:\n");
		} else {
			ESP_LOGI(EXAMPLE_TAG,
					"***** pair status = fail, reason = 0x%x *****\n",
					param->ble_security.auth_cmpl.fail_reason);
		}
		show_bonded_devices();
		break;
	}
	case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
		EXAMPLE_DEBUG(EXAMPLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
#if DEBUG_ON
            esp_log_buffer_hex(EXAMPLE_TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
            #endif
		EXAMPLE_DEBUG(EXAMPLE_TAG, "------------------------------------");
		break;
	}
	default:
		break;
	}
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if,
		prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
	EXAMPLE_DEBUG(EXAMPLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
	esp_gatt_status_t status = ESP_GATT_OK;
	if (prepare_write_env->prepare_buf == NULL) {
		prepare_write_env->prepare_buf = (uint8_t*) malloc(
		PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
		prepare_write_env->prepare_len = 0;
		if (prepare_write_env->prepare_buf == NULL) {
			ESP_LOGE(EXAMPLE_TAG, "%s, Gatt_server prep no mem", __func__);
			status = ESP_GATT_NO_RESOURCES;
		}
	} else {
		if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
			status = ESP_GATT_INVALID_OFFSET;
		} else if ((param->write.offset + param->write.len)
				> PREPARE_BUF_MAX_SIZE) {
			status = ESP_GATT_INVALID_ATTR_LEN;
		}
	}
	/*send response when param->write.need_rsp is true */
	if (param->write.need_rsp) {
		esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t*) malloc(
				sizeof(esp_gatt_rsp_t));
		if (gatt_rsp != NULL) {
			gatt_rsp->attr_value.len = param->write.len;
			gatt_rsp->attr_value.handle = param->write.handle;
			gatt_rsp->attr_value.offset = param->write.offset;
			gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
			memcpy(gatt_rsp->attr_value.value, param->write.value,
					param->write.len);
			esp_err_t response_err = esp_ble_gatts_send_response(gatts_if,
					param->write.conn_id, param->write.trans_id, status,
					gatt_rsp);
			if (response_err != ESP_OK) {
				ESP_LOGE(EXAMPLE_TAG, "Send response error");
			}
			free(gatt_rsp);
		} else {
			ESP_LOGE(EXAMPLE_TAG, "%s, malloc failed", __func__);
		}
	}
	if (status != ESP_GATT_OK) {
		return;
	}
	memcpy(prepare_write_env->prepare_buf + param->write.offset,
			param->write.value, param->write.len);
	prepare_write_env->prepare_len += param->write.len;

}
uint8_t long_write[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
		0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env,
		esp_ble_gatts_cb_param_t *param) {
	if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC
			&& prepare_write_env->prepare_buf) {
		if (prepare_write_env->prepare_len == 256) {
			bool long_write_success = true;
			for (uint16_t i = 0; i < prepare_write_env->prepare_len; i++) {
				if (prepare_write_env->prepare_buf[i] != long_write[i % 16]) {
					long_write_success = false;
					break;
				}
			}
			if (long_write_success) {
				ESP_LOGI(EXAMPLE_TAG, "(4) ***** long write success ***** \n");
			}
		}
	} else {
		ESP_LOGI(EXAMPLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
	}
	if (prepare_write_env->prepare_buf) {
		free(prepare_write_env->prepare_buf);
		prepare_write_env->prepare_buf = NULL;
	}
	prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
		esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	switch (event) {
	case ESP_GATTS_REG_EVT: {
		esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(
		SAMPLE_DEVICE_NAME);
		if (set_dev_name_ret) {
			ESP_LOGE(EXAMPLE_TAG, "set device name failed, error code = %x",
					set_dev_name_ret);
		}
#ifdef CONFIG_SET_RAW_ADV_DATA
		esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data,
				sizeof(raw_adv_data));
		if (raw_adv_ret) {
			ESP_LOGE(EXAMPLE_TAG,
					"config raw adv data failed, error code = %x ",
					raw_adv_ret);
		}
		adv_config_done |= ADV_CONFIG_FLAG;
		esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(
				raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
		if (raw_scan_ret) {
			ESP_LOGE(EXAMPLE_TAG,
					"config raw scan rsp data failed, error code = %x",
					raw_scan_ret);
		}
		adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(EXAMPLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(EXAMPLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
		esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db,
				gatts_if, HRS_IDX_NB, SVC_INST_ID);
		if (create_attr_ret) {
			ESP_LOGE(EXAMPLE_TAG, "create attr table failed, error code = %x",
					create_attr_ret);
		}
	}
		break;
	case ESP_GATTS_READ_EVT:
		//ESP_LOGE(EXAMPLE_TAG, "ESP_GATTS_READ_EVT, handle=0x%d, offset=%d", param->read.handle, param->read.offset);
		if (gatt_db_handle_table[IDX_CHAR_VAL_A] == param->read.handle) {
			ESP_LOGE(EXAMPLE_TAG, "(2) ***** read char1 ***** \n");
		}

		break;
	case ESP_GATTS_WRITE_EVT:
		if (!param->write.is_prep) {
			// the data length of gattc write  must be less than GATTS_EXAMPLE_CHAR_VAL_LEN_MAX.

			if (gatt_db_handle_table[IDX_CHAR_VAL_A] == param->write.handle
					&& param->write.len == 8) {
				ESP_LOGI("Mobile", "%s", param->write.value);
				memcpy(text,param->write.value,8);
				write_text(count, 32, text, 8);
				count++;
			}

			/* send response when param->write.need_rsp is true*/
			if (param->write.need_rsp) {
				esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
						param->write.trans_id, ESP_GATT_OK, NULL);
			}
		} else {
			/* handle prepare write */
			example_prepare_write_event_env(gatts_if, &prepare_write_env,
					param);
		}
		break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		// the length of gattc prepare write data must be less than GATTS_EXAMPLE_CHAR_VAL_LEN_MAX.
		ESP_LOGI(EXAMPLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT, Length=%d",
				prepare_write_env.prepare_len);
		example_exec_write_event_env(&prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		EXAMPLE_DEBUG(EXAMPLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	case ESP_GATTS_CONF_EVT:
		EXAMPLE_DEBUG(EXAMPLE_TAG, "ESP_GATTS_CONF_EVT, status = %d", param->conf.status);
		break;
	case ESP_GATTS_START_EVT:
		EXAMPLE_DEBUG(EXAMPLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_CONNECT_EVT:
		ESP_LOGI(EXAMPLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d",
				param->connect.conn_id);
		/* start security connect with peer device when receive the connect event sent by the master */
		esp_ble_set_encryption(param->connect.remote_bda,
				ESP_BLE_SEC_ENCRYPT_MITM);
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(EXAMPLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = %d",
				param->disconnect.reason);
		esp_ble_gap_start_advertising(&adv_params);
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
		if (param->add_attr_tab.status != ESP_GATT_OK) {
			ESP_LOGE(EXAMPLE_TAG,
					"create attribute table failed, error code=0x%x",
					param->add_attr_tab.status);
		} else if (param->add_attr_tab.num_handle != HRS_IDX_NB) {
			ESP_LOGE(EXAMPLE_TAG,
					"create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
					param->add_attr_tab.num_handle, HRS_IDX_NB);
		} else {
			ESP_LOGI(EXAMPLE_TAG,
					"create attribute table successfully, the number handle = %d\n",
					param->add_attr_tab.num_handle);
			memcpy(gatt_db_handle_table, param->add_attr_tab.handles,
					sizeof(gatt_db_handle_table));
			esp_ble_gatts_start_service(gatt_db_handle_table[IDX_SVC]);
		}
		break;
	}
	default:
		break;
	}
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
		esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
		} else {
			ESP_LOGE(EXAMPLE_TAG, "reg app failed, app_id %04x, status %d",
					param->reg.app_id, param->reg.status);
			return;
		}
	}
	do {
		int idx;
		for (idx = 0; idx < PROFILE_NUM; idx++) {
			/* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
			if (gatts_if == ESP_GATT_IF_NONE
					|| gatts_if == heart_rate_profile_tab[idx].gatts_if) {
				if (heart_rate_profile_tab[idx].gatts_cb) {
					heart_rate_profile_tab[idx].gatts_cb(event, gatts_if,
							param);
				}
			}
		}
	} while (0);
}

void app_main(void) {

	esp_err_t ret;

	/* Initialize NVS. */
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(EXAMPLE_TAG, "%s enable controller failed: %s", __func__,
				esp_err_to_name(ret));
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(EXAMPLE_TAG, "%s enable controller failed: %s", __func__,
				esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(EXAMPLE_TAG, "%s init bluetooth failed: %s", __func__,
				esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(EXAMPLE_TAG, "%s enable bluetooth failed: %s", __func__,
				esp_err_to_name(ret));
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret) {
		ESP_LOGE(EXAMPLE_TAG, "gatts register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret) {
		ESP_LOGE(EXAMPLE_TAG, "gap register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gatts_app_register(ESP_APP_ID);
	if (ret) {
		ESP_LOGE(EXAMPLE_TAG, "gatts app register error, error code = %x", ret);
		return;
	}

	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(33);
	if (local_mtu_ret) {
		ESP_LOGE(EXAMPLE_TAG, "set local  MTU failed, error code = %x",
				local_mtu_ret);
	}

	/* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
	esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND; //bonding with peer device after authentication
	esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE; //set the IO capability to No output No input
	uint8_t key_size = 16;      //the key size should be 7~16 bytes
	uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	uint32_t passkey = 205217;
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey,
			sizeof(uint32_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req,
			sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap,
			sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size,
			sizeof(uint8_t));
	/* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
	 and the response key means which key you can distribute to the Master;
	 If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
	 and the init key means which key you can distribute to the slave. */
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key,
			sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key,
			sizeof(uint8_t));
	i2c_master_init();
	ssd1306_init();
	task_ssd1306_display_clear();
	ESP_LOGI("SET UP SSD1306", "DONE");
}


