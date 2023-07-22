#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#if defined(CONFIG_NRF_MODEM_LIB)
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>
#include <nrf_modem_gnss.h>
#include <nrf_modem.h>
#endif
#include <net/aws_iot.h>
#include <zephyr/sys/reboot.h>
#include <date_time.h>
#include <zephyr/dfu/mcuboot.h>
#include <cJSON.h>
#include <cJSON_os.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_AWS_IOT_SAMPLE_DEVICE_ID_USE_HW_ID)
#include <hw_id.h>
#endif


// todo kconfig it ?
#define GNSS_FIX_INTERVAL (25)

static const char * custom_topic = "dbg";

LOG_MODULE_REGISTER(aws_iot_sample_log, CONFIG_AWS_IOT_SAMPLE_LOG_LEVEL);

BUILD_ASSERT(!IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT),
	     "not support LTE auto-init and connect");

#define APP_TOPICS_COUNT CONFIG_AWS_IOT_APP_SUBSCRIPTION_LIST_COUNT

static struct k_work_delayable shadow_update_work;
static struct k_work_delayable gnss_work;
static struct k_work_delayable connect_work;
static struct k_work shadow_update_version_work;
static bool cloud_connected;

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(date_time_obtained, 0, 1);

static struct nrf_modem_gnss_pvt_data_frame last_pvt;
//static struct nrf_modem_gnss_nmea_data_frame last_nmea;
K_MSGQ_DEFINE(nmea_queue, sizeof(struct nrf_modem_gnss_nmea_data_frame *), 10, 4);
static K_SEM_DEFINE(pvt_data_sem, 0, 1);


static int json_add_obj(cJSON *parent, const char *str, cJSON *item)
{
	cJSON_AddItemToObject(parent, str, item);

	return 0;
}

static int json_add_str(cJSON *parent, const char *str, const char *item)
{
	cJSON *json_str;

	json_str = cJSON_CreateString(item);
	if (json_str == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_str);
}

static int json_add_number(cJSON *parent, const char *str, double item)
{
	cJSON *json_num;

	json_num = cJSON_CreateNumber(item);
	if (json_num == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_num);
}

static int send_pvt()
{
	bool valid = last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID;

	if (!cloud_connected || !valid) {
		// silently skip tilll we get fix
		return 0;
	}

	int err = 0;
	int64_t message_ts = 0;
	char *message = "\0";
	LOG_INF("send_pvt start");

	cJSON *root_obj = cJSON_CreateObject();
	cJSON *state_obj = cJSON_CreateObject();
	if (root_obj == NULL || state_obj == NULL) {
		cJSON_Delete(root_obj);
		cJSON_Delete(state_obj);
		err = -ENOMEM;
		return err;
	}

	err = date_time_now(&message_ts);
	if (err) {
		LOG_ERR("date_time_now %d", err);
		goto cleanup;
	}
	err += json_add_number(state_obj, "ts",  message_ts);
	err += json_add_number(state_obj, "lat", last_pvt.latitude);
	err += json_add_number(state_obj, "lon", last_pvt.longitude);
	//err += json_add_number(state_obj, "alt", last_pvt.altitude);
	err += json_add_number(state_obj, "acc", last_pvt.accuracy);
	//err += json_add_number(state_obj, "speed", last_pvt.speed);
	//err += json_add_number(state_obj, "speed_accuracy", last_pvt.speed_accuracy);
	//err += json_add_number(state_obj, "heading", last_pvt.heading);
	//err += json_add_number(state_obj, "heading_accuracy", last_pvt.heading_accuracy);
	err += json_add_obj(root_obj, "state", state_obj);

	if (err) {
		LOG_ERR("json_add..., error: %d", err);
		goto cleanup;
	}

	message = cJSON_Print(root_obj);
	if (message == NULL) {
		LOG_ERR("cJSON_Print, error: returned NULL");
		err = -ENOMEM;
		goto cleanup;
	}

	struct aws_iot_data tx_data = {
		.qos = MQTT_QOS_0_AT_MOST_ONCE,
		.ptr = message,
		.len = strlen(message)
	};

	tx_data.topic.str = "gps";
	tx_data.topic.len = 3;

	LOG_INF("GNSS: sending to AWS IoT ");

	err = aws_iot_send(&tx_data);
	if (err) {
		LOG_ERR("GNSS: aws_iot_send, error: %d", err);
	}

	cJSON_FreeString(message);

cleanup:

	cJSON_Delete(root_obj);
	return err;
}

static void gnss_event_handler(int event)
{
	if (event != NRF_MODEM_GNSS_EVT_PVT) 
		return;
	switch (event) {
	case NRF_MODEM_GNSS_EVT_PVT:
		int retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
		if (retval == 0) {
			//LOG_INF("gnss_event_handler new pvt");
			k_sem_give(&pvt_data_sem);
		}
		break;
	default:
		break;
	}
}

static int gnss_init_and_start(void)
{
	/* Enable GNSS. */
	if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS) != 0) {
		LOG_ERR("Failed to activate GNSS functional mode");
		return -1;
	}

	/* Configure GNSS. */
	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
		LOG_ERR("Failed to set GNSS event handler");
		return -1;
	}

	/* Enable all supported NMEA messages. */
	
	// uint16_t nmea_mask = NRF_MODEM_GNSS_NMEA_RMC_MASK |
	// 		     NRF_MODEM_GNSS_NMEA_GGA_MASK |
	// 		     NRF_MODEM_GNSS_NMEA_GLL_MASK |
	// 		     NRF_MODEM_GNSS_NMEA_GSA_MASK |
	// 		     NRF_MODEM_GNSS_NMEA_GSV_MASK;
	/* disable nmea and use pvt onl */
	if (nrf_modem_gnss_nmea_mask_set(0) != 0) {
		LOG_ERR("Failed to set GNSS NMEA mask");
		return -1;
	}

	/* This use case flag should always be set. */
	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;

	if (nrf_modem_gnss_use_case_set(use_case) != 0) {
		LOG_WRN("Failed to set GNSS use case");
	}

	/* Default to no power saving. */
	uint8_t power_mode = NRF_MODEM_GNSS_PSM_DISABLED;

// #if defined(GNSS_SAMPLE_POWER_SAVING_MODERATE)
// 	power_mode = NRF_MODEM_GNSS_PSM_DUTY_CYCLING_PERFORMANCE;
// #elif defined(GNSS_SAMPLE_POWER_SAVING_HIGH)
// 	power_mode = NRF_MODEM_GNSS_PSM_DUTY_CYCLING_POWER;
// #endif

	if (nrf_modem_gnss_power_mode_set(power_mode) != 0) {
		LOG_ERR("Failed to set GNSS power saving mode");
		return -1;
	}

	/* Default to continuous tracking. */
	uint16_t fix_retry = 0;
	uint16_t fix_interval = GNSS_FIX_INTERVAL;

	if (nrf_modem_gnss_fix_retry_set(fix_retry) != 0) {
		LOG_ERR("Failed to set GNSS fix retry");
		return -1;
	}

	if (nrf_modem_gnss_fix_interval_set(fix_interval) != 0) {
		LOG_ERR("Failed to set GNSS fix interval");
		return -1;
	}

// finally start it up 
	if (nrf_modem_gnss_start() != 0) {
		LOG_ERR("Failed to start GNSS");
		return -1;
	}
	return 0;
}

static int shadow_update(bool version_number_include)
{
	return 0;

	int err;
	char *message;
	int64_t message_ts = 0;
	int16_t bat_voltage = 0;

	err = date_time_now(&message_ts);
	if (err) {
		LOG_ERR("date_time_now, error: %d", err);
		return err;
	}

#if defined(CONFIG_NRF_MODEM_LIB)
	/* Request battery voltage data from the modem. */
	err = modem_info_short_get(MODEM_INFO_BATTERY, &bat_voltage);
	if (err != sizeof(bat_voltage)) {
		LOG_ERR("modem_info_short_get, error: %d", err);
		return err;
	}
#endif

	cJSON *root_obj = cJSON_CreateObject();
	cJSON *state_obj = cJSON_CreateObject();
	cJSON *reported_obj = cJSON_CreateObject();

	if (root_obj == NULL || state_obj == NULL || reported_obj == NULL) {
		cJSON_Delete(root_obj);
		cJSON_Delete(state_obj);
		cJSON_Delete(reported_obj);
		err = -ENOMEM;
		return err;
	}

	if (version_number_include) {
		err = json_add_str(reported_obj, "app_version", CONFIG_AWS_IOT_SAMPLE_APP_VERSION);
	} else {
		err = 0;
	}

	err += json_add_number(reported_obj, "batv", bat_voltage);
	err += json_add_number(reported_obj, "ts", message_ts);
	err += json_add_obj(state_obj, "reported", reported_obj);
	err += json_add_obj(root_obj, "state", state_obj);

	if (err) {
		LOG_ERR("json_add, error: %d", err);
		goto cleanup;
	}

	message = cJSON_Print(root_obj);
	if (message == NULL) {
		LOG_ERR("cJSON_Print, error: returned NULL");
		err = -ENOMEM;
		goto cleanup;
	}

	struct aws_iot_data tx_data = {
		.qos = MQTT_QOS_0_AT_MOST_ONCE,
		.topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE,
		.ptr = message,
		.len = strlen(message)
	};

	LOG_INF("Publishing: %s to AWS IoT broker", message);

	err = aws_iot_send(&tx_data);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
	}

	cJSON_FreeString(message);

cleanup:

	cJSON_Delete(root_obj);

	return err;
}

static void connect_work_fn(struct k_work *work)
{
	int err;

	if (cloud_connected) {
		return;
	}

	err = aws_iot_connect(NULL);
	if (err) {
		LOG_ERR("aws_iot_connect, error: %d", err);
	}

	LOG_INF("Next connection retry in %d seconds",
		CONFIG_AWS_IOT_SAMPLE_CONNECTION_RETRY_TIMEOUT_SECONDS);

	k_work_schedule(&connect_work,
			K_SECONDS(CONFIG_AWS_IOT_SAMPLE_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

static void gnss_work_fn(struct k_work *work)
{
	// wait loop on sem versus reschedule
	while (true)
	{
		k_sem_take(&pvt_data_sem, K_FOREVER);
		int err = send_pvt();
		if (err != 0)
		{
			LOG_ERR("send_pvt returned %d", err);
		}
	}
}

static void shadow_update_work_fn(struct k_work *work)
{
	int err;

	if (!cloud_connected) {
		return;
	}

	err = shadow_update(false);
	if (err) {
		LOG_ERR("shadow_update, error: %d", err);
	}

	LOG_INF("Next data publication in %d seconds",
		CONFIG_AWS_IOT_SAMPLE_PUBLICATION_INTERVAL_SECONDS);

	k_work_schedule(&shadow_update_work,
			K_SECONDS(CONFIG_AWS_IOT_SAMPLE_PUBLICATION_INTERVAL_SECONDS));
}

static void shadow_update_version_work_fn(struct k_work *work)
{
	int err;

	err = shadow_update(true);
	if (err) {
		LOG_ERR("shadow_update, error: %d", err);
	}
}

static void print_received_data(const char *buf, const char *topic,
				size_t topic_len)
{
	char *str = NULL;
	cJSON *root_obj = NULL;

	root_obj = cJSON_Parse(buf);
	if (root_obj == NULL) {
		LOG_ERR("cJSON Parse failure");
		return;
	}

	str = cJSON_Print(root_obj);
	if (str == NULL) {
		LOG_ERR("Failed to print JSON object");
		goto clean_exit;
	}

	LOG_INF("Data received from AWS IoT console: Topic: %.*s Message: %s",
		topic_len, topic, str);

	cJSON_FreeString(str);

clean_exit:
	cJSON_Delete(root_obj);
}

void aws_iot_event_handler(const struct aws_iot_evt *const evt)
{
	switch (evt->type) {
	case AWS_IOT_EVT_CONNECTING:
		LOG_INF("AWS_IOT_EVT_CONNECTING");
		break;
	case AWS_IOT_EVT_CONNECTED:
		LOG_INF("AWS_IOT_EVT_CONNECTED");

		cloud_connected = true;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&connect_work);

		if (evt->data.persistent_session) {
			LOG_INF("Persistent session enabled");
		}

#if defined(CONFIG_NRF_MODEM_LIB)
		/** Successfully connected to AWS IoT broker, mark image as
		 *  working to avoid reverting to the former image upon reboot.
		 */
		boot_write_img_confirmed();
#endif

		/** Send version number to AWS IoT broker to verify that the
		 *  FOTA update worked.
		 */
		k_work_submit(&shadow_update_version_work);

		/** Start sequential shadow data updates.
		 */
		k_work_schedule(&shadow_update_work,
				K_SECONDS(CONFIG_AWS_IOT_SAMPLE_PUBLICATION_INTERVAL_SECONDS));

#if defined(CONFIG_NRF_MODEM_LIB)
		int err = lte_lc_psm_req(true);
		if (err) {
			LOG_ERR("Requesting PSM failed, error: %d", err);
		}
#endif
		break;
	case AWS_IOT_EVT_READY:
		LOG_INF("AWS_IOT_EVT_READY");
		break;
	case AWS_IOT_EVT_DISCONNECTED:
		LOG_INF("AWS_IOT_EVT_DISCONNECTED");
		cloud_connected = false;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&shadow_update_work);
		k_work_schedule(&connect_work, K_NO_WAIT);
		break;
	case AWS_IOT_EVT_DATA_RECEIVED:
		LOG_INF("AWS_IOT_EVT_DATA_RECEIVED");
		print_received_data(evt->data.msg.ptr, evt->data.msg.topic.str,
				    evt->data.msg.topic.len);
		break;
	case AWS_IOT_EVT_PUBACK:
		LOG_INF("AWS_IOT_EVT_PUBACK, message ID: %d", evt->data.message_id);
		break;
	case AWS_IOT_EVT_FOTA_START:
		LOG_INF("AWS_IOT_EVT_FOTA_START");
		break;
	case AWS_IOT_EVT_FOTA_ERASE_PENDING:
		LOG_INF("AWS_IOT_EVT_FOTA_ERASE_PENDING");
		LOG_INF("Disconnect LTE link or reboot");
#if defined(CONFIG_NRF_MODEM_LIB)
		err = lte_lc_offline();
		if (err) {
			LOG_ERR("Error disconnecting from LTE");
		}
#endif
		break;
	case AWS_IOT_EVT_FOTA_ERASE_DONE:
		LOG_INF("AWS_FOTA_EVT_ERASE_DONE");
		LOG_INF("Reconnecting the LTE link");
#if defined(CONFIG_NRF_MODEM_LIB)
		err = lte_lc_connect();
		if (err) {
			LOG_ERR("Error connecting to LTE");
		}
#endif
		break;
	case AWS_IOT_EVT_FOTA_DONE:
		LOG_INF("AWS_IOT_EVT_FOTA_DONE");
		LOG_INF("FOTA done, rebooting device");
		aws_iot_disconnect();
		sys_reboot(0);
		break;
	case AWS_IOT_EVT_FOTA_DL_PROGRESS:
		LOG_INF("AWS_IOT_EVT_FOTA_DL_PROGRESS, (%d%%)", evt->data.fota_progress);
	case AWS_IOT_EVT_ERROR:
		LOG_INF("AWS_IOT_EVT_ERROR, %d", evt->data.err);
		break;
	case AWS_IOT_EVT_FOTA_ERROR:
		LOG_INF("AWS_IOT_EVT_FOTA_ERROR");
		break;
	default:
		LOG_WRN("Unknown AWS IoT event type: %d", evt->type);
		break;
	}
}

static void work_init(void)
{
	k_work_init_delayable(&shadow_update_work, shadow_update_work_fn);
	k_work_init_delayable(&connect_work, connect_work_fn);
	k_work_init_delayable(&gnss_work, gnss_work_fn);
	k_work_init(&shadow_update_version_work, shadow_update_version_work_fn);
}

#if defined(CONFIG_NRF_MODEM_LIB)
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		     (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			break;
		}

		LOG_INF("Network registration status: %s",
			evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
			"Connected - home network" : "Connected - roaming");

		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		LOG_INF("PSM parameter update: TAU: %d, Active time: %d",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE: {
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
			       "eDRX parameter update: eDRX: %f, PTW: %f",
			       evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0) {
			LOG_INF("%s", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s",
			evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
			"Connected" : "Idle");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		LOG_INF("LTE cell changed: Cell ID: %d, Tracking area: %d",
			evt->cell.id, evt->cell.tac);
		break;
	default:
		break;
	}
}

static void nrf_modem_lib_dfu_handler(int err)
{
	switch (err) {
	case NRF_MODEM_DFU_RESULT_OK:
		LOG_INF("Modem update suceeded, reboot");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_MODEM_DFU_RESULT_UUID_ERROR:
	case NRF_MODEM_DFU_RESULT_AUTH_ERROR:
		LOG_INF("Modem update failed, error: %d", err);
		LOG_INF("Modem will use old firmware");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_MODEM_DFU_RESULT_HARDWARE_ERROR:
	case NRF_MODEM_DFU_RESULT_INTERNAL_ERROR:
		LOG_INF("Modem update malfunction, error: %d, reboot", err);
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_MODEM_DFU_RESULT_VOLTAGE_LOW:
		LOG_INF("Modem update cancelled due to low power, error: %d", err);
		LOG_INF("Please reboot once you have sufficient power for the DFU");
		break;
	default:
		break;
	}
}
#endif

static int app_topics_subscribe(void)
{
	int err;

	const struct aws_iot_topic_data topics_list[APP_TOPICS_COUNT] = {
		[0].str = custom_topic,
		[0].len = strlen(custom_topic)
	};

	err = aws_iot_subscription_topics_add(topics_list, ARRAY_SIZE(topics_list));
	if (err) {
		LOG_ERR("aws_iot_subscription_topics_add, error: %d", err);
	}

	return err;
}

static void date_time_event_handler(const struct date_time_evt *evt)
{
	switch (evt->type) {
	case DATE_TIME_OBTAINED_MODEM:
		/* Fall through */
	case DATE_TIME_OBTAINED_NTP:
		/* Fall through */
	case DATE_TIME_OBTAINED_EXT:
		LOG_INF("Date time obtained");
		k_sem_give(&date_time_obtained);
		
		/* De-register handler. At this point the sample will have
		 * date time to depend on indefinitely until a reboot occurs.
		 */
		date_time_register_handler(NULL);
		break;
	case DATE_TIME_NOT_OBTAINED:
		LOG_INF("DATE_TIME_NOT_OBTAINED");
		break;
	default:
		LOG_ERR("Unknown event: %d", evt->type);
		break;
	}
}

int main(void)
{
	int err;

	LOG_INF("The AWS IoT sample started, version: %s", CONFIG_AWS_IOT_SAMPLE_APP_VERSION);

	cJSON_Init();

// do modem init like other example

#if defined(CONFIG_NRF_MODEM_LIB)
	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("Modem library initialization failed, error: %d", err);
		return 0;
	}

	nrf_modem_lib_dfu_handler(err);
#endif

#if defined(CONFIG_NRF_MODEM_LIB)
	err = lte_lc_init_and_connect_async(lte_handler);
	if (err) {
		LOG_ERR("Modem could not be configured, error: %d", err);
		return 0;
	}

	err = modem_info_init();
	if (err) {
		LOG_ERR("Failed initializing modem info module, error: %d", err);
	}

	k_sem_take(&lte_connected, K_FOREVER);
#endif

	err = gnss_init_and_start();
	if (err) {
		LOG_ERR("gnss_init_and_start failed, error: %d", err);
		return 0;
	}
#if defined(CONFIG_AWS_IOT_SAMPLE_DEVICE_ID_USE_HW_ID)
	char device_id[HW_ID_LEN] = { 0 };

	err = hw_id_get(device_id, ARRAY_SIZE(device_id));
	if (err) {
		LOG_ERR("Failed to retrieve device ID, error: %d", err);
		return 0;
	}

	struct aws_iot_config config = {
	    .client_id = device_id,
	    .client_id_len = strlen(device_id)
	};

	LOG_INF("Device id: %s", device_id);

	err = aws_iot_init(&config, aws_iot_event_handler);
#else
	err = aws_iot_init(NULL, aws_iot_event_handler);
#endif
	if (err) {
		LOG_ERR("AWS IoT library could not be initialized, error: %d", err);
	}

	/** Subscribe to customizable non-shadow specific topics
	 *  to AWS IoT backend.
	 */
	err = app_topics_subscribe();
	if (err) {
		LOG_ERR("Adding application specific topics failed, error: %d", err);
	}

	work_init();
	/* Trigger a date time update. The date_time API is used to timestamp data that is sent
	 * to AWS IoT.
	 */
	date_time_update_async(date_time_event_handler);

	/* Postpone connecting to AWS IoT until date time has been obtained. */
	k_sem_take(&date_time_obtained, K_FOREVER);
	k_work_schedule(&connect_work, K_NO_WAIT);

	// start gnss worker loop soon too
	k_work_schedule(&gnss_work, K_SECONDS(1));

	return 0;
}
