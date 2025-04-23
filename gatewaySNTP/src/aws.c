#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/net/conn_mgr_connectivity.h>
#include <zephyr/net/conn_mgr_monitor.h>
#include <zephyr/drivers/gpio.h>
#include "iot_connections.h"
#include <stdio.h>
#include <stdlib.h>
#include <modem/modem_info.h>
#include <time.h>
#include <zephyr/net/sntp.h>
#include "gsm.h"
#include "observer.h"
#include <zephyr/data/json.h>
#include <zephyr/drivers/flash.h>

#define publishPayloadSize CONFIG_BLE_RECEIVING_PAYLOAD
#define LED0_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Register log module */
LOG_MODULE_REGISTER(aws_iot_sample, CONFIG_AWS_IOT_SAMPLE_LOG_LEVEL);

#define SNTP_SERVER "0.pool.ntp.org"

/* Macros used to subscribe to specific Zephyr NET management events. */
#define L4_EVENT_MASK (NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED)
#define CONN_LAYER_EVENT_MASK (NET_EVENT_CONN_IF_FATAL_ERROR)

#define MODEM_FIRMWARE_VERSION_SIZE_MAX 50

/* Macro called upon a fatal error, reboots the device. */
#define FATAL_ERROR()                              \
	LOG_ERR("Fatal error! Rebooting the device."); \
	LOG_PANIC();                                   \
	IF_ENABLED(CONFIG_REBOOT, (sys_reboot(0)))

/* Forward declarations. */
static void shadow_update_work_fn(struct k_work *work);
static void connect_work_fn(struct k_work *work);
static void aws_iot_event_handler(const struct aws_iot_evt *const evt);

/* Work items used to control some aspects of the sample. */
static K_WORK_DELAYABLE_DEFINE(shadow_update_work, shadow_update_work_fn);
static K_WORK_DELAYABLE_DEFINE(connect_work, connect_work_fn);

#define SPI_FLASH_SECTOR_SIZE 4096
extern int readIndex, writeIndex, flashLimit, filterVar;
extern bool memFilled;
bool once = true; // run only one time
extern struct k_mutex test_mutex;
static int aws_iot_client_init(void)
{
	int err;
	struct aws_iot_config config = {0};

	err = aws_iot_init(&config, aws_iot_event_handler);
	if (err)
	{
		LOG_ERR("AWS IoT library could not be initialized, error: %d", err);
		FATAL_ERROR();
		return err;
	}

	return 0;
}

struct publish_payload
{
	uint32_t counter[publishPayloadSize];
	size_t counter_array_len;
};

static const struct json_obj_descr json_descr[] = {
	JSON_OBJ_DESCR_ARRAY(struct publish_payload, counter, 10 + 1, counter_array_len, JSON_TOK_NUMBER),
};

/* System Workqueue handlers. */
static void shadow_update_work_fn(struct k_work *work)
{
	uint8_t buf[publishPayloadSize] = {0};
	char temp;

	const struct device *flashDev;
	flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));

	struct publish_payload pl;
	int err;

	int index = 0;
	char message[CONFIG_AWS_IOT_SAMPLE_JSON_MESSAGE_SIZE_MAX] = {0};

	if (((writeIndex - readIndex) >= publishPayloadSize) || memFilled)
	{

		if (readIndex >= flashLimit)
		{
			readIndex = 0x102000;
			memFilled = false;
		}
		int rc;
		printk("flash reading from index %x", readIndex);
		rc = flash_read(flashDev, readIndex, buf, publishPayloadSize);
		if (rc != 0)
		{
			/* IF FAILED! */
			printf("Flash read failed! %d\n", rc);
			return;
		}

		const uint8_t *rp = buf;
		const uint8_t *rpe = rp + publishPayloadSize;

		while (rp < rpe)
		{
			/* temp has is 1 byte char.*/
			temp = *rp;

			/* Appending them in pl.counter so as to make MQTT payload. The index and rp are incremented every time. Thus, they are iterating over data. */
			pl.counter[index] = temp;
			index++;
			++rp;
		}
		pl.counter_array_len = publishPayloadSize;

		/* JSON encoding. */
		json_obj_encode_buf(json_descr, ARRAY_SIZE(json_descr), &pl, message, sizeof(message));

		struct aws_iot_data tx_data = {
			.qos = CONFIG_AWS_QOS,
			.topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE,
		};

		tx_data.ptr = message;
		tx_data.len = strlen(message);

		err = aws_iot_send(&tx_data);
		if (err)
		{
			LOG_ERR("aws_iot_send, error: %d", err);
			// FATAL_ERROR();
			return;
		}
		// readIndex += publishPayloadSize;
	}
	(void)k_work_reschedule(&shadow_update_work,
							K_SECONDS(CONFIG_AWS_IOT_SAMPLE_PUBLICATION_INTERVAL_SECONDS));
}

static void connect_work_fn(struct k_work *work)
{
	int err;

	LOG_INF("Connecting to AWS IoT");

	err = aws_iot_connect(NULL);
	if (err)
	{
		LOG_ERR("aws_iot_connect, error: %d", err);
	}

	LOG_INF("Next connection retry in %d seconds",
			CONFIG_AWS_IOT_SAMPLE_CONNECTION_RETRY_TIMEOUT_SECONDS);

	(void)k_work_reschedule(&connect_work,
							K_SECONDS(CONFIG_AWS_IOT_SAMPLE_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

/* Functions that are executed on specific connection-related events. */

static void on_aws_iot_evt_connected(const struct aws_iot_evt *const evt)
{
	// rebootCounter = 0;
	(void)k_work_cancel_delayable(&connect_work);

	/* If persistent session is enabled, the AWS IoT library will not subscribe to any topics.
	 * Topics from the last session will be used.
	 */
	if (evt->data.persistent_session)
	{
		LOG_WRN("Persistent session is enabled, using subscriptions "
				"from the previous session");
	}

	/* Mark image as working to avoid reverting to the former image after a reboot. */
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
	LOG_INF("Confirming image");
	boot_write_img_confirmed();
#endif

	/* Start sequential updates to AWS IoT. */
	(void)k_work_reschedule(&shadow_update_work, K_NO_WAIT);
}

static void on_aws_iot_evt_disconnected(void)
{
	(void)k_work_cancel_delayable(&shadow_update_work);
	(void)k_work_reschedule(&connect_work, K_SECONDS(5));
}

static void on_aws_iot_evt_fota_done(const struct aws_iot_evt *const evt)
{
	int err;

	/* Tear down MQTT connection. */
	(void)aws_iot_disconnect();
	(void)k_work_cancel_delayable(&connect_work);

	/* If modem FOTA has been carried out, the modem needs to be reinitialized.
	 * This is carried out by bringing the network interface down/up.
	 */
	if (evt->data.image & DFU_TARGET_IMAGE_TYPE_ANY_MODEM)
	{
		LOG_INF("Modem FOTA done, reinitializing the modem");

		err = conn_mgr_all_if_down(true);
		if (err)
		{
			LOG_ERR("conn_mgr_all_if_down, error: %d", err);
			// FATAL_ERROR();
			return;
		}

		err = conn_mgr_all_if_up(true);
		if (err)
		{
			LOG_ERR("conn_mgr_all_if_up, error: %d", err);
			// FATAL_ERROR();
			return;
		}
	}
	else if (evt->data.image & DFU_TARGET_IMAGE_TYPE_ANY_APPLICATION)
	{
		LOG_INF("Application FOTA done, rebooting");
		IF_ENABLED(CONFIG_REBOOT, (sys_reboot(0)));
	}
	else
	{
		LOG_WRN("Unexpected FOTA image type");
	}
}
void on_net_event_l4_connected(void)
{
	(void)k_work_reschedule(&connect_work, K_SECONDS(5));
}

void on_net_event_l4_disconnected(void)
{
	(void)aws_iot_disconnect();
	(void)k_work_cancel_delayable(&connect_work);
	(void)k_work_cancel_delayable(&shadow_update_work);
}

/* Event handlers */

static void aws_iot_event_handler(const struct aws_iot_evt *const evt)
{
	switch (evt->type)
	{
	case AWS_IOT_EVT_CONNECTING:
		LOG_INF("AWS_IOT_EVT_CONNECTING");
		break;
	case AWS_IOT_EVT_CONNECTED:
		LOG_INF("AWS_IOT_EVT_CONNECTED");
		LOG_INF("LED 2 IS HIGH\n");
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
		gpio_pin_set_dt(&led, 1);
		on_aws_iot_evt_connected(evt);
		break;
	case AWS_IOT_EVT_READY:
		LOG_INF("AWS_IOT_EVT_READY");
		break;
	case AWS_IOT_EVT_DISCONNECTED:
		LOG_INF("AWS_IOT_EVT_DISCONNECTED");
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
		gpio_pin_set_dt(&led, 0);
		on_aws_iot_evt_disconnected();
		break;
	case AWS_IOT_EVT_DATA_RECEIVED:
		LOG_INF("AWS_IOT_EVT_DATA_RECEIVED");

		LOG_INF("Received message: \"%.*s\" on topic: \"%.*s\"", evt->data.msg.len,
				evt->data.msg.ptr,
				evt->data.msg.topic.len,
				evt->data.msg.topic.str);
		break;
	case AWS_IOT_EVT_PUBACK:
		if (once)
		{
			once = false;
			return;
		}
		int rc;
		const struct device *flash_dev;
		flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
		if (!device_is_ready(flash_dev))
		{
			printk("%s: flash device device not ready.\n", flash_dev->name);
			return;
		}

		LOG_INF("AWS_IOT_EVT_PUBACK, message ID: %d", evt->data.message_id);
		readIndex += publishPayloadSize;
		printk("Read Index : %ld\n", readIndex);
		k_mutex_lock(&test_mutex, K_FOREVER);
		rc = flash_erase(flash_dev, CONFIG_FLASH_READ_ADD, SPI_FLASH_SECTOR_SIZE);
		if (rc != 0)
		{
			printk("Flash erase failed with error code %d\n", rc);
			return;
		}
		const int wr_len = sizeof(int);
		rc = flash_write(flash_dev, CONFIG_FLASH_READ_ADD, &readIndex, wr_len);
		if (rc == 0)
		{
			printk("Updated read_index : %d\n", readIndex);
		}
		else
			printk("Failed to update read_index\n");
		k_mutex_unlock(&test_mutex);
		filterVar = 0;
		break;
	case AWS_IOT_EVT_PINGRESP:
		LOG_INF("AWS_IOT_EVT_PINGRESP");
		break;
	case AWS_IOT_EVT_FOTA_START:
		LOG_INF("AWS_IOT_EVT_FOTA_START");
		break;
	case AWS_IOT_EVT_FOTA_ERASE_PENDING:
		LOG_INF("AWS_IOT_EVT_FOTA_ERASE_PENDING");
		break;
	case AWS_IOT_EVT_FOTA_ERASE_DONE:
		LOG_INF("AWS_FOTA_EVT_ERASE_DONE");
		break;
	case AWS_IOT_EVT_FOTA_DONE:
		LOG_INF("AWS_IOT_EVT_FOTA_DONE");
		on_aws_iot_evt_fota_done(evt);
		break;
	case AWS_IOT_EVT_FOTA_DL_PROGRESS:
		LOG_INF("AWS_IOT_EVT_FOTA_DL_PROGRESS, (%d%%)", evt->data.fota_progress);
		break;
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

int init_AWS(void)
{
	int err;

	err = aws_iot_client_init();
	if (err)
	{
		LOG_ERR("aws_iot_client_init, error: %d", err);
		FATAL_ERROR();
		return err;
	}

	return 0;
}
