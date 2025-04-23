#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <stdio.h>
#include "observer.h" // Necessary for sensor-related operations
#include "aws.h"      // Required for AWS integration
#include "gsm.h"      // GSM communication

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Defining the mutex as to lock the flash while reading and writing and then unlocking it. */
K_MUTEX_DEFINE(test_mutex);

#define STACKSIZE 4096
#define THREAD0_PRIORITY 7
#define SPI_FLASH_SECTOR_SIZE 4096
static void sntp_check(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(connect_work, sntp_check);
/* Index writeIndex to write data in flash */
extern int flashLimit, writeIndex, readIndex;
extern bool memFilled;
int epochArr[10] = {0};
int epochTime[10] = {0};
int filterVar = 0;
uint8_t rssi;

static void sntp_check(struct k_work *work)
{

    fetch_time_from_sntp(epochArr);

    for (int epochIdx = 0; epochIdx < 10; epochIdx++)
    {
        epochTime[epochIdx] = (uint8_t)epochArr[epochIdx];
    }

    /* Reschedule the work for periodic checking */
    k_work_reschedule(&connect_work, K_SECONDS(2));
}

#if defined(CONFIG_BT_EXT_ADV)
static bool data_cb(struct bt_data *data, void *user_data)
{

    uint8_t writeArray[CONFIG_BLE_RECEIVING_PAYLOAD];

    /*making the write_array zero*/
    for (int zeroIdx = 0; zeroIdx < sizeof(writeArray); zeroIdx++)
    {
        writeArray[zeroIdx] = 0;
    }

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    gpio_pin_toggle_dt(&led0);

    /* data_cb is executed whenever some advertising packet comes. */
    printk("IN DATA CB");

    switch (data->type)
    {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
    case BT_DATA_MANUFACTURER_DATA:

        if (filterVar == data->data[0])
        {
            return;
        }
        else
        {

            filterVar = data->data[0];
        }

        printk("VALUE OF writeIndex  = %x \n", writeIndex);

        const struct device *flash_dev;
        flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));

        // Storing chunk_size BYTES OF DATA AT ONCE //

        printk("\n %u \n", data->data_len);
        for (int dataIdx = 0; dataIdx < data->data_len; dataIdx++)
        {
            uint8_t data_holding_byte = data->data[dataIdx];
            // printk("%u ", data_holding_byte);
            writeArray[dataIdx] = data_holding_byte;
        }

        for (int dataIdx = data->data_len; dataIdx < CONFIG_BLE_RECEIVING_PAYLOAD; dataIdx++)
        {
            writeArray[dataIdx] = 0;
        }

        writeArray[CONFIG_BLE_RECEIVING_PAYLOAD - 11] = CONFIG_DEVICE_ID; /// device id of gateway
        writeArray[CONFIG_BLE_RECEIVING_PAYLOAD - 12] = CONFIG_GROUP_ID; /// device id of gateway
        writeArray[CONFIG_BLE_RECEIVING_PAYLOAD - 13] = rssi;             // range value

        /*Appending time at the end of the data_array*/
        for (int epochIdx = 0; epochIdx < 10; epochIdx++)
        {
            writeArray[CONFIG_BLE_RECEIVING_PAYLOAD + epochIdx - 10] = (uint8_t)epochTime[epochIdx];
        }
        /* leng is the length of the write array. */
        const size_t arrayLen = CONFIG_BLE_RECEIVING_PAYLOAD;

        k_mutex_lock(&test_mutex, K_FOREVER);

        printk("LOCK BEFORE WRITING!! \n");

        /* Erasing the flash as per erase sector size*/
        if (writeIndex % SPI_FLASH_SECTOR_SIZE == 0)
        {
            int ret = flash_erase(flash_dev, writeIndex, SPI_FLASH_SECTOR_SIZE);
            if (ret == 0)
            {
                printk("Erased flash sector %d to %d \n", writeIndex, writeIndex + SPI_FLASH_SECTOR_SIZE - 1);
            }
        }

        /* Writing in flash. */
        int rc = flash_write(flash_dev, writeIndex, writeArray, arrayLen);
        if (rc != 0)
        {
            printf("Flash write failed! %d\n", rc);
            return;
        }
        else
        {
            /* Incrementing index to write */
            writeIndex = writeIndex + CONFIG_BLE_RECEIVING_PAYLOAD;
            printf("write successful \n");
        }
        for (int dataIdx = 0; dataIdx < CONFIG_BLE_RECEIVING_PAYLOAD; dataIdx++)
        {

            printk("%u ", writeArray[dataIdx]);
        }

        // === UNLOCKING IN write === //
        k_mutex_unlock(&test_mutex);

        printk("UNLOCK AFTER WRITING!! \n");

        if (writeIndex >= flashLimit)
        {
            writeIndex = 0x102000;
            memFilled = true;
        }

        k_mutex_lock(&test_mutex, K_FOREVER);
        rc = flash_erase(flash_dev, CONFIG_FLASH_WRITE_ADD, SPI_FLASH_SECTOR_SIZE);
        if (rc != 0)
        {
            printk("Flash erase failed with error code %d\n", rc);
            return;
        }
        const int wr_len = sizeof(int);
        rc = flash_write(flash_dev, CONFIG_FLASH_WRITE_ADD, &writeIndex, wr_len);
        if (rc == 0)
        {
            printk("Updated writeIndex : %d\n", writeIndex);
        }
        else
            printk("Failed to update writeIndex\n");
        k_mutex_unlock(&test_mutex);

        return false;
    default:
        return true;
    }
}
static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
    char le_addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
    printk("Received packet from %s\n", le_addr);

    rssi = -(info->rssi);
    printk("%d\n", rssi);

    bt_data_parse(buf, data_cb, NULL);
}

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
};
#endif /* CONFIG_BT_EXT_ADV */

int observer_start(void)
{

    bt_addr_le_t addr1;

    bt_addr_le_from_str(CONFIG_BLE_ADD, "random", &addr1);

    int err = bt_le_filter_accept_list_add(&addr1);
    if (err == 0)
    {
        printk("Successfully added the address to Filter Accept List..\n");
    }
    else
    {
        printk("Failed to update Filter Accept List..\n");
    }

    struct bt_le_scan_param scan_param = {
        .type = BT_LE_SCAN_TYPE_PASSIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE | BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST,
        .interval = 0x0030,
        .window = 0x0030,
    };

    bt_le_scan_cb_register(&scan_callbacks);
    err = bt_le_scan_start(&scan_param, NULL);

    if (err)
    {
        printk("Start scanning failed (err %d)\n", err);
        return err;
    }
    printk("Started scanning...\n");
    return 0;
}

int init_observer(void)
{
    /* Observer Initialization. */
    printk("observer Started");

    /* Enabling Bluetooth. */
    int err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }
    k_work_reschedule(&connect_work, K_SECONDS(2));

    return 0;
}

/* Defining the thread for observer. It will thus have the locking and unlocking property to mutex.
    It will start after 2500ms. */
K_THREAD_DEFINE(thread0_id, STACKSIZE, observer_start, NULL, NULL, NULL,
                THREAD0_PRIORITY, 0, 30000);
