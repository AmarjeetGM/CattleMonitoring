#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include "gsm.h"
#include "aws.h"
#include "observer.h"

#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

int readIndex = 0x102000;
int writeIndex = 0x102000;
int flashLimit = 0x800000;
bool memFilled = false;

void gsm_enable()
{
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	gpio_pin_set_dt(&led1, 0);
	K_MSEC(300);
	gpio_pin_set_dt(&led1, 1);
	K_MSEC(100);
}

void main()
{
	int rc;
	int temp_read_index, temp_write_index;
	const struct device *flash_dev;
	flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	int len = sizeof(int);
	if (!device_is_ready(flash_dev))
	{
		printk("%s: flash device device not ready.\n", flash_dev->name);
		return;
	}
	rc = flash_read(flash_dev, CONFIG_FLASH_READ_ADD, &temp_read_index, len);
	if (rc == 0)
	{
		printk("Successfully read Read_index : %d\n", temp_read_index);
		if (temp_read_index > 0x102000 && temp_read_index < flashLimit)
		{
			readIndex = temp_read_index;
		}
		else
		{

			printk("It does not have stored read memory\n");
		}
	}
	else
	{

		printk("Failed to read Read_index\n");
	}
	rc = flash_read(flash_dev, CONFIG_FLASH_WRITE_ADD, &temp_write_index, len);
	if (rc == 0)
	{
		printk("Successfully read write_index : %d\n", temp_write_index);
		if (temp_write_index > 0x102000 && temp_write_index < flashLimit)
		{
			writeIndex = temp_write_index;
		}
		else
		{

			printk("It does not have stored write memory\n");
		}
	}
	else
	{

		printk("Failed to read write_index\n");
	}
	if (writeIndex < readIndex)
	{
		readIndex = 0x102000;
		writeIndex = 0x102000;
		printk("Read & write resets\n");
	}
	else
	{
		// nothing to do
	}
	printk("\nFIRMWARE VERSION: %s\n", CONFIG_AWS_IOT_SAMPLE_APP_VERSION);
	/*Enabling gsm*/
	gsm_enable();
	/*Initializing gsm AT commands function*/
	init_gsm();
	/*sensor initialization*/
	init_observer();
	/*Initializing AWS function*/
	init_AWS();
}