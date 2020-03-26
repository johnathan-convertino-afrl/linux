// SPDX-License-Identifier: GPL-2.0
/*
 * ADM1266 - Cascadable Super Sequencer with Margin
 * Control and Fault Recording
 *
 * Copyright 2020 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "pmbus.h"

#define ADM1266_PDIO_CONFIG	0xD4
#define ADM1266_GPIO_CONFIG	0xE1
#define ADM1266_PDIO_STATUS	0xE9
#define ADM1266_GPIO_STATUS	0xEA

/* ADM1266 GPIO defines */
#define ADM1266_GPIO_NR			9
#define ADM1266_GPIO_FUNCTIONS(x)      FIELD_GET(BIT(0), x)
#define ADM1266_GPIO_INPUT_EN(x)       FIELD_GET(BIT(2), x)
#define ADM1266_GPIO_OUTPUT_EN(x)      FIELD_GET(BIT(3), x)
#define ADM1266_GPIO_OPEN_DRAIN(x)     FIELD_GET(BIT(4), x)

/* ADM1266 PDIO defines */
#define ADM1266_PDIO_NR			16
#define ADM1266_PDIO_PIN_CFG(x)		FIELD_GET(GENMASK(15, 13), x)
#define ADM1266_PDIO_GLITCH_FILT(x)	FIELD_GET(GENMASK(12, 9), x)
#define ADM1266_PDIO_OUT_CFG(x)		FIELD_GET(GENMASK(2, 0), x)

struct adm1266_data {
	struct pmbus_driver_info info;
	struct gpio_chip gc;
	char pin_names[ADM1266_PDIO_NR + ADM1266_GPIO_NR][7];
	struct i2c_client *client;
};

static const unsigned int adm1266_gpio_mapping[ADM1266_GPIO_NR][2] = {
	{1, 0},
	{2, 1},
	{3, 2},
	{4, 8},
	{5, 9},
	{6, 10},
	{7, 11},
	{8, 6},
	{9, 7},
};

static const char *adm1266_names[ADM1266_GPIO_NR + ADM1266_PDIO_NR] = {
	"GPIO1", "GPIO2", "GPIO3", "GPIO4", "GPIO5", "GPIO6", "GPIO7", "GPIO8",
	"GPIO9", "PDIO1", "PDIO2", "PDIO3", "PDIO4", "PDIO5", "PDIO6",
	"PDIO7", "PDIO8", "PDIO9", "PDIO10", "PDIO11", "PDIO12", "PDIO13",
	"PDIO14", "PDIO15", "PDIO16",
};

/* Block Write/Read command.
 * @client: Handle to slave device
 * @cmd: Byte interpreted by slave
 * @w_len: Size of write data block; SMBus allows at most 32 bytes
 * @data_w: byte array which will be written.
 * @data_r: Byte array into which data will be read; big enough to hold
 *	the data returned by the slave.  SMBus allows at most 32 bytes.
 *
 * Different from Block Read as it sends data and
 * waits for the slave to return a value dependent
 * on that data.  The protocol is simply a Write Word
 * followed by a Read Word without the Read-Word
 * command field and the Write-Word STOP bit.
 *
 * Returns number of bytes read or negative errno.
 */
static int adm1266_block_wr(struct i2c_client *client, u8 cmd, u8 w_len,
			    u8 *data_w, u8 *data_r)
{
	union i2c_smbus_data smbus_data;
	int ret;

	if (w_len > I2C_SMBUS_BLOCK_MAX)
		w_len = I2C_SMBUS_BLOCK_MAX;

	smbus_data.block[0] = w_len;
	memcpy(smbus_data.block + 1, data_w, w_len);

	ret = i2c_smbus_xfer(client->adapter, client->addr,
			     client->flags, I2C_SMBUS_READ, cmd,
			     I2C_SMBUS_BLOCK_PROC_CALL, &smbus_data);

	if (ret < 0)
		return ret;

	if (smbus_data.block[0] > I2C_SMBUS_BLOCK_MAX)
		return -EIO;

	memcpy(data_r, &smbus_data.block[1], smbus_data.block[0]);

	return smbus_data.block[0];
}

static int adm1266_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct adm1266_data *data = gpiochip_get_data(chip);
	u8 read_buf[I2C_SMBUS_BLOCK_MAX + 1];
	unsigned long pins_status;
	unsigned int pmbus_cmd;
	int ret;

	if (offset < ADM1266_GPIO_NR)
		pmbus_cmd = ADM1266_GPIO_STATUS;
	else
		pmbus_cmd = ADM1266_PDIO_STATUS;

	ret = i2c_smbus_read_block_data(data->client, pmbus_cmd,
					read_buf);
	if (ret < 0)
		return ret;

	pins_status = read_buf[0] + (read_buf[1] << 8);
	if (offset < ADM1266_GPIO_NR)
		return test_bit(adm1266_gpio_mapping[offset][1], &pins_status);
	else
		return test_bit(offset - ADM1266_GPIO_NR, &pins_status);
}

static int adm1266_gpio_get_multiple(struct gpio_chip *chip,
				     unsigned long *mask,
				     unsigned long *bits)
{
	struct adm1266_data *data = gpiochip_get_data(chip);
	u8 gpio_data[I2C_SMBUS_BLOCK_MAX + 1];
	u8 pdio_data[I2C_SMBUS_BLOCK_MAX + 1];
	unsigned long gpio_status;
	unsigned long pdio_status;
	unsigned int gpio_nr;
	int ret;

	ret = i2c_smbus_read_block_data(data->client, ADM1266_GPIO_STATUS,
					gpio_data);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_block_data(data->client, ADM1266_PDIO_STATUS,
					pdio_data);
	if (ret < 0)
		return ret;

	gpio_status = gpio_data[0] + (gpio_data[1] << 8);
	pdio_status = pdio_data[0] + (pdio_data[1] << 8);
	*bits = 0;
	for_each_set_bit(gpio_nr, mask, ADM1266_GPIO_NR) {
		if (test_bit(adm1266_gpio_mapping[gpio_nr][1], &gpio_status))
			set_bit(gpio_nr, bits);
	}

	for_each_set_bit_from(gpio_nr, mask,
			      ADM1266_GPIO_NR + ADM1266_PDIO_STATUS) {
		if (test_bit(gpio_nr - ADM1266_GPIO_NR, &pdio_status))
			set_bit(gpio_nr, bits);
	}

	return 0;
}

static void adm1266_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct adm1266_data *data = gpiochip_get_data(chip);
	u8 write_buf[I2C_SMBUS_BLOCK_MAX + 1];
	u8 read_buf[I2C_SMBUS_BLOCK_MAX + 1];
	unsigned long gpio_config;
	unsigned long pdio_config;
	unsigned long pin_cfg;
	int ret;
	int i;

	for (i = 0; i < ADM1266_GPIO_NR; i++) {
		write_buf[0] = adm1266_gpio_mapping[i][1];
		ret = adm1266_block_wr(data->client, ADM1266_GPIO_CONFIG, 1,
				       write_buf,
				       read_buf);
		if (ret < 0)
			dev_err(&data->client->dev, "GPIOs scan failed(%d).\n",
				ret);

		gpio_config = read_buf[0];
		seq_puts(s, adm1266_names[i]);

		seq_puts(s, " ( ");
		if (!ADM1266_GPIO_FUNCTIONS(gpio_config)) {
			seq_puts(s, "high-Z )\n");
			continue;
		}
		if (ADM1266_GPIO_INPUT_EN(gpio_config))
			seq_puts(s, "input ");
		if (ADM1266_GPIO_OUTPUT_EN(gpio_config))
			seq_puts(s, "output ");
		if (ADM1266_GPIO_OPEN_DRAIN(gpio_config))
			seq_puts(s, "open-drain )\n");
		else
			seq_puts(s, "push-pull )\n");
	}

	write_buf[0] = 0xFF;
	ret = adm1266_block_wr(data->client, ADM1266_PDIO_CONFIG, 1,
			       write_buf, read_buf);
	if (ret < 0)
		dev_err(&data->client->dev, "PDIOs scan failed(%d).\n", ret);

	for (i = 0; i < ADM1266_PDIO_NR; i++) {
		seq_puts(s, adm1266_names[ADM1266_GPIO_NR + i]);

		pdio_config = read_buf[2 * i];
		pdio_config += (read_buf[2 * i + 1] << 8);
		pin_cfg = ADM1266_PDIO_PIN_CFG(pdio_config);

		seq_puts(s, " ( ");
		if (!pin_cfg || pin_cfg > 5) {
			seq_puts(s, "high-Z )\n");
			continue;
		}

		if (pin_cfg & BIT(0))
			seq_puts(s, "output ");

		if (pin_cfg & BIT(1))
			seq_puts(s, "input ");

		seq_puts(s, ")\n");
	}
}

static int adm1266_config_gpio(struct adm1266_data *data)
{
	const char *name = dev_name(&data->client->dev);
	int ret;

	data->gc.label = name;
	data->gc.parent = &data->client->dev;
	data->gc.owner = THIS_MODULE;
	data->gc.base = -1;
	data->gc.names = adm1266_names;
	data->gc.ngpio = ADM1266_PDIO_NR + ADM1266_GPIO_NR;
	data->gc.get = adm1266_gpio_get;
	data->gc.get_multiple = adm1266_gpio_get_multiple;
	data->gc.dbg_show = adm1266_gpio_dbg_show;

	ret = devm_gpiochip_add_data(&data->client->dev, &data->gc, data);
	if (ret)
		dev_err(&data->client->dev, "GPIO registering failed (%d)\n",
			ret);

	return ret;
}

static int adm1266_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pmbus_driver_info *info;
	struct adm1266_data *data;
	u32 funcs;
	int ret;
	int i;

	data = devm_kzalloc(&client->dev, sizeof(struct adm1266_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	ret = adm1266_config_gpio(data);
	if (ret < 0)
		return ret;

	info = &data->info;
	info->pages = 17;
	info->format[PSC_VOLTAGE_OUT] = linear;
	funcs = PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT;
	for (i = 0; i < info->pages; i++)
		info->func[i] = funcs;

	return pmbus_do_probe(client, id, info);
}

static const struct of_device_id adm1266_of_match[] = {
	{ .compatible = "adi,adm1266" },
	{ },
};
MODULE_DEVICE_TABLE(of, adm1266_of_match);

static const struct i2c_device_id adm1266_id[] = {
	{ "adm1266", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, adm1266_id);

static struct i2c_driver adm1266_driver = {
	.driver = {
		   .name = "adm1266",
		   .of_match_table = adm1266_of_match,
		  },
	.probe = adm1266_probe,
	.remove = pmbus_do_remove,
	.id_table = adm1266_id,
};

module_i2c_driver(adm1266_driver);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("PMBus driver for Analog Devices ADM1266");
MODULE_LICENSE("GPL v2");
