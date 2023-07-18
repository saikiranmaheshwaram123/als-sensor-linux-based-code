#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>

#define APDS9960_REGMAP_NAME	"apds9960_regmap"
#define APDS9960_DRV_NAME	"apds9960"

#define APDS9960_REG_ALS_BASE	0x94
#define APDS9960_REG_ALS_CHANNEL(_colour) \
	(APDS9960_REG_ALS_BASE + (IDX_ALS_##_colour * 2))

enum apds9960_als_channel_idx {
	IDX_ALS_CLEAR, IDX_ALS_RED, IDX_ALS_GREEN, IDX_ALS_BLUE,
};

#define APDS9960_REG_ATIME	0x81

#define APDS9960_MAX_ALS_THRES_VAL	0xffff
#define APDS9960_MAX_INT_TIME_IN_US	1000000

static const struct regmap_range apds9960_readable_ranges[] = {
	regmap_reg_range(APDS9960_REG_ATIME, APDS9960_REG_ALS_BASE + 6),
};

static const struct regmap_access_table apds9960_readable_table = {
	.yes_ranges	= apds9960_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(apds9960_readable_ranges),
};

struct apds9960_data {
	struct i2c_client *client;
	struct iio_dev *indio_dev;
	struct mutex lock;
	struct regmap *regmap;
	int als_int;
	int als_gain;
	int als_adc_int_us;
};

static const struct reg_default apds9960_reg_defaults[] = {
	/* Default ALS integration time = 2.48ms */
	{ APDS9960_REG_ATIME, 0xff },
};

static const struct regmap_range apds9960_volatile_ranges[] = {
	regmap_reg_range(APDS9960_REG_ALS_BASE,
				APDS9960_REG_ALS_BASE + 2),
};

static const struct regmap_access_table apds9960_volatile_table = {
	.yes_ranges	= apds9960_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(apds9960_volatile_ranges),
};

static const struct regmap_range apds9960_precious_ranges[] = {
	regmap_reg_range(APDS9960_REG_ALS_BASE + 4,
				APDS9960_REG_ALS_BASE + 6),
};

static const struct regmap_access_table apds9960_precious_table = {
	.yes_ranges	= apds9960_precious_ranges,
	.n_yes_ranges	= ARRAY_SIZE(apds9960_precious_ranges),
};

static const struct regmap_range apds9960_readable_ranges[] = {
	regmap_reg_range(APDS9960_REG_ATIME, APDS9960_REG_ALS_BASE + 6),
};

static const struct regmap_access_table apds9960_readable_table = {
	.yes_ranges	= apds9960_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(apds9960_readable_ranges),
};

static const struct regmap_config apds9960_regmap_config = {
	.name = APDS9960_REGMAP_NAME,
	.reg_bits = 8,
	.val_bits = 8,
	.use_single_read = true,
	.use_single_write = true,

	.volatile_table = &apds9960_volatile_table,
	.precious_table = &apds9960_precious_table,
	.rd_table = &apds9960_readable_table,

	.reg_defaults = apds9960_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(apds9960_reg_defaults),
	.max_register = APDS9960_REG_ALS_BASE + 6,
	.cache_type = REGCACHE_RBTREE,
};

static const struct iio_chan_spec apds9960_channels[] = {
	/* ALS */
	{
		.type = IIO_INTENSITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_INT_TIME),
		.channel2 = IIO_MOD_LIGHT_CLEAR,
		.address = APDS9960_REG_ALS_CHANNEL(CLEAR),
		.modified = 1,
		.scan_index = -1,
	},
	/* RGB Sensor */
	{
		.type = IIO_INTENSITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_INT_TIME),
		.channel2 = IIO_MOD_LIGHT_RED,
		.address = APDS9960_REG_ALS_CHANNEL(RED),
		.modified = 1,
		.scan_index = -1,
	},
	{
		.type = IIO_INTENSITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_INT_TIME),
		.channel2 = IIO_MOD_LIGHT_GREEN,
		.address = APDS9960_REG_ALS_CHANNEL(GREEN),
		.modified = 1,
		.scan_index = -1,
	},
	{
		.type = IIO_INTENSITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_INT_TIME),
		.channel2 = IIO_MOD_LIGHT_BLUE,
		.address = APDS9960_REG_ALS_CHANNEL(BLUE),
		.modified = 1,
		.scan_index = -1,
	},
	/* End of channels */
	{},
};

static int apds9960_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	int ret;

	if (mask == IIO_CHAN_INFO_SCALE) {
		switch (chan->channel2) {
		case IIO_MOD_LIGHT_CLEAR:
		case IIO_MOD_LIGHT_RED:
		case IIO_MOD_LIGHT_GREEN:
		case IIO_MOD_LIGHT_BLUE:
			ret = iio_read_channel_ext_info(indio_dev, chan,
					chan->channel2, val, val2);
			if (ret)
				return ret;
			*val = 0;
			*val2 = 10000; /* 10000 lux, [lx] */
			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}

static int apds9960_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct apds9960_data *data = iio_priv(indio_dev);
	int reg;

	if (mask != IIO_CHAN_INFO_INT_TIME)
		return -EINVAL;

	/*
	 * Calculate ADC integration time in microseconds.
	 * To reduce rounding errors, we work with 64-bit integers.
	 */
	data->als_adc_int_us = 1000000LL * (256 - val) *
			       data->als_gain / 1000;

	if (data->als_adc_int_us < 1000)
		data->als_adc_int_us = 1000;
	else if (data->als_adc_int_us > APDS9960_MAX_INT_TIME_IN_US)
		data->als_adc_int_us = APDS9960_MAX_INT_TIME_IN_US;

	data->als_adc_int_us = clamp(data->als_adc_int_us, 1000LL,
				     APDS9960_MAX_INT_TIME_IN_US);

	switch (data->als_gain) {
	case 1:
		reg = 0;
		break;
	case 4:
		reg = 1;
		break;
	case 16:
		reg = 2;
		break;
	case 64:
		reg = 3;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(data->regmap, APDS9960_REG_ATIME,
				  0xff, 255 - val);
}

static irqreturn_t apds9960_als_irq_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct apds9960_data *data = iio_priv(indio_dev);

	iio_push_event(indio_dev, data->als_int,
		       iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static int apds9960_als_read_event_config(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir)
{
	struct apds9960_data *data = iio_priv(indio_dev);

	return regmap_read(data->regmap, APDS9960_REG_ALS_CHANNEL(CLEAR),
			   &data->als_int);
}

static int apds9960_als_write_event_config(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   enum iio_event_type type,
					   enum iio_event_direction dir,
					   int state)
{
	struct apds9960_data *data = iio_priv(indio_dev);

	return regmap_write(data->regmap, APDS9960_REG_ALS_CHANNEL(CLEAR),
			    state);
}

static int apds9960_als_buffer_postenable(struct iio_dev *indio_dev)
{
	return iio_triggered_buffer_postenable(indio_dev);
}

static int apds9960_als_buffer_predisable(struct iio_dev *indio_dev)
{
	return iio_triggered_buffer_predisable(indio_dev);
}

static const struct iio_buffer_setup_ops apds9960_buffer_setup_ops = {
	.postenable = &apds9960_als_buffer_postenable,
	.predisable = &apds9960_als_buffer_predisable,
};

static int apds9960_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct apds9960_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);

	data->regmap = devm_regmap_init_i2c(client, &apds9960_regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(&client->dev, "Failed to initialize register map: %d\n",
			ret);
		return ret;
	}

	ret = regmap_write(data->regmap, APDS9960_REG_ATIME, 0xff);
	if (ret) {
		dev_err(&client->dev, "Failed to write ATIME register: %d\n",
			ret);
		return ret;
	}

	ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev, NULL,
					      NULL,
					      &apds9960_buffer_setup_ops);
	if (ret) {
		dev_err(&client->dev, "Failed to setup buffer: %d\n", ret);
		return ret;
	}

	indio_dev->name = APDS9960_DRV_NAME;
	indio_dev->channels = apds9960_channels;
	indio_dev->num_channels = ARRAY_SIZE(apds9960_channels);
	indio_dev->info = &apds9960_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = apds9960_scan_masks;
	indio_dev->channels[0].ext_info = apds9960_intensity_ext_info;

	ret = iio_triggered_event_setup(indio_dev, 0,
					&apds9960_als_write_event_config,
					&apds9960_als_read_event_config);
	if (ret) {
		dev_err(&client->dev,
			"Failed to setup trigger event: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, apds9960_als_irq_handler,
					IRQF_TRIGGER_FALLING |
					IRQF_ONESHOT,
					APDS9960_DRV_NAME, indio_dev);
	if (ret) {
		dev_err(&client->dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	ret = pm_runtime_set_active(&client->dev);
	if (ret)
		return ret;

	pm_runtime_enable(&client->dev);

	ret = iio_device_register(indio_dev);
	if (ret) {
		pm_runtime_disable(&client->dev);
		pm_runtime_put_noidle(&client->dev);
	}

	return ret;
}

static int apds9960_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_triggered_event_cleanup(indio_dev);

	iio_device_unregister(indio_dev);
	pm_runtime_disable(&client->dev);
	pm_runtime_put_noidle(&client->dev);

	return 0;
}

static const struct acpi_device_id apds9960_acpi_match[] = {
	{ "APDS9960", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, apds9960_acpi_match);

static const struct i2c_device_id apds9960_id[] = {
	{ APDS9960_DRV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, apds9960_id);

#ifdef CONFIG_OF
static const struct of_device_id apds9960_of_match[] = {
	{ .compatible = "avago,apds9960", },
	{ }
};
MODULE_DEVICE_TABLE(of, apds9960_of_match);
#endif

static struct i2c_driver apds9960_driver = {
	.driver = {
		.name = APDS9960_DRV_NAME,
		.acpi_match_table = apds9960_acpi_match,
		.of_match_table = of_match_ptr(apds9960_of_match),
	},
	.probe = apds9960_probe,
	.remove = apds9960_remove,
	.id_table = apds9960_id,
};
module_i2c_driver(apds9960_driver);

MODULE_AUTHOR("Alexandre Belloni <alexandre.belloni@bootlin.com>");
MODULE_DESCRIPTION("APDS9960 ALS and RGB color sensor driver");
MODULE_LICENSE("GPL v2");

