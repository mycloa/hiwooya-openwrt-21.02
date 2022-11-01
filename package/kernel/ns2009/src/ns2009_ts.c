/*
 * drivers/input/touchscreen/ns2009.c
 *
 * Copyright (c) 2008 MtekVision Co., Ltd.
 *	Kwangwoo Lee <kwlee@mtekvision.com>
 *
 * Using code from:
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include "ns2009.h"


#define ns2009_MEASURE_TEMP0		(0x0 << 4)
#define ns2009_MEASURE_AUX		(0x2 << 4)
#define ns2009_MEASURE_TEMP1		(0x4 << 4)
#define ns2009_ACTIVATE_XN		(0x8 << 4)
#define ns2009_ACTIVATE_YN		(0x9 << 4)
#define ns2009_ACTIVATE_YP_XN		(0xa << 4)
#define ns2009_SETUP			(0xb << 4)
#define ns2009_MEASURE_X		(0xc << 4)
#define ns2009_MEASURE_Y		(0xd << 4)
#define ns2009_MEASURE_Z1		(0xe << 4)
#define ns2009_MEASURE_Z2		(0xf << 4)

#define ns2009_POWER_OFF_IRQ_EN	(0x0 << 2)
#define ns2009_ADC_ON_IRQ_DIS0		(0x1 << 2)
#define ns2009_ADC_OFF_IRQ_EN		(0x2 << 2)
#define ns2009_ADC_ON_IRQ_DIS1		(0x3 << 2)

#define ns2009_12BIT			(0x0 << 1)
#define ns2009_8BIT			(0x1 << 1)

#define	MAX_12BIT			((1 << 12) - 1)

#define ADC_ON_12BIT	(ns2009_12BIT | ns2009_ADC_ON_IRQ_DIS0)

#define READ_Y		(ADC_ON_12BIT | ns2009_MEASURE_Y)
#define READ_Z1		(ADC_ON_12BIT | ns2009_MEASURE_Z1)
#define READ_Z2		(ADC_ON_12BIT | ns2009_MEASURE_Z2)
#define READ_X		(ADC_ON_12BIT | ns2009_MEASURE_X)
#define PWRDOWN		(ns2009_12BIT | ns2009_POWER_OFF_IRQ_EN)

struct ts_event {
	u16	x;
	u16	y;
	u16	z1, z2;
};

struct ns2009 {
	struct input_dev	*input;
	char			phys[32];

	struct i2c_client	*client;

	u16			model;
	u16			x_plate_ohms;
	u16			max_rt;
	unsigned long		poll_period;
	int			fuzzx;
	int			fuzzy;
	int			fuzzz;

	unsigned		gpio;
	int			irq;

	wait_queue_head_t	wait;
	bool			stopped;

	int			(*get_pendown_state)(struct device *);
	void			(*clear_penirq)(void);
};

static inline int ns2009_xfer(struct ns2009 *tsc, u8 cmd)
{
	s32 data;
	u16 val;

	data = i2c_smbus_read_word_data(tsc->client, cmd);
	if (data < 0) {
		dev_err(&tsc->client->dev, "i2c io error: %d\n", data);
		return data;
	}

	/* The protocol and raw data format from i2c interface:
	 * S Addr Wr [A] Comm [A] S Addr Rd [A] [DataLow] A [DataHigh] NA P
	 * Where DataLow has [D11-D4], DataHigh has [D3-D0 << 4 | Dummy 4bit].
	 */
	val = swab16(data) >> 4;

	dev_dbg(&tsc->client->dev, "data: 0x%x, val: 0x%x\n", data, val);
	//printk("data: 0x%x, val: 0x%x\n", data, val);

	return val;
}

static void ns2009_read_values(struct ns2009 *tsc, struct ts_event *tc)
{
	/* y- still on; turn on only y+ (and ADC) */
	tc->y = ns2009_xfer(tsc, READ_Y);

	/* turn y- off, x+ on, then leave in lowpower */
	tc->x = ns2009_xfer(tsc, READ_X);

	/* turn y+ off, x- on; we'll use formula #1 */
	tc->z1 = ns2009_xfer(tsc, READ_Z1);
	tc->z2 = ns2009_xfer(tsc, READ_Z2);

	/* Prepare for next touch reading - power down ADC, enable PENIRQ */
	ns2009_xfer(tsc, PWRDOWN);
}

static u32 ns2009_calculate_pressure(struct ns2009 *tsc, struct ts_event *tc)
{
	u32 rt = 0;

	/* range filtering */
	if (tc->x == MAX_12BIT)
		tc->x = 0;

	if (likely(tc->x && tc->z1)) {
		/* compute touch pressure resistance using equation #1 */
		rt = tc->z2 - tc->z1;
		rt *= tc->x;
		rt *= tsc->x_plate_ohms;
		rt /= tc->z1;
		rt = (rt + 2047) >> 12;
	}

	return rt;
}

static bool ns2009_is_pen_down(struct ns2009 *ts)
{
	/*
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we assume that the pen is down and expect caller
	 * to fall back on the pressure reading.
	 */

	if (!ts->get_pendown_state)
		return true;

	return ts->get_pendown_state(&ts->client->dev);
}

static irqreturn_t ns2009_soft_irq(int irq, void *handle)
{
	struct ns2009 *ts = handle;
	struct input_dev *input = ts->input;
	struct ts_event tc;
	u32 rt;

	dev_dbg(&ts->client->dev, "%s: Entering\n", __func__);	
	printk("%s: Entering\n", __func__);	

	while (!ts->stopped && ns2009_is_pen_down(ts)) {

		/* pen is down, continue with the measurement */
		ns2009_read_values(ts, &tc);

		rt = ns2009_calculate_pressure(ts, &tc);

		if (!rt && !ts->get_pendown_state) {
			/*
			 * If pressure reported is 0 and we don't have
			 * callback to check pendown state, we have to
			 * assume that pen was lifted up.
			 */
			break;
		}

		if (rt <= ts->max_rt) {
			dev_dbg(&ts->client->dev,
				"DOWN point(%4d,%4d), pressure (%4u)\n",
				tc.x, tc.y, rt);

			input_report_key(input, BTN_TOUCH, 1);
			input_report_abs(input, ABS_X, tc.x);
			input_report_abs(input, ABS_Y, tc.y);
			input_report_abs(input, ABS_PRESSURE, rt);

			input_sync(input);

		} else {
			/*
			 * Sample found inconsistent by debouncing or pressure is
			 * beyond the maximum. Don't report it to user space,
			 * repeat at least once more the measurement.
			 */
			dev_dbg(&ts->client->dev,	\
				"ignored pressure %d\n", rt);
		}

		wait_event_timeout(ts->wait, ts->stopped,	\
				   msecs_to_jiffies(ts->poll_period));
	}

	dev_dbg(&ts->client->dev,
		"UP\n");

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);

	if (ts->clear_penirq)
		ts->clear_penirq();

	/* Now that the touch has been handled, re-enabled the PENIRQ */
	enable_irq(ts->irq);	

	return IRQ_HANDLED;
}

static irqreturn_t ns2009_hard_irq(int irq, void *handle)
{
	struct ns2009 *ts = handle;
	
	dev_dbg(&ts->client->dev,	\
		"%s: Entering\n", __func__);
	
	if (ns2009_is_pen_down(ts))	
	{
		/* Once the IRQ handling starts, disable the interrupt */	
		disable_irq_nosync(ts->irq);
		dev_dbg(&ts->client->dev, "%s: Pen down returns true\n", __func__);				
		return IRQ_WAKE_THREAD;
	}

	if (ts->clear_penirq)
	{
		dev_dbg(&ts->client->dev, "%s: Clear pending interrupt", __func__);
		ts->clear_penirq();
	}

	return IRQ_HANDLED;
}

static void ns2009_stop(struct ns2009 *ts)
{
	ts->stopped = true;
	mb();
	wake_up(&ts->wait);

	disable_irq(ts->irq);
}

static int ns2009_open(struct input_dev *input_dev)
{
	struct ns2009 *ts = input_get_drvdata(input_dev);
	int err;

	ts->stopped = false;
	mb();

	enable_irq(ts->irq);

	/* Prepare for touch readings - power down ADC and enable PENIRQ */
	err = ns2009_xfer(ts, PWRDOWN);
	if (err < 0) {
		ns2009_stop(ts);
		return err;
	}

	return 0;
}

static void ns2009_close(struct input_dev *input_dev)
{
	struct ns2009 *ts = input_get_drvdata(input_dev);

	ns2009_stop(ts);
}

#ifdef CONFIG_OF
static int ns2009_get_pendown_state_gpio(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ns2009 *ts = i2c_get_clientdata(client);

	return !gpio_get_value(ts->gpio);
}

static int ns2009_probe_dt(struct i2c_client *client, struct ns2009 *ts)
{
	struct device_node *np = client->dev.of_node;
	u32 val32;
	u64 val64;

	if (!np) {
		dev_err(&client->dev, "missing device tree data\n");
		return -EINVAL;
	}

	if (!of_property_read_u32(np, "ti,max-rt", &val32))
		ts->max_rt = val32;
	else
		ts->max_rt = MAX_12BIT;

	if (!of_property_read_u32(np, "ti,fuzzx", &val32))
		ts->fuzzx = val32;

	if (!of_property_read_u32(np, "ti,fuzzy", &val32))
		ts->fuzzy = val32;

	if (!of_property_read_u32(np, "ti,fuzzz", &val32))
		ts->fuzzz = val32;

	if (!of_property_read_u64(np, "ti,poll-period", &val64))
		ts->poll_period = val64;
	else
		ts->poll_period = 1;

	if (!of_property_read_u32(np, "ti,x-plate-ohms", &val32)) {
		ts->x_plate_ohms = val32;
	} else {
		dev_err(&client->dev, "missing ti,x-plate-ohms devicetree property.");
		return -EINVAL;
	}

	ts->gpio = of_get_gpio(np, 0);

	if (gpio_is_valid(ts->gpio)){
		ts->get_pendown_state = ns2009_get_pendown_state_gpio;

		/* If GPIO is valid, then request the GPIO for use */
		//dev_dbg(&ts->client->dev,
		printk(
			"%s: Requesting GPIO\n", __func__);
		
		if (gpio_request(ts->gpio, "nPENIRQ"))
		{
			//dev_dbg(&ts->client->dev,
			printk(
				"%s: Requesting GPIO failed", __func__);
			return 0;
		}
		
		/* Set the direction to be input - may not be required */
		//dev_dbg(&ts->client->dev,
		printk(
			"%s: Setting direction for GPIO\n", __func__);
		if (gpio_direction_input(ts->gpio))
		{
			gpio_free(ts->gpio);
			return 0;
		}
		
		/* Set the debounce limit/value for the GPIO 
		//SR_DEBUG(&client->dev, "%s: Setting debounce for GPIO", __func__);
		if (gpio_set_debounce(ts->gpio, 50000))
		{
			gpio_free(ts->gpio);
			return 0;
		}*/

	}
	else
		dev_warn(&client->dev,
			 "GPIO not specified in DT (of_get_gpio returned %d)\n",
			 ts->gpio);
	
	printk("GPIO set is ok\n");
	return 0;
}
#else
static int ns2009_probe_dt(struct i2c_client *client, struct ns2009 *ts)
{
	dev_err(&client->dev, "platform data is required!\n");
	return -EINVAL;
}
#endif

static int ns2009_probe_pdev(struct i2c_client *client, struct ns2009 *ts,
			      const struct ns2009_platform_data *pdata,
			      const struct i2c_device_id *id)
{
	ts->model             = pdata->model;
	ts->x_plate_ohms      = pdata->x_plate_ohms;
	ts->max_rt            = pdata->max_rt ? : MAX_12BIT;
	ts->poll_period       = pdata->poll_period ? : 1;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;
	ts->fuzzx             = pdata->fuzzx;
	ts->fuzzy             = pdata->fuzzy;
	ts->fuzzz             = pdata->fuzzz;

	if (pdata->x_plate_ohms == 0) {
		dev_err(&client->dev, "x_plate_ohms is not set up in platform data");
		return -EINVAL;
	}

	return 0;
}

static void ns2009_call_exit_platform_hw(void *data)
{
	struct device *dev = data;
	const struct ns2009_platform_data *pdata = dev_get_platdata(dev);

	pdata->exit_platform_hw();
}

static int ns2009_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	const struct ns2009_platform_data *pdata = dev_get_platdata(&client->dev);
	struct ns2009 *ts;
	struct input_dev *input_dev;
	int err;


	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = devm_kzalloc(&client->dev, sizeof(struct ns2009), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	if (pdata)
		err = ns2009_probe_pdev(client, ts, pdata, id);
	else
		err = ns2009_probe_dt(client, ts);
	if (err)
		return err;

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev)
		return -ENOMEM;

	i2c_set_clientdata(client, ts);

	ts->client = client;

	client->irq = gpio_to_irq(ts->gpio);
	ts->irq = client->irq;
	ts->input = input_dev;
	printk("%s: IRQ number is %d", __func__, ts->irq);

	init_waitqueue_head(&ts->wait);

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "ns2009 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->open = ns2009_open;
	input_dev->close = ns2009_close;

	input_set_drvdata(input_dev, ts);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, ts->fuzzx, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, ts->fuzzy, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT,
			     ts->fuzzz, 0);

	if (pdata) {
		if (pdata->exit_platform_hw) {
			err = devm_add_action(&client->dev,
					      ns2009_call_exit_platform_hw,
					      &client->dev);
			if (err) {
				dev_err(&client->dev,
					"Failed to register exit_platform_hw action, %d\n",
					err);
				return err;
			}
		}

		if (pdata->init_platform_hw)
			pdata->init_platform_hw();
	}

	err = devm_request_threaded_irq(&client->dev, ts->irq,
					ns2009_hard_irq, ns2009_soft_irq,
					IRQF_TRIGGER_FALLING/*IRQF_ONESHOT*/,
					client->dev.driver->name, ts);
	if (err) {
		dev_err(&client->dev, "Failed to request irq %d: %d\n",
			ts->irq, err);
		return err;
	}

	ns2009_stop(ts);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"Failed to register input device: %d\n", err);
		return err;
	}

	return 0;
}

static const struct i2c_device_id ns2009_idtable[] = {
	{ "NS2009,TOUCHSCREEN", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ns2009_idtable);

#ifdef CONFIG_OF
static const struct of_device_id ns2009_of_match[] = {
	{ .compatible = "NS2009,TOUCHSCREEN" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ns2009_of_match);
#endif

static struct i2c_driver ns2009_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ns2009_ts",
		.of_match_table = of_match_ptr(ns2009_of_match),
	},
	.id_table	= ns2009_idtable,
	.probe		= ns2009_probe,
};

module_i2c_driver(ns2009_driver);

MODULE_AUTHOR("Kwangwoo Lee <kwlee@mtekvision.com>");
MODULE_DESCRIPTION("ns2009 TouchScreen Driver");
MODULE_LICENSE("GPL");
