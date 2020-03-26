/*
 * rotary_encoder.c
 *
 * A single level trigger interrupt driver for rotary encoders connected to cheetah GPIO lines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/rotary_encoder.h>

#include <asm/mach-cheetah/cheetah.h>

#define DRV_NAME "cheetah-rotary-encoder"

#define STATE_LOW   0
#define STATE_HIGH  1

//#define IGNORE_ROTARY_ENCODER_NOISE    2

struct rotary_encoder {
	struct input_dev *input;
	struct rotary_encoder_platform_data *pdata;
	unsigned int axis;
	unsigned int irq_a;
	int state_a;
	int state_b;
};

static irqreturn_t rotary_encoder_irq(int irq, void *dev_id)
{
	struct rotary_encoder *encoder = dev_id;
	struct rotary_encoder_platform_data *pdata = encoder->pdata;
	unsigned int v = GPREG(GPVAL);
	int a = !!(v & 1<<(pdata->gpio_a));
	int b = !!(v & 1<<(pdata->gpio_b));
	int event = 0;
#ifdef IGNORE_ROTARY_ENCODER_NOISE
	static unsigned long next = 0;
	unsigned long now = 0;
#endif

	//sent event at falling edge
	if ((encoder->state_a != a) && (a == STATE_HIGH) && (encoder->state_b != b)) {
		b = encoder->state_b;
		event = 1;
	}

	//update state
	if ((encoder->state_a != a) && (a == STATE_LOW))
		encoder->state_b = b;
	encoder->state_a = a;

	//update trigger level
	set_irq_type(IRQ_GPIOX, (a) ? IRQ_TYPE_LEVEL_LOW : IRQ_TYPE_LEVEL_HIGH);

#ifdef IGNORE_ROTARY_ENCODER_NOISE
	/* don't care about jiffies overflow */
	now = jiffies;
	if ((now + IGNORE_ROTARY_ENCODER_NOISE) < next)
		next = now + IGNORE_ROTARY_ENCODER_NOISE;
	if ((event) && (now > next)) {
		next = now + IGNORE_ROTARY_ENCODER_NOISE;
#else
	if (event) {
#endif
		input_report_rel(encoder->input, pdata->axis, b ? -1 : 1);
		input_sync(encoder->input);
	}

	return IRQ_HANDLED;
}

static int __devinit rotary_encoder_probe(struct platform_device *pdev)
{
	struct rotary_encoder_platform_data *pdata = pdev->dev.platform_data;
	struct rotary_encoder *encoder;
	struct input_dev *input;
	int err;
	unsigned int v = GPREG(GPVAL);

	if (!pdata) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -ENOENT;
	}

	encoder = kzalloc(sizeof(struct rotary_encoder), GFP_KERNEL);
	input = input_allocate_device();
	if (!encoder || !input) {
		dev_err(&pdev->dev, "failed to allocate memory for device\n");
		err = -ENOMEM;
		goto exit_free_mem;
	}

	encoder->input = input;
	encoder->pdata = pdata;
	encoder->irq_a = gpio_to_irq(pdata->gpio_a);
	encoder->state_a = !!(v & 1<<(pdata->gpio_a));
	encoder->state_b = !!(v & 1<<(pdata->gpio_b));
	printk("encoder->irq_a=%d\n",encoder->irq_a);

	/* create and register the input driver */
	input->name = pdev->name;
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;

	if (pdata->relative_axis) {
		input->evbit[0] = BIT_MASK(EV_REL);
		input->relbit[0] = BIT_MASK(pdata->axis);
	} else {
		err = -EINVAL;
		goto exit_free_mem;
	}

	err = input_register_device(input);
	if (err) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto exit_free_mem;
	}

	/* request the GPIOs */
	err = gpio_request(pdata->gpio_a, DRV_NAME);
	if (err) {
		dev_err(&pdev->dev, "unable to request GPIO %d\n",
			pdata->gpio_a);
		goto exit_unregister_input;
	}
	gpio_direction_input(pdata->gpio_a);

	err = gpio_request(pdata->gpio_b, DRV_NAME);
	if (err) {
		dev_err(&pdev->dev, "unable to request GPIO %d\n",
			pdata->gpio_b);
		goto exit_free_gpio_a;
	}
	gpio_direction_input(pdata->gpio_b);

	/* request the IRQs */
	err = request_irq(IRQ_GPIOX, &rotary_encoder_irq,
			  encoder->state_a ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH,
			  DRV_NAME, encoder);
	if (err) {
		dev_err(&pdev->dev, "unable to request IRQ %d\n",
			IRQ_GPIOX);
		goto exit_free_gpio_b;
	}

	platform_set_drvdata(pdev, encoder);

	return 0;

exit_free_gpio_b:
	gpio_free(pdata->gpio_b);
exit_free_gpio_a:
	gpio_free(pdata->gpio_a);
exit_unregister_input:
	input_unregister_device(input);
	input = NULL; /* so we don't try to free it */
exit_free_mem:
	input_free_device(input);
	kfree(encoder);
	return err;
}

static int __devexit rotary_encoder_remove(struct platform_device *pdev)
{
	struct rotary_encoder *encoder = platform_get_drvdata(pdev);
	struct rotary_encoder_platform_data *pdata = pdev->dev.platform_data;

	free_irq(IRQ_GPIOX, encoder);
	gpio_free(pdata->gpio_a);
	gpio_free(pdata->gpio_b);
	input_unregister_device(encoder->input);
	platform_set_drvdata(pdev, NULL);
	kfree(encoder);

	return 0;
}

static struct platform_driver rotary_encoder_driver = {
	.probe		= rotary_encoder_probe,
	.remove		= __devexit_p(rotary_encoder_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	}
};

static int __init rotary_encoder_init(void)
{
	return platform_driver_register(&rotary_encoder_driver);
}

static void __exit rotary_encoder_exit(void)
{
	platform_driver_unregister(&rotary_encoder_driver);
}

module_init(rotary_encoder_init);
module_exit(rotary_encoder_exit);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DESCRIPTION("cheetah GPIO rotary encoder driver");
MODULE_AUTHOR("Daniel Mack <daniel@caiaq.de>");
MODULE_LICENSE("GPL v2");

