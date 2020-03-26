
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <asm/mach-cheetah/cta_keypad.h>
#include <asm/mach-cheetah/cheetah.h>

#define RAKEY			"/lib/wdk/rakey"
#define RAKEY_PATH		"PATH=/usr/bin:/bin:/sbin:/lib/wdk"
/*
gpio	row
22		2	-----1-----2-----3
23		1	-----4-----5-----6
13		0	-----7-----8-----9
				 |	   |	 |
			col	 0	   1	 2
		gpio	19	  20	21
*/

static char *keyaction[] =
{
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "forward",
    "backward",
    "switch",
};

unsigned short **keycodes;

struct matrix_keypad {
	const struct cta_keypad_platform_data *pdata;
	unsigned int row_shift;

	uint32_t last_key_state[MATRIX_MAX_COLS];
	struct delayed_work work;
	bool scan_pending;
	bool stopped;
};

static inline void
cta_keypad_build_keymap(const struct cta_keymap_data *keymap_data,
               unsigned int row_shift,
               unsigned short **keymap)
{
    int i;

    for (i = 0; i < keymap_data->keymap_size; i++) {
        unsigned int key = keymap_data->keymap[i];
        unsigned int row = KEY_ROW(key);
        unsigned int col = KEY_COL(key);
        unsigned short code = KEY_VAL(key);
		
		keymap[row][col] = code;
    }
}

/*
 * NOTE: normally the GPIO has to be put into HiZ when de-activated to cause
 * minmal side effect when scanning other columns, here it is configured to
 * be input, and it should work on most platforms.
 */
static void __activate_col(const struct cta_keypad_platform_data *pdata,
			   int col, bool on)
{
	bool level_on = !pdata->active_low;

	if (on) {
		gpio_direction_output(pdata->col_gpios[col], level_on);
	} else {
		gpio_set_value_cansleep(pdata->col_gpios[col], !level_on);
		gpio_direction_input(pdata->col_gpios[col]);
	}
}

static void activate_col(const struct cta_keypad_platform_data *pdata,
			 int col, bool on)
{
	__activate_col(pdata, col, on);

	if (on && pdata->col_scan_delay_us)
		udelay(pdata->col_scan_delay_us);
}

static void activate_all_cols(const struct cta_keypad_platform_data *pdata,
			      bool on)
{
	int col;

	for (col = 0; col < pdata->num_col_gpios; col++)
		__activate_col(pdata, col, on);
}

static bool row_asserted(const struct cta_keypad_platform_data *pdata,
			 int row)
{
	return gpio_get_value_cansleep(pdata->row_gpios[row]) ?
			!pdata->active_low : pdata->active_low;
}

//static void keypad_create_event(int key)
static void keypad_create_event(char *action)
{
	char *argv[3];
	char *envp[2];
	int ret;
	
	argv[0] = RAKEY;
	argv[1] = action;
	argv[2] = NULL;

	/*env*/
	envp[0]	= RAKEY_PATH; 
	envp[1]	= NULL; 
	//printk("!!! argv[0]:%s argv[1]:%s envp[0]:%s\n",argv[0], argv[1], envp[0]);	
	
	ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
	return;

}

static void matrix_keypad_scan(struct work_struct *work)
{
	struct matrix_keypad *keypad =
		container_of(work, struct matrix_keypad, work.work);
	const struct cta_keypad_platform_data *pdata = keypad->pdata;
	uint32_t new_state[MATRIX_MAX_COLS];
	int row, col, key, delay=10;

	/* de-activate all columns for scanning */
	activate_all_cols(pdata, false);

	memset(new_state, 0, sizeof(new_state));

	/* assert each column and read the row status out */
	for (col = 0; col < pdata->num_col_gpios; col++) {

		activate_col(pdata, col, true);

		for (row = 0; row < pdata->num_row_gpios; row++)
		{
			new_state[col] |=
				row_asserted(pdata, row) ? (1 << row) : 0;
		}
		activate_col(pdata, col, false);
	}
	for (col = 0; col < pdata->num_col_gpios; col++) {
		uint32_t bits_changed;

		bits_changed = keypad->last_key_state[col] ^ new_state[col];
		if (bits_changed == 0)
			continue;

		for (row = 0; row < pdata->num_row_gpios; row++) {
			if (((bits_changed & (1 << row)) == 0) || ((new_state[col] & (1 << row)) == 0))
				continue;

			//printk("!!!!! PRESS:%d\n",keycodes[row][col]);
			key = keycodes[row][col];
			keypad_create_event((char *)keyaction[key-1]);
			delay = 200;
		}
	}

	memcpy(keypad->last_key_state, new_state, sizeof(new_state));

	activate_all_cols(pdata, true);
	schedule_delayed_work(&keypad->work,delay);
}

static int __devinit init_matrix_gpio(struct platform_device *pdev,
					struct matrix_keypad *keypad)
{
	const struct cta_keypad_platform_data *pdata = keypad->pdata;
	int i;

	/* initialized strobe lines as outputs, activated */
	for (i = 0; i < pdata->num_col_gpios; i++) {
		gpio_direction_output(pdata->col_gpios[i], !pdata->active_low);
	}

	for (i = 0; i < pdata->num_row_gpios; i++) {
		gpio_direction_input(pdata->row_gpios[i]);
	}
	return 0;
}

static int __devinit cta_keypad_probe(struct platform_device *pdev)
{
	const struct cta_keypad_platform_data *pdata;
	const struct cta_keymap_data *keymap_data;
	struct matrix_keypad *keypad;
	unsigned int row_shift;
	int i, err;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}

	keymap_data = pdata->keymap_data;
	if (!keymap_data) {
		dev_err(&pdev->dev, "no keymap data defined\n");
		return -EINVAL;
	}

	row_shift = get_count_order(pdata->num_col_gpios);

	keypad = kzalloc(sizeof(struct matrix_keypad), GFP_KERNEL);
	
	keycodes = (unsigned short **)kzalloc(MATRIX_MAX_ROWS * sizeof(unsigned short), GFP_KERNEL);
	for (i = 0; i < MATRIX_MAX_ROWS; i++)
	{
		keycodes[i] = (unsigned short *)kzalloc(MATRIX_MAX_COLS * sizeof(unsigned short), GFP_KERNEL);
	}

	if (!keypad || !keycodes) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	keypad->pdata = pdata;
	keypad->row_shift = row_shift;
	keypad->stopped = true;
	INIT_DELAYED_WORK(&keypad->work, matrix_keypad_scan);
	
	cta_keypad_build_keymap(keymap_data, row_shift,
				  keycodes);

	err = init_matrix_gpio(pdev, keypad);
	if (err)
		goto err_free_mem;

	device_init_wakeup(&pdev->dev, pdata->wakeup);
	platform_set_drvdata(pdev, keypad);

	schedule_delayed_work(&keypad->work, 0);

	return 0;

err_free_mem:
	kfree(keycodes);
	kfree(keypad);
	return err;
}

static int __devexit cta_keypad_remove(struct platform_device *pdev)
{
	struct matrix_keypad *keypad = platform_get_drvdata(pdev);
	const struct cta_keypad_platform_data *pdata = keypad->pdata;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->num_row_gpios; i++) {
		gpio_free(pdata->row_gpios[i]);
	}

	for (i = 0; i < pdata->num_col_gpios; i++)
		gpio_free(pdata->col_gpios[i]);

	platform_set_drvdata(pdev, NULL);
	kfree(keycodes);
	kfree(keypad);

	return 0;
}

static struct platform_driver cta_keypad_driver = {
	.probe		= cta_keypad_probe,
	.remove		= __devexit_p(cta_keypad_remove),
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= "cta-gpio-keypad",
		.owner	= THIS_MODULE,
	},
};

static int __init cta_keypad_init(void)
{
	GPREG(PINMUX) &= ~(EN_MDIO_AUX_FNC);	//gpio 23
	GPREG(PINMUX) &= ~(EN_ADC_OUT_FNC);		//gpio 21
	GPREG(PINMUX) &= ~(EN_ANA_MON_FNC);		//gpio 13
	return platform_driver_register(&cta_keypad_driver);
}

static void __exit cta_keypad_exit(void)
{
	platform_driver_unregister(&cta_keypad_driver);
}

module_init(cta_keypad_init);
module_exit(cta_keypad_exit);
