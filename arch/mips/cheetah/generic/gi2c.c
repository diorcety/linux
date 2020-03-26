/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                           |
|                                                                              |
+=============================================================================*/
/*! 
*   \file gi2c.c 
*   \brief GPIO I2C Driver
*   \author Montage
*/
/*=============================================================================+
| Included Files                                                               |
+=============================================================================*/
#include <linux/init.h>
#include <linux/module.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/gpio.h>
/*=============================================================================+
| Define                                                                       |
+=============================================================================*/

#define GI2C_DBUG_LEVEL 0
#define GI2C_DEBUG(level, s, params...) do{ if(level >= GI2C_DBUG_LEVEL)\
                        printk(KERN_INFO "[%s, %d]: " s "\n", __FUNCTION__, __LINE__, params);\
                    }while(0)

#if CONFIG_CHEETAH_GI2C_SCL_GPIO_NUM == CONFIG_CHEETAH_GI2C_SDA_GPIO_NUM
#error "GI2C SCL/SDA GPIO NUM are the same!!"
#endif
#if CONFIG_CHEETAH_GI2C_SCL_GPIO_NUM < 0 || CONFIG_CHEETAH_GI2C_SCL_GPIO_NUM > CHEETAH_GPIO_NUMS - 1
#error "GI2C SCL GPIO NUM is out of range!!"
#endif
#if CONFIG_CHEETAH_GI2C_SDA_GPIO_NUM < 0 || CONFIG_CHEETAH_GI2C_SDA_GPIO_NUM > CHEETAH_GPIO_NUMS - 1
#error "GI2C SDA GPIO NUM is out of range!!"
#endif
#define GPIO_I2C_SCL (1 << CONFIG_CHEETAH_GI2C_SCL_GPIO_NUM)
#define GPIO_I2C_SDA (1 << CONFIG_CHEETAH_GI2C_SDA_GPIO_NUM)
/*=============================================================================+
| Variables                                                                    |
+=============================================================================*/

/*=============================================================================+
| Function Prototypes                                                          |
+=============================================================================*/

/*=============================================================================+
| Functions                                                                    |
+=============================================================================*/

int cheetah_gi2c_init(void)
{
    int ret_scl, ret_sda;

    /*initialize the in and out port value and */
    ret_scl = gpio_request(CONFIG_CHEETAH_GI2C_SCL_GPIO_NUM, "gi2c");
    ret_sda = gpio_request(CONFIG_CHEETAH_GI2C_SDA_GPIO_NUM, "gi2c");
    if (ret_scl || ret_sda) {
        if (ret_scl)
            gpio_free(CONFIG_CHEETAH_GI2C_SCL_GPIO_NUM);
        if (ret_sda)
            gpio_free(CONFIG_CHEETAH_GI2C_SDA_GPIO_NUM);
        printk(KERN_ERR "gpio_request is failed for gi2c\n");
        return -EINVAL;
    }

    /* set high value to out line */
    gpio_direction_output(CONFIG_CHEETAH_GI2C_SCL_GPIO_NUM, 1);
    gpio_direction_output(CONFIG_CHEETAH_GI2C_SDA_GPIO_NUM, 1);

    return 0;
}
module_init(cheetah_gi2c_init);
