#define pr_fmt(fmt) "%s:%d,"fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/wakelock.h>

#define AS3647_GPIO_TORCH 				18
#define AS3647_GPIO_STROBE 				19
#define AS3647_REG_CHIPID 				0x00
#define AS3647_REG_CURRENT_SET 			0x01
#define AS3647_REG_TXMASK 				0x03
#define AS3647_REG_LOW_VOLTAGE 			0x04
#define AS3647_REG_FLASH_TIMER 			0x05
#define AS3647_REG_CONTROL 				0x06
#define AS3647_REG_STROBE_SIGNALLING 	0x07
#define AS3647_REG_FAULT 				0x08
#define AS3647_REG_PWM_AND_INDICATOR 	0x09
#define AS3647_REG_MINIMUM_LED_CURRENT 	0x0E
#define AS3647_REG_ACTUAL_LED_CURRENT 	0x0F

#define AS3647_WRITE_REG(addr, data) \
    ({ \
        if (as3647_in_suspend) { \
            pr_err("as3647 in suspend, discard i2c write.\n"); \
        } else { \
            i2c_smbus_write_byte_data(p_as3647_data->client, addr, data); \
        } \
    })
    
#define AS3647_READ_REG(addr) \
    ({ \
        if (as3647_in_suspend) { \
            pr_err("as3647 in suspend, discard i2c read.\n"); \
        } else { \
            i2c_smbus_read_byte_data(p_as3647_data->client, addr); \
        } \
    })

struct as3647_data {
	struct i2c_client *client;
	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	struct mutex update_lock;
	struct work_struct work;
	u8 brightness;
    struct wake_lock wakelock;
};

static struct as3647_data *p_as3647_data;
static bool as3647_in_suspend = false;

static void as3647_flash_on(void)
{
	u8 data_fault = 0;
	
	mutex_lock(&p_as3647_data->update_lock);
	/* reset fault reg */
	data_fault = i2c_smbus_read_byte_data(p_as3647_data->client, AS3647_REG_FAULT);
	/* Current Set */
	AS3647_WRITE_REG(AS3647_REG_CURRENT_SET, p_as3647_data->brightness);
	/* Flash Timer */
	AS3647_WRITE_REG(AS3647_REG_FLASH_TIMER, 0x80); // 264ms
	/* Strobe Signalling */
	AS3647_WRITE_REG(AS3647_REG_STROBE_SIGNALLING, 0x00); // strobe input disabled
	/* Control */
	AS3647_WRITE_REG(AS3647_REG_CONTROL, 0x0B); // flash mode, automatically cleared after a flash pulse
#if 0 // TXMask:0x68, pin has no effect
	/* set torch/strobe gpio */
	if (gpio_request(AS3647_GPIO_TORCH, "as3647_gpio_torch") < 0) {
		pr_err("%s,%d, gpio %d request fail\n", __func__, __LINE__, AS3647_GPIO_TORCH);
		return;
	}
	gpio_tlmm_config(GPIO_CFG(AS3647_GPIO_TORCH, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG_ENABLE);
	gpio_direction_output(AS3647_GPIO_TORCH, 1);
	gpio_free(AS3647_GPIO_TORCH);
	
	if (gpio_request(AS3647_GPIO_STROBE, "as3647_gpio_strobe") < 0) {
		pr_err("%s,%d, gpio %d request fail\n", __func__, __LINE__, AS3647_GPIO_STROBE);
		return;
	}
	gpio_tlmm_config(GPIO_CFG(AS3647_GPIO_STROBE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG_ENABLE);
	gpio_direction_output(AS3647_GPIO_STROBE, 1);
	gpio_free(AS3647_GPIO_STROBE);
#endif
	mutex_unlock(&p_as3647_data->update_lock);
}

static void as3647_torch_on(void)
{
	u8 data_fault = 0;

	mutex_lock(&p_as3647_data->update_lock);
	/* reset fault reg */
	data_fault = i2c_smbus_read_byte_data(p_as3647_data->client, AS3647_REG_FAULT);
	/* Current Set */
	AS3647_WRITE_REG(AS3647_REG_CURRENT_SET, p_as3647_data->brightness);
	/* Control */
	AS3647_WRITE_REG(AS3647_REG_CONTROL, 0x0A); // assist light mode
	mutex_unlock(&p_as3647_data->update_lock);
}

static void as3647_off(void)
{
	mutex_lock(&p_as3647_data->update_lock);
	/* Control */
	AS3647_WRITE_REG(AS3647_REG_CONTROL, 0x00); // outputs disabled
	mutex_unlock(&p_as3647_data->update_lock);
}

static void led_i2c_brightness_set_led_work(struct work_struct *work)
{
	printk(KERN_DEBUG"%s:%d,brightness:%d\n", __func__, __LINE__, p_as3647_data->brightness);
    if(p_as3647_data->brightness > LED_HALF)
	{
        as3647_flash_on();
    } else if(p_as3647_data->brightness > LED_OFF)
    {
        as3647_torch_on();
		wake_lock(&p_as3647_data->wakelock); // fix sleep mode crash
    } else
    {
	    as3647_off();
		wake_unlock(&p_as3647_data->wakelock); // fix sleep mode crash
    }
}

static void led_i2c_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	if (value > 0xff || value < 0)
	{
		pr_err("%s:%d, value overflow:%d\n", __func__, __LINE__, value);
	} else if (value != p_as3647_data->brightness)
	{
		p_as3647_data->brightness = value;
		schedule_work(&p_as3647_data->work);
	}
}

static enum led_brightness led_i2c_brightness_get(struct led_classdev *led_cdev)
{
	return p_as3647_data->brightness;
}

static int as3647_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
    struct device_node *node = client->dev.of_node;
	struct device_node *sub_node = NULL;
	int32_t rc = 0;
	u8 chip_id = 0;

    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
    // Check and setup i2c client
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -1;
	}
	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
	{
	    dev_err(&client->dev, "%s: I2C error2!\n", __func__);
		return -1;
	}
	
	chip_id = i2c_smbus_read_byte_data(client, AS3647_REG_CHIPID);
	if (chip_id < 0 || ((chip_id & 0xf8) != 0xb0)) {
		dev_err(&client->dev, "no chip or wrong chip detected, ids=%d", chip_id);
		return -EINVAL;
	}
	
    if (node) {
		p_as3647_data = devm_kzalloc(&client->dev, sizeof(struct as3647_data), GFP_KERNEL);
		if (!p_as3647_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
	} else {
		dev_err(&client->dev, "Invalid of node\n");
		return -EINVAL;
	}

	// parse of node "qcom,flash"
	sub_node = of_find_node_by_name(node, "qcom,flash");
	if (!sub_node)
	{
		pr_err("%s:%d, qcom,flash node not found\n", __func__, __LINE__);
		goto end_fail;
	}
    rc = of_property_read_string(sub_node, "linux,default-trigger",
		&p_as3647_data->cdev_flash.default_trigger);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Failed to read flash default-trigger. rc = %d\n", __func__, rc);
		goto end_fail;
	}
    rc = of_property_read_string(sub_node, "linux,name", &p_as3647_data->cdev_flash.name);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Failed to read flash name. rc = %d\n", __func__, rc);
		goto end_fail;
	}
    p_as3647_data->cdev_flash.max_brightness = LED_FULL;
	p_as3647_data->cdev_flash.brightness_set = led_i2c_brightness_set;
	p_as3647_data->cdev_flash.brightness_get = led_i2c_brightness_get;
    rc = led_classdev_register(&client->dev, &p_as3647_data->cdev_flash);
	if (rc) {
		dev_err(&client->dev, "%s: Failed to register led dev flash. rc = %d\n", __func__, rc);
		goto end_fail;
	}
	
	// parse of node "qcom,torch"
	sub_node = of_find_node_by_name(node, "qcom,torch");
	if (!sub_node)
	{
		pr_err("%s:%d, qcom,torch node not found\n", __func__, __LINE__);
		goto unregister_cdev_flash;
	}
    rc = of_property_read_string(sub_node, "linux,default-trigger",
		&p_as3647_data->cdev_torch.default_trigger);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Failed to read torch default-trigger. rc = %d\n", __func__, rc);
		goto unregister_cdev_flash;
	}
    rc = of_property_read_string(sub_node, "linux,name", &p_as3647_data->cdev_torch.name);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Failed to read torch name. rc = %d\n", __func__, rc);
		goto unregister_cdev_flash;
	}
    p_as3647_data->cdev_torch.max_brightness = LED_FULL;
	p_as3647_data->cdev_torch.brightness_set = led_i2c_brightness_set;
	p_as3647_data->cdev_torch.brightness_get = led_i2c_brightness_get;
    rc = led_classdev_register(&client->dev, &p_as3647_data->cdev_torch);
	if (rc) {
		dev_err(&client->dev, "%s: Failed to register led dev torch. rc = %d\n", __func__, rc);
		goto unregister_cdev_flash;
	}

    INIT_WORK(&p_as3647_data->work, led_i2c_brightness_set_led_work);
	mutex_init(&p_as3647_data->update_lock);
	p_as3647_data->client = client;
	i2c_set_clientdata(client, p_as3647_data);
	wake_lock_init(&p_as3647_data->wakelock, WAKE_LOCK_SUSPEND, "as3647_wakelock");

	return 0;
unregister_cdev_flash:
	led_classdev_unregister(&p_as3647_data->cdev_flash);
end_fail:
    devm_kfree(&client->dev, p_as3647_data);
	p_as3647_data = NULL;
	
	return rc;
}

static int as3647_remove(struct i2c_client *client)
{
	struct as3647_data *data = i2c_get_clientdata(client);

    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	cancel_work_sync(&data->work);
	mutex_destroy(&data->update_lock);
	led_classdev_unregister(&data->cdev_flash);
	led_classdev_unregister(&data->cdev_torch);
	devm_kfree(&client->dev, data);
	i2c_set_clientdata(client, NULL);
	
	return 0;
}

static const struct i2c_device_id as3647_id[] = {
	{ "as3647", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, as3647_id);

static struct of_device_id as3647_match_table[] = {
    {.compatible = "qcom,led-flash",},
    {},
};

static int as3647_suspend(struct device *dev)
{
	printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
    as3647_in_suspend = true;
    return 0;
}

static int as3647_resume(struct device *dev)
{
    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
    as3647_in_suspend = false;
    return 0;
}

static const struct dev_pm_ops as3647_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(as3647_suspend, as3647_resume)
};

static struct i2c_driver as3647_driver = {
	.driver = {
		.name   = "as3647",
        .owner = THIS_MODULE,
        .of_match_table = as3647_match_table,
        .pm = &as3647_pm_ops,
	},
	.probe  = as3647_probe,
	.remove = as3647_remove,
	.id_table = as3647_id,
};

static int __init as3647_init(void)
{
	printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	return i2c_add_driver(&as3647_driver);
}

static void __exit as3647_exit(void)
{
	printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	i2c_del_driver(&as3647_driver);
}

MODULE_AUTHOR("Ulrich Herrmann <ulrich.herrmann@austriamicrosystems.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AS3647 LED flash light");
module_init(as3647_init);
module_exit(as3647_exit);
