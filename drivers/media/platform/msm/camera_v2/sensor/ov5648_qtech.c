/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt) "%s:%d "fmt, __func__, __LINE__

#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"

#define OV5648_QTECH_SENSOR_NAME "ov5648_qtech"
DEFINE_MSM_MUTEX(ov5648_qtech_mut);

extern bool camera_in_suspend;
static struct msm_sensor_ctrl_t ov5648_qtech_s_ctrl;

static struct msm_sensor_power_setting ov5648_qtech_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov5648_qtech_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov5648_qtech_i2c_id[] = {
	{OV5648_QTECH_SENSOR_NAME,
		(kernel_ulong_t)&ov5648_qtech_s_ctrl},
	{ }
};

static int32_t msm_ov5648_qtech_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	return msm_sensor_i2c_probe(client, id, &ov5648_qtech_s_ctrl);
}

static int ov5648_qtech_suspend(struct device *dev)
{
    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
    camera_in_suspend = true;
    return 0;
}

static int ov5648_qtech_resume(struct device *dev)
{
    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
    camera_in_suspend = false;
    return 0;
}

static const struct dev_pm_ops ov5648_qtech_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(ov5648_qtech_suspend, ov5648_qtech_resume)
};

static struct i2c_driver ov5648_qtech_i2c_driver = {
	.id_table = ov5648_qtech_i2c_id,
	.probe  = msm_ov5648_qtech_i2c_probe,
	.driver = {
		.name = OV5648_QTECH_SENSOR_NAME,
		.owner = THIS_MODULE,
		.pm = &ov5648_qtech_pm_ops,
	},
};

static struct msm_camera_i2c_client ov5648_qtech_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int32_t ov5648_qtech_otp_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
    return 0;
}

static int32_t ov5648_qtech_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	int32_t ctype_value = 0;
	
	printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	s_ctrl->stop_setting_valid = 0;
	power_setting_array = &s_ctrl->power_setting_array;
	if (data->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			data->gpio_conf->cam_gpiomux_conf_tbl,
			data->gpio_conf->cam_gpiomux_conf_tbl_size);
	}

	rc = msm_camera_request_gpio_table(
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
	for (index = 0; index < power_setting_array->size; index++) {
		power_setting = &power_setting_array->power_setting[index];
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			if (power_setting->seq_val >= s_ctrl->clk_info_size) {
				pr_err("%s clk index %d >= max %d\n", __func__,
					power_setting->seq_val,
					s_ctrl->clk_info_size);
				goto power_up_failed;
			}
			if (power_setting->config_val)
				s_ctrl->clk_info[power_setting->seq_val].
					clk_rate = power_setting->config_val;

			rc = msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				1);
			if (rc < 0) {
				pr_err("%s: clk enable failed\n",
					__func__);
				goto power_up_failed;
			}
			break;
		case SENSOR_GPIO:
			if (power_setting->seq_val >= SENSOR_GPIO_MAX ||
				!data->gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_GPIO_MAX);
				goto power_up_failed;
			}
			pr_debug("%s:%d gpio set val %d\n", __func__, __LINE__,
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val]);
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val],
				power_setting->config_val);
			break;
		case SENSOR_VREG:
			if (power_setting->seq_val >= CAM_VREG_MAX) {
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_GPIO_MAX);
				goto power_up_failed;
			}
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				1);
			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
			{
				struct v4l2_subdev *i2c_mux_sd = dev_get_drvdata(&data->i2c_conf->mux_dev->dev);
				v4l2_subdev_call(i2c_mux_sd, core, ioctl, VIDIOC_MSM_I2C_MUX_INIT, NULL);
				v4l2_subdev_call(i2c_mux_sd, core, ioctl, VIDIOC_MSM_I2C_MUX_CFG, (void *)&data->i2c_conf->i2c_mux_mode);			
			}
			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}

	// check ctype pin
	printk(KERN_DEBUG"%s:%d,check ctype pin GPIO83", __func__, __LINE__);
	if (0 > gpio_request(83, "ov5648_qtech_ctype")) 
	{
		pr_err("gpio request fail\n");
		goto power_up_failed;
	}
	gpio_tlmm_config(GPIO_CFG(83, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_direction_input(83);
	ctype_value = gpio_get_value(83);
	gpio_free(83);
	if (0 != ctype_value)
	{
		pr_err("inappropriate driver for not ov5648 qtech module.\n");
		rc = -ENODEV;
		goto power_up_failed;
	}

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("%s cci_init failed\n", __func__);
			goto power_up_failed;
		}
	}

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0) {
		pr_err("%s:%d match id failed rc %d\n", __func__, __LINE__, rc);
		goto power_up_failed;
	}

	return 0;
power_up_failed:
	pr_err("%s:%d failed\n", __func__, __LINE__);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	for (index--; index >= 0; index--) {
		power_setting = &power_setting_array->power_setting[index];
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				0);
			break;
		case SENSOR_GPIO:
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
			{
				struct v4l2_subdev *i2c_mux_sd = dev_get_drvdata(&data->i2c_conf->mux_dev->dev);
				v4l2_subdev_call(i2c_mux_sd, core, ioctl, VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
			}
			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}
	msm_camera_request_gpio_table(
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 0);
	return rc;
}

static struct msm_sensor_fn_t ov5648_qtech_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov5648_qtech_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.otp_config = ov5648_qtech_otp_config,
};

static struct msm_sensor_ctrl_t ov5648_qtech_s_ctrl = {
	.sensor_i2c_client = &ov5648_qtech_sensor_i2c_client,
	.power_setting_array.power_setting = ov5648_qtech_power_setting,
	.power_setting_array.size =
			ARRAY_SIZE(ov5648_qtech_power_setting),
	.msm_sensor_mutex = &ov5648_qtech_mut,
	.sensor_v4l2_subdev_info = ov5648_qtech_subdev_info,
	.sensor_v4l2_subdev_info_size =
			ARRAY_SIZE(ov5648_qtech_subdev_info),
	.func_tbl = &ov5648_qtech_func_tbl,
};

static const struct of_device_id ov5648_qtech_dt_match[] = {
	{
		.compatible = "qtech,ov5648_qtech",
		.data = &ov5648_qtech_s_ctrl
	},
	{}
};

MODULE_DEVICE_TABLE(of, ov5648_qtech_dt_match);

static struct platform_driver ov5648_qtech_platform_driver = {
	.driver = {
		.name = "qtech,ov5648_qtech",
		.owner = THIS_MODULE,
		.of_match_table = ov5648_qtech_dt_match,
	},
};

static int32_t ov5648_qtech_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	match = of_match_device(ov5648_qtech_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov5648_qtech_init_module(void)
{
	int32_t rc = 0;

	printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov5648_qtech_platform_driver,
		ov5648_qtech_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&ov5648_qtech_i2c_driver);
}

static void __exit ov5648_qtech_exit_module(void)
{
	printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	if (ov5648_qtech_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov5648_qtech_s_ctrl);
		platform_driver_unregister(&ov5648_qtech_platform_driver);
	} else
		i2c_del_driver(&ov5648_qtech_i2c_driver);
	return;
}

module_init(ov5648_qtech_init_module);
module_exit(ov5648_qtech_exit_module);
MODULE_DESCRIPTION("ov5648_qtech");
MODULE_LICENSE("GPL v2");
