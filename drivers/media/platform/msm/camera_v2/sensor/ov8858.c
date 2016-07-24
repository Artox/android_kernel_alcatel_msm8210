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
#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>

#define OV8858_SENSOR_NAME "ov8858"
DEFINE_MSM_MUTEX(ov8858_mut);

//#define OV8858_DEBUG
#undef CDBG
#ifdef OV8858_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define OTP_FEATURE
#ifdef OTP_FEATURE
#define RG_RATIO_TYPICAL 321
#define BG_RATIO_TYPICAL 307

struct ov8858_otp_struct {
	int rg_ratio;
	int bg_ratio;
	int lenc[110];
};
#endif //OTP_FEATURE

static struct msm_sensor_ctrl_t ov8858_s_ctrl;

extern bool camera_in_suspend;

static struct msm_sensor_power_setting ov8858_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
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
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
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
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
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

static struct v4l2_subdev_info ov8858_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov8858_i2c_id[] = {
	{OV8858_SENSOR_NAME, (kernel_ulong_t)&ov8858_s_ctrl},
	{ }
};

static int32_t msm_ov8858_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov8858_s_ctrl);
}

static int ov8858_suspend(struct device *dev)
{
    pr_err("camera_in_suspend:%d\n", camera_in_suspend);
    camera_in_suspend = true;
    return 0;
}

static int ov8858_resume(struct device *dev)
{
    pr_err("camera_in_suspend:%d\n", camera_in_suspend);
    camera_in_suspend = false;
    return 0;
}

static const struct dev_pm_ops ov8858_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(ov8858_suspend, ov8858_resume)
};

static struct i2c_driver ov8858_i2c_driver = {
	.id_table = ov8858_i2c_id,
	.probe  = msm_ov8858_i2c_probe,
	.driver = {
		.name = OV8858_SENSOR_NAME,
		.owner = THIS_MODULE,
		.pm = &ov8858_pm_ops,
	},
};

static struct msm_camera_i2c_client ov8858_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov8858_dt_match[] = {
	{.compatible = "qcom,ov8858", .data = &ov8858_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov8858_dt_match);

static struct platform_driver ov8858_platform_driver = {
	.driver = {
		.name = "qcom,ov8858",
		.owner = THIS_MODULE,
		.of_match_table = ov8858_dt_match,
	},
};

static int32_t ov8858_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov8858_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov8858_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov8858_platform_driver,
		ov8858_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov8858_i2c_driver);
}

static void __exit ov8858_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov8858_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov8858_s_ctrl);
		platform_driver_unregister(&ov8858_platform_driver);
	} else
		i2c_del_driver(&ov8858_i2c_driver);
	return;
}

static int32_t ov8858_sensor_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

static int32_t ov8858_sensor_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}

int32_t ov8858_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;

	CDBG("%s:%d\n", __func__, __LINE__);
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
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
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
				ov8858_sensor_enable_i2c_mux(data->i2c_conf);
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

	CDBG("%s exit\n", __func__);
	return 0;
power_up_failed:
	pr_err("%s:%d failed\n", __func__, __LINE__);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	for (index--; index >= 0; index--) {
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
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
				ov8858_sensor_disable_i2c_mux(data->i2c_conf);
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

int32_t ov8858_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	s_ctrl->stop_setting_valid = 0;

	CDBG("%s:%d\n", __func__, __LINE__);
	power_setting_array = &s_ctrl->power_setting_array;

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	for (index = (power_setting_array->size - 1); index >= 0; index--) {
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				0);
			break;
		case SENSOR_GPIO:
			if (power_setting->seq_val >= SENSOR_GPIO_MAX ||
				!data->gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_GPIO_MAX);
				continue;
			}
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			if (power_setting->seq_val >= CAM_VREG_MAX) {
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_GPIO_MAX);
				continue;
			}
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
				ov8858_sensor_disable_i2c_mux(data->i2c_conf);
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
	CDBG("%s exit\n", __func__);

	return 0;
}

#ifdef OTP_FEATURE
static void OV8858_write_i2c(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t data)
{
	long rc = 0;
	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, \
		addr, data, MSM_CAMERA_I2C_BYTE_DATA);
	if(0 > rc)
		CDBG("%s, %d, i2c write(0x%x) fail:%ld\n", __func__, __LINE__, addr, rc);
}

static int OV8858_read_i2c(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr)
{
	uint16_t data = 0;
	long rc = 0;
	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, \
		addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
	if(0 > rc)
		CDBG("%s, %d, i2c read(0x%x) fail:%ld\n", __func__, __LINE__, addr, rc);

	return data;
}

static int ov8858_otp_readLENC(struct msm_sensor_ctrl_t *s_ctrl, int nGroup, 
	struct ov8858_otp_struct *current_otp)
{
	int rc = 0;
	uint16_t flag = 0;
	uint32_t i = 0, start_addr = 0, end_addr = 0;

	if(NULL == current_otp) {
		CDBG("%s, %d, OTP struct point is NULL\n", __func__, __LINE__);
		return 0;
	}

	OV8858_write_i2c(s_ctrl, 0x3d84, 0xc0); //enable OTP read
	OV8858_write_i2c(s_ctrl, 0x3d88, 0x70); //start address
	OV8858_write_i2c(s_ctrl, 0x3d89, 0x3a);
	OV8858_write_i2c(s_ctrl, 0x3d8a, 0x70); //end address
	OV8858_write_i2c(s_ctrl, 0x3d8b, 0x3a);
	OV8858_write_i2c(s_ctrl, 0x3d81, 0x01); //load OTP
	msleep(5);
	OV8858_write_i2c(s_ctrl, 0x3d81, 0); //disable load OTP
	flag = OV8858_read_i2c(s_ctrl, 0x703a); //read flag
	OV8858_write_i2c(s_ctrl, 0x703a, 0); //clear OTP buffer
	
	if(0 == nGroup && (0x01 == ((flag >> 6) & 0x03))) {
		start_addr = 0x703b;
		end_addr = 0x70a8;
		rc = 1;
	} else if(1 == nGroup && (0x01 == ((flag >> 4) & 0x03))) {
		start_addr = 0x70a9;
		end_addr = 0x7116;
		rc = 1;
	} else if(2 == nGroup && (0x01 == ((flag >> 2) & 0x03))) {
		start_addr = 0x7117;
		end_addr = 0x7184;
		rc = 1;
	} else {
		rc = 0;
	}

	if(rc) {
		OV8858_write_i2c(s_ctrl, 0x3d84, 0xc0); //enable OTP read
		OV8858_write_i2c(s_ctrl, 0x3d88, (start_addr >> 8) & 0xff); //start address
		OV8858_write_i2c(s_ctrl, 0x3d89, start_addr & 0xff);
		OV8858_write_i2c(s_ctrl, 0x3d8a, (end_addr >> 8) & 0xff); //end address
		OV8858_write_i2c(s_ctrl, 0x3d8b, end_addr & 0xff);
		OV8858_write_i2c(s_ctrl, 0x3d81, 0x01); //load OTP
		msleep(5);
		OV8858_write_i2c(s_ctrl, 0x3d81, 0); //disable load OTP
		for(i = 0; i < 110; i++) {
			current_otp->lenc[i] = OV8858_read_i2c(s_ctrl, start_addr + i); //read LENC data
		}
		/*move to clearbuff config*/
		for(i = start_addr; i <= end_addr; i++) { //clear OTP buffer
			OV8858_write_i2c(s_ctrl, i, 0);
		}
		CDBG("%s, %d, OTP LENC group(%d), [%d, %d, %d, %d, %d, %d...]\n", __func__, __LINE__, nGroup, \
			current_otp->lenc[0], current_otp->lenc[1], current_otp->lenc[2], \
			current_otp->lenc[3], current_otp->lenc[4], current_otp->lenc[5]);
	} else
		CDBG("%s, %d, OTP LENC group(%d) invalid or empty\n", __func__, __LINE__, nGroup);

	return rc;
}

static int ov8858_otp_readWB(struct msm_sensor_ctrl_t *s_ctrl, int nGroup, 
	struct ov8858_otp_struct *current_otp)
{
	int rc = 0;
	uint16_t flag = 0;
	uint32_t wb_lsb = 0, start_addr = 0, end_addr = 0;

	if(NULL == current_otp) {
		CDBG("%s, %d, OTP struct point is NULL\n", __func__, __LINE__);
		return 0;
	}

	OV8858_write_i2c(s_ctrl, 0x3d84, 0xc0); //enable OTP read
	OV8858_write_i2c(s_ctrl, 0x3d88, 0x70); //start address
	OV8858_write_i2c(s_ctrl, 0x3d89, 0x20);
	OV8858_write_i2c(s_ctrl, 0x3d8a, 0x70);	//end address
	OV8858_write_i2c(s_ctrl, 0x3d8b, 0x20);
	OV8858_write_i2c(s_ctrl, 0x3d81, 0x01); //load OTP
	msleep(5);
	OV8858_write_i2c(s_ctrl, 0x3d81, 0); //disable load OTP
	flag = OV8858_read_i2c(s_ctrl, 0x7020);
	OV8858_write_i2c(s_ctrl, 0x7020, 0); //clear OTP buffer
	
	if(0 == nGroup && (0x01 == ((flag >> 6) & 0x03))) {
		start_addr = 0x7021;
		end_addr = 0x7025;
		rc = 1;
	} else if(1 == nGroup && (0x01 == ((flag >> 4) & 0x03))) {
		start_addr = 0x7026;
		end_addr = 0x702a;
		rc = 1;
	} else if(2 == nGroup && (0x01 == ((flag >> 2) & 0x03))) {
		start_addr = 0x702b;
		end_addr = 0x702f;
		rc = 1;
	} else {
		rc = 0;
	}

	if(rc) {
		OV8858_write_i2c(s_ctrl, 0x3d84, 0xc0); //enable OTP read
		OV8858_write_i2c(s_ctrl, 0x3d88, (start_addr >> 8) & 0xff); //start address
		OV8858_write_i2c(s_ctrl, 0x3d89, start_addr & 0xff);
		OV8858_write_i2c(s_ctrl, 0x3d8A, (end_addr >> 8) & 0xff); //end address
		OV8858_write_i2c(s_ctrl, 0x3d8B, end_addr & 0xff);
		OV8858_write_i2c(s_ctrl, 0x3d81, 0x01); //load OTP
		msleep(5);
		OV8858_write_i2c(s_ctrl, 0x3d81, 0); //disable load OTP
		wb_lsb = OV8858_read_i2c(s_ctrl, start_addr + 4);
		current_otp->rg_ratio = (OV8858_read_i2c(s_ctrl, start_addr) << 2) | ((wb_lsb >> 6) & 0x03);
		current_otp->bg_ratio = (OV8858_read_i2c(s_ctrl, start_addr + 1) << 2) | ((wb_lsb >> 4) & 0x03);
		/*move to clearbuff config*/
		for(i = start_addr; i <= end_addr; i++) { //clear OTP buffer
			OV8858_write_i2c(s_ctrl, i, 0);
		}
		CDBG("%s, %d, OTP WB group(%d), rg_ratio:%d, bg_ratio:%d\n", __func__, __LINE__, nGroup, \
			current_otp->rg_ratio, current_otp->bg_ratio);
	} else
		CDBG("%s, %d, OTP WB group(%d) invalid or empty\n", __func__, __LINE__, nGroup);

	return rc;
}

static int32_t ov8858_otp_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct ov8858_otp_struct current_otp = {0};
	int i = 0, temp = 0, R_gain = 0, B_gain = 0, G_gain = 0, G_gain_B = 0, G_gain_R = 0;

	CDBG("enter\n");
	if(NULL == s_ctrl) {
		CDBG("%s, %d, msm_sensor_ctrl_t struct point is NULL\n", __func__, __LINE__);
		return 0;
	}
	
	mutex_lock(s_ctrl->msm_sensor_mutex);
	//LENC
	for(i = 0; i < 3; i++) {
		if(ov8858_otp_readLENC(s_ctrl, i, &current_otp))
			break;
	}
	if(3 > i) {
		//update LENC
		temp = OV8858_read_i2c(s_ctrl, 0x5000);
		temp = 0x80 | temp;
		OV8858_write_i2c(s_ctrl, 0x5000, temp);
		for(i = 0; i < 110; i++) {
			OV8858_write_i2c(s_ctrl, 0x5800 + i, current_otp.lenc[i]);
		}
	}

	//WB
	for(i = 0; i < 3; i++) {
		if(ov8858_otp_readWB(s_ctrl, i, &current_otp))
			break;
	}
	if(3 > i) {
		if(!current_otp.bg_ratio || !current_otp.rg_ratio) {
			printk(KERN_ERR "!!!read out zero OTP value, bg:%d, rg:%d\n", 
				current_otp.bg_ratio, current_otp.rg_ratio);
			goto ex;
		}
		//calculate gains
		//0x400 = 1x gain
		if(current_otp.bg_ratio < BG_RATIO_TYPICAL) {
			if(current_otp.rg_ratio < RG_RATIO_TYPICAL) {
				G_gain = 0x400;
				B_gain = 0x400 * BG_RATIO_TYPICAL / current_otp.bg_ratio;
				R_gain = 0x400 * RG_RATIO_TYPICAL / current_otp.rg_ratio; 
			} else {
				R_gain = 0x400;
				G_gain = 0x400 * current_otp.rg_ratio / RG_RATIO_TYPICAL;
				B_gain = G_gain * BG_RATIO_TYPICAL / current_otp.bg_ratio;
			}
		} else {
			if(current_otp.rg_ratio < RG_RATIO_TYPICAL) {
				B_gain = 0x400;
				G_gain = 0x400 * current_otp.bg_ratio / BG_RATIO_TYPICAL;
				R_gain = G_gain * RG_RATIO_TYPICAL / current_otp.rg_ratio;
			} else {
				G_gain_B = 0x400 * current_otp.bg_ratio / BG_RATIO_TYPICAL;
				G_gain_R = 0x400 * current_otp.rg_ratio / RG_RATIO_TYPICAL;
				if(G_gain_B > G_gain_R) {
					B_gain = 0x400;
					G_gain = G_gain_B;
					R_gain = G_gain * RG_RATIO_TYPICAL / current_otp.rg_ratio;
				} else {
					R_gain = 0x400;
					G_gain = G_gain_R;
					B_gain = G_gain * BG_RATIO_TYPICAL / current_otp.bg_ratio;
				}
			}    
		}
		CDBG("%s, %d, OTP WB after calculate, G_gain:%d, B_gain:%d, R_gain:%d\n", __func__, __LINE__, \
			G_gain, B_gain, R_gain);
		//update WB
		if(R_gain > 0x400) {
			OV8858_write_i2c(s_ctrl, 0x5032, R_gain >> 8);
			OV8858_write_i2c(s_ctrl, 0x5033, R_gain & 0x00ff);
		}
		if(G_gain > 0x400) {
			OV8858_write_i2c(s_ctrl, 0x5034, G_gain >> 8);
			OV8858_write_i2c(s_ctrl, 0x5035, G_gain & 0x00ff);
		}
		if(B_gain > 0x400) {
			OV8858_write_i2c(s_ctrl, 0x5036, B_gain >> 8);
			OV8858_write_i2c(s_ctrl, 0x5037, B_gain & 0x00ff);
		}
	}

ex:
    mutex_unlock(s_ctrl->msm_sensor_mutex);
	CDBG("exit\n");

	return 0;
}
#endif //OTP_FEATURE

static struct msm_sensor_fn_t ov8858_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov8858_sensor_power_up,
	.sensor_power_down = ov8858_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef OTP_FEATURE
	.otp_config = ov8858_otp_config,
#endif //OTP_FEATURE
};

static struct msm_sensor_ctrl_t ov8858_s_ctrl = {
	.sensor_i2c_client = &ov8858_sensor_i2c_client,
	.power_setting_array.power_setting = ov8858_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov8858_power_setting),
	.msm_sensor_mutex = &ov8858_mut,
	.sensor_v4l2_subdev_info = ov8858_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8858_subdev_info),
	.func_tbl = &ov8858_sensor_func_tbl,
};

module_init(ov8858_init_module);
module_exit(ov8858_exit_module);
MODULE_DESCRIPTION("ov8858");
MODULE_LICENSE("GPL v2");
