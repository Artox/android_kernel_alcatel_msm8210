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
*/
#define pr_fmt(fmt) "%s:%d "fmt, __func__, __LINE__

#include "msm_sensor.h"
#define S5K4H5YC_SENSOR_NAME "s5k4h5yc-0x30"
DEFINE_MSM_MUTEX(s5k4h5yc_mut);

//#define S5K4H5YC_0x30_DEBUG
#undef CDBG
#ifdef S5K4H5YC_0x30_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define OTP_FEATURE
#ifdef OTP_FEATURE
#define RG_RATIO_TYPICAL 817
#define BG_RATIO_TYPICAL 640

struct s5k4h5yc_otp_struct {
	 int rg_ratio;
	 int bg_ratio;
};
#endif //OTP_FEATURE

static struct msm_sensor_ctrl_t s5k4h5yc_s_ctrl;

extern bool camera_in_suspend;

static struct msm_sensor_power_setting s5k4h5yc_power_setting[] = {
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
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_AF_PWDM,
        .config_val = GPIO_OUT_LOW,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_AF_PWDM,
        .config_val = GPIO_OUT_HIGH,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_STANDBY,
        .config_val = GPIO_OUT_LOW,
        .delay = 5,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_STANDBY,
        .config_val = GPIO_OUT_HIGH,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_CLK,
        .seq_val = SENSOR_CAM_MCLK,
        .config_val = 24000000,
        .delay = 2,
    },
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 18,
	},
};

static struct v4l2_subdev_info s5k4h5yc_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SGRBG10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static struct msm_camera_i2c_client s5k4h5yc_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct i2c_device_id s5k4h5yc_i2c_id[] = {
	{S5K4H5YC_SENSOR_NAME,
		(kernel_ulong_t)&s5k4h5yc_s_ctrl},
	{ }
};

static int32_t msm_s5k4h5yc_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &s5k4h5yc_s_ctrl);
}

static int s5k4h5yc_0x30_suspend(struct device *dev)
{
    pr_err("camera_in_suspend:%d\n", camera_in_suspend);
    camera_in_suspend = true;
    return 0;
}

static int s5k4h5yc_0x30_resume(struct device *dev)
{
    pr_err("camera_in_suspend:%d\n", camera_in_suspend);
    camera_in_suspend = false;
    return 0;
}

static const struct dev_pm_ops s5k4h5yc_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(s5k4h5yc_0x30_suspend, s5k4h5yc_0x30_resume)
};

static struct i2c_driver s5k4h5yc_i2c_driver = {
	.id_table = s5k4h5yc_i2c_id,
	.probe  = msm_s5k4h5yc_i2c_probe,
	.driver = {
		.name = S5K4H5YC_SENSOR_NAME,
		.owner = THIS_MODULE,
		.pm = &s5k4h5yc_pm_ops,
	},
};

#ifdef OTP_FEATURE
static void s5k4h5yc_write_i2c(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t data)
{
	long rc = 0;
	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, \
		addr, data, MSM_CAMERA_I2C_BYTE_DATA);
	if(0 > rc)
		CDBG("%s, %d, i2c write(0x%x) fail:%ld\n", __func__, __LINE__, addr, rc);
}

static int s5k4h5yc_read_i2c(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr)
{
	uint16_t data = 0;
	long rc = 0;
	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, \
		addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
	if(0 > rc)
		CDBG("%s, %d, i2c read(0x%x) fail:%ld\n", __func__, __LINE__, addr, rc);

	return data;
}

static int s5k4h5yc_otp_readWB(struct msm_sensor_ctrl_t *s_ctrl, int nGroup, 
	struct s5k4h5yc_otp_struct *current_otp)
{
	int rc = 0;
	uint16_t flag = 0;

	if(NULL == current_otp) {
		CDBG("%s, %d, OTP struct point is NULL\n", __func__, __LINE__);
		return 0;
	}

	if(0 == nGroup) {
		s5k4h5yc_write_i2c(s_ctrl, 0x3A02, 0); //set page 00
		s5k4h5yc_write_i2c(s_ctrl, 0x3A00, 0x01); //OTP chip enable and read start
		msleep(5);
		flag = s5k4h5yc_read_i2c(s_ctrl, 0x3A1A); //WB 0 flag
		if(0x01 == flag) { //group 0
			current_otp->rg_ratio = \
				((s5k4h5yc_read_i2c(s_ctrl, 0x3A0B) & 0x03) << 8) | \
				(s5k4h5yc_read_i2c(s_ctrl, 0x3A0C) & 0xFF);
			current_otp->bg_ratio = \
				((s5k4h5yc_read_i2c(s_ctrl, 0x3A0D) & 0x03) << 8) | \
				(s5k4h5yc_read_i2c(s_ctrl, 0x3A0E) & 0xFF);
			rc = 1;
		} else {
			rc = 0;
		}
		s5k4h5yc_write_i2c(s_ctrl, 0x3A00, 0); //OTP chip disable and read end
	} else if(1 == nGroup) {
		s5k4h5yc_write_i2c(s_ctrl, 0x3A02, 0); //set page 00
		s5k4h5yc_write_i2c(s_ctrl, 0x3A00, 0x01); //OTP chip enable and read start
		msleep(5);
		flag = s5k4h5yc_read_i2c(s_ctrl, 0x3A31); //WB 1 flag
		if(0x01 == flag) { //group 1
			current_otp->rg_ratio = \
				((s5k4h5yc_read_i2c(s_ctrl, 0x3A22) & 0x03) << 8) | \
				(s5k4h5yc_read_i2c(s_ctrl, 0x3A23) & 0xFF);
			current_otp->bg_ratio = \
				((s5k4h5yc_read_i2c(s_ctrl, 0x3A24) & 0x03) << 8) | \
				(s5k4h5yc_read_i2c(s_ctrl, 0x3A25) & 0xFF);
			rc = 1;
		} else {
			rc = 0;
		}
		s5k4h5yc_write_i2c(s_ctrl, 0x3A00, 0); //OTP chip disable and read end
	} else {
		rc = 0;
	}

	if(rc)
		CDBG("%s, %d, OTP WB group(%d), rg_ratio:%d, bg_ratio:%d\n", __func__, __LINE__, nGroup, \
			current_otp->rg_ratio, current_otp->bg_ratio);
	else
		CDBG("%s, %d, OTP WB group(%d) invalid or empty\n", __func__, __LINE__, nGroup);

	return rc;
}

static int32_t s5k4h5yc_otp_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct s5k4h5yc_otp_struct current_otp = {0};
	int i = 0, R_gain = 0, B_gain = 0, G_gain = 0, nG_B_gain = 0, nG_R_gain = 0;

    CDBG("enter\n");
	if(NULL == s_ctrl) {
		CDBG("%s, %d, msm_sensor_ctrl_t struct point is NULL\n", __func__, __LINE__);
		return 0;
	}
	
	mutex_lock(s_ctrl->msm_sensor_mutex);
	//WB
	for(i = 0; i < 2; i++) {
		if(s5k4h5yc_otp_readWB(s_ctrl, i, &current_otp))
			break;
	}
	if(2 > i) {
		if(!current_otp.bg_ratio || !current_otp.rg_ratio) {
			printk(KERN_ERR "!!!read out zero OTP value, bg:%d, rg:%d\n", 
				current_otp.bg_ratio, current_otp.rg_ratio);
			goto ex;
		}
		//calculate gains
		if(current_otp.bg_ratio < BG_RATIO_TYPICAL) {
			if(current_otp.rg_ratio < RG_RATIO_TYPICAL) {
				G_gain = 0x100;
				B_gain = 0x100 * BG_RATIO_TYPICAL / current_otp.bg_ratio;
				R_gain = 0x100 * RG_RATIO_TYPICAL / current_otp.rg_ratio;
			} else {
				R_gain = 0x100;
				G_gain = 0x100 * current_otp.rg_ratio / RG_RATIO_TYPICAL;
				B_gain = G_gain * BG_RATIO_TYPICAL / current_otp.bg_ratio;
			}
		} else {
			if(current_otp.rg_ratio < RG_RATIO_TYPICAL) {
				B_gain = 0x100;
				G_gain = 0x100 * current_otp.bg_ratio / BG_RATIO_TYPICAL;
				R_gain = G_gain * RG_RATIO_TYPICAL / current_otp.rg_ratio;
			} else {
				nG_B_gain = 0x100 * current_otp.bg_ratio / BG_RATIO_TYPICAL;
				nG_R_gain = 0x100 * current_otp.rg_ratio / RG_RATIO_TYPICAL;
				if(nG_B_gain > nG_R_gain) {
					B_gain = 0x100;
					G_gain = nG_B_gain;
					R_gain = G_gain * RG_RATIO_TYPICAL / current_otp.rg_ratio;
				} else {
					R_gain = 0x100;
					G_gain = nG_R_gain;
					B_gain = G_gain * BG_RATIO_TYPICAL / current_otp.bg_ratio;
				}
			}
		}
		CDBG("%s, %d, after calculate G_gain:%d, B_gain:%d, R_gain:%d\n", __func__, __LINE__, \
			G_gain, B_gain, R_gain);
		if(R_gain < 0x100)
			R_gain = 0x100;
		if(G_gain < 0x100)
			G_gain = 0x100;
		if(B_gain < 0x100)
			B_gain = 0x100;
		s5k4h5yc_write_i2c(s_ctrl, 0x0210, (R_gain >> 8) & 0x0F);
		s5k4h5yc_write_i2c(s_ctrl, 0x0211, R_gain & 0xFF);
		s5k4h5yc_write_i2c(s_ctrl, 0x0212, (B_gain >> 8) & 0x0F);
		s5k4h5yc_write_i2c(s_ctrl, 0x0213, B_gain & 0xFF);
		s5k4h5yc_write_i2c(s_ctrl, 0x020E, (G_gain >> 8) & 0x0F);
		s5k4h5yc_write_i2c(s_ctrl, 0x020F, G_gain & 0xFF);
		s5k4h5yc_write_i2c(s_ctrl, 0x0214, (G_gain >> 8) & 0x0F);
		s5k4h5yc_write_i2c(s_ctrl, 0x0215, G_gain & 0xFF);
	}

ex:
    mutex_unlock(s_ctrl->msm_sensor_mutex);
    CDBG("exit\n");
    
	return 0;
}
#endif //OTP_FEATURE

static struct msm_sensor_fn_t s5k4h5yc_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef OTP_FEATURE
	.otp_config = s5k4h5yc_otp_config,
#endif //OTP_FEATURE
};

static struct msm_sensor_ctrl_t s5k4h5yc_s_ctrl = {
	.sensor_i2c_client = &s5k4h5yc_sensor_i2c_client,
	.power_setting_array.power_setting = s5k4h5yc_power_setting,
	.power_setting_array.size =
			ARRAY_SIZE(s5k4h5yc_power_setting),
	.msm_sensor_mutex = &s5k4h5yc_mut,
	.sensor_v4l2_subdev_info = s5k4h5yc_subdev_info,
	.sensor_v4l2_subdev_info_size =
			ARRAY_SIZE(s5k4h5yc_subdev_info),
	.func_tbl = &s5k4h5yc_sensor_func_tbl,
};

static const struct of_device_id s5k4h5yc_dt_match[] = {
	{
		.compatible = "qcom,s5k4h5yc-0x30",
		.data = &s5k4h5yc_s_ctrl
	},
	{}
};

MODULE_DEVICE_TABLE(of, s5k4h5yc_dt_match);

static struct platform_driver s5k4h5yc_platform_driver = {
	.driver = {
		.name = "qcom,s5k4h5yc-0x30",
		.owner = THIS_MODULE,
		.of_match_table = s5k4h5yc_dt_match,
	},
};

static int32_t s5k4h5yc_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(s5k4h5yc_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);

    return rc;
}

static int __init s5k4h5yc_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&s5k4h5yc_platform_driver,
		s5k4h5yc_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&s5k4h5yc_i2c_driver);
}

static void __exit s5k4h5yc_exit_module(void)
{
	if (s5k4h5yc_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&s5k4h5yc_s_ctrl);
		platform_driver_unregister(&s5k4h5yc_platform_driver);
	} else
		i2c_del_driver(&s5k4h5yc_i2c_driver);
	return;
}

module_init(s5k4h5yc_init_module);
module_exit(s5k4h5yc_exit_module);
MODULE_DESCRIPTION("s5k4h5yc-0x30");
MODULE_LICENSE("GPL v2");


