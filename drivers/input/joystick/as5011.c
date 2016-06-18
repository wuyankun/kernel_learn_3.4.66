/*
 * Copyright (c) 2010, 2011 Fabien Marteau <fabien.marteau@armadeus.com>
 * Sponsored by ARMadeus Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Driver for Austria Microsystems joysticks AS5011
 *
 * TODO:
 *	- Power on the chip when open() and power down when close()
 *	- Manage power mode
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input/as5011.h>
#include <linux/slab.h>
#include <linux/module.h>

#define DRIVER_DESC "Driver for Austria Microsystems AS5011 joystick"
#define MODULE_DEVICE_ALIAS "as5011"

MODULE_AUTHOR("Fabien Marteau <fabien.marteau@armadeus.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/* registers */
#define AS5011_CTRL1		0x76
#define AS5011_CTRL2		0x75
#define AS5011_XP		0x43
#define AS5011_XN		0x44
#define AS5011_YP		0x53
#define AS5011_YN		0x54
#define AS5011_X_REG		0x41
#define AS5011_Y_REG		0x42
#define AS5011_X_RES_INT	0x51
#define AS5011_Y_RES_INT	0x52

/* CTRL1 bits */
#define AS5011_CTRL1_LP_PULSED		0x80
#define AS5011_CTRL1_LP_ACTIVE		0x40
#define AS5011_CTRL1_LP_CONTINUE	0x20
#define AS5011_CTRL1_INT_WUP_EN		0x10
#define AS5011_CTRL1_INT_ACT_EN		0x08
#define AS5011_CTRL1_EXT_CLK_EN		0x04
#define AS5011_CTRL1_SOFT_RST		0x02
#define AS5011_CTRL1_DATA_VALID		0x01

/* CTRL2 bits */
#define AS5011_CTRL2_EXT_SAMPLE_EN	0x08
#define AS5011_CTRL2_RC_BIAS_ON		0x04
#define AS5011_CTRL2_INV_SPINNING	0x02

#define AS5011_MAX_AXIS	80
#define AS5011_MIN_AXIS	(-80)
#define AS5011_FUZZ	8
#define AS5011_FLAT	40

struct as5011_device {
	struct input_dev *input_dev;
	struct i2c_client *i2c_client;
	unsigned int button_gpio;
	unsigned int button_irq;
	unsigned int axis_irq;
};

static int as5011_i2c_write(struct i2c_client *client,
			    uint8_t aregaddr,
			    uint8_t avalue)
{
	uint8_t data[2] = { aregaddr, avalue };
	struct i2c_msg msg = {//组织个i2c的消息格式
		client->addr, I2C_M_IGNORE_NAK, 2, (uint8_t *)data
	};
	int error;

	error = i2c_transfer(client->adapter, &msg, 1);//调用接口函数传出去
	return error < 0 ? error : 0;
}

static int as5011_i2c_read(struct i2c_client *client,
			   uint8_t aregaddr, signed char *value)
{
	uint8_t data[2] = { aregaddr };
	struct i2c_msg msg_set[2] = {
		{ client->addr, I2C_M_REV_DIR_ADDR, 1, (uint8_t *)data },
		{ client->addr, I2C_M_RD | I2C_M_NOSTART, 1, (uint8_t *)data }
	};
	int error;

	error = i2c_transfer(client->adapter, msg_set, 2);
	if (error < 0)
		return error;

	*value = data[0] & 0x80 ? -1 * (1 + ~data[0]) : data[0];//组织数据
	return 0;
}

static irqreturn_t as5011_button_interrupt(int irq, void *dev_id)
{
	struct as5011_device *as5011 = dev_id;
	int val = gpio_get_value_cansleep(as5011->button_gpio);//获取一个gpio的值，这里是可以睡眠的获取

	input_report_key(as5011->input_dev, BTN_JOYSTICK, !val);
	input_sync(as5011->input_dev);//产生一个input事件

	return IRQ_HANDLED;
}

static irqreturn_t as5011_axis_interrupt(int irq, void *dev_id)
{
	struct as5011_device *as5011 = dev_id;
	int error;
	signed char x, y;

	error = as5011_i2c_read(as5011->i2c_client, AS5011_X_RES_INT, &x);//通过i2c接口读取寄存器
	if (error < 0)
		goto out;

	error = as5011_i2c_read(as5011->i2c_client, AS5011_Y_RES_INT, &y);
	if (error < 0)
		goto out;

	input_report_abs(as5011->input_dev, ABS_X, x);
	input_report_abs(as5011->input_dev, ABS_Y, y);
	input_sync(as5011->input_dev);//产生触摸位置的事件

out:
	return IRQ_HANDLED;
}

static int __devinit as5011_configure_chip(struct as5011_device *as5011,
				const struct as5011_platform_data *plat_dat)
{
	struct i2c_client *client = as5011->i2c_client;
	int error;
	signed char value;

	/* chip soft reset */
	error = as5011_i2c_write(client, AS5011_CTRL1,
				 AS5011_CTRL1_SOFT_RST);
	if (error < 0) {
		dev_err(&client->dev, "Soft reset failed\n");
		return error;
	}

	mdelay(10);

	error = as5011_i2c_write(client, AS5011_CTRL1,
				 AS5011_CTRL1_LP_PULSED |
				 AS5011_CTRL1_LP_ACTIVE |
				 AS5011_CTRL1_INT_ACT_EN);
	if (error < 0) {
		dev_err(&client->dev, "Power config failed\n");
		return error;
	}

	error = as5011_i2c_write(client, AS5011_CTRL2,
				 AS5011_CTRL2_INV_SPINNING);
	if (error < 0) {
		dev_err(&client->dev, "Can't invert spinning\n");
		return error;
	}

	/* write threshold */
	error = as5011_i2c_write(client, AS5011_XP, plat_dat->xp);
	if (error < 0) {
		dev_err(&client->dev, "Can't write threshold\n");
		return error;
	}

	error = as5011_i2c_write(client, AS5011_XN, plat_dat->xn);
	if (error < 0) {
		dev_err(&client->dev, "Can't write threshold\n");
		return error;
	}

	error = as5011_i2c_write(client, AS5011_YP, plat_dat->yp);
	if (error < 0) {
		dev_err(&client->dev, "Can't write threshold\n");
		return error;
	}

	error = as5011_i2c_write(client, AS5011_YN, plat_dat->yn);
	if (error < 0) {
		dev_err(&client->dev, "Can't write threshold\n");
		return error;
	}

	/* to free irq gpio in chip */
	error = as5011_i2c_read(client, AS5011_X_RES_INT, &value);
	if (error < 0) {
		dev_err(&client->dev, "Can't read i2c X resolution value\n");
		return error;
	}

	return 0;
}

static int __devinit as5011_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	const struct as5011_platform_data *plat_data;//平台数据指针，自定义结构体，const类型的指针
	struct as5011_device *as5011;//驱动上下文环境，指针
	struct input_dev *input_dev;//从功能上划分为input设备，指针
	int irq;
	int error;

	plat_data = client->dev.platform_data;//获取平台数据，直接赋值，对平台数据的格式有要求
	if (!plat_data)
		return -EINVAL;

	if (!plat_data->axis_irq) {//数据合法性校验
		dev_err(&client->dev, "No axis IRQ?\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_PROTOCOL_MANGLING)) {//i2c支持的协议检查，这里是？
		dev_err(&client->dev,
			"need i2c bus that supports protocol mangling\n");
		return -ENODEV;
	}

	as5011 = kmalloc(sizeof(struct as5011_device), GFP_KERNEL);//分配内存
	input_dev = input_allocate_device();//分配一个input设备
	if (!as5011 || !input_dev) {//检查结果
		dev_err(&client->dev,
			"Can't allocate memory for device structure\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	as5011->i2c_client = client;//关联i2c接口设备
	as5011->input_dev = input_dev;//关联逻辑input设备
	as5011->button_gpio = plat_data->button_gpio;
	as5011->axis_irq = plat_data->axis_irq;

	input_dev->name = "Austria Microsystem as5011 joystick";//input设备的name直接赋值
	input_dev->id.bustype = BUS_I2C;//总线类型
	input_dev->dev.parent = &client->dev;//设备的父设备，是i2c设备，i2c设备的子设备为input设备

	__set_bit(EV_KEY, input_dev->evbit);//支持的事件种类，触摸屏至少包含按键和绝对
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_JOYSTICK, input_dev->keybit);//按键的种类

	input_set_abs_params(input_dev, ABS_X,//绝对设备的最大最小值定义
		AS5011_MIN_AXIS, AS5011_MAX_AXIS, AS5011_FUZZ, AS5011_FLAT);
	input_set_abs_params(as5011->input_dev, ABS_Y,
		AS5011_MIN_AXIS, AS5011_MAX_AXIS, AS5011_FUZZ, AS5011_FLAT);

	error = gpio_request(as5011->button_gpio, "AS5011 button");//用gpio口实现的中断线,先申请gpio资源
	if (error < 0) {
		dev_err(&client->dev, "Failed to request button gpio\n");
		goto err_free_mem;
	}

	irq = gpio_to_irq(as5011->button_gpio);//将gpio转变为irq中断
	if (irq < 0) {
		dev_err(&client->dev,
			"Failed to get irq number for button gpio\n");
		goto err_free_button_gpio;
	}

	as5011->button_irq = irq;//记录中断号

	error = request_threaded_irq(as5011->button_irq,//管理一个内核中断线程中断，这里触发方式为上升沿和下降沿触发，管理驱动上下文结构体
				     NULL, as5011_button_interrupt,
				     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				     "as5011_button", as5011);
	if (error < 0) {
		dev_err(&client->dev,
			"Can't allocate button irq %d\n", as5011->button_irq);
		goto err_free_button_gpio;
	}

	error = as5011_configure_chip(as5011, plat_data);//初始化芯片
	if (error)
		goto err_free_button_irq;

	error = request_threaded_irq(as5011->axis_irq, NULL,//另外一个中断资源的申请
				     as5011_axis_interrupt,
				     plat_data->axis_irqflags,
				     "as5011_joystick", as5011);
	if (error) {
		dev_err(&client->dev,
			"Can't allocate axis irq %d\n", plat_data->axis_irq);
		goto err_free_button_irq;
	}

	error = input_register_device(as5011->input_dev);//注册input设备
	if (error) {
		dev_err(&client->dev, "Failed to register input device\n");
		goto err_free_axis_irq;
	}

	i2c_set_clientdata(client, as5011);//将驱动上下文环境保存到i2c设备的私有数据中

	return 0;

err_free_axis_irq:
	free_irq(as5011->axis_irq, as5011);
err_free_button_irq:
	free_irq(as5011->button_irq, as5011);
err_free_button_gpio:
	gpio_free(as5011->button_gpio);
err_free_mem:
	input_free_device(input_dev);
	kfree(as5011);

	return error;
}

static int __devexit as5011_remove(struct i2c_client *client)
{
	struct as5011_device *as5011 = i2c_get_clientdata(client);

	free_irq(as5011->axis_irq, as5011);
	free_irq(as5011->button_irq, as5011);
	gpio_free(as5011->button_gpio);//释放gpio资源

	input_unregister_device(as5011->input_dev);//去注册
	kfree(as5011);//释放内存

	return 0;
}

static const struct i2c_device_id as5011_id[] = {
	{ MODULE_DEVICE_ALIAS, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, as5011_id);

static struct i2c_driver as5011_driver = {
	.driver = {
		.name = "as5011",
	},
	.probe		= as5011_probe,
	.remove		= __devexit_p(as5011_remove),//__devexit_p这种实现在新的内核中不在使用
	.id_table	= as5011_id,
};

module_i2c_driver(as5011_driver);
