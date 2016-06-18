/*
 * Driver for cypress touch screen controller
 *
 * Copyright (c) 2009 Aava Mobile
 *
 * Some cleanups by Alan Cox <alan@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input/cy8ctmg110_pdata.h>

#define CY8CTMG110_DRIVER_NAME      "cy8ctmg110"

/* Touch coordinates */
#define CY8CTMG110_X_MIN		0
#define CY8CTMG110_Y_MIN		0
#define CY8CTMG110_X_MAX		759
#define CY8CTMG110_Y_MAX		465


/* cy8ctmg110 register definitions */
#define CY8CTMG110_TOUCH_WAKEUP_TIME	0
#define CY8CTMG110_TOUCH_SLEEP_TIME	2
#define CY8CTMG110_TOUCH_X1		3
#define CY8CTMG110_TOUCH_Y1		5
#define CY8CTMG110_TOUCH_X2		7
#define CY8CTMG110_TOUCH_Y2		9
#define CY8CTMG110_FINGERS		11
#define CY8CTMG110_GESTURE		12
#define CY8CTMG110_REG_MAX		13


/*
 * The touch driver structure.
 */
struct cy8ctmg110 {
	struct input_dev *input;
	char phys[32];
	struct i2c_client *client;
	int reset_pin;
	int irq_pin;
};

/*
 * cy8ctmg110_power is the routine that is called when touch hardware
 * will powered off or on.
 */
static void cy8ctmg110_power(struct cy8ctmg110 *ts, bool poweron)
{
	if (ts->reset_pin)
		gpio_direction_output(ts->reset_pin, 1 - poweron);//通过激活是否给一个gpio线输出灌电流来使能设备
}

static int cy8ctmg110_write_regs(struct cy8ctmg110 *tsc, unsigned char reg,
		unsigned char len, unsigned char *value)
{
	struct i2c_client *client = tsc->client;
	int ret;
	unsigned char i2c_data[6];

	BUG_ON(len > 5);

	i2c_data[0] = reg;
	memcpy(i2c_data + 1, value, len);

	ret = i2c_master_send(client, i2c_data, len + 1);//发送数据
	if (ret != len + 1) {
		dev_err(&client->dev, "i2c write data cmd failed\n");
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int cy8ctmg110_read_regs(struct cy8ctmg110 *tsc,
		unsigned char *data, unsigned char len, unsigned char cmd)
{
	struct i2c_client *client = tsc->client;
	int ret;
	struct i2c_msg msg[2] = {
		/* first write slave position to i2c devices */
		{ client->addr, 0, 1, &cmd },
		/* Second read data from position */
		{ client->addr, I2C_M_RD, len, data }
	};

	ret = i2c_transfer(client->adapter, msg, 2);//读入数据
	if (ret < 0)
		return ret;

	return 0;
}

static int cy8ctmg110_touch_pos(struct cy8ctmg110 *tsc)
{
	struct input_dev *input = tsc->input;//都没有直接用驱动上下文环境的指针，而是赋值给一个变量
	unsigned char reg_p[CY8CTMG110_REG_MAX];
	int x, y;

	memset(reg_p, 0, CY8CTMG110_REG_MAX);

	/* Reading coordinates */
	if (cy8ctmg110_read_regs(tsc, reg_p, 9, CY8CTMG110_TOUCH_X1) != 0)
		return -EIO;

	y = reg_p[2] << 8 | reg_p[3];
	x = reg_p[0] << 8 | reg_p[1];

	/* Number of touch */
	if (reg_p[8] == 0) {
		input_report_key(input, BTN_TOUCH, 0);
	} else  {
		input_report_key(input, BTN_TOUCH, 1);
		input_report_abs(input, ABS_X, x);
		input_report_abs(input, ABS_Y, y);
	}

	input_sync(input);//上报一次数据

	return 0;
}

static int cy8ctmg110_set_sleepmode(struct cy8ctmg110 *ts, bool sleep)
{
	unsigned char reg_p[3];

	if (sleep) {
		reg_p[0] = 0x00;
		reg_p[1] = 0xff;
		reg_p[2] = 5;
	} else {
		reg_p[0] = 0x10;
		reg_p[1] = 0xff;
		reg_p[2] = 0;
	}

	return cy8ctmg110_write_regs(ts, CY8CTMG110_TOUCH_WAKEUP_TIME, 3, reg_p);
}

static irqreturn_t cy8ctmg110_irq_thread(int irq, void *dev_id)
{
	struct cy8ctmg110 *tsc = dev_id;

	cy8ctmg110_touch_pos(tsc);//中断产生，则触发一次数据上报

	return IRQ_HANDLED;
}

static int __devinit cy8ctmg110_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	const struct cy8ctmg110_pdata *pdata = client->dev.platform_data;//自定义的平台数据格式
	struct cy8ctmg110 *ts;//驱动上下文环境指针
	struct input_dev *input_dev;//逻辑平台设备指针
	int err;

	/* No pdata no way forward */
	if (pdata == NULL) {//平台数据检查
		dev_err(&client->dev, "no pdata\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_READ_WORD_DATA))//i2c协议支持检查，是否支持SMBUS的字读取
		return -EIO;

	ts = kzalloc(sizeof(struct cy8ctmg110), GFP_KERNEL);//分配内存
	input_dev = input_allocate_device();//分配input设备
	if (!ts || !input_dev) {//检查结果
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;//关联i2c设备
	ts->input = input_dev;//关联input设备
	ts->reset_pin = pdata->reset_pin;//重置的gpio管脚，通过平台数据传入
	ts->irq_pin = pdata->irq_pin;//中断管脚

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));//平台设备需要的物理路径，i2c设备路径，加上input0

	input_dev->name = CY8CTMG110_DRIVER_NAME " Touchscreen";//input设备的名字直接命名
	input_dev->phys = ts->phys;//物理路径
	input_dev->id.bustype = BUS_I2C;//总线类型
	input_dev->dev.parent = &client->dev;//设备父设备关联，input设备是i2c设备的子设备

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);//支持的事件种类
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);//按键种类定义

	input_set_abs_params(input_dev, ABS_X,//绝对设备的上下限值
			CY8CTMG110_X_MIN, CY8CTMG110_X_MAX, 4, 0);
	input_set_abs_params(input_dev, ABS_Y,
			CY8CTMG110_Y_MIN, CY8CTMG110_Y_MAX, 4, 0);

	if (ts->reset_pin) {
		err = gpio_request(ts->reset_pin, NULL);//先申请gpio资源
		if (err) {
			dev_err(&client->dev,
				"Unable to request GPIO pin %d.\n",
				ts->reset_pin);
			goto err_free_mem;
		}
	}

	cy8ctmg110_power(ts, true);
	cy8ctmg110_set_sleepmode(ts, false);

	err = gpio_request(ts->irq_pin, "touch_irq_key");//gpio资源申请
	if (err < 0) {
		dev_err(&client->dev,
			"Failed to request GPIO %d, error %d\n",
			ts->irq_pin, err);
		goto err_shutoff_device;
	}

	err = gpio_direction_input(ts->irq_pin);//配置中断gpio线为输入脚
	if (err < 0) {
		dev_err(&client->dev,
			"Failed to configure input direction for GPIO %d, error %d\n",
			ts->irq_pin, err);
		goto err_free_irq_gpio;
	}

	client->irq = gpio_to_irq(ts->irq_pin);//gpio转变为中断资源
	if (client->irq < 0) {
		err = client->irq;
		dev_err(&client->dev,
			"Unable to get irq number for GPIO %d, error %d\n",
			ts->irq_pin, err);
		goto err_free_irq_gpio;
	}

	err = request_threaded_irq(client->irq, NULL, cy8ctmg110_irq_thread,
				   IRQF_TRIGGER_RISING, "touch_reset_key", ts);//上升沿触发,普通中断使用
	if (err < 0) {
		dev_err(&client->dev,
			"irq %d busy? error %d\n", client->irq, err);
		goto err_free_irq_gpio;
	}

	err = input_register_device(input_dev);//input设备注册
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);//i2c设备关联驱动上下文件指针，保存到私有数据指针
	device_init_wakeup(&client->dev, 1);//？唤醒设备？
	return 0;

err_free_irq:
	free_irq(client->irq, ts);
err_free_irq_gpio:
	gpio_free(ts->irq_pin);
err_shutoff_device:
	cy8ctmg110_set_sleepmode(ts, true);
	cy8ctmg110_power(ts, false);
	if (ts->reset_pin)
		gpio_free(ts->reset_pin);
err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

#ifdef CONFIG_PM
static int cy8ctmg110_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8ctmg110 *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);
	else {
		cy8ctmg110_set_sleepmode(ts, true);
		cy8ctmg110_power(ts, false);
	}
	return 0;
}

static int cy8ctmg110_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8ctmg110 *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);
	else {
		cy8ctmg110_power(ts, true);
		cy8ctmg110_set_sleepmode(ts, false);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(cy8ctmg110_pm, cy8ctmg110_suspend, cy8ctmg110_resume);
#endif

static int __devexit cy8ctmg110_remove(struct i2c_client *client)
{
	struct cy8ctmg110 *ts = i2c_get_clientdata(client);

	cy8ctmg110_set_sleepmode(ts, true);
	cy8ctmg110_power(ts, false);//电源gpio的状态关闭

	free_irq(client->irq, ts);//释放中断资源
	input_unregister_device(ts->input);
	gpio_free(ts->irq_pin);//释放gpio支援
	if (ts->reset_pin)
		gpio_free(ts->reset_pin);//重置gpio资源释放
	kfree(ts);//是否驱动上下文的堆内存

	return 0;
}

static const struct i2c_device_id cy8ctmg110_idtable[] = {
	{ CY8CTMG110_DRIVER_NAME, 1 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, cy8ctmg110_idtable);

static struct i2c_driver cy8ctmg110_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= CY8CTMG110_DRIVER_NAME,
#ifdef CONFIG_PM
		.pm	= &cy8ctmg110_pm,
#endif
	},
	.id_table	= cy8ctmg110_idtable,
	.probe		= cy8ctmg110_probe,
	.remove		= __devexit_p(cy8ctmg110_remove),
};

module_i2c_driver(cy8ctmg110_driver);

MODULE_AUTHOR("Samuli Konttila <samuli.konttila@aavamobile.com>");
MODULE_DESCRIPTION("cy8ctmg110 TouchScreen Driver");
MODULE_LICENSE("GPL v2");
