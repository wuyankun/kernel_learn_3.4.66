/*
 * Texas Instruments TNETV107X Touchscreen Driver
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <mach/tnetv107x.h>

#define TSC_PENUP_POLL		(HZ / 5)
#define IDLE_TIMEOUT		100 /* msec */

/*
 * The first and last samples of a touch interval are usually garbage and need
 * to be filtered out with these devices.  The following definitions control
 * the number of samples skipped.
 */
#define TSC_HEAD_SKIP		1
#define TSC_TAIL_SKIP		1
#define TSC_SKIP		(TSC_HEAD_SKIP + TSC_TAIL_SKIP + 1)
#define TSC_SAMPLES		(TSC_SKIP + 1)

/* Register Offsets */
struct tsc_regs {
	u32	rev;
	u32	tscm;
	u32	bwcm;
	u32	swc;
	u32	adcchnl;
	u32	adcdata;
	u32	chval[4];
};

/* TSC Mode Configuration Register (tscm) bits */
#define WMODE		BIT(0)
#define TSKIND		BIT(1)
#define ZMEASURE_EN	BIT(2)
#define IDLE		BIT(3)
#define TSC_EN		BIT(4)
#define STOP		BIT(5)
#define ONE_SHOT	BIT(6)
#define SINGLE		BIT(7)
#define AVG		BIT(8)
#define AVGNUM(x)	(((x) & 0x03) <<  9)
#define PVSTC(x)	(((x) & 0x07) << 11)
#define PON		BIT(14)
#define PONBG		BIT(15)
#define AFERST		BIT(16)

/* ADC DATA Capture Register bits */
#define DATA_VALID	BIT(16)

/* Register Access Macros */
#define tsc_read(ts, reg)		__raw_readl(&(ts)->regs->reg)
#define tsc_write(ts, reg, val)		__raw_writel(val, &(ts)->regs->reg);
#define tsc_set_bits(ts, reg, val)	\
	tsc_write(ts, reg, tsc_read(ts, reg) | (val))
#define tsc_clr_bits(ts, reg, val)	\
	tsc_write(ts, reg, tsc_read(ts, reg) & ~(val))

struct sample {
	int x, y, p;
};

struct tsc_data {
	struct input_dev		*input_dev;
	struct resource			*res;
	struct tsc_regs __iomem		*regs;
	struct timer_list		timer;
	spinlock_t			lock;
	struct clk			*clk;
	struct device			*dev;
	int				sample_count;
	struct sample			samples[TSC_SAMPLES];
	int				tsc_irq;
};

static int tsc_read_sample(struct tsc_data *ts, struct sample* sample)
{
	int	x, y, z1, z2, t, p = 0;
	u32	val;

	val = tsc_read(ts, chval[0]);
	if (val & DATA_VALID)
		x = val & 0xffff;
	else
		return -EINVAL;

	y  = tsc_read(ts, chval[1]) & 0xffff;
	z1 = tsc_read(ts, chval[2]) & 0xffff;
	z2 = tsc_read(ts, chval[3]) & 0xffff;

	if (z1) {
		t = ((600 * x) * (z2 - z1));
		p = t / (u32) (z1 << 12);
		if (p < 0)
			p = 0;
	}

	sample->x  = x;
	sample->y  = y;
	sample->p  = p;

	return 0;
}

static void tsc_poll(unsigned long data)
{
	struct tsc_data *ts = (struct tsc_data *)data;
	unsigned long flags;
	int i, val, x, y, p;

	spin_lock_irqsave(&ts->lock, flags);//关闭中断，获取自旋锁

	if (ts->sample_count >= TSC_SKIP) {
		input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_sync(ts->input_dev);
	} else if (ts->sample_count > 0) {
		/*
		 * A touch event lasted less than our skip count.  Salvage and
		 * report anyway.
		 */
		for (i = 0, val = 0; i < ts->sample_count; i++)
			val += ts->samples[i].x;
		x = val / ts->sample_count;

		for (i = 0, val = 0; i < ts->sample_count; i++)
			val += ts->samples[i].y;
		y = val / ts->sample_count;

		for (i = 0, val = 0; i < ts->sample_count; i++)
			val += ts->samples[i].p;
		p = val / ts->sample_count;

		input_report_abs(ts->input_dev, ABS_X, x);
		input_report_abs(ts->input_dev, ABS_Y, y);
		input_report_abs(ts->input_dev, ABS_PRESSURE, p);
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_sync(ts->input_dev);
	}

	ts->sample_count = 0;

	spin_unlock_irqrestore(&ts->lock, flags);
}

static irqreturn_t tsc_irq(int irq, void *dev_id)
{
	struct tsc_data *ts = (struct tsc_data *)dev_id;
	struct sample *sample;
	int index;

	spin_lock(&ts->lock);//自旋锁，在中断中获取自旋锁，不是会导致嵌套吗，也没关闭中断,不懂

	index = ts->sample_count % TSC_SAMPLES;
	sample = &ts->samples[index];
	if (tsc_read_sample(ts, sample) < 0)
		goto out;

	if (++ts->sample_count >= TSC_SKIP) {
		index = (ts->sample_count - TSC_TAIL_SKIP - 1) % TSC_SAMPLES;
		sample = &ts->samples[index];

		input_report_abs(ts->input_dev, ABS_X, sample->x);
		input_report_abs(ts->input_dev, ABS_Y, sample->y);
		input_report_abs(ts->input_dev, ABS_PRESSURE, sample->p);
		if (ts->sample_count == TSC_SKIP)
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_sync(ts->input_dev);//上报事件
	}
	mod_timer(&ts->timer, jiffies + TSC_PENUP_POLL);//更新timer
out:
	spin_unlock(&ts->lock);
	return IRQ_HANDLED;
}

static int tsc_start(struct input_dev *dev)//input设备打开
{
	struct tsc_data *ts = input_get_drvdata(dev);
	unsigned long timeout = jiffies + msecs_to_jiffies(IDLE_TIMEOUT);
	u32 val;

	clk_enable(ts->clk);//激活时钟

	/* Go to idle mode, before any initialization */
	while (time_after(timeout, jiffies)) {//短延时，忙等，占用CPU资源，sleep会释放
		if (tsc_read(ts, tscm) & IDLE)
			break;
	}

	if (time_before(timeout, jiffies)) {
		dev_warn(ts->dev, "timeout waiting for idle\n");
		clk_disable(ts->clk);
		return -EIO;
	}

	/* Configure TSC Control register*/
	val = (PONBG | PON | PVSTC(4) | ONE_SHOT | ZMEASURE_EN);
	tsc_write(ts, tscm, val);

	/* Bring TSC out of reset: Clear AFE reset bit */
	val &= ~(AFERST);
	tsc_write(ts, tscm, val);

	/* Configure all pins for hardware control*/
	tsc_write(ts, bwcm, 0);

	/* Finally enable the TSC */
	tsc_set_bits(ts, tscm, TSC_EN);

	return 0;
}

static void tsc_stop(struct input_dev *dev)//input设备关闭
{
	struct tsc_data *ts = input_get_drvdata(dev);

	tsc_clr_bits(ts, tscm, TSC_EN);
	synchronize_irq(ts->tsc_irq);//？接口的含义
	del_timer_sync(&ts->timer);//删除定时器
	clk_disable(ts->clk);//关闭时钟
}

static int __devinit tsc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tsc_data *ts;
	int error = 0;
	u32 rev = 0;

	ts = kzalloc(sizeof(struct tsc_data), GFP_KERNEL);
	if (!ts) {
		dev_err(dev, "cannot allocate device info\n");
		return -ENOMEM;
	}

	ts->dev = dev;
	spin_lock_init(&ts->lock);//自旋锁
	setup_timer(&ts->timer, tsc_poll, (unsigned long)ts);//定时器
	platform_set_drvdata(pdev, ts);//关联驱动上下文指针

	//平台中断资源
	ts->tsc_irq = platform_get_irq(pdev, 0);
	if (ts->tsc_irq < 0) {
		dev_err(dev, "cannot determine device interrupt\n");
		error = -ENODEV;
		goto error_res;
	}
	//平台内存资源
	ts->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ts->res) {
		dev_err(dev, "cannot determine register area\n");
		error = -ENODEV;
		goto error_res;
	}
	//需要申请一下
	if (!request_mem_region(ts->res->start, resource_size(ts->res),
				pdev->name)) {
		dev_err(dev, "cannot claim register memory\n");
		ts->res = NULL;
		error = -EINVAL;
		goto error_res;
	}
	//ioremap出来
	ts->regs = ioremap(ts->res->start, resource_size(ts->res));
	if (!ts->regs) {
		dev_err(dev, "cannot map register memory\n");
		error = -ENOMEM;
		goto error_map;
	}
	//时钟获取
	ts->clk = clk_get(dev, NULL);
	if (IS_ERR(ts->clk)) {
		dev_err(dev, "cannot claim device clock\n");
		error = PTR_ERR(ts->clk);
		goto error_clk;
	}
	//中断注册，普通中断
	error = request_threaded_irq(ts->tsc_irq, NULL, tsc_irq, 0,
				     dev_name(dev), ts);
	if (error < 0) {
		dev_err(ts->dev, "Could not allocate ts irq\n");
		goto error_irq;
	}
	//input设备
	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		dev_err(dev, "cannot allocate input device\n");
		error = -ENOMEM;
		goto error_input;
	}
	input_set_drvdata(ts->input_dev, ts);//input设备关联驱动上下指针

	ts->input_dev->name       = pdev->name;//input设备初始化
	ts->input_dev->id.bustype = BUS_HOST;//总线类似是HOST
	ts->input_dev->dev.parent = &pdev->dev;//平台设备的子设备是input设备
	ts->input_dev->open	  = tsc_start;//input设备的打开关闭实现
	ts->input_dev->close	  = tsc_stop;

	clk_enable(ts->clk);//激活时钟
	rev = tsc_read(ts, rev);
	ts->input_dev->id.product = ((rev >>  8) & 0x07);
	ts->input_dev->id.version = ((rev >> 16) & 0xfff);
	clk_disable(ts->clk);//关闭时钟

	__set_bit(EV_KEY,    ts->input_dev->evbit);
	__set_bit(EV_ABS,    ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_X, 0, 0xffff, 5, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, 0xffff, 5, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 4095, 128, 0);

	error = input_register_device(ts->input_dev);//input设备注册
	if (error < 0) {
		dev_err(dev, "failed input device registration\n");
		goto error_reg;
	}

	return 0;

error_reg:
	input_free_device(ts->input_dev);
error_input:
	free_irq(ts->tsc_irq, ts);
error_irq:
	clk_put(ts->clk);
error_clk:
	iounmap(ts->regs);
error_map:
	release_mem_region(ts->res->start, resource_size(ts->res));
error_res:
	platform_set_drvdata(pdev, NULL);
	kfree(ts);

	return error;
}

static int __devexit tsc_remove(struct platform_device *pdev)
{
	struct tsc_data *ts = platform_get_drvdata(pdev);

	input_unregister_device(ts->input_dev);
	free_irq(ts->tsc_irq, ts);
	clk_put(ts->clk);
	iounmap(ts->regs);
	release_mem_region(ts->res->start, resource_size(ts->res));
	platform_set_drvdata(pdev, NULL);
	kfree(ts);

	return 0;
}

static struct platform_driver tsc_driver = {
	.probe		= tsc_probe,
	.remove		= __devexit_p(tsc_remove),
	.driver.name	= "tnetv107x-ts",
	.driver.owner	= THIS_MODULE,
};
module_platform_driver(tsc_driver);

MODULE_AUTHOR("Cyril Chemparathy");
MODULE_DESCRIPTION("TNETV107X Touchscreen Driver");
MODULE_ALIAS("platform:tnetv107x-ts");
MODULE_LICENSE("GPL");
