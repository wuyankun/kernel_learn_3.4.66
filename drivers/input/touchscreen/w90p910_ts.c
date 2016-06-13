/*
 * Copyright (c) 2008 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/delay.h> //timer相关的定义
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>//时钟
#include <linux/input.h>//input设备
#include <linux/interrupt.h>//中断
#include <linux/slab.h>//kmalloc内存分配

/* ADC controller bit defines */
#define ADC_DELAY	0xf00
#define ADC_DOWN	0x01
#define ADC_TSC_Y	(0x01 << 8)
#define ADC_TSC_X	(0x00 << 8)
#define TSC_FOURWIRE	(~(0x03 << 1))
#define ADC_CLK_EN	(0x01 << 28)	/* ADC clock enable */
#define ADC_READ_CON	(0x01 << 12)
#define ADC_CONV	(0x01 << 13)
#define ADC_SEMIAUTO	(0x01 << 14)
#define ADC_WAITTRIG	(0x03 << 14)
#define ADC_RST1	(0x01 << 16)
#define ADC_RST0	(0x00 << 16)
#define ADC_EN		(0x01 << 17)
#define ADC_INT		(0x01 << 18)
#define WT_INT		(0x01 << 20)
#define ADC_INT_EN	(0x01 << 21)
#define LVD_INT_EN	(0x01 << 22)
#define WT_INT_EN	(0x01 << 23)
#define ADC_DIV		(0x04 << 1)	/* div = 6 */

enum ts_state {//触摸屏的全局状态
	TS_WAIT_NEW_PACKET,	/* We are waiting next touch report */
	TS_WAIT_X_COORD,	/* We are waiting for ADC to report X coord */
	TS_WAIT_Y_COORD,	/* We are waiting for ADC to report Y coord */
	TS_IDLE,		/* Input device is closed, don't do anything */
};

struct w90p910_ts {//驱动上下文环境
	struct input_dev *input;//逻辑划分为一个input设备
	struct timer_list timer;//包含一个定时器
	struct clk *clk;//管理时钟
	int irq_num;//中断号，中断线记录
	void __iomem *ts_reg;//IO内存记录
	spinlock_t lock;//自旋锁
	enum ts_state state;//全局的状态记录
};
//报告一个事件，提交一个input_event事件
static void w90p910_report_event(struct w90p910_ts *w90p910_ts, bool down)
{
	struct input_dev *dev = w90p910_ts->input;

	if (down) {
		input_report_abs(dev, ABS_X,
				 __raw_readl(w90p910_ts->ts_reg + 0x0c));
		input_report_abs(dev, ABS_Y,
				 __raw_readl(w90p910_ts->ts_reg + 0x10));
	}

	input_report_key(dev, BTN_TOUCH, down);
	input_sync(dev);
}

static void w90p910_prepare_x_reading(struct w90p910_ts *w90p910_ts)//先处于X方向上的数据
{
	unsigned long ctlreg;

	__raw_writel(ADC_TSC_X, w90p910_ts->ts_reg + 0x04);
	ctlreg = __raw_readl(w90p910_ts->ts_reg);
	ctlreg &= ~(ADC_WAITTRIG | WT_INT | WT_INT_EN);
	ctlreg |= ADC_SEMIAUTO | ADC_INT_EN | ADC_CONV;
	__raw_writel(ctlreg, w90p910_ts->ts_reg);

	w90p910_ts->state = TS_WAIT_X_COORD;//标记X方向数据正确，更新全局的设备状态
}

static void w90p910_prepare_y_reading(struct w90p910_ts *w90p910_ts)
{
	unsigned long ctlreg;

	__raw_writel(ADC_TSC_Y, w90p910_ts->ts_reg + 0x04);
	ctlreg = __raw_readl(w90p910_ts->ts_reg);
	ctlreg &= ~(ADC_WAITTRIG | ADC_INT | WT_INT_EN);
	ctlreg |= ADC_SEMIAUTO | ADC_INT_EN | ADC_CONV;
	__raw_writel(ctlreg, w90p910_ts->ts_reg);

	w90p910_ts->state = TS_WAIT_Y_COORD;//标记Y方向数据正确，更新全局的设备状态
}

static void w90p910_prepare_next_packet(struct w90p910_ts *w90p910_ts)
{
	unsigned long ctlreg;

	ctlreg = __raw_readl(w90p910_ts->ts_reg);
	ctlreg &= ~(ADC_INT | ADC_INT_EN | ADC_SEMIAUTO | ADC_CONV);
	ctlreg |= ADC_WAITTRIG | WT_INT_EN;
	__raw_writel(ctlreg, w90p910_ts->ts_reg);

	w90p910_ts->state = TS_WAIT_NEW_PACKET;
}

static irqreturn_t w90p910_ts_interrupt(int irq, void *dev_id)
{
	struct w90p910_ts *w90p910_ts = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&w90p910_ts->lock, flags);//不容许中断嵌套

	switch (w90p910_ts->state) {//根据硬件中断和当前设备处于的状态，来推动事件处理
	case TS_WAIT_NEW_PACKET://第一次中断来后，设备默认处于等待新的数据包
		/*
		 * The controller only generates interrupts when pen
		 * is down.
		 */
		del_timer(&w90p910_ts->timer);//先删除定时器
		w90p910_prepare_x_reading(w90p910_ts);//先去读取X方向的数据，再产生一次中断
		break;


	case TS_WAIT_X_COORD:
		w90p910_prepare_y_reading(w90p910_ts);//再去读取Y方向的数据，再产生一次中断
		break;

	case TS_WAIT_Y_COORD:
		w90p910_report_event(w90p910_ts, true);//上报一次数据
		w90p910_prepare_next_packet(w90p910_ts);//更新全局的状态，等待下一个包数据
		mod_timer(&w90p910_ts->timer, jiffies + msecs_to_jiffies(100));//重新启动定时器，当前时刻后的100ms
		break;

	case TS_IDLE:
		break;
	}

	spin_unlock_irqrestore(&w90p910_ts->lock, flags);

	return IRQ_HANDLED;
}

static void w90p910_check_pen_up(unsigned long data)//定时器处理程序，每隔一定间隔来查询，是否有ADC数据
{
	struct w90p910_ts *w90p910_ts = (struct w90p910_ts *) data;
	unsigned long flags;

	spin_lock_irqsave(&w90p910_ts->lock, flags);

	if (w90p910_ts->state == TS_WAIT_NEW_PACKET &&
	    !(__raw_readl(w90p910_ts->ts_reg + 0x04) & ADC_DOWN)) {

		w90p910_report_event(w90p910_ts, false);//当前处于等待新的数据包中，但没有新的数据产生
	}

	spin_unlock_irqrestore(&w90p910_ts->lock, flags);
}

static int w90p910_open(struct input_dev *dev)
{
	struct w90p910_ts *w90p910_ts = input_get_drvdata(dev);
	unsigned long val;

	/* enable the ADC clock */
	clk_enable(w90p910_ts->clk);

	__raw_writel(ADC_RST1, w90p910_ts->ts_reg);
	msleep(1);
	__raw_writel(ADC_RST0, w90p910_ts->ts_reg);
	msleep(1);

	/* set delay and screen type */
	val = __raw_readl(w90p910_ts->ts_reg + 0x04);
	__raw_writel(val & TSC_FOURWIRE, w90p910_ts->ts_reg + 0x04);
	__raw_writel(ADC_DELAY, w90p910_ts->ts_reg + 0x08);

	w90p910_ts->state = TS_WAIT_NEW_PACKET;//设备被打开后，设备处于等待新的数据包的状态
	wmb();//内存写入屏障的例子程序，不可以跨越写的顺序

	/* set trigger mode */
	val = __raw_readl(w90p910_ts->ts_reg);
	val |= ADC_WAITTRIG | ADC_DIV | ADC_EN | WT_INT_EN;
	__raw_writel(val, w90p910_ts->ts_reg);

	return 0;
}

static void w90p910_close(struct input_dev *dev)
{
	struct w90p910_ts *w90p910_ts = input_get_drvdata(dev);
	unsigned long val;

	/* disable trigger mode */

	spin_lock_irq(&w90p910_ts->lock);//关闭掉中断

	w90p910_ts->state = TS_IDLE;

	val = __raw_readl(w90p910_ts->ts_reg);
	val &= ~(ADC_WAITTRIG | ADC_DIV | ADC_EN | WT_INT_EN | ADC_INT_EN);
	__raw_writel(val, w90p910_ts->ts_reg);

	spin_unlock_irq(&w90p910_ts->lock);

	/* Now that interrupts are shut off we can safely delete timer */
	del_timer_sync(&w90p910_ts->timer);//删除掉timer定时器，同步的方式

	/* stop the ADC clock */
	clk_disable(w90p910_ts->clk);//关闭时钟
}

static int __devinit w90x900ts_probe(struct platform_device *pdev)
{
	struct w90p910_ts *w90p910_ts;
	struct input_dev *input_dev;
	struct resource *res;
	int err;

	w90p910_ts = kzalloc(sizeof(struct w90p910_ts), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!w90p910_ts || !input_dev) {
		err = -ENOMEM;
		goto fail1;
	}

	w90p910_ts->input = input_dev;
	w90p910_ts->state = TS_IDLE;//默认状态为待机状态
	spin_lock_init(&w90p910_ts->lock);
	setup_timer(&w90p910_ts->timer, w90p910_check_pen_up,
		    (unsigned long)w90p910_ts);//设置timer，初始化timer的结构体

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENXIO;
		goto fail1;
	}

	if (!request_mem_region(res->start, resource_size(res),
				pdev->name)) {
		err = -EBUSY;
		goto fail1;
	}

	w90p910_ts->ts_reg = ioremap(res->start, resource_size(res));
	if (!w90p910_ts->ts_reg) {
		err = -ENOMEM;
		goto fail2;
	}

	w90p910_ts->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(w90p910_ts->clk)) {
		err = PTR_ERR(w90p910_ts->clk);
		goto fail3;
	}

	input_dev->name = "W90P910 TouchScreen";
	input_dev->phys = "w90p910ts/event0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor  = 0x0005;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &pdev->dev;
	input_dev->open = w90p910_open;
	input_dev->close = w90p910_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, 0x400, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 0x400, 0, 0);

	input_set_drvdata(input_dev, w90p910_ts);

	w90p910_ts->irq_num = platform_get_irq(pdev, 0);//中断号从平台资源处获得
	if (request_irq(w90p910_ts->irq_num, w90p910_ts_interrupt,//激活中断，绑定中断处理程序和中断处理程序传入的资源
			0, "w90p910ts", w90p910_ts)) {
		err = -EBUSY;
		goto fail4;
	}

	err = input_register_device(w90p910_ts->input);
	if (err)
		goto fail5;

	platform_set_drvdata(pdev, w90p910_ts);

	return 0;

fail5:	free_irq(w90p910_ts->irq_num, w90p910_ts);
fail4:	clk_put(w90p910_ts->clk);
fail3:	iounmap(w90p910_ts->ts_reg);
fail2:	release_mem_region(res->start, resource_size(res));
fail1:	input_free_device(input_dev);
	kfree(w90p910_ts);
	return err;
}

static int __devexit w90x900ts_remove(struct platform_device *pdev)
{
	struct w90p910_ts *w90p910_ts = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(w90p910_ts->irq_num, w90p910_ts);//中断释放
	del_timer_sync(&w90p910_ts->timer);//删除定时器，同步的方式
	iounmap(w90p910_ts->ts_reg);//IO内存解除映射

	clk_put(w90p910_ts->clk);//停掉时钟

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));//是否内存区域

	input_unregister_device(w90p910_ts->input);//去注册input设备
	kfree(w90p910_ts);//释放内存

	platform_set_drvdata(pdev, NULL);//平台设备的私有数据设置为空

	return 0;
}

static struct platform_driver w90x900ts_driver = {
	.probe		= w90x900ts_probe,
	.remove		= __devexit_p(w90x900ts_remove),
	.driver		= {
		.name	= "nuc900-ts",
		.owner	= THIS_MODULE,
	},
};
module_platform_driver(w90x900ts_driver);

MODULE_AUTHOR("Wan ZongShun <mcuos.com@gmail.com>");
MODULE_DESCRIPTION("w90p910 touch screen driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc900-ts");
