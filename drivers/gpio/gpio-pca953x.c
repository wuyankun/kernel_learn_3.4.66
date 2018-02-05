/*
 *  PCA953x 4/8/16 bit I/O ports //TI公司的i2c转gpio的芯片
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/slab.h>
#ifdef CONFIG_OF_GPIO
#include <linux/of_platform.h>
#endif

#define PCA953X_INPUT		0 //pca953x的4个命令字
#define PCA953X_OUTPUT		1
#define PCA953X_INVERT		2
#define PCA953X_DIRECTION	3

#define PCA957X_IN		0
#define PCA957X_INVRT		1
#define PCA957X_BKEN		2
#define PCA957X_PUPD		3
#define PCA957X_CFG		4
#define PCA957X_OUT		5
#define PCA957X_MSK		6
#define PCA957X_INTS		7

#define PCA_GPIO_MASK		0x00FF
#define PCA_INT			0x0100
#define PCA953X_TYPE		0x1000
#define PCA957X_TYPE		0x2000

static const struct i2c_device_id pca953x_id[] = {//很多类似芯片
	{ "pca9534", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca9535", 16 | PCA953X_TYPE | PCA_INT, },
	{ "pca9536", 4  | PCA953X_TYPE, },
	{ "pca9537", 4  | PCA953X_TYPE | PCA_INT, },
	{ "pca9538", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca9539", 16 | PCA953X_TYPE | PCA_INT, },
	{ "pca9554", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca9555", 16 | PCA953X_TYPE | PCA_INT, },
	{ "pca9556", 8  | PCA953X_TYPE, },
	{ "pca9557", 8  | PCA953X_TYPE, },
	{ "pca9574", 8  | PCA957X_TYPE | PCA_INT, },
	{ "pca9575", 16 | PCA957X_TYPE | PCA_INT, },

	{ "max7310", 8  | PCA953X_TYPE, },
	{ "max7312", 16 | PCA953X_TYPE | PCA_INT, },
	{ "max7313", 16 | PCA953X_TYPE | PCA_INT, },
	{ "max7315", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca6107", 8  | PCA953X_TYPE | PCA_INT, },
	{ "tca6408", 8  | PCA953X_TYPE | PCA_INT, },
	{ "tca6416", 16 | PCA953X_TYPE | PCA_INT, },
	/* NYET:  { "tca6424", 24, }, */
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca953x_id);

struct pca953x_chip {//驱动上下文
	unsigned gpio_start;///sys/class/gpio/gpiochipN gpio标号起始
	uint16_t reg_output;
	uint16_t reg_direction;
	struct mutex i2c_lock;

#ifdef CONFIG_GPIO_PCA953X_IRQ
	struct mutex irq_lock;
	uint16_t irq_mask;
	uint16_t irq_stat;
	uint16_t irq_trig_raise;
	uint16_t irq_trig_fall;
	int	 irq_base;
#endif

	struct i2c_client *client;//i2c从设备，i2c客户端
	struct gpio_chip gpio_chip;//本质是个gpio管脚扩展芯片
	const char *const *names;
	int	chip_type;
};

static int pca953x_write_reg(struct pca953x_chip *chip, int reg, uint16_t val)
{
	int ret = 0;

	if (chip->gpio_chip.ngpio <= 8)//如果芯片的gpio管脚小于等于8个，直接写值
		ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	else {
		switch (chip->chip_type) {
		case PCA953X_TYPE:
			ret = i2c_smbus_write_word_data(chip->client,
							reg << 1, val);//如果是953x系列的，大于8个gpio管脚,reg<<1乘上2
			break;
		case PCA957X_TYPE:
			ret = i2c_smbus_write_byte_data(chip->client, reg << 1,
							val & 0xff);//reg<<1 写LSB，按字节写
			if (ret < 0)
				break;
			ret = i2c_smbus_write_byte_data(chip->client,
							(reg << 1) + 1,//reg <<1 +1 ,写MSB，按字节写，拆分
							(val & 0xff00) >> 8);
			break;
		}
	}

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int pca953x_read_reg(struct pca953x_chip *chip, int reg, uint16_t *val)
{
	int ret;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_read_byte_data(chip->client, reg);//小于等于8个管脚，直接读取值,读字节
	else
		ret = i2c_smbus_read_word_data(chip->client, reg << 1);//否则真实读取的是reg<<1的reg值，读双字

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	*val = (uint16_t)ret;
	return 0;
}

static int pca953x_gpio_direction_input(struct gpio_chip *gc, unsigned off)//设置某个管脚为输入管脚,输入管脚不带值
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);//找到驱动上下文

	mutex_lock(&chip->i2c_lock);//上锁
	reg_val = chip->reg_direction | (1u << off);//更新方向的变量值，全局记录所有的管脚方向

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_DIRECTION;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_CFG;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);//写寄存器值
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;//成功后更新记录的全局变量
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pca953x_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)//设备某个管脚为输出管脚，值为高还是低
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);//上下文

	mutex_lock(&chip->i2c_lock);//上锁
	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << off);//更新方向信息
	else
		reg_val = chip->reg_output & ~(1u << off);

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_OUTPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_OUT;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << off);
	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_DIRECTION;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_CFG;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);//更新值的信息
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pca953x_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);//找到驱动上下文，根据地址偏移量计算

	mutex_lock(&chip->i2c_lock);
	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_INPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_IN;
		break;
	}
	ret = pca953x_read_reg(chip, offset, &reg_val);//读取到寄存器值
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;//根据具体bit位，判断是返回1还是0
}

static void pca953x_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);//找到驱动上下文

	mutex_lock(&chip->i2c_lock);//互斥访问，防止其他人同时进行操作
	if (val)//on or off
		reg_val = chip->reg_output | (1u << off);//设置某位为高电平
	else
		reg_val = chip->reg_output & ~(1u << off);//设置某位为低电平

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_OUTPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_OUT;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);//写寄存器
	if (ret)
		goto exit;

	chip->reg_output = reg_val;//更新上下文环境，用一变量记录当前所有管脚的电平信息
exit:
	mutex_unlock(&chip->i2c_lock);
}

static void pca953x_setup_gpio(struct pca953x_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = pca953x_gpio_direction_input;//gpio_chip需要实现填充，4个函数实现
	gc->direction_output = pca953x_gpio_direction_output;
	gc->get = pca953x_gpio_get_value;
	gc->set = pca953x_gpio_set_value;
	gc->can_sleep = 1;//是否可以sleep

	gc->base = chip->gpio_start;//标号开始
	gc->ngpio = gpios;//共支持多少个gpio管脚
	gc->label = chip->client->name;//设备标签，用i2c设备的名字，默认为路径
	gc->dev = &chip->client->dev;//dev关联起来
	gc->owner = THIS_MODULE;//所有者和名字
	gc->names = chip->names;
}

#ifdef CONFIG_GPIO_PCA953X_IRQ
static int pca953x_gpio_to_irq(struct gpio_chip *gc, unsigned off)//gpio_to_irq支持
{
	struct pca953x_chip *chip;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);
	return chip->irq_base + off;//芯片的irq基数+偏移量
}

static void pca953x_irq_mask(struct irq_data *d)//mask
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);//从ird_data找到上下文

	chip->irq_mask &= ~(1 << (d->irq - chip->irq_base));//用偏移量来计算需要屏蔽掉哪个管脚
}

static void pca953x_irq_unmask(struct irq_data *d)//unmask
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);

	chip->irq_mask |= 1 << (d->irq - chip->irq_base);
}

static void pca953x_irq_bus_lock(struct irq_data *d)//锁住irq
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);

	mutex_lock(&chip->irq_lock);
}

static void pca953x_irq_bus_sync_unlock(struct irq_data *d)//同步解锁
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);
	uint16_t new_irqs;
	uint16_t level;

	/* Look for any newly setup interrupt */
	new_irqs = chip->irq_trig_fall | chip->irq_trig_raise;//获取两种中断触发的值|
	new_irqs &= ~chip->reg_direction;

	while (new_irqs) {//变量每一个bit位的值
		level = __ffs(new_irqs);
		pca953x_gpio_direction_input(&chip->gpio_chip, level);
		new_irqs &= ~(1 << level);
	}

	mutex_unlock(&chip->irq_lock);
}

static int pca953x_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);
	uint16_t level = d->irq - chip->irq_base;//得到偏移量，得到是哪个管脚
	uint16_t mask = 1 << level;//转换为对应的bit值

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_FALLING)
		chip->irq_trig_fall |= mask;//两个边沿触发全局变量
	else
		chip->irq_trig_fall &= ~mask;

	if (type & IRQ_TYPE_EDGE_RISING)
		chip->irq_trig_raise |= mask;
	else
		chip->irq_trig_raise &= ~mask;

	return 0;
}

static struct irq_chip pca953x_irq_chip = {//irq_chip结构体初始化，如果是一个irq的芯片,是否屏蔽中断，是否上锁
	.name			= "pca953x",
	.irq_mask		= pca953x_irq_mask,
	.irq_unmask		= pca953x_irq_unmask,
	.irq_bus_lock		= pca953x_irq_bus_lock,
	.irq_bus_sync_unlock	= pca953x_irq_bus_sync_unlock,
	.irq_set_type		= pca953x_irq_set_type,
};

static uint16_t pca953x_irq_pending(struct pca953x_chip *chip)
{
	uint16_t cur_stat;
	uint16_t old_stat;
	uint16_t pending;
	uint16_t trigger;
	int ret, offset = 0;

	switch (chip->chip_type) {//只有输入脚才可能为中断脚
	case PCA953X_TYPE:
		offset = PCA953X_INPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_IN;
		break;
	}
	ret = pca953x_read_reg(chip, offset, &cur_stat);
	if (ret)
		return 0;

	/* Remove output pins from the equation */
	cur_stat &= chip->reg_direction;

	old_stat = chip->irq_stat;
	trigger = (cur_stat ^ old_stat) & chip->irq_mask;//读取旧的和新的比较，然后判断是否满足条件

	if (!trigger)
		return 0;

	chip->irq_stat = cur_stat;

	pending = (old_stat & chip->irq_trig_fall) |
		  (cur_stat & chip->irq_trig_raise);
	pending &= trigger;

	return pending;//是否满足了中断条件
}

static irqreturn_t pca953x_irq_handler(int irq, void *devid)//中断处理函数
{
	struct pca953x_chip *chip = devid;
	uint16_t pending;
	uint16_t level;

	pending = pca953x_irq_pending(chip);

	if (!pending)
		return IRQ_HANDLED;

	do {
		level = __ffs(pending);
		handle_nested_irq(level + chip->irq_base);//处理对应管脚上的中断服务

		pending &= ~(1 << level);
	} while (pending);//依次处理满足条件的管脚

	return IRQ_HANDLED;
}

static int pca953x_irq_setup(struct pca953x_chip *chip,
			     const struct i2c_device_id *id,
			     int irq_base)
{
	struct i2c_client *client = chip->client;
	int ret, offset = 0;

	if (irq_base != -1
			&& (id->driver_data & PCA_INT)) {//判断irq_base不等于-1，且芯片声明是支持中断功能的，如果是-1，芯片的中断发给谁
		int lvl;

		switch (chip->chip_type) {
		case PCA953X_TYPE:
			offset = PCA953X_INPUT;
			break;
		case PCA957X_TYPE:
			offset = PCA957X_IN;
			break;
		}
		ret = pca953x_read_reg(chip, offset, &chip->irq_stat);//读取输入管脚的值
		if (ret)
			goto out_failed;

		/*
		 * There is no way to know which GPIO line generated the
		 * interrupt.  We have to rely on the previous read for
		 * this purpose.
		 */
		chip->irq_stat &= chip->reg_direction;//符合方向的，得到初始化状态
		mutex_init(&chip->irq_lock);

		chip->irq_base = irq_alloc_descs(-1, irq_base, chip->gpio_chip.ngpio, -1);//申请ngpio个中断资源
		if (chip->irq_base < 0)//irq_base是中断的基数
			goto out_failed;

		for (lvl = 0; lvl < chip->gpio_chip.ngpio; lvl++) {
			int irq = lvl + chip->irq_base;

			irq_clear_status_flags(irq, IRQ_NOREQUEST);//通用中断接口
			irq_set_chip_data(irq, chip);//irq_data到irq_chip
			irq_set_chip(irq, &pca953x_irq_chip);
			irq_set_nested_thread(irq, true);
#ifdef CONFIG_ARM
			set_irq_flags(irq, IRQF_VALID);
#else
			irq_set_noprobe(irq);
#endif
		}
		//采用中断线程的方式
		ret = request_threaded_irq(client->irq,//芯片的中断
					   NULL,
					   pca953x_irq_handler,//中断处理函数
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,//低电平触发，只命中一次
					   dev_name(&client->dev), chip);
		if (ret) {
			dev_err(&client->dev, "failed to request irq %d\n",
				client->irq);
			goto out_failed;
		}

		chip->gpio_chip.to_irq = pca953x_gpio_to_irq;//接口函数初始化
	}

	return 0;

out_failed:
	chip->irq_base = -1;
	return ret;
}

static void pca953x_irq_teardown(struct pca953x_chip *chip)
{
	if (chip->irq_base != -1) {
		irq_free_descs(chip->irq_base, chip->gpio_chip.ngpio);//中断资源ngpio
		free_irq(chip->client->irq, chip);//芯片本身的中断
	}
}
#else /* CONFIG_GPIO_PCA953X_IRQ */
static int pca953x_irq_setup(struct pca953x_chip *chip,
			     const struct i2c_device_id *id,
			     int irq_base)
{
	struct i2c_client *client = chip->client;

	if (irq_base != -1 && (id->driver_data & PCA_INT))
		dev_warn(&client->dev, "interrupt support not compiled in\n");

	return 0;
}

static void pca953x_irq_teardown(struct pca953x_chip *chip)
{
}
#endif

/*
 * Handlers for alternative sources of platform_data
 */
#ifdef CONFIG_OF_GPIO
/*
 * Translate OpenFirmware node properties into platform_data
 * WARNING: This is DEPRECATED and will be removed eventually!
 */
static void
pca953x_get_alt_pdata(struct i2c_client *client, int *gpio_base, int *invert)
{
	struct device_node *node;
	const __be32 *val;
	int size;

	node = client->dev.of_node;
	if (node == NULL)
		return;

	*gpio_base = -1;
	val = of_get_property(node, "linux,gpio-base", &size);
	WARN(val, "%s: device-tree property 'linux,gpio-base' is deprecated!", __func__);
	if (val) {
		if (size != sizeof(*val))
			dev_warn(&client->dev, "%s: wrong linux,gpio-base\n",
				 node->full_name);
		else
			*gpio_base = be32_to_cpup(val);//从设备树中得到gpio_base的值，以及优化级别
	}

	val = of_get_property(node, "polarity", NULL);
	WARN(val, "%s: device-tree property 'polarity' is deprecated!", __func__);
	if (val)
		*invert = *val;
}
#else
static void
pca953x_get_alt_pdata(struct i2c_client *client, int *gpio_base, int *invert)
{
	*gpio_base = -1;
}
#endif

static int __devinit device_pca953x_init(struct pca953x_chip *chip, int invert)
{
	int ret;

	ret = pca953x_read_reg(chip, PCA953X_OUTPUT, &chip->reg_output);
	if (ret)
		goto out;

	ret = pca953x_read_reg(chip, PCA953X_DIRECTION,
			       &chip->reg_direction);
	if (ret)
		goto out;

	/* set platform specific polarity inversion */
	ret = pca953x_write_reg(chip, PCA953X_INVERT, invert);//初始化芯片，设置极性，即默认的高低电平
out:
	return ret;
}

static int __devinit device_pca957x_init(struct pca953x_chip *chip, int invert)
{
	int ret;
	uint16_t val = 0;

	/* Let every port in proper state, that could save power */
	pca953x_write_reg(chip, PCA957X_PUPD, 0x0);//节能
	pca953x_write_reg(chip, PCA957X_CFG, 0xffff);
	pca953x_write_reg(chip, PCA957X_OUT, 0x0);

	ret = pca953x_read_reg(chip, PCA957X_IN, &val);
	if (ret)
		goto out;
	ret = pca953x_read_reg(chip, PCA957X_OUT, &chip->reg_output);
	if (ret)
		goto out;
	ret = pca953x_read_reg(chip, PCA957X_CFG, &chip->reg_direction);
	if (ret)
		goto out;

	/* set platform specific polarity inversion */
	pca953x_write_reg(chip, PCA957X_INVRT, invert);

	/* To enable register 6, 7 to controll pull up and pull down */
	pca953x_write_reg(chip, PCA957X_BKEN, 0x202);

	return 0;
out:
	return ret;
}

static int __devinit pca953x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)//探测，本质上为i2c设备
{
	struct pca953x_platform_data *pdata;//附带数据
	struct pca953x_chip *chip;//驱动上下文
	int irq_base=0, invert=0;
	int ret;

	chip = kzalloc(sizeof(struct pca953x_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	pdata = client->dev.platform_data;//如果有平台数据
	if (pdata) {
		irq_base = pdata->irq_base;//中断和gpio的基数是通过平台设备的数据初始化的
		chip->gpio_start = pdata->gpio_base;
		invert = pdata->invert;
		chip->names = pdata->names;
	} else {
		pca953x_get_alt_pdata(client, &chip->gpio_start, &invert);
#ifdef CONFIG_OF_GPIO
		/* If I2C node has no interrupts property, disable GPIO interrupts */
		if (of_find_property(client->dev.of_node, "interrupts", NULL) == NULL)
			irq_base = -1;
#endif
	}

	chip->client = client;//关联i2c设备，互斥初始化，芯片类型初始化等

	chip->chip_type = id->driver_data & (PCA953X_TYPE | PCA957X_TYPE);

	mutex_init(&chip->i2c_lock);

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	pca953x_setup_gpio(chip, id->driver_data & PCA_GPIO_MASK);

	if (chip->chip_type == PCA953X_TYPE)//芯片状态初始化
		ret = device_pca953x_init(chip, invert);
	else
		ret = device_pca957x_init(chip, invert);
	if (ret)
		goto out_failed;

	ret = pca953x_irq_setup(chip, id, irq_base);//把irq_base传入，申请中断资源等
	if (ret)
		goto out_failed;

	ret = gpiochip_add(&chip->gpio_chip);//添加一个gpio_chip
	if (ret)
		goto out_failed_irq;

	if (pdata && pdata->setup) {//如果有setup接口，就调用
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);//i2c的私有数据关联
	return 0;

out_failed_irq:
	pca953x_irq_teardown(chip);
out_failed:
	kfree(chip);
	return ret;
}

static int pca953x_remove(struct i2c_client *client)
{
	struct pca953x_platform_data *pdata = client->dev.platform_data;
	struct pca953x_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	if (pdata && pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
			return ret;
		}
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
		return ret;
	}

	pca953x_irq_teardown(chip);
	kfree(chip);
	return 0;
}

static struct i2c_driver pca953x_driver = {//i2c驱动结构体
	.driver = {
		.name	= "pca953x",
	},
	.probe		= pca953x_probe,
	.remove		= pca953x_remove,
	.id_table	= pca953x_id,
};

static int __init pca953x_init(void)
{
	return i2c_add_driver(&pca953x_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(pca953x_init);

static void __exit pca953x_exit(void)
{
	i2c_del_driver(&pca953x_driver);
}
module_exit(pca953x_exit);

MODULE_AUTHOR("eric miao <eric.miao@marvell.com>");
MODULE_DESCRIPTION("GPIO expander driver for PCA953x");
MODULE_LICENSE("GPL");
