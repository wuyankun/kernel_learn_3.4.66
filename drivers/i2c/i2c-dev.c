/*
    i2c-dev.c - i2c-bus driver, char device interface

    Copyright (C) 1995-97 Simon G. Vogl
    Copyright (C) 1998-99 Frodo Looijaard <frodol@dds.nl>
    Copyright (C) 2003 Greg Kroah-Hartman <greg@kroah.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

/* Note that this is a complete rewrite of Simon Vogl's i2c-dev module.
   But I have used so much of his original code and ideas that it seems
   only fair to recognize him as co-author -- Frodo */

/* The I2C_RDWR ioctl code is written by Kolja Waschk <waschk@telos.de> */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/slab.h>//内存分配
#include <linux/init.h>
#include <linux/list.h>//列表接口
#include <linux/i2c.h>//i2c的命令字定义
#include <linux/i2c-dev.h>//
#include <linux/jiffies.h>//间隔
#include <linux/uaccess.h>

/*
 * An i2c_dev represents an i2c_adapter ... an I2C or SMBus master, not a
 * slave (i2c_client) with which messages will be exchanged.  It's coupled
 * with a character special file which is accessed by user mode drivers.
 *
 * The list of i2c_dev structures is parallel to the i2c_adapter lists
 * maintained by the driver model, and is updated using bus notifications.
 */
 //i2c 设备代表的是适配器，即I2c或者SMbus的总控芯片，不是从设备
 //表现为一个特殊的字符设备，用来提供给用户空间的驱动来控制使用
struct i2c_dev {
	struct list_head list;//构建双向链表，连接到适配器总的链表中
	struct i2c_adapter *adap;//本质是一个i2c的适配器
	struct device *dev;//设备的基本属性
};

#define I2C_MINORS	256
static LIST_HEAD(i2c_dev_list);//全局的i2c_dev的链表，驱动用来管理
static DEFINE_SPINLOCK(i2c_dev_list_lock);//链表操作的自旋锁

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)//通过次设备号来寻找主设备
{
	struct i2c_dev *i2c_dev;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list) {
		if (i2c_dev->adap->nr == index)//遍历寻找到对应的设备，依次递增，按照次设备号就能找到对应的适配器
			goto found;
	}
	i2c_dev = NULL;
found:
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {//字符设备最大的次设备数量为256
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
		       adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);//分配一个新的设备上下文
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);//增加到全局的设备链表的末尾
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);//移除一个i2c_dev，从链表中删除特定的i2c_dev
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

static ssize_t show_adapter_name(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_dev *i2c_dev = i2c_dev_get_by_minor(MINOR(dev->devt));

	if (!i2c_dev)
		return -ENODEV;
	return sprintf(buf, "%s\n", i2c_dev->adap->name);
}
static DEVICE_ATTR(name, S_IRUGO, show_adapter_name, NULL);

/* ------------------------------------------------------------------------- */

/*
 * After opening an instance of this character special file, a file
 * descriptor starts out associated only with an i2c_adapter (and bus).
 *
 * Using the I2C_RDWR ioctl(), you can then *immediately* issue i2c_msg
 * traffic to any devices on the bus used by that adapter.  That's because
 * the i2c_msg vectors embed all the addressing information they need, and
 * are submitted directly to an i2c_adapter.  However, SMBus-only adapters
 * don't support that interface.
 *
 * To use read()/write() system calls on that file descriptor, or to use
 * SMBus interfaces (and work with SMBus-only hosts!), you must first issue
 * an I2C_SLAVE (or I2C_SLAVE_FORCE) ioctl.  That configures an anonymous
 * (never registered) i2c_client so it holds the addressing information
 * needed by those system calls and by this SMBus interface.
 */
//i2c的适配器可以立即通过i2c_rdwr的ioctl命令来操作对应总线上挂载的i2c从设备
//如果使用的是SMBus适配器，则需要首选通过i2c_slave或者i2c_slave_force来制定接下来
//需要操作的从设备地址
static ssize_t i2cdev_read(struct file *file, char __user *buf, size_t count,
		loff_t *offset)
{
	char *tmp;
	int ret;

	struct i2c_client *client = file->private_data;//将当期需要操作的从器件的地址保存在私有变量中

	if (count > 8192)//这里为啥要限制读取的最大长度为8192Byte?
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	pr_debug("i2c-dev: i2c-%d reading %zu bytes.\n",
		iminor(file->f_path.dentry->d_inode), count);

	ret = i2c_master_recv(client, tmp, count);//直接调用这个函数来读取总线上的数据
	if (ret >= 0)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;
	kfree(tmp);
	return ret;
}

static ssize_t i2cdev_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	int ret;
	char *tmp;
	struct i2c_client *client = file->private_data;

	if (count > 8192)
		count = 8192;

	tmp = memdup_user(buf, count);//从用户空间拷贝数据过来，这里没有用copy_from_user
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	pr_debug("i2c-dev: i2c-%d writing %zu bytes.\n",
		iminor(file->f_path.dentry->d_inode), count);

	ret = i2c_master_send(client, tmp, count);
	kfree(tmp);
	return ret;
}

static int i2cdev_check(struct device *dev, void *addrp)
{
	struct i2c_client *client = i2c_verify_client(dev);

	if (!client || client->addr != *(unsigned int *)addrp)//从器件地址不为空，且保存的适配器地址正确
		return 0;

	return dev->driver ? -EBUSY : 0;
}

/* walk up mux tree */
static int i2cdev_check_mux_parents(struct i2c_adapter *adapter, int addr)
{
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result;

	result = device_for_each_child(&adapter->dev, &addr, i2cdev_check);
	if (!result && parent)
		result = i2cdev_check_mux_parents(parent, addr);

	return result;
}

/* recurse down mux tree */
static int i2cdev_check_mux_children(struct device *dev, void *addrp)
{
	int result;

	if (dev->type == &i2c_adapter_type)
		result = device_for_each_child(dev, addrp,
						i2cdev_check_mux_children);
	else
		result = i2cdev_check(dev, addrp);

	return result;
}

/* This address checking function differs from the one in i2c-core
   in that it considers an address with a registered device, but no
   driver bound to it, as NOT busy. */
static int i2cdev_check_addr(struct i2c_adapter *adapter, unsigned int addr)
{
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result = 0;

	if (parent)
		result = i2cdev_check_mux_parents(parent, addr);

	if (!result)
		result = device_for_each_child(&adapter->dev, &addr,
						i2cdev_check_mux_children);

	return result;
}

static noinline int i2cdev_ioctl_rdrw(struct i2c_client *client,
		unsigned long arg)
{
	struct i2c_rdwr_ioctl_data rdwr_arg;//应用空间传递的参数的结构体
	struct i2c_msg *rdwr_pa;//i2c消息的结构体
	u8 __user **data_ptrs;
	int i, res;

	if (copy_from_user(&rdwr_arg,
			   (struct i2c_rdwr_ioctl_data __user *)arg,
			   sizeof(rdwr_arg)))
		return -EFAULT;

	/* Put an arbitrary limit on the number of messages that can
	 * be sent at once */
	if (rdwr_arg.nmsgs > I2C_RDRW_IOCTL_MAX_MSGS)//规定最大的传输大小限制
		return -EINVAL;

	rdwr_pa = memdup_user(rdwr_arg.msgs,//拷贝出纯正的i2c_msg的数据
			      rdwr_arg.nmsgs * sizeof(struct i2c_msg));
	if (IS_ERR(rdwr_pa))
		return PTR_ERR(rdwr_pa);

	data_ptrs = kmalloc(rdwr_arg.nmsgs * sizeof(u8 __user *), GFP_KERNEL);
	if (data_ptrs == NULL) {
		kfree(rdwr_pa);
		return -ENOMEM;
	}

	res = 0;
	for (i = 0; i < rdwr_arg.nmsgs; i++) {
		/* Limit the size of the message to a sane amount;
		 * and don't let length change either. */
		if ((rdwr_pa[i].len > 8192) ||
		    (rdwr_pa[i].flags & I2C_M_RECV_LEN)) {
			res = -EINVAL;
			break;
		}
		data_ptrs[i] = (u8 __user *)rdwr_pa[i].buf;
		rdwr_pa[i].buf = memdup_user(data_ptrs[i], rdwr_pa[i].len);
		if (IS_ERR(rdwr_pa[i].buf)) {
			res = PTR_ERR(rdwr_pa[i].buf);
			break;
		}
	}
	if (res < 0) {
		int j;
		for (j = 0; j < i; ++j)
			kfree(rdwr_pa[j].buf);
		kfree(data_ptrs);
		kfree(rdwr_pa);
		return res;
	}

	res = i2c_transfer(client->adapter, rdwr_pa, rdwr_arg.nmsgs);//调用底层的transfer接口传输数据，这里看来不是纯正的适配器驱动
	while (i-- > 0) {//只是将适配器的驱动增加一个接口层
		if (res >= 0 && (rdwr_pa[i].flags & I2C_M_RD)) {
			if (copy_to_user(data_ptrs[i], rdwr_pa[i].buf,
					 rdwr_pa[i].len))
				res = -EFAULT;
		}
		kfree(rdwr_pa[i].buf);
	}
	kfree(data_ptrs);
	kfree(rdwr_pa);
	return res;
}

static noinline int i2cdev_ioctl_smbus(struct i2c_client *client,
		unsigned long arg)
{
	struct i2c_smbus_ioctl_data data_arg;
	union i2c_smbus_data temp;
	int datasize, res;

	if (copy_from_user(&data_arg,
			   (struct i2c_smbus_ioctl_data __user *) arg,
			   sizeof(struct i2c_smbus_ioctl_data)))
		return -EFAULT;
	if ((data_arg.size != I2C_SMBUS_BYTE) &&
	    (data_arg.size != I2C_SMBUS_QUICK) &&
	    (data_arg.size != I2C_SMBUS_BYTE_DATA) &&
	    (data_arg.size != I2C_SMBUS_WORD_DATA) &&
	    (data_arg.size != I2C_SMBUS_PROC_CALL) &&
	    (data_arg.size != I2C_SMBUS_BLOCK_DATA) &&
	    (data_arg.size != I2C_SMBUS_I2C_BLOCK_BROKEN) &&
	    (data_arg.size != I2C_SMBUS_I2C_BLOCK_DATA) &&
	    (data_arg.size != I2C_SMBUS_BLOCK_PROC_CALL)) {
		dev_dbg(&client->adapter->dev,
			"size out of range (%x) in ioctl I2C_SMBUS.\n",
			data_arg.size);
		return -EINVAL;
	}//SMBus只能操作特定的数据字节大小类型
	/* Note that I2C_SMBUS_READ and I2C_SMBUS_WRITE are 0 and 1,
	   so the check is valid if size==I2C_SMBUS_QUICK too. */
	if ((data_arg.read_write != I2C_SMBUS_READ) &&
	    (data_arg.read_write != I2C_SMBUS_WRITE)) {
		dev_dbg(&client->adapter->dev,
			"read_write out of range (%x) in ioctl I2C_SMBUS.\n",
			data_arg.read_write);
		return -EINVAL;
	}//只有读写两个方向，检查参数是否合法

	/* Note that command values are always valid! */

	if ((data_arg.size == I2C_SMBUS_QUICK) ||
	    ((data_arg.size == I2C_SMBUS_BYTE) &&
	    (data_arg.read_write == I2C_SMBUS_WRITE)))
		/* These are special: we do not use data *///这里如何理解？写字节，却不用数据，Quick命令等同写字节
		return i2c_smbus_xfer(client->adapter, client->addr,
				      client->flags, data_arg.read_write,
				      data_arg.command, data_arg.size, NULL);

	if (data_arg.data == NULL) {//其他情况，则数据不能为空
		dev_dbg(&client->adapter->dev,
			"data is NULL pointer in ioctl I2C_SMBUS.\n");
		return -EINVAL;
	}

	if ((data_arg.size == I2C_SMBUS_BYTE_DATA) ||
	    (data_arg.size == I2C_SMBUS_BYTE))
		datasize = sizeof(data_arg.data->byte);
	else if ((data_arg.size == I2C_SMBUS_WORD_DATA) ||
		 (data_arg.size == I2C_SMBUS_PROC_CALL))
		datasize = sizeof(data_arg.data->word);
	else /* size == smbus block, i2c block, or block proc. call */
		datasize = sizeof(data_arg.data->block);

	if ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
	    (data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
	    (data_arg.size == I2C_SMBUS_I2C_BLOCK_DATA) ||
	    (data_arg.read_write == I2C_SMBUS_WRITE)) {
		if (copy_from_user(&temp, data_arg.data, datasize))
			return -EFAULT;
	}
	if (data_arg.size == I2C_SMBUS_I2C_BLOCK_BROKEN) {//特殊处理
		/* Convert old I2C block commands to the new
		   convention. This preserves binary compatibility. */
		data_arg.size = I2C_SMBUS_I2C_BLOCK_DATA;
		if (data_arg.read_write == I2C_SMBUS_READ)
			temp.block[0] = I2C_SMBUS_BLOCK_MAX;
	}
	res = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
	      data_arg.read_write, data_arg.command, data_arg.size, &temp);//真正读写的底层接口
	if (!res && ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
		     (data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
		     (data_arg.read_write == I2C_SMBUS_READ))) {
		if (copy_to_user(data_arg.data, &temp, datasize))//若为读取，则拷贝给用户空间
			return -EFAULT;
	}
	return res;
}

static long i2cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = file->private_data;//从上下文环境中拿到虚拟的从器件(未初始化完全的从器件)
	unsigned long funcs;

	dev_dbg(&client->adapter->dev, "ioctl, cmd=0x%02x, arg=0x%02lx\n",
		cmd, arg);

	switch (cmd) {
	case I2C_SLAVE:
	case I2C_SLAVE_FORCE:
		/* NOTE:  devices set up to work with "new style" drivers
		 * can't use I2C_SLAVE, even when the device node is not
		 * bound to a driver.  Only I2C_SLAVE_FORCE will work.
		 *
		 * Setting the PEC flag here won't affect kernel drivers,
		 * which will be using the i2c_client node registered with
		 * the driver model core.  Likewise, when that client has
		 * the PEC flag already set, the i2c-dev driver won't see
		 * (or use) this setting.
		 */
		 //一些SMBus设备支持PEC校验标识(多出一个字节长度)，一般用奇偶校验实现
		 //某些新类型的设备，需要使用I2C_SLAVE_FORCE
		if ((arg > 0x3ff) ||
		    (((client->flags & I2C_M_TEN) == 0) && arg > 0x7f))//如果未制定使用10bit的数据长度，检查传入的地址数据是否有效
			return -EINVAL;
		if (cmd == I2C_SLAVE && i2cdev_check_addr(client->adapter, arg))
			return -EBUSY;
		/* REVISIT: address could become busy later */
		client->addr = arg;
		return 0;
	case I2C_TENBIT:
		if (arg)
			client->flags |= I2C_M_TEN;//是否使用10bit地址长度，更新上下文环境
		else
			client->flags &= ~I2C_M_TEN;
		return 0;
	case I2C_PEC:
		if (arg)
			client->flags |= I2C_CLIENT_PEC;//是否使用校验标识
		else
			client->flags &= ~I2C_CLIENT_PEC;
		return 0;
	case I2C_FUNCS:
		funcs = i2c_get_functionality(client->adapter);//获取适配器支持的特征属性
		return put_user(funcs, (unsigned long __user *)arg);//标识位代表适配器具有不同的功能

	case I2C_RDWR:
		return i2cdev_ioctl_rdrw(client, arg);//rdwr为纯正的i2c设备使用，内部使用的接口不同

	case I2C_SMBUS:
		return i2cdev_ioctl_smbus(client, arg);//smbus为纯正的SMBus设备使用，内部使用的更底层的translate接口不同

	case I2C_RETRIES:
		client->adapter->retries = arg;//设置translate重试次数
		break;
	case I2C_TIMEOUT:
		/* For historical reasons, user-space sets the timeout
		 * value in units of 10 ms.
		 */
		client->adapter->timeout = msecs_to_jiffies(arg * 10);//以10ms为单位，设置通信的超声时间
		break;
	default:
		/* NOTE:  returning a fault code here could cause trouble
		 * in buggy userspace code.  Some old kernel bugs returned
		 * zero in this case, and userspace code might accidentally
		 * have depended on that bug.
		 */
		return -ENOTTY;//其他的ioctl命令返回没有tty
	}
	return 0;
}

static int i2cdev_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);//通过inode得到次设备号
	struct i2c_client *client;
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;

	i2c_dev = i2c_dev_get_by_minor(minor);//通过次设备号，找到设备的上下文环境
	if (!i2c_dev)
		return -ENODEV;

	adap = i2c_get_adapter(i2c_dev->adap->nr);//根据适配器的编号，找到对应的适配器
	if (!adap)
		return -ENODEV;

	/* This creates an anonymous i2c_client, which may later be
	 * pointed to some address using I2C_SLAVE or I2C_SLAVE_FORCE.
	 *
	 * This client is ** NEVER REGISTERED ** with the driver model
	 * or I2C core code!!  It just holds private copies of addressing
	 * information and maybe a PEC flag.
	 */
	client = kzalloc(sizeof(*client), GFP_KERNEL);//先虚构一个i2c的从器件，稍后会被指定从器件的地址
	if (!client) {
		i2c_put_adapter(adap);//如果不能创建这个从器件，则销毁这个适配器
		return -ENOMEM;
	}
	snprintf(client->name, I2C_NAME_SIZE, "i2c-dev %d", adap->nr);//设备节点的名字为适配器的编号

	client->adapter = adap;//该从器件关联的适配器关联起来
	file->private_data = client;//把虚拟的从器件关联到私有数据指针上，即加到上下文环境中

	return 0;
}

static int i2cdev_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;//从私有数据(上下文环境)从拿出从器件

	i2c_put_adapter(client->adapter);//销毁对应的适配器
	kfree(client);//释放虚拟的从器件占用的空间
	file->private_data = NULL;//上下文环境置为空

	return 0;
}

static const struct file_operations i2cdev_fops = {//fops
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,//不支持偏移操作，用no_llseek来填充
	.read		= i2cdev_read,
	.write		= i2cdev_write,
	.unlocked_ioctl	= i2cdev_ioctl,//最重要的ioctl方法
	.open		= i2cdev_open,//打开，重要
	.release	= i2cdev_release,
};

/* ------------------------------------------------------------------------- */

static struct class *i2c_dev_class;

static int i2cdev_attach_adapter(struct device *dev, void *dummy)
{
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;
	int res;

	if (dev->type != &i2c_adapter_type)
		return 0;
	adap = to_i2c_adapter(dev);//如果该设备的类型为一个适配器

	i2c_dev = get_free_i2c_dev(adap);//申请一个新的i2c_dev
	if (IS_ERR(i2c_dev))
		return PTR_ERR(i2c_dev);

	/* register this i2c device with the driver core */
	i2c_dev->dev = device_create(i2c_dev_class, &adap->dev,//构建i2c_dev下的device的属性，填充sysfs树
				     MKDEV(I2C_MAJOR, adap->nr), NULL,
				     "i2c-%d", adap->nr);
	if (IS_ERR(i2c_dev->dev)) {
		res = PTR_ERR(i2c_dev->dev);
		goto error;
	}
	res = device_create_file(i2c_dev->dev, &dev_attr_name);//在对应目录下生成文件
	if (res)
		goto error_destroy;

	pr_debug("i2c-dev: adapter [%s] registered as minor %d\n",
		 adap->name, adap->nr);
	return 0;
error_destroy:
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, adap->nr));
error:
	return_i2c_dev(i2c_dev);
	return res;
}

static int i2cdev_detach_adapter(struct device *dev, void *dummy)
{
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;

	if (dev->type != &i2c_adapter_type)
		return 0;
	adap = to_i2c_adapter(dev);

	i2c_dev = i2c_dev_get_by_minor(adap->nr);//通过适配器的编号，找到对应的i2c-dev
	if (!i2c_dev) /* attach_adapter must have failed */
		return 0;

	device_remove_file(i2c_dev->dev, &dev_attr_name);//删除sysfs下的文件结点
	return_i2c_dev(i2c_dev);//从i2c_dev的链表中移除
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, adap->nr));//销毁设备，更新sysfs

	pr_debug("i2c-dev: adapter [%s] unregistered\n", adap->name);
	return 0;
}

static int i2cdev_notifier_call(struct notifier_block *nb, unsigned long action,
			 void *data)
{
	struct device *dev = data;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		return i2cdev_attach_adapter(dev, NULL);//增加一个设备
	case BUS_NOTIFY_DEL_DEVICE:
		return i2cdev_detach_adapter(dev, NULL);//移除一个设备
	}

	return 0;
}

static struct notifier_block i2cdev_notifier = {//当发生跟踪事件时，执行的回调函数
	.notifier_call = i2cdev_notifier_call,
};

/* ------------------------------------------------------------------------- */

/*
 * module load/unload record keeping
 */

static int __init i2c_dev_init(void)
{
	int res;

	printk(KERN_INFO "i2c /dev entries driver\n");

	res = register_chrdev(I2C_MAJOR, "i2c", &i2cdev_fops);//注册一个字符设备，关联对应的fops指针结构体
	if (res)
		goto out;

	i2c_dev_class = class_create(THIS_MODULE, "i2c-dev");
	if (IS_ERR(i2c_dev_class)) {
		res = PTR_ERR(i2c_dev_class);
		goto out_unreg_chrdev;
	}

	/* Keep track of adapters which will be added or removed later */
	res = bus_register_notifier(&i2c_bus_type, &i2cdev_notifier);//保持跟踪适配器的接入和移除，动态的热插拔事件
	if (res)
		goto out_unreg_class;

	/* Bind to already existing adapters right away */
	i2c_for_each_dev(NULL, i2cdev_attach_adapter);//探测已经接入的适配器设备

	return 0;

out_unreg_class:
	class_destroy(i2c_dev_class);
out_unreg_chrdev:
	unregister_chrdev(I2C_MAJOR, "i2c");
out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
	return res;
}

static void __exit i2c_dev_exit(void)
{
	bus_unregister_notifier(&i2c_bus_type, &i2cdev_notifier);
	i2c_for_each_dev(NULL, i2cdev_detach_adapter);
	class_destroy(i2c_dev_class);//去初始化
	unregister_chrdev(I2C_MAJOR, "i2c");
}

MODULE_AUTHOR("Frodo Looijaard <frodol@dds.nl> and "
		"Simon G. Vogl <simon@tk.uni-linz.ac.at>");
MODULE_DESCRIPTION("I2C /dev entries driver");
MODULE_LICENSE("GPL");

module_init(i2c_dev_init);
module_exit(i2c_dev_exit);
