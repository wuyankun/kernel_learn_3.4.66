/*
 * HID raw devices, giving access to raw HID events.
 *
 * In comparison to hiddev, this device does not process the
 * hid events at all (no parsing, no lookups). This lets applications
 * to work on raw hid events as they want to, and avoids a need to
 * use a transport-specific userspace libhid/libusb libraries.
 *
 *  Copyright (c) 2007 Jiri Kosina
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <linux/sched.h>

#include <linux/hidraw.h>

static int hidraw_major;//主设备号用静态变量保存
static struct cdev hidraw_cdev;//hidraw的字符设备，静态变量
static struct class *hidraw_class;//hidraw的class类型
static struct hidraw *hidraw_table[HIDRAW_MAX_DEVICES];//创建了一个静态的数组，关联每个创建的hidraw设备信息
static DEFINE_MUTEX(minors_lock);//子设备号互斥锁

static ssize_t hidraw_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)//read的实现
{
	struct hidraw_list *list = file->private_data;//得到驱动context
	int ret = 0, len;
	DECLARE_WAITQUEUE(wait, current);//定义等待队列一个链表元素

	mutex_lock(&list->read_mutex);//驱动context下的读互斥锁

	while (ret == 0) {//ret初始化为0
		if (list->head == list->tail) {//如果队列为空
			add_wait_queue(&list->hidraw->wait, &wait);//将当前的链表元素添加到等待队列头部中，构成等待链表
			set_current_state(TASK_INTERRUPTIBLE);//将当前的task(线程）标记修改为可被中断唤醒的休眠状态

			while (list->head == list->tail) {//当前链表为空
				if (file->f_flags & O_NONBLOCK) {//打开方式是非阻塞的
					ret = -EAGAIN;//返回个E_AGAIN，跳出
					break;
				}
				if (signal_pending(current)) {//当前task收到了信号
					ret = -ERESTARTSYS;//返回E_RE_START_SYS，跳出
					break;
				}
				if (!list->hidraw->exist) {//如果底层设备不存在，状态发生了改变
					ret = -EIO;//返回E_IO，IO错误，跳出
					break;
				}

				/* allow O_NONBLOCK to work well from other threads */
				mutex_unlock(&list->read_mutex);//上述跳出的场景释放锁
				schedule();//申请让度CPU时间片，使得调度器稍后重新调度
				mutex_lock(&list->read_mutex);//重新获得读锁
				set_current_state(TASK_INTERRUPTIBLE);//设置为可被打断的休眠状态
			}

			set_current_state(TASK_RUNNING);//队列不为空，设置task为运行状态
			remove_wait_queue(&list->hidraw->wait, &wait);//将task从等待队列头部移除
		}

		if (ret)
			goto out;

		len = list->buffer[list->tail].len > count ?//两者取小，队列中的实际数据个数和用户想拿到的数据个数比较
			count : list->buffer[list->tail].len;

		if (copy_to_user(buffer, list->buffer[list->tail].value, len)) {//拷贝给用户
			ret = -EFAULT;
			goto out;
		}
		ret = len;

		kfree(list->buffer[list->tail].value);
		list->tail = (list->tail + 1) & (HIDRAW_BUFFER_SIZE - 1);//？
	}
out:
	mutex_unlock(&list->read_mutex);
	return ret;
}

/* The first byte is expected to be a report number.
 * This function is to be called with the minors_lock mutex held */
static ssize_t hidraw_send_report(struct file *file, const char __user *buffer, size_t count, unsigned char report_type)
{
	unsigned int minor = iminor(file->f_path.dentry->d_inode);//通过filp得到inode，再通过inode得到minor号
	struct hid_device *dev;//另外一层hid_device结构
	__u8 *buf;
	int ret = 0;

	if (!hidraw_table[minor]) {//入参检查，设备状态检查，设备不存在
		ret = -ENODEV;
		goto out;
	}

	dev = hidraw_table[minor]->hid;//得到hidraw的下一层hid_device

	if (!dev->hid_output_raw_report) {//入参检查，设备状态检查，设备不存在
		ret = -ENODEV;
		goto out;
	}

	if (count > HID_MAX_BUFFER_SIZE) {//入参检查，参数错误
		hid_warn(dev, "pid %d passed too large report\n",
			 task_pid_nr(current));//通过current指针得到pid号
		ret = -EINVAL;
		goto out;
	}

	if (count < 2) {//入参检查，参数错误
		hid_warn(dev, "pid %d passed too short report\n",
			 task_pid_nr(current));
		ret = -EINVAL;
		goto out;
	}

	buf = kmalloc(count * sizeof(__u8), GFP_KERNEL);//分配内核空间内存
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user(buf, buffer, count)) {//将用户空间哦内存数据，拷贝到内核空间的内存中
		ret = -EFAULT;
		goto out_free;
	}

	ret = dev->hid_output_raw_report(dev, buf, count, report_type);//调用hidraw->hid_device的report接口，真正的发送报告
out_free:
	kfree(buf);
out:
	return ret;
}

/* the first byte is expected to be a report number */
static ssize_t hidraw_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)//write的实现
{
	ssize_t ret;
	mutex_lock(&minors_lock);
	ret = hidraw_send_report(file, buffer, count, HID_OUTPUT_REPORT);//发送报告OUTPUT
	mutex_unlock(&minors_lock);
	return ret;
}


/* This function performs a Get_Report transfer over the control endpoint
 * per section 7.2.1 of the HID specification, version 1.1.  The first byte
 * of buffer is the report number to request, or 0x0 if the defice does not
 * use numbered reports. The report_type parameter can be HID_FEATURE_REPORT
 * or HID_INPUT_REPORT.  This function is to be called with the minors_lock
 *  mutex held. */
static ssize_t hidraw_get_report(struct file *file, char __user *buffer, size_t count, unsigned char report_type)
{
	unsigned int minor = iminor(file->f_path.dentry->d_inode);//参考hidraw_set_report
	struct hid_device *dev;
	__u8 *buf;
	int ret = 0, len;
	unsigned char report_number;

	dev = hidraw_table[minor]->hid;

	if (!dev->hid_get_raw_report) {
		ret = -ENODEV;
		goto out;
	}

	if (count > HID_MAX_BUFFER_SIZE) {
		printk(KERN_WARNING "hidraw: pid %d passed too large report\n",
				task_pid_nr(current));
		ret = -EINVAL;
		goto out;
	}

	if (count < 2) {
		printk(KERN_WARNING "hidraw: pid %d passed too short report\n",
				task_pid_nr(current));
		ret = -EINVAL;
		goto out;
	}

	buf = kmalloc(count * sizeof(__u8), GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	/* Read the first byte from the user. This is the report number,
	 * which is passed to dev->hid_get_raw_report(). */
	if (copy_from_user(&report_number, buffer, 1)) {
		ret = -EFAULT;
		goto out_free;
	}

	ret = dev->hid_get_raw_report(dev, report_number, buf, count, report_type);

	if (ret < 0)
		goto out_free;

	len = (ret < count) ? ret : count;

	if (copy_to_user(buffer, buf, len)) {
		ret = -EFAULT;
		goto out_free;
	}

	ret = len;

out_free:
	kfree(buf);
out:
	return ret;
}

static unsigned int hidraw_poll(struct file *file, poll_table *wait)//poll的实现
{
	struct hidraw_list *list = file->private_data;//得到驱动上下文context

	poll_wait(file, &list->hidraw->wait, wait);//等待队列
	if (list->head != list->tail)//如果不是空链表
		return POLLIN | POLLRDNORM;//标志为应用空间可读，poll是相对用户而言
	if (!list->hidraw->exist)//如果设备已经不存在了
		return POLLERR | POLLHUP;//标志为发生错误活在挂起
	return 0;
}

static int hidraw_open(struct inode *inode, struct file *file)//open的实现
{
	unsigned int minor = iminor(inode);//从inode中得到次设备号
	struct hidraw *dev;//从功能划分属于hidraw设备
	struct hidraw_list *list;//驱动上下文环境context
	int err = 0;

	if (!(list = kzalloc(sizeof(struct hidraw_list), GFP_KERNEL))) {//分配一个驱动上下文环境的内存空间
		err = -ENOMEM;
		goto out;
	}

	mutex_lock(&minors_lock);//所有次设备共用的次设备互斥锁，来访问全局变量hidraw_table[]
	if (!hidraw_table[minor]) {//对应的下标号中没有初始化，提示设备不存在
		err = -ENODEV;
		goto out_unlock;
	}

	list->hidraw = hidraw_table[minor];//将hidraw_table[]的初始化的hidraw关联起来
	mutex_init(&list->read_mutex);//初始化驱动上下文中的读互斥锁
	list_add_tail(&list->node, &hidraw_table[minor]->list);//增加到队列的尾部？
	file->private_data = list;//将驱动上下文保存到文件句柄的私有数据中

	dev = hidraw_table[minor];//将hidraw_table[]用临时变量dev（hidraw）关联起来
	if (!dev->open++) {//若设备未打开过，设备打开次数+1
		err = hid_hw_power(dev->hid, PM_HINT_FULLON);//将该设备的电源拉起来
		if (err < 0) {
			dev->open--;//打开失败，设备打开次数-1
			goto out_unlock;
		}

		err = hid_hw_open(dev->hid);//调用hid_hw层级的open进一步初始化hidraw设备上下文
		if (err < 0) {
			hid_hw_power(dev->hid, PM_HINT_NORMAL);//将设备的电源置于NORMAL状态下
			dev->open--;
		}
	}

out_unlock:
	mutex_unlock(&minors_lock);
out:
	if (err < 0)
		kfree(list);
	return err;

}

static int hidraw_release(struct inode * inode, struct file * file)
{
	unsigned int minor = iminor(inode);//从inode得到对应的次设备号
	struct hidraw *dev;//临时变量，关联hidraw设备
	struct hidraw_list *list = file->private_data;//从文件句柄的私有变量中拿到驱动上下文环境context
	int ret;

	mutex_lock(&minors_lock);//操作hidraw_table[]之前获得互斥锁
	if (!hidraw_table[minor]) {
		ret = -ENODEV;
		goto unlock;
	}

	list_del(&list->node);//将链表删除
	dev = hidraw_table[minor];//将hidraw设备用dev指针表示
	if (!--dev->open) {//不是最后一次关闭
		if (list->hidraw->exist) {//hidraw设备还存在
			hid_hw_power(dev->hid, PM_HINT_NORMAL);//将电源置于Normal状态
			hid_hw_close(dev->hid);//调用更底层的hid_hw_close关闭
		} else {
			kfree(list->hidraw);//否则将这个节点内存free掉
		}
	}
	kfree(list);//释放驱动上下文环境内存context
	ret = 0;
unlock:
	mutex_unlock(&minors_lock);

	return ret;
}

static long hidraw_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	unsigned int minor = iminor(inode);//得到对应的次设备号
	long ret = 0;
	struct hidraw *dev;
	void __user *user_arg = (void __user*) arg;//格式转化，从ulong转换为void*，从数据类型转换为指针

	mutex_lock(&minors_lock);
	dev = hidraw_table[minor];
	if (!dev) {
		ret = -ENODEV;
		goto out;
	}

	switch (cmd) {
		case HIDIOCGRDESCSIZE://HID_IOC_G_RDESC_SIZE(report descriptor size)
			if (put_user(dev->hid->rsize, (int __user *)arg))
				ret = -EFAULT;
			break;

		case HIDIOCGRDESC://HID_IOC_G_RDESC(report descriptor)
			{
				__u32 len;

				if (get_user(len, (int __user *)arg))
					ret = -EFAULT;
				else if (len > HID_MAX_DESCRIPTOR_SIZE - 1)
					ret = -EINVAL;
				else if (copy_to_user(user_arg + offsetof(
					struct hidraw_report_descriptor,
					value[0]),
					dev->hid->rdesc,
					min(dev->hid->rsize, len)))
					ret = -EFAULT;
				break;
			}
		case HIDIOCGRAWINFO://HID_IOC_G_RAW_INFO
			{
				struct hidraw_devinfo dinfo;

				dinfo.bustype = dev->hid->bus;//总线类型
				dinfo.vendor = dev->hid->vendor;//厂商
				dinfo.product = dev->hid->product;//产品
				if (copy_to_user(user_arg, &dinfo, sizeof(dinfo)))
					ret = -EFAULT;
				break;
			}
		default:
			{
				struct hid_device *hid = dev->hid;
				if (_IOC_TYPE(cmd) != 'H') {//类型为H
					ret = -EINVAL;
					break;
				}

				if (_IOC_NR(cmd) == _IOC_NR(HIDIOCSFEATURE(0))) {//HID_IOC_S_FEATURE
					int len = _IOC_SIZE(cmd);
					ret = hidraw_send_report(file, user_arg, len, HID_FEATURE_REPORT);
					break;
				}
				if (_IOC_NR(cmd) == _IOC_NR(HIDIOCGFEATURE(0))) {//HID_IOC_G_FEATURE
					int len = _IOC_SIZE(cmd);
					ret = hidraw_get_report(file, user_arg, len, HID_FEATURE_REPORT);
					break;
				}

				/* Begin Read-only ioctls. */
				if (_IOC_DIR(cmd) != _IOC_READ) {
					ret = -EINVAL;
					break;
				}

				if (_IOC_NR(cmd) == _IOC_NR(HIDIOCGRAWNAME(0))) {//HID_IOC_G_RAW_NAME
					int len = strlen(hid->name) + 1;
					if (len > _IOC_SIZE(cmd))
						len = _IOC_SIZE(cmd);
					ret = copy_to_user(user_arg, hid->name, len) ?
						-EFAULT : len;
					break;
				}

				if (_IOC_NR(cmd) == _IOC_NR(HIDIOCGRAWPHYS(0))) {//HID_IOC_G_RAW_PHY
					int len = strlen(hid->phys) + 1;
					if (len > _IOC_SIZE(cmd))
						len = _IOC_SIZE(cmd);
					ret = copy_to_user(user_arg, hid->phys, len) ?
						-EFAULT : len;
					break;
				}
			}

		ret = -ENOTTY;
	}
out:
	mutex_unlock(&minors_lock);
	return ret;
}

static const struct file_operations hidraw_ops = {
	.owner =        THIS_MODULE,
	.read =         hidraw_read,//read和write接口
	.write =        hidraw_write,
	.poll =         hidraw_poll,//支持poll的系统调用
	.open =         hidraw_open,//open和close接口
	.release =      hidraw_release,
	.unlocked_ioctl = hidraw_ioctl,//新版本的ioctl形式
#ifdef CONFIG_COMPAT//兼容的ioctl形式
	.compat_ioctl   = hidraw_ioctl,
#endif
	.llseek =	noop_llseek,//不支持seek
};

void hidraw_report_event(struct hid_device *hid, u8 *data, int len)
{
	struct hidraw *dev = hid->hidraw;
	struct hidraw_list *list;

	list_for_each_entry(list, &dev->list, node) {
		list->buffer[list->head].value = kmemdup(data, len, GFP_ATOMIC);
		list->buffer[list->head].len = len;
		list->head = (list->head + 1) & (HIDRAW_BUFFER_SIZE - 1);
		kill_fasync(&list->fasync, SIGIO, POLL_IN);//发送信号SIGIO
	}

	wake_up_interruptible(&dev->wait);//唤醒等待队列（该队列可以被中断唤醒）
}
EXPORT_SYMBOL_GPL(hidraw_report_event);

int hidraw_connect(struct hid_device *hid)//hidraw设备连接上
{
	int minor, result;
	struct hidraw *dev;//定义一个这样的设备

	/* we accept any HID device, no matter the applications */

	dev = kzalloc(sizeof(struct hidraw), GFP_KERNEL);//分配hidraw设备上下文内存空间
	if (!dev)
		return -ENOMEM;

	result = -EINVAL;

	mutex_lock(&minors_lock);

	for (minor = 0; minor < HIDRAW_MAX_DEVICES; minor++) {//在数组中找到一个空闲的，将设备保存起来，将指针的地址保存下来
		if (hidraw_table[minor])
			continue;
		hidraw_table[minor] = dev;
		result = 0;
		break;
	}

	if (result) {//根据result的值判断是否成功
		mutex_unlock(&minors_lock);
		kfree(dev);
		goto out;
	}

	dev->dev = device_create(hidraw_class, &hid->dev, MKDEV(hidraw_major, minor),
				 NULL, "%s%d", "hidraw", minor);//创建一个device节点，和hidraw的class关联在一起

	if (IS_ERR(dev->dev)) {//device创建失败，会清理
		hidraw_table[minor] = NULL;
		mutex_unlock(&minors_lock);
		result = PTR_ERR(dev->dev);
		kfree(dev);
		goto out;
	}

	mutex_unlock(&minors_lock);
	init_waitqueue_head(&dev->wait);//hidraw设备有一个等待队列头部需要初始化
	INIT_LIST_HEAD(&dev->list);//hidraw设备有一个链表需要初始化

	dev->hid = hid;//hidraw设备包含一个hid_device
	dev->minor = minor;//hidraw保存次设备号

	dev->exist = 1;//存在属性置为1
	hid->hidraw = dev;//将hid_device和hidraw关联起来，相互可以查找

out:
	return result;

}
EXPORT_SYMBOL_GPL(hidraw_connect);

void hidraw_disconnect(struct hid_device *hid)
{
	struct hidraw *hidraw = hid->hidraw;//通过hid_device找到了hidraw，参见504行

	mutex_lock(&minors_lock);
	hidraw->exist = 0;//存在属性置为0

	device_destroy(hidraw_class, MKDEV(hidraw_major, hidraw->minor));//删除对应的device节点信息，hidraw中存有次设备号，参见501行

	hidraw_table[hidraw->minor] = NULL;//数组(每个元素为一个指针)对应的位置为NULL

	if (hidraw->open) {//如果已经打开
		hid_hw_close(hid);//调用底层的hw层借口close关闭
		wake_up_interruptible(&hidraw->wait);//唤醒睡眠在该hid_device上面的task(线程）
	} else {
		kfree(hidraw);
	}
	mutex_unlock(&minors_lock);
}
EXPORT_SYMBOL_GPL(hidraw_disconnect);

int __init hidraw_init(void)
{
	int result;
	dev_t dev_id;

	result = alloc_chrdev_region(&dev_id, HIDRAW_FIRST_MINOR,
			HIDRAW_MAX_DEVICES, "hidraw");//分配一个主设备号，/dev/hidrawN

	hidraw_major = MAJOR(dev_id);//设备号得到主设备号

	if (result < 0) {
		pr_warn("can't get major number\n");
		result = 0;
		goto out;
	}

	hidraw_class = class_create(THIS_MODULE, "hidraw");//创建/sys/class/hidraw类型
	if (IS_ERR(hidraw_class)) {
		result = PTR_ERR(hidraw_class);
		unregister_chrdev(hidraw_major, "hidraw");
		goto out;
	}

        cdev_init(&hidraw_cdev, &hidraw_ops);//关联fops
        cdev_add(&hidraw_cdev, dev_id, HIDRAW_MAX_DEVICES);//添加char设备
out:
	return result;
}

void hidraw_exit(void)
{
	dev_t dev_id = MKDEV(hidraw_major, 0);//从static变量中拿到major_number，得到设备号

	cdev_del(&hidraw_cdev);//删除字节设备
	class_destroy(hidraw_class);//删除hidraw类型
	unregister_chrdev_region(dev_id, HIDRAW_MAX_DEVICES);//将所有的子设备销毁

}
