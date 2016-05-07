/*
* cypress_cy7c63.c//赛普拉斯
*
* Copyright (c) 2006-2007 Oliver Bock (bock@tfh-berlin.de)
*
*	This driver is based on the Cypress USB Driver by Marcus Maul
*	(cyport) and the 2.0 version of Greg Kroah-Hartman's
*	USB Skeleton driver.//USB骨架驱动
*
*	This is a generic driver for the Cypress CY7C63xxx family.
*	For the time being it enables you to read from and write to
*	the single I/O ports of the device.
*
*	Supported vendors:	AK Modul-Bus Computer GmbH
*				(Firmware "Port-Chip")
*
*	Supported devices:	CY7C63001A-PC
*				CY7C63001C-PXC
*				CY7C63001C-SXC
*
*	Supported functions:	Read/Write Ports
*
*
*	For up-to-date information please visit:
*	http://www.obock.de/kernel/cypress
*
*	This program is free software; you can redistribute it and/or
*	modify it under the terms of the GNU General Public License as
*	published by the Free Software Foundation, version 2.
*/

#include <linux/init.h>//模块相关的modules_init
#include <linux/module.h>//模块相关的modules_init
#include <linux/kernel.h>
#include <linux/slab.h>//内存分配算法kmalloc
#include <linux/usb.h>//usb设备相关的

#define DRIVER_AUTHOR		"Oliver Bock (bock@tfh-berlin.de)"//模块描述的宏定义
#define DRIVER_DESC		"Cypress CY7C63xxx USB driver"

#define CYPRESS_VENDOR_ID	0xa2c//vid
#define CYPRESS_PRODUCT_ID	0x8//pid

#define CYPRESS_READ_PORT	0x4//读写端口port
#define CYPRESS_WRITE_PORT	0x5

#define CYPRESS_READ_RAM	0x2//读写ram
#define CYPRESS_WRITE_RAM	0x3
#define CYPRESS_READ_ROM	0x1//读rom

#define CYPRESS_READ_PORT_ID0	0//读写端口port的ID0和ID1
#define CYPRESS_WRITE_PORT_ID0	0
#define CYPRESS_READ_PORT_ID1	0x2
#define CYPRESS_WRITE_PORT_ID1	1

#define CYPRESS_MAX_REQSIZE	8//最大的请求大小


/* table of devices that work with this driver */
static const struct usb_device_id cypress_table[] = {//兼容设备表
	{ USB_DEVICE(CYPRESS_VENDOR_ID, CYPRESS_PRODUCT_ID) },
	{ }
};
MODULE_DEVICE_TABLE(usb, cypress_table);

/* structure to hold all of our device specific stuff *///驱动上下文，包含一些特有的属性
struct cypress {
	struct usb_device *	udev;
	unsigned char		port[2];
};

/* used to send usb control messages to device */
static int vendor_command(struct cypress *dev, unsigned char request,//驱动上下文，操作码，地址(端口号),数据
			  unsigned char address, unsigned char data)
{
	int retval = 0;
	unsigned int pipe;
	unsigned char *iobuf;

	/* allocate some memory for the i/o buffer*/
	iobuf = kzalloc(CYPRESS_MAX_REQSIZE, GFP_KERNEL);//申请缓存空间，用于io的缓存
	if (!iobuf) {
		dev_err(&dev->udev->dev, "Out of memory!\n");
		retval = -ENOMEM;
		goto error;
	}

	dev_dbg(&dev->udev->dev, "Sending usb_control_msg (data: %d)\n", data);//调试

	/* prepare usb control message and send it upstream */
	pipe = usb_rcvctrlpipe(dev->udev, 0);//申请一个recv的控制pipe管道，用于构建一个接收的控制urb
	retval = usb_control_msg(dev->udev, pipe, request,//这里没有用urb，而是用了封装的usb_control_msg接口
				 USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_OTHER,
				 address, data, iobuf, CYPRESS_MAX_REQSIZE,
				 USB_CTRL_GET_TIMEOUT);

	/* store returned data (more READs to be added) */
	switch (request) {//针对内容的处理
		case CYPRESS_READ_PORT:
			if (address == CYPRESS_READ_PORT_ID0) {
				dev->port[0] = iobuf[1];
				dev_dbg(&dev->udev->dev,
					"READ_PORT0 returned: %d\n",
					dev->port[0]);
			}
			else if (address == CYPRESS_READ_PORT_ID1) {
				dev->port[1] = iobuf[1];
				dev_dbg(&dev->udev->dev,
					"READ_PORT1 returned: %d\n",
					dev->port[1]);
			}
			break;
	}

	kfree(iobuf);
error:
	return retval;
}

/* write port value */
static ssize_t write_port(struct device *dev, struct device_attribute *attr,//写端口
			  const char *buf, size_t count,
			  int port_num, int write_id)
{
	int value = -1;
	int result = 0;

	struct usb_interface *intf = to_usb_interface(dev);//从标准设备到接口设备
	struct cypress *cyp = usb_get_intfdata(intf);//从接口设备拿到驱动的上下文环境

	dev_dbg(&cyp->udev->dev, "WRITE_PORT%d called\n", port_num);//调试

	/* validate input data */
	if (sscanf(buf, "%d", &value) < 1) {//校验输入
		result = -EINVAL;
		goto error;
	}
	if (value < 0 || value > 255) {//输入合法性检查
		result = -EINVAL;
		goto error;
	}
	//驱动上下文，操作码，端口号，值
	result = vendor_command(cyp, CYPRESS_WRITE_PORT, write_id,//调用vendor_command进行处理
				(unsigned char)value);

	dev_dbg(&cyp->udev->dev, "Result of vendor_command: %d\n\n", result);//调试
error:
	return result < 0 ? result : count;
}

/* attribute callback handler (write) */
static ssize_t set_port0_handler(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	return write_port(dev, attr, buf, count, 0, CYPRESS_WRITE_PORT_ID0);
}

/* attribute callback handler (write) */
static ssize_t set_port1_handler(struct device *dev,//属性回调关联，用户空间到内核空间的交互，buf，buf_count
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	return write_port(dev, attr, buf, count, 1, CYPRESS_WRITE_PORT_ID1);//调用write_port
}

/* read port value */
static ssize_t read_port(struct device *dev, struct device_attribute *attr,
			 char *buf, int port_num, int read_id)
{
	int result = 0;

	struct usb_interface *intf = to_usb_interface(dev);//标准的设备到接口设备
	struct cypress *cyp = usb_get_intfdata(intf);//从接口的附加属性拿到驱动上下文环境

	dev_dbg(&cyp->udev->dev, "READ_PORT%d called\n", port_num);//调试

	result = vendor_command(cyp, CYPRESS_READ_PORT, read_id, 0);//调用接口vendor_command，读取时多余的变量设置为0

	dev_dbg(&cyp->udev->dev, "Result of vendor_command: %d\n\n", result);//调试

	return sprintf(buf, "%d", cyp->port[port_num]);//格式化输出
}

/* attribute callback handler (read) */
static ssize_t get_port0_handler(struct device *dev,
				 struct device_attribute *attr, char *buf)//属性的回调关联，函数定义是固定的,用户空间读，传入指针
{
	return read_port(dev, attr, buf, 0, CYPRESS_READ_PORT_ID0);
}

/* attribute callback handler (read) */
static ssize_t get_port1_handler(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return read_port(dev, attr, buf, 1, CYPRESS_READ_PORT_ID1);
}

static DEVICE_ATTR(port0, S_IRUGO | S_IWUSR, get_port0_handler, set_port0_handler);//设备属性声明，关联操作方法

static DEVICE_ATTR(port1, S_IRUGO | S_IWUSR, get_port1_handler, set_port1_handler);


static int cypress_probe(struct usb_interface *interface,
			 const struct usb_device_id *id)
{
	struct cypress *dev = NULL;//驱动上下文环境，声明个指针，赋值为空
	int retval = -ENOMEM;//返回值，初始化为申请内存失败

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);//为驱动上下文申请内存，物理上连续的内存空间
	if (dev == NULL) {
		dev_err(&interface->dev, "Out of memory!\n");
		goto error_mem;
	}

	dev->udev = usb_get_dev(interface_to_usbdev(interface));//接口转变为设备

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);//接口保留驱动上下文的指针

	/* create device attribute files */
	retval = device_create_file(&interface->dev, &dev_attr_port0);//创建设备的属性文件，用户程序可以通过文件操作
	if (retval)
		goto error;
	retval = device_create_file(&interface->dev, &dev_attr_port1);
	if (retval)
		goto error;

	/* let the user know that the device is now attached *///一通知信息
	dev_info(&interface->dev,
		 "Cypress CY7C63xxx device now attached\n");
	return 0;

error://若发生错误，执行清理工作
	device_remove_file(&interface->dev, &dev_attr_port0);
	device_remove_file(&interface->dev, &dev_attr_port1);
	usb_set_intfdata(interface, NULL);
	usb_put_dev(dev->udev);
	kfree(dev);

error_mem:
	return retval;
}

static void cypress_disconnect(struct usb_interface *interface)
{
	struct cypress *dev;

	dev = usb_get_intfdata(interface);//从接口中拿出驱动上下文环境

	/* remove device attribute files *///做一些资源释放清理工作，同代码行235部分的内容
	device_remove_file(&interface->dev, &dev_attr_port0);
	device_remove_file(&interface->dev, &dev_attr_port1);
	/* the intfdata can be set to NULL only after the
	 * device files have been removed */
	usb_set_intfdata(interface, NULL);

	usb_put_dev(dev->udev);

	dev_info(&interface->dev,
		 "Cypress CY7C63xxx device now disconnected\n");

	kfree(dev);
}

static struct usb_driver cypress_driver = {
	.name = "cypress_cy7c63",//驱动模块名称**.ko
	.probe = cypress_probe,//探测函数，总线型驱动，设备接入时执行
	.disconnect = cypress_disconnect,//设备断开后执行
	.id_table = cypress_table,//驱动支持的设备列表
};

module_usb_driver(cypress_driver);//简化的模块装载和卸载，标准化接口，用于没有额外的变量需要释放和申请

MODULE_AUTHOR(DRIVER_AUTHOR);//模块作者
MODULE_DESCRIPTION(DRIVER_DESC);//模块描述

MODULE_LICENSE("GPL");//模块遵照的协议
