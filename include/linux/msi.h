#ifndef LINUX_MSI_H
#define LINUX_MSI_H

#include <linux/kobject.h>
#include <linux/list.h>

struct msi_msg {//msi消息的结构体，地址低32位，高32位，数据32位
	u32	address_lo;	/* low 32 bits of msi message address */
	u32	address_hi;	/* high 32 bits of msi message address */
	u32	data;		/* 16 bits of msi message data */
};

/* Helper functions */
struct irq_data;
struct msi_desc;//前项申明，用于gcc编译需要
extern void mask_msi_irq(struct irq_data *data);//mask和非mask某一个msi_irq
extern void unmask_msi_irq(struct irq_data *data);
extern void __read_msi_msg(struct msi_desc *entry, struct msi_msg *msg);//读写一个msi_msg消息数据
extern void __get_cached_msi_msg(struct msi_desc *entry, struct msi_msg *msg);
extern void __write_msi_msg(struct msi_desc *entry, struct msi_msg *msg);
extern void read_msi_msg(unsigned int irq, struct msi_msg *msg);
extern void get_cached_msi_msg(unsigned int irq, struct msi_msg *msg);
extern void write_msi_msg(unsigned int irq, struct msi_msg *msg);

struct msi_desc {
	struct {
		__u8	is_msix	: 1;
		__u8	multiple: 3;	/* log2 number of messages */
		__u8	maskbit	: 1; 	/* mask-pending bit supported ?   */
		__u8	is_64	: 1;	/* Address size: 0=32bit 1=64bit  */
		__u8	pos;	 	/* Location of the msi capability */
		__u16	entry_nr;    	/* specific enabled entry 	  */
		unsigned default_irq;	/* default pre-assigned irq	  */
	} msi_attrib;//属性相关的标识

	u32 masked;			/* mask bits */
	unsigned int irq;//默认中断线号
	struct list_head list;//组成链表

	union {
		void __iomem *mask_base;
		u8 mask_pos;
	};//基地址
	struct pci_dev *dev;//管理pci设备

	/* Last set MSI message */
	struct msi_msg msg;//消息内容

	struct kobject kobj;//内核对象
};

/*
 * The arch hook for setup up msi irqs//体系相关的msi接口的实现
 */
int arch_setup_msi_irq(struct pci_dev *dev, struct msi_desc *desc);
void arch_teardown_msi_irq(unsigned int irq);
extern int arch_setup_msi_irqs(struct pci_dev *dev, int nvec, int type);
extern void arch_teardown_msi_irqs(struct pci_dev *dev);
extern int arch_msi_check_device(struct pci_dev* dev, int nvec, int type);


#endif /* LINUX_MSI_H */
