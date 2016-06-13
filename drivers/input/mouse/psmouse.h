#ifndef _PSMOUSE_H
#define _PSMOUSE_H

#define PSMOUSE_CMD_SETSCALE11	0x00e6//ps2鼠标的一些命令字，协议定义
#define PSMOUSE_CMD_SETSCALE21	0x00e7
#define PSMOUSE_CMD_SETRES	0x10e8
#define PSMOUSE_CMD_GETINFO	0x03e9
#define PSMOUSE_CMD_SETSTREAM	0x00ea
#define PSMOUSE_CMD_SETPOLL	0x00f0
#define PSMOUSE_CMD_POLL	0x00eb	/* caller sets number of bytes to receive */
#define PSMOUSE_CMD_RESET_WRAP	0x00ec
#define PSMOUSE_CMD_GETID	0x02f2
#define PSMOUSE_CMD_SETRATE	0x10f3
#define PSMOUSE_CMD_ENABLE	0x00f4
#define PSMOUSE_CMD_DISABLE	0x00f5
#define PSMOUSE_CMD_RESET_DIS	0x00f6
#define PSMOUSE_CMD_RESET_BAT	0x02ff

#define PSMOUSE_RET_BAT		0xaa
#define PSMOUSE_RET_ID		0x00
#define PSMOUSE_RET_ACK		0xfa
#define PSMOUSE_RET_NAK		0xfe

enum psmouse_state {//鼠标状态枚举
	PSMOUSE_IGNORE,
	PSMOUSE_INITIALIZING,
	PSMOUSE_RESYNCING,
	PSMOUSE_CMD_MODE,
	PSMOUSE_ACTIVATED,
};

/* psmouse protocol handler return codes */
typedef enum {
	PSMOUSE_BAD_DATA,
	PSMOUSE_GOOD_DATA,
	PSMOUSE_FULL_PACKET
} psmouse_ret_t;

struct psmouse {//驱动上下文结构，环境上下文
	void *private;//私有数据指针,泛型
	struct input_dev *dev;//本身是一个input设备，逻辑划分
	struct ps2dev ps2dev;//本身是一个ps2设备，接口划分
	struct delayed_work resync_work;//延迟工作队列
	char *vendor;//厂商信息
	char *name;//设备名称信息
	unsigned char packet[8];//上传的数据包，这里分配最大的缓冲区，为8个无符号字节
	unsigned char badbyte;
	unsigned char pktcnt;//包的计数
	unsigned char pktsize;//包大小设置，不同类型的ps2鼠标包大小不一样
	unsigned char type;//类型，区分不同的鼠标类型
	bool ignore_parity;//？字面意思，忽视平等
	bool acks_disable_command;
	unsigned int model;
	unsigned long last;
	unsigned long out_of_sync_cnt;
	unsigned long num_resyncs;
	enum psmouse_state state;//鼠标状态信息
	char devname[64];//设备名称，sys文件系统需要
	char phys[32];//物理路径

	unsigned int rate;//放大倍率
	unsigned int resolution;
	unsigned int resetafter;
	unsigned int resync_time;
	bool smartscroll;	/* Logitech only */

	psmouse_ret_t (*protocol_handler)(struct psmouse *psmouse);//协议处理，并且返回协议规定的返回值类型枚举
	void (*set_rate)(struct psmouse *psmouse, unsigned int rate);//设置一些属性的接口函数
	void (*set_resolution)(struct psmouse *psmouse, unsigned int resolution);

	int (*reconnect)(struct psmouse *psmouse);//重连，断开，清除，poll等待，激活，非激活
	void (*disconnect)(struct psmouse *psmouse);
	void (*cleanup)(struct psmouse *psmouse);
	int (*poll)(struct psmouse *psmouse);

	void (*pt_activate)(struct psmouse *psmouse);
	void (*pt_deactivate)(struct psmouse *psmouse);
};

enum psmouse_type {//鼠标类型枚举，PS2协议已经比较旧了
	PSMOUSE_NONE,
	PSMOUSE_PS2,
	PSMOUSE_PS2PP,
	PSMOUSE_THINKPS,
	PSMOUSE_GENPS,
	PSMOUSE_IMPS,
	PSMOUSE_IMEX,
	PSMOUSE_SYNAPTICS,
	PSMOUSE_ALPS,
	PSMOUSE_LIFEBOOK,
	PSMOUSE_TRACKPOINT,
	PSMOUSE_TOUCHKIT_PS2,
	PSMOUSE_CORTRON,
	PSMOUSE_HGPK,
	PSMOUSE_ELANTECH,
	PSMOUSE_FSP,
	PSMOUSE_SYNAPTICS_RELATIVE,
	PSMOUSE_AUTO		/* This one should always be last */
};
//一些接口函数的定义
void psmouse_queue_work(struct psmouse *psmouse, struct delayed_work *work,
		unsigned long delay);
int psmouse_sliced_command(struct psmouse *psmouse, unsigned char command);
int psmouse_reset(struct psmouse *psmouse);
void psmouse_set_state(struct psmouse *psmouse, enum psmouse_state new_state);
void psmouse_set_resolution(struct psmouse *psmouse, unsigned int resolution);
psmouse_ret_t psmouse_process_byte(struct psmouse *psmouse);
int psmouse_activate(struct psmouse *psmouse);
int psmouse_deactivate(struct psmouse *psmouse);

struct psmouse_attribute {//属性结构体定义
	struct device_attribute dattr;
	void *data;
	ssize_t (*show)(struct psmouse *psmouse, void *data, char *buf);
	ssize_t (*set)(struct psmouse *psmouse, void *data,
			const char *buf, size_t count);
	bool protect;
};
#define to_psmouse_attr(a)	container_of((a), struct psmouse_attribute, dattr)

ssize_t psmouse_attr_show_helper(struct device *dev, struct device_attribute *attr,
				 char *buf);
ssize_t psmouse_attr_set_helper(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count);

#define __PSMOUSE_DEFINE_ATTR_VAR(_name, _mode, _data, _show, _set, _protect)	\
static struct psmouse_attribute psmouse_attr_##_name = {			\
	.dattr	= {								\
		.attr	= {							\
			.name	= __stringify(_name),				\
			.mode	= _mode,					\
		},								\
		.show	= psmouse_attr_show_helper,				\
		.store	= psmouse_attr_set_helper,				\
	},									\
	.data	= _data,							\
	.show	= _show,							\
	.set	= _set,								\
	.protect = _protect,							\
}

#define __PSMOUSE_DEFINE_ATTR(_name, _mode, _data, _show, _set, _protect)	\
	static ssize_t _show(struct psmouse *, void *, char *);			\
	static ssize_t _set(struct psmouse *, void *, const char *, size_t);	\
	__PSMOUSE_DEFINE_ATTR_VAR(_name, _mode, _data, _show, _set, _protect)

#define PSMOUSE_DEFINE_ATTR(_name, _mode, _data, _show, _set)			\
	__PSMOUSE_DEFINE_ATTR(_name, _mode, _data, _show, _set, true)

#define PSMOUSE_DEFINE_RO_ATTR(_name, _mode, _data, _show)			\
	static ssize_t _show(struct psmouse *, void *, char *);			\
	__PSMOUSE_DEFINE_ATTR_VAR(_name, _mode, _data, _show, NULL, true)

#define PSMOUSE_DEFINE_WO_ATTR(_name, _mode, _data, _set)			\
	static ssize_t _set(struct psmouse *, void *, const char *, size_t);	\
	__PSMOUSE_DEFINE_ATTR_VAR(_name, _mode, _data, NULL, _set, true)

#ifndef psmouse_fmt
#define psmouse_fmt(fmt)	KBUILD_BASENAME ": " fmt
#endif
//调试打印接口的扩展，使用宏函数实现
#define psmouse_dbg(psmouse, format, ...)		\
	dev_dbg(&(psmouse)->ps2dev.serio->dev,		\
		psmouse_fmt(format), ##__VA_ARGS__)
#define psmouse_info(psmouse, format, ...)		\
	dev_info(&(psmouse)->ps2dev.serio->dev,		\
		 psmouse_fmt(format), ##__VA_ARGS__)
#define psmouse_warn(psmouse, format, ...)		\
	dev_warn(&(psmouse)->ps2dev.serio->dev,		\
		 psmouse_fmt(format), ##__VA_ARGS__)
#define psmouse_err(psmouse, format, ...)		\
	dev_err(&(psmouse)->ps2dev.serio->dev,		\
		psmouse_fmt(format), ##__VA_ARGS__)
#define psmouse_notice(psmouse, format, ...)		\
	dev_notice(&(psmouse)->ps2dev.serio->dev,	\
		   psmouse_fmt(format), ##__VA_ARGS__)
#define psmouse_printk(level, psmouse, format, ...)	\
	dev_printk(level,				\
		   &(psmouse)->ps2dev.serio->dev,	\
		   psmouse_fmt(format), ##__VA_ARGS__)


#endif /* _PSMOUSE_H */
