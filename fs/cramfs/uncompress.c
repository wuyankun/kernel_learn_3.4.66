/*
 * uncompress.c
 *
 * (C) Copyright 1999 Linus Torvalds
 *
 * cramfs interfaces to the uncompression library. There's really just
 * three entrypoints:
 *
 *  - cramfs_uncompress_init() - called to initialize the thing.
 *  - cramfs_uncompress_exit() - tell me when you're done
 *  - cramfs_uncompress_block() - uncompress a block.
 *
 * NOTE NOTE NOTE! The uncompression is entirely single-threaded. We
 * only have one stream, and we'll initialize it only once even if it
 * then is used by multiple filesystems.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/vmalloc.h>
#include <linux/zlib.h>
#include <linux/cramfs_fs.h>

static z_stream stream;
static int initialized;//静态变量

/* Returns length of decompressed data. */
int cramfs_uncompress_block(void *dst, int dstlen, void *src, int srclen)//uncompress解压缩的
{
	int err;

	stream.next_in = src;//源泛型指针
	stream.avail_in = srclen;//源长度

	stream.next_out = dst;//目标泛型指针
	stream.avail_out = dstlen;//目标长度

	err = zlib_inflateReset(&stream);
	if (err != Z_OK) {
		printk("zlib_inflateReset error %d\n", err);
		zlib_inflateEnd(&stream);
		zlib_inflateInit(&stream);
	}

	err = zlib_inflate(&stream, Z_FINISH);
	if (err != Z_STREAM_END)
		goto err;
	return stream.total_out;

err:
	printk("Error %d while decompressing!\n", err);
	printk("%p(%d)->%p(%d)\n", src, srclen, dst, dstlen);
	return -EIO;
}

int cramfs_uncompress_init(void)
{
	if (!initialized++) {
		stream.workspace = vmalloc(zlib_inflate_workspacesize());//逻辑地址是连续的vmalloc，使用zlib的接口函数
		if ( !stream.workspace ) {
			initialized = 0;
			return -ENOMEM;
		}
		stream.next_in = NULL;
		stream.avail_in = 0;
		zlib_inflateInit(&stream);//inflate是充气的，膨胀的
	}
	return 0;
}

void cramfs_uncompress_exit(void)
{
	if (!--initialized) {
		zlib_inflateEnd(&stream);//释放相关的资源
		vfree(stream.workspace);
	}
}
