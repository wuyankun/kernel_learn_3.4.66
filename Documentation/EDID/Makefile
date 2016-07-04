
SOURCES	:= $(wildcard [0-9]*x[0-9]*.S)#学习下这种写法，采用模式匹配方法找到所有的源文件

BIN	:= $(patsubst %.S, %.bin, $(SOURCES))#通过模式替换源文件，得到同名的bin目标文件

IHEX	:= $(patsubst %.S, %.bin.ihex, $(SOURCES))#通过模式替换源文件，得到同名的.bin.ihex文件

CODE	:= $(patsubst %.S, %.c, $(SOURCES))#通过模式替换源文件，得到同名的.c文件

all:	$(BIN) $(IHEX) $(CODE)#3种后缀的文件都是目标文件，第一非伪目标

clean:
	@rm -f *.o *.bin.ihex *.bin *.c

%.o:	%.S#由.S生成.o文件
	@cc -c $^

%.bin:	%.o#由.o生成.bin文件
	@objcopy -Obinary $^ $@

%.bin.ihex:	%.o
	@objcopy -Oihex $^ $@
	@dos2unix $@ 2>/dev/null#由dos格式的回车换行，转换为unix格式

%.c:	%.bin#由.bin文件生成.c文件
	@echo "{" >$@; hexdump -f hex $^ >>$@; echo "};" >>$@
