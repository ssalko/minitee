NAME = FreeRTOSDemo

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

#LIBS = -lnosys

LDSCRIPT = qemu.ld

CFLAGS = -I"include" -I"." \
	-O3 \
	-Wall \
	-fmessage-length=0 \
	-mcpu=cortex-a15 \
	-g \
	-nostartfiles \
	-Wl,-T$(LDSCRIPT)

#LDFLAGS = -T $(LDSCRIPT) -nostartfiles

S_SRCS = entry.s monitor.s

KERNEL_SRCS := kernel/mem.c kernel/tasks.c kernel/queue.c kernel/list.c kernel/mem/heap_2.c
DRIVER_SRCS := drivers/serial.c drivers/pl011.c
C_SRCS = main.c $(KERNEL_SRCS) $(DRIVER_SRCS) # port.c

#
# Define all object files.
#
C_OBJS = $(C_SRCS:%.c=%.o)
S_OBJS = $(S_SRCS:%.s=%.o)

rtosdemo.bin: rtosdemo.elf
	$(OBJCOPY) -O binary $< $@

rtosdemo.elf: $(C_OBJS) $(S_OBJS)
	$(CC) $(C_OBJS) $(S_OBJS) $(CFLAGS) $(LIBS) -o $@

$(C_OBJS): %.o : %.c
	@echo $(C_OBJS)
	$(CC) -c $(CFLAGS) $< -o $@

$(S_OBJS): %.o : %.s
	@echo $(S_OBJS)
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f *.o kernel/*.o kernel/mem/*.o  *.bin *.elf
