#
# Copyright: Oleg Borodin <onborodin@gmail.com> 2018
#

.SECONDARY:

CFLAGS+= -I. -O2 -DSTM32F4 #-std=c99
CFLAGS+= -mfloat-abi=hard
CFLAGS+= -mcpu=cortex-m4
CFLAGS+= -mthumb
CFLAGS+= -fno-common -ffunction-sections -fdata-sections
CFLAGS+= -g -gdwarf-2
CFLAGS+= -Wall


LDFLAGS+= ${CFLAGS}
LDFLAGS+= --static
#LDFLAGS+= -nostartfiles
LDFLAGS+= -T main.ld

LDFLAGS+= -Wl,-Map=main.map
LDFLAGS+= -Wl,--cref -Wl,--gc-sections
LDFLAGS+= -lopencm3_stm32f4
LDFLAGS+= -Wl,--start-group -lc -lm -lgcc -lnosys -Wl,--end-group

TARGET= arm-eabi

all: main.bin

OBJS+= main.o
OBJS+= syscall.o
OBJS+= usartu.o
OBJS+= mpu6050.o
OBJS+= geometry.o
OBJS+= madgwick.o
OBJS+= i2cdev.o

main.elf: $(OBJS)
	$(TARGET)-gcc $(^F) $(LDFLAGS) -o $@
	$(TARGET)-size --format=berkeley $@

%.o: %.c
	$(TARGET)-gcc $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(TARGET)-gcc $(CFLAGS) -c -o $@ $<

%.bin: %.elf
	$(TARGET)-objcopy -O binary $< $@

%.elf: %.o
	$(TARGET)-gcc $(^F) $(LDFLAGS) -o $@
	$(TARGET)-size --format=berkeley $@

clean:
	rm -f *.i *.o *.elf *.bin *.map *~ *.hex *.d *.s

flash: main.bin
	@openocd \
	    -c 'puts "--- START --------------------"' \
	    -f 'interface/stlink.cfg' \
	    -f 'target/stm32f4x.cfg'  \
	    -c 'puts "--- INIT --------------------"' \
	    -c "init" \
	    -c "reset halt" \
	    -c 'puts "--- WRITE --------------------"' \
	    -c "flash write_image erase $< 0x08000000"\
	    -c 'puts "--- VERIFY --------------------"' \
	    -c "verify_image $<" \
	    -c 'puts "--- RESET --------------------"' \
	    -c "reset" \
	    -c 'puts "--- DONE --------------------"' \
	    -c "shutdown"

debug: main.bin
	@openocd \
	    -c 'puts "--- START --------------------"' \
	    -f 'interface/stlink.cfg' \
	    -f 'target/stm32f4x.cfg'  \
	    -c 'puts "--- INIT --------------------"' \
	    -c "init" \
	    -c "halt" \
	    -c "poll"
#EOF
