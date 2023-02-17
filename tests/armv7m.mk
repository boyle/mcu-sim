SOC = armv7m
CPU = cortex-m4

BIN = $(SOC)-test
SRC = $(SOC)-test.c startup.c

LD = $(SOC).ld
SOC_DEFINE = __$(shell tr '[:lower:]' '[:upper:]' <<< $(SOC))__

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

CFLAGS += -mcpu=$(CPU) -mthumb
CFLAGS += -W -Wall -Werror --std=c17 -Os
CFLAGS += -fdata-sections -ffunction-sections
# -fdata-sections      Place data items into their own section.
# -ffunction-sections  Place each function into its own section.
CFLAGS += -funsigned-char -funsigned-bitfields
# -funsigned-bitfields  When "signed" or "unsigned" is not given make the bitfield unsigned.
# -funsigned-char       Make "char" unsigned by default.
CFLAGS += -MD -MP -MT $(*F).o -MF .$(@F).d
# -MD  Generate make dependencies and compile.
# -MP  Generate phony targets for all headers.
# -MT <target>  Add a target that does not require quoting.
# -MF <file> Write dependency output to the given file.

LDFLAGS += -Wall -Werror
LDFLAGS += -mcpu=$(CPU) -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--unresolved-symbols=report-all
LDFLAGS += -Wl,--warn-common
ifdef V
LDFLAGS += -Wl,--cref
endif
LDFLAGS += -Wl,--script=$(LD)
# --gc-sections  Remove unused sections (on some targets); as split by -f{data,function}-sections
LDFLAGS += -Wl,--print-gc-sections
$(V)LDFLAGS += -Wl,--no-print-gc-sections
# when verbose, print deleted sections (dead code)


CFLAGS += -I vendor
CFLAGS += -D$(SOC_DEFINE) -DDONT_USE_CMSIS_INIT

LIBS = # none

BOOTLOADER_SIZE = 8192
FLASH_TOTAL = (256*1024)
FLASH_SPACE = ($(FLASH_TOTAL)-$(BOOTLOADER_SIZE))

BIN2UF2 = ./bin2uf2

OBJS = $(notdir %/$(subst .c,.o,$(SRC)))
OUT = $(BIN).elf $(BIN).hex $(BIN).bin $(BIN).asm # $(BIN).uf2

all: $(OUT) size
.phony: all clean size

%.o: %.c
	echo " [CC]   $@"
	$(CC) $(CFLAGS) -c $< -o $@
$(BIN).elf: $(OBJS)
	echo " [LD]   $@"
	$(CC) $(LDFLAGS) $^ $(LIBS) -o $@
%.hex: %.elf
	echo " [HEX]  $@"
	$(OBJCOPY) -O ihex $< $@
%.bin: %.elf
	echo " [BIN]  $@"
	$(OBJCOPY) -O binary $< $@
%.asm: %.elf
	echo " [ASM]  $@"
	$(OBJDUMP) --disassemble $< > $@
%.uf2: %.bin $(BIN2UF2)
	echo -n " [UF2]  $@: " && $(BIN2UF2) samd21 $< $@
size: $(BIN).elf
	echo -n " [SIZE] " && $(SIZE) -G $< | awk '{ s=$$1+$$2 } END { print s " B; " ($(FLASH_SPACE)-s)/1024 " kB of " ($(FLASH_SPACE)/1024) " kB flash remains" }'
ifdef V
	$(SIZE) -G $<
endif

clean:
	rm -f $(OUT) $(OBJS) $(patsubst %.o, .%.o.d, $(OBJS))

DEST = DEST_DIR
upload: | $(BIN).uf2
	cp $(BIN).uf2 $(DEST)

TTY=/dev/ttyACM0
tty:
	echo "'Ctrl-a K' to close the serial port" && sleep 1
	screen $(TTY) 115200

ATMEL_VER=1.3.395
CMSIS_VER=5.8.0

ATMEL_URL=http://packs.download.atmel.com/Atmel.SAMD21_DFP.$(ATMEL_VER).atpack
ATMEL_OUT=atmel-samd21-$(ATMEL_VER).zip
CMSIS_URL=https://raw.githubusercontent.com/ARM-software/CMSIS_5/$(CMSIS_VER)/CMSIS/Core/Include

update:
	mkdir -p vendor
	[ -f "$(ATMEL_OUT)" ] || wget -q $(ATMEL_URL) -O $(ATMEL_OUT)
	unzip -qo $(ATMEL_OUT) -d vendor/
	cd vendor && rm -rf component instance pio
	mv -f vendor/samd21a/include/* vendor/
	cd vendor && rm -rf *.pdsc package.content samd21a samd21b samd21c samd21d scripts sam.h system_samd21.h component-version.h
	find vendor/ -name samd21\[je\]\*.h -delete # everything but samd21g*.h
	find vendor/ -name samd21\?1\[567\]\*.h -delete # everything but samd21g18a.h
	find vendor/ -name samd21g18au.h -delete # everything but samd21g18a.h
	rm -f $(ATMEL_OUT)
	for i in core_cm0plus.h cmsis_version.h cmsis_compiler.h cmsis_gcc.h; do \
		rm -f vendor/$$i; \
		wget -q $(CMSIS_URL)/$$i -O vendor/$$i; \
	done

$(V).SILENT:
-include $(wildcard .*.d)
