 
all: default

INCLUDE_PATH += CMSIS/Include 
INCLUDE_PATH += CMSIS/Device/ST/STM32L4xx/Include 
INCLUDE_PATH += src/ 
INCLUDE_PATH += STM32L4xx_HAL_Driver/Inc 
INCLUDE_PATH += ../../include 
SOURCE_PATH += src/ 
SOURCE_PATH += STM32L4xx_HAL_Driver/Src
SOURCE_PATH += ../../src/ 

SOURCE_FILES = $(foreach dir,$(SOURCE_PATH),$(shell find $(dir) -name "*.[cs]") )
OBJECT_FILES = $(foreach file, $(SOURCE_FILES), build/$(basename $(notdir $(file))).o) 
#$(foreach dir,$(SOURCE_PATH),$(shell find $(dir) -name "*.[cs]" | sed -e "s/[cs]$/o/") )
VPATH = $(SOURCE_PATH)
DEPEND_FILES = $(foreach file, $(SOURCE_FILES), build/$(basename $(notdir $(file))).d)

TARGET := arm-none-eabi-

DEFINES = DEBUG USE_HAL_DRIVER STM32L476xx

CFLAGS += $(addprefix -D,$(DEFINES)) $(addprefix -I,$(INCLUDE_PATH)) -std=gnu11 -g3
CFLAGS += -Og -ffunction-sections -fdata-sections -Wall -fomit-frame-pointer -fstack-usage -MMD -MP
CFLAGS += -mcpu=cortex-m4 --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb

LDFLAGS += -TSTM32L476RGTX_FLASH.ld
LDFLAGS += -Wl,-Map="build/map" 
LDFLAGS += -Wl,--gc-sections -static 
LDFLAGS += -mcpu=cortex-m4 --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb

LIBS += -lm -lc

#-include $(DEPEND_FILES)

default: build/elf

prepare:
	mkdir -p build/ 

clean:
	rm -fr build/

build/elf build/map build/asm: $(OBJECT_FILES)
	$(TARGET)gcc -o "$@" $(LDFLAGS) $(OBJECT_FILES) $(LIBS) 
	$(TARGET)objdump -h -S build/elf > "build/asm"

build/%.o: %.c prepare
	$(TARGET)gcc -o "$@" "$<" -c $(CFLAGS) -MT"$@" -MF"$(@:%.o=%.d)" 

build/%.o: %.s prepare
	$(TARGET)gcc -o "$@" "$<" -c $(CFLAGS) -MT"$@" -MF"$(@:%.o=%.d)" 
