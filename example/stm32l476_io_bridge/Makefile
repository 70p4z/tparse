 
all: default

ifeq ($(BUILD_DIR),)
BUILD_DIR = build
endif

DEFINES += VERSION=\"0.10\"

INCLUDE_PATH += CMSIS/Include 
INCLUDE_PATH += CMSIS/Device/ST/STM32L4xx/Include 
INCLUDE_PATH += src/ 
INCLUDE_PATH += STM32L4xx_HAL_Driver/Inc 
INCLUDE_PATH += ../../include 
SOURCE_PATH += src/ 
SOURCE_PATH += STM32L4xx_HAL_Driver/Src
SOURCE_PATH += ../../src/ 

SOURCE_FILES = $(foreach dir,$(SOURCE_PATH),$(shell find $(dir) -name "*.[cs]") )
OBJECT_FILES = $(foreach file, $(SOURCE_FILES), $(BUILD_DIR)/$(basename $(notdir $(file))).o) 
#$(foreach dir,$(SOURCE_PATH),$(shell find $(dir) -name "*.[cs]" | sed -e "s/[cs]$/o/") )
VPATH = $(SOURCE_PATH)
DEPEND_FILES = $(foreach file, $(SOURCE_FILES), $(BUILD_DIR)/$(basename $(notdir $(file))).d)

TARGET := arm-none-eabi-

DEFINES += DEBUG USE_HAL_DRIVER STM32L476xx

CFLAGS += $(addprefix -D,$(DEFINES)) $(addprefix -I,$(INCLUDE_PATH)) -std=gnu11 -g3
CFLAGS += -Og -ffunction-sections -fdata-sections -Wall -fomit-frame-pointer -fstack-usage -MMD -MP
CFLAGS += -mcpu=cortex-m4 --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb

LDFLAGS += -TSTM32L476RGTX_FLASH.ld
LDFLAGS += -Wl,-Map="$(BUILD_DIR)/output.map" 
LDFLAGS += -Wl,--gc-sections -static 
LDFLAGS += -mcpu=cortex-m4 --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb

LIBS += -lm -lc

-include $(DEPEND_FILES)

default: $(BUILD_DIR)/output.elf

prepare:
	mkdir -p $(BUILD_DIR)/ 

clean:
	rm -fr $(BUILD_DIR)/

$(BUILD_DIR)/output.elf $(BUILD_DIR)/output.map $(BUILD_DIR)/output.asm $(BUILD_DIR)/output.bin: $(OBJECT_FILES)
	$(TARGET)gcc -o "$@" $(LDFLAGS) $(OBJECT_FILES) $(LIBS) 
	$(TARGET)objdump -h -S $(BUILD_DIR)/output.elf > "$(BUILD_DIR)/output.asm"
	$(TARGET)objcopy -O ihex -S $(BUILD_DIR)/output.elf $(BUILD_DIR)/output.hex
	srec_cat -o -binary $(BUILD_DIR)/output.hex -intel -offset - 0x08000000 > $(BUILD_DIR)/output.bin

$(BUILD_DIR)/%.o: %.c Makefile | prepare
	$(TARGET)gcc -o "$@" "$<" -c $(CFLAGS) -MT"$@" -MF"$(@:%.o=%.d)" 

$(BUILD_DIR)/%.o: %.s Makefile | prepare
	$(TARGET)gcc -o "$@" "$<" -c $(CFLAGS) -MT"$@" -MF"$(@:%.o=%.d)" 
