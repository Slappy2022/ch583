TOOL_CHAIN_PATH ?= /opt/gcc-riscv/xpacks/.bin
PROJECT_NAME    = BLE

TOP_DIR     := .
OUTPUT_DIR  := $(TOP_DIR)/../Build
STARTUP_DIR := $(TOP_DIR)/../SRC/Startup
USER_DIR    := $(TOP_DIR)/BLE_BLINKY/APP
PROFILE_DIR := $(TOP_DIR)/BLE_BLINKY/Profile
SERVICE_DIR := $(TOP_DIR)/BLE_BLINKY/APP/ble_uart_service
LIB2_DIR    := $(TOP_DIR)/BLE_BLINKY/lib

LD_FILE     := $(TOP_DIR)/../SRC/Ld/Link.ld
MAP_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).map
ELF_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).elf
HEX_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).hex
LST_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).lst
SIZ_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).siz

INCLUDES := $(INCLUDES)
INCLUDES += -I $(TOP_DIR)/BLE_BLINKY/APP/include
INCLUDES += -I $(TOP_DIR)/BLE_BLINKY/APP/ble_uart_service
INCLUDES += -I $(TOP_DIR)/BLE_BLINKY/Profile/include
INCLUDES += -I $(TOP_DIR)/BLE_BLINKY/lib

CCFLAGS := -march=rv32imac_zicsr_zifencei \
           -mabi=ilp32 \
           -msmall-data-limit=8 \
           -mno-save-restore \
           -Os \
           -fmessage-length=0 \
           -fsigned-char \
           -ffunction-sections \
           -fdata-sections \
           -Wunused -Wuninitialized -g

all: $(HEX_FILE) $(LST_FILE) $(SIZ_FILE)

STARTUP_SRCS := $(wildcard $(STARTUP_DIR)/*.S)
STARTUP_OBJS := $(patsubst $(STARTUP_DIR)/%.S, $(OUTPUT_DIR)/startup/%.o, $(STARTUP_SRCS)) # patsubst: in #3 replace #1 to #2

USER_SRCS := $(wildcard $(USER_DIR)/*.c)
USER_OBJS := $(patsubst $(USER_DIR)/%.c, $(OUTPUT_DIR)/user/%.o, $(USER_SRCS))

PROFILE_SRCS := $(wildcard $(PROFILE_DIR)/*.c)
PROFILE_OBJS := $(patsubst $(PROFILE_DIR)/%.c, $(OUTPUT_DIR)/profile/%.o, $(PROFILE_SRCS))

SERVICE_SRCS := $(wildcard $(SERVICE_DIR)/*.c)
SERVICE_OBJS := $(patsubst $(SERVICE_DIR)/%.c, $(OUTPUT_DIR)/service/%.o, $(SERVICE_SRCS))

$(OUTPUT_DIR)/startup/%.o: $(STARTUP_DIR)/%.S
	@mkdir -p $(@D)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) -x assembler -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

$(OUTPUT_DIR)/user/%.o: $(USER_DIR)/%.c
	@mkdir -p $(@D)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) $(INCLUDES) -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

$(OUTPUT_DIR)/profile/%.o: $(PROFILE_DIR)/%.c
	@mkdir -p $(@D)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) $(INCLUDES) -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

$(OUTPUT_DIR)/service/%.o: $(SERVICE_DIR)/%.c
	@mkdir -p $(@D)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) $(INCLUDES) -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

$(ELF_FILE): $(STARTUP_OBJS) $(USER_OBJS) $(PROFILE_OBJS) $(SERVICE_OBJS)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) -T $(LD_FILE) -nostartfiles -Xlinker --gc-sections -Wl,-Map,$(MAP_FILE) --specs=nano.specs --specs=nosys.specs \
		-o $(ELF_FILE) $(USER_OBJS) $(STARTUP_OBJS) $(PROFILE_OBJS) $(SERVICE_OBJS) \
		-L$(LIB2_DIR) -lCH58xBLE

$(HEX_FILE): $(ELF_FILE)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-objcopy -O ihex $(ELF_FILE) $(HEX_FILE)

$(LST_FILE): $(ELF_FILE)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-objdump --all-headers --demangle --disassemble $(ELF_FILE) > $(LST_FILE)

$(SIZ_FILE): $(ELF_FILE)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-size --format=berkeley $(ELF_FILE)

.PHONY: clean
clean:
	rm -rf $(OUTPUT_DIR)/*
