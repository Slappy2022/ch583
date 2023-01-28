TOOL_CHAIN_PATH ?= /opt/gcc-riscv/xpacks/.bin
PROJECT_NAME    = UART3

TOP_DIR     := ..
OUTPUT_DIR  := $(TOP_DIR)/Build
STARTUP_DIR := $(TOP_DIR)/SRC/Startup
CORE_DIR    := $(TOP_DIR)/SRC/RVMSIS
SPL_DIR     := $(TOP_DIR)/SRC/StdPeriphDriver
USER_DIR    := $(TOP_DIR)/$(PROJECT_NAME)/src

LD_FILE     := $(TOP_DIR)/SRC/Ld/Link.ld
MAP_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).map
ELF_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).elf
HEX_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).hex
LST_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).lst
SIZ_FILE    := $(OUTPUT_DIR)/$(PROJECT_NAME).siz

INCLUDES := $(INCLUDES)
INCLUDES += -I $(TOP_DIR)/SRC/RVMSIS
INCLUDES += -I $(TOP_DIR)/SRC/StdPeriphDriver/inc

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

CORE_SRCS := $(wildcard $(CORE_DIR)/*.c)
CORE_OBJS := $(patsubst $(CORE_DIR)/%.c, $(OUTPUT_DIR)/core/%.o, $(CORE_SRCS))

SPL_SRCS := $(wildcard $(SPL_DIR)/*.c)
SPL_OBJS := $(patsubst $(SPL_DIR)/%.c, $(OUTPUT_DIR)/spl/%.o, $(SPL_SRCS))

USER_SRCS := $(wildcard $(USER_DIR)/*.c)
USER_OBJS := $(patsubst $(USER_DIR)/%.c, $(OUTPUT_DIR)/user/%.o, $(USER_SRCS))


$(OUTPUT_DIR)/startup/%.o: $(STARTUP_DIR)/%.S
	@mkdir -p $(@D)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) -x assembler -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

$(OUTPUT_DIR)/core/%.o: $(CORE_DIR)/%.c
	@mkdir -p $(@D)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) $(INCLUDES) -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

$(OUTPUT_DIR)/spl/%.o: $(SPL_DIR)/%.c
	@mkdir -p $(@D)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) $(INCLUDES) -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

$(OUTPUT_DIR)/user/%.o: $(USER_DIR)/%.c
	@mkdir -p $(@D)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) $(INCLUDES) -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

$(ELF_FILE): $(STARTUP_OBJS) $(CORE_OBJS) $(SPL_OBJS) $(USER_OBJS)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-gcc $(CCFLAGS) -T $(LD_FILE) -nostartfiles -Xlinker --gc-sections -Wl,-Map,$(MAP_FILE) --specs=nano.specs --specs=nosys.specs -o $(ELF_FILE) $(USER_OBJS) $(STARTUP_OBJS) $(CORE_OBJS) $(SPL_OBJS) -L$(SPL_DIR) -lISP583

$(HEX_FILE): $(ELF_FILE)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-objcopy -O ihex $(ELF_FILE) $(HEX_FILE)

$(LST_FILE): $(ELF_FILE)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-objdump --all-headers --demangle --disassemble $(ELF_FILE) > $(LST_FILE)

$(SIZ_FILE): $(ELF_FILE)
	$(TOOL_CHAIN_PATH)/riscv-none-elf-size --format=berkeley $(ELF_FILE)

.PHONY: clean
clean:
	rm -f $(OUTPUT_DIR)/*
