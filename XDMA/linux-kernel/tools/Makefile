TOP ?= $(shell git rev-parse --show-toplevel)

# BlackParrot paths
BP_SDK_DIR ?= $(abspath $(TOP)/../black-parrot-sdk)
BP_RTL_DIR ?= $(abspath $(TOP)/../black-parrot)
FPGA_DIR   ?= $(abspath $(TOP)/../fpga)
ifeq ($(BP_RTL_DIR),)
$(error "Please point BP_RTL_DIR to a BlackParrot directory, after running make prep")
endif
ifeq ($(BP_SDK_DIR),)
$(error "Please point BP_SDK_DIR to a BlackParrot directory, after running make sdk")
endif

RISCV_OBJCOPY  ?= $(BP_SDK_DIR)/install/bin/riscv64-unknown-linux-gnu-objcopy
RISCV_OBJDUMP  ?= $(BP_SDK_DIR)/install/bin/riscv64-unknown-linux-gnu-objdump
NBF_PY ?= $(FPGA_DIR)/software/nbf.py

CC ?= gcc

all: reg_rw dma_to_device dma_from_device performance nbf

dma_to_device: dma_to_device.o
	$(CC) -lrt -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

dma_from_device: dma_from_device.o
	$(CC) -lrt -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

performance: performance.o
	$(CC) -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

reg_rw: reg_rw.o
	$(CC) -o $@ $<

nbf: nbf.o
	$(CC) -pthread -o $@ $<

%.o: %.c
	$(CC) -c -std=c99 -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

clean:
	rm -rf reg_rw *.o *.bin dma_to_device dma_from_device performance nbf

clean_progs:
	rm -rf *.riscv *.word *.nbf *.mem

NBF_NCPUS ?= 1
NBF_MEM_SIZE ?= 256
NBF_BOOTROM ?= bootrom.mem
NBF_BOOTROM_SIZE ?= 64
NBF_UCODE ?=
NBF_SKIP_ZEROS ?= 1
NBF_BOOT_PC ?=
NBF_CONFIG ?= 0
NBF_DEBUG ?= 1
NBF_VERIFY ?= 0
NBF_START_CACHED ?= 1

#NBF_UCODE_MEM ?= $(BP_SDK_DIR)/bedrock/roms/moesif.mem

NBF_FLAGS ?= --ncpus $(NBF_NCPUS) --data_width 64
NBF_FLAGS += --bootrom bootrom.mem --bootrom_size $(NBF_BOOTROM_SIZE)
ifneq ($(NBF_UCODE),)
NBF_FLAGS += --ucode $(NBF_UCODE)
endif
ifneq ($(NBF_BOOT_PC),)
NBF_FLAGS += --boot_pc $(NBF_BOOT_PC)
endif
ifeq ($(NBF_SKIP_ZEROS), 1)
NBF_FLAGS += --skip_zeros
endif
ifeq ($(NBF_CONFIG), 1)
NBF_FLAGS += --config
endif
ifeq ($(NBF_DEBUG), 1)
NBF_FLAGS += --debug
endif
ifeq ($(NBF_VERIFY), 1)
NBF_FLAGS += --verify
endif
ifeq ($(NBF_START_CACHED), 1)
NBF_FLAGS += --start_cached
endif
NBF_FLAGS += --mem_size=$(NBF_MEM_SIZE)

%.riscv:
	cp $(BP_SDK_DIR)/prog/*/$@ $@

%.mem: %.riscv
	$(RISCV_OBJCOPY) -O verilog $< $@

%.dump: %.riscv
	$(RISCV_OBJDUMP) -D $< > $@

%.nbf: %.mem bootrom.mem
	python $(NBF_PY) $(NBF_FLAGS) --mem $< > $@

%.word: %.nbf
	python nbf_to_word.py $< > $@

%.run: %.word nbf
	sudo ./nbf /dev/xdma0_user run $< $(NBF_NCPUS) 2>&1 | tee -i run.log

