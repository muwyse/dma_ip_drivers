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

NCPUS ?= 1
UCODE ?= 0
UCODE_MEM ?= $(BP_SDK_DIR)/bedrock/roms/moesif.mem
SKIP_ZEROS ?= 0
MEM_SIZE ?= 64
CONFIG ?= 0

NBF_INPUTS ?= --ncpus $(NCPUS)
ifeq ($(UCODE),1)
NBF_INPUTS += --ucode $(UCODE_MEM)
endif
ifeq ($(SKIP_ZEROS), 1)
NBF_INPUTS += --skip_zeros
endif
ifneq ($(CONFIG), 0)
NBF_INPUTS += --config
endif
NBF_INPUTS += --mem_size=$(MEM_SIZE)
#NBF_INPUTS += --boot_pc=0x80000000
NBF_INPUTS += --bootrom bootrom.mem
NBF_INPUTS += --mem

%.riscv:
	cp $(BP_SDK_DIR)/prog/*/$@ $@

%.mem: %.riscv
	$(RISCV_OBJCOPY) -O verilog $< $@

%.dump: %.riscv
	$(RISCV_OBJDUMP) -D $< > $@

%.nbf: %.mem bootrom.mem
	#sed -i "s/@8/@0/g" $<
	python $(NBF_PY) $(NBF_INPUTS) $< > $@

%.word: %.nbf
	python nbf_to_word.py $< > $@

# args: device nbf_file NCPUS
# w means write
%.run: %.word nbf
	sudo ./nbf /dev/xdma0_user $< $(NCPUS) 2>&1 | tee -i run.log

