CC ?= gcc

all: reg_rw dma_to_device dma_from_device performance nbf nbf_tapeout_rc0 nbf_mc_tapeout_rc3

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

nbf_tapeout_rc0: nbf_tapeout_rc0.o
	$(CC) -pthread -o $@ $<

nbf_mc_tapeout_rc3: nbf_mc_tapeout_rc3.o
	$(CC) -pthread -o $@ $<

%.o: %.c
	$(CC) -c -std=c99 -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

clean:
	rm -rf reg_rw *.o *.bin dma_to_device dma_from_device performance *.riscv *.word *.nbf *.mem

BLACKPARROT_RTL_DIR ?= $(HOME)/black-parrot/bsg_fpga/vcu128/rtl
BLACKPARROT_SDK_DIR ?= $(HOME)/black-parrot/bsg_fpga/vcu128/sdk
ifeq ($(BLACKPARROT_RTL_DIR),)
$(error "Please point BLACKPARROT_RTL_DIR to a BlackParrot directory, after running make prep")
endif
ifeq ($(BLACKPARROT_SDK_DIR),)
$(error "Please point BLACKPARROT_SDK_DIR to a BlackParrot directory, after running make sdk")
endif

RISCV_OBJCOPY  ?= $(BLACKPARROT_SDK_DIR)/install/bin/riscv64-unknown-linux-gnu-objcopy
NBF_PY ?= $(BLACKPARROT_RTL_DIR)/bp_common/software/py/nbf.py
UCODE ?= $(BLACKPARROT_SDK_DIR)/bedrock/roms/mesi.mem
NCPUS ?= 1

%.riscv:
	cp $(BLACKPARROT_SDK_DIR)/prog/*/$@ $@

%.mem: %.riscv
	$(RISCV_OBJCOPY) -O verilog $< $@

%.nbf: %.mem
	python $(NBF_PY) --ncpus $(NCPUS) --config --ucode $(UCODE) --mem $< > $@

%.word: %.nbf
	python nbf_to_word.py $< > $@

%.run: %.word nbf
	sudo ./nbf /dev/xdma0_user 0x10 w 0x1 $<

