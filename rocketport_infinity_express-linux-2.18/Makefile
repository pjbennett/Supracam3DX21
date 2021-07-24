PWD  := $(shell pwd)
KVER =  $(shell uname -r)

# allow LINUX_SRC to be specified on the command-line
ifeq ($(origin LINUX_SRC), undefined)
  #  Find the kernel source.
  ifeq ($(shell test -d /usr/src/linux && echo true),true)
    LINUX_SRC=/usr/src/linux
  endif
  ifeq ($(shell test -d /usr/src/kernel-headers-$(KVER) && echo true),true)
    LINUX_SRC=/usr/src/kernel-source-$(KVER)
  endif
  ifeq ($(shell test -d /usr/src/kernel-headers-$(KVER) && echo true),true)
    LINUX_SRC=/usr/src/kernel-headers-$(KVER)
  endif
  ifeq ($(shell test -d /lib/modules/$(KVER)/build && echo true),true) 
    LINUX_SRC=/lib/modules/$(KVER)/build
  endif
endif

ifeq ($(MAKELEVEL),0)
  $(info LINUX_SRC: $(LINUX_SRC))
endif

all:
	$(MAKE) -C $(LINUX_SRC) SUBDIRS=$(PWD) modules

# turn off "caret" diagnostics that mess up our error messages
CFLAGS_rp2.o := $(call cc-option,-fno-diagnostics-show-caret)

ifeq ($(RP2_MONITOR_ENABLE),y)
CFLAGS_rp2.o += -DRP2_MONITOR_ENABLE
endif

obj-m = rp2.o

.PHONY: clean
clean:
	rm -rf *.o *.ko *.ver *.mod.[co] *.mod .$(OBJ)*.cmd .tmp* *~ .*.o.d *.symvers *.markers *.order .cache.mk

DEFAULT_CONFIG="polling_mode=100 port_modes=0-255:rs232"

.PHONY: install
install:
	./install.sh rp2 $(DEFAULT_CONFIG) $(KVER) 

.PHONY: install_rebuild_initramfs
install_rebuild_initramfs:
	REBUILD_INITRAMFS=y ./install.sh rp2 $(DEFAULT_CONFIG) $(KVER)

