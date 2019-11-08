
ifndef KERNEL_SRC
KERNEL_VER := $(shell uname -r)
KERNEL_SRC := /lib/modules/$(KERNEL_VER)/build
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD)/src EMUCDIR=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD)/src EMUCDIR=$(PWD) clean

