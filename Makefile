obj-m := tps23861.o
KERNELDIR := /lib/modules/$(shell uname -r)/build
MODFLAGS:=-Wall -DMODULE -D__KERNEL__ -W -Wall -Wstrict-prototypes -Wmissing-prototypes

PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean: 
	@rm -f *.o .*.cmd .*.flags *.mod.c *.order 
	@rm -f .*.*.cmd *~ *.*~ TODO.* 
	@rm -fR .tmp* 
	@rm -rf .tmp_versions 
distclean: clean 
	@rm *.ko *.symvers
