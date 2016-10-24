obj-m := usb_driver1.o
usb_driver1-y := usb.o scsiglue.o transport.o protocol.o usual-tables.o

CROSS_COMPILER = 0
ifeq ('$(CROSS_COMPILER)', '0')

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

else

all:
	make -C /home/venkyvenkatesh/linux-4.1/ M=$(PWD) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules

clean:
	make -C /home/venkyvenkatesh/linux-4.1/ M=$(PWD) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean

endif
