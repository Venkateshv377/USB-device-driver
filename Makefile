obj-m := venky-storage.o
venky-storage-y := usb.o protocol.o transport.o scsiglue.o initializers.o usual-tables.o sierra_ms.o option_ms.o

CROSS_COMPILER = 0
ifeq ('$(CROSS_COMPILER)', '0')

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

else

all:
	make -C /home/venkyvenkatesh/linux-4.1 M=$(PWD) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules

clean:
	make -C /home/venkyvenkatesh/linux-4.1 M=$(PWD) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean

endif
