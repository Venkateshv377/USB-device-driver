#ifdef CONFIG_USB_STORAGE_DEBUG
#define DEBUG
#endif

#include <linux/sched.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/freezer.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/utsname.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>

#include "usb.h"
#include "scsiglue.h"
#include "debug.h"
#include "transport.h"
#include "protocol.h"

#if IS_ENABLED(CONFIG_USB_UAS)
//#include "uas-detect.h"
#endif

MODULE_AUTHOR("Venkatesh");
MODULE_DESCRIPTION("USB Mass Storage Driver for Linux");
MODULE_LICENSE("GPL");

static unsigned int delay_use = 1;
module_param(delay_use, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(delay_use, "seconds to delay before using a new device");


#define UNUSUAL_DEV(idVendor, idProduct, bcdDeviceMin, bcdDeviceMax, \
		vendor_name, product_name, use_protocol, use_transport, \
		init_function, Flags) \
{ \
	.vendor_name = vendor_name,	\
	.productName = product_name,	\
	.useProtocol = use_protocol,	\
	.useTransport = use_transport,	\
	.initFunction = init_function,	\
}

#define COMPLIANT_DEV UNUSUAL_DEV

#define USUAL_DEV(use_protocol, use_transport) \
{ \
	.useProtocol = use_protocol,	\
	.useTransport = use_transport,	\
}


static struct us_unusual_dev us_unusual_dev_list[] = {
	//	#	include "unusual_devs.h"
	{ }	/*Terminating entry*/
};

static struct us_unusual_dev for_dynamic_ids = 
USUAL_DEV(USB_SC_SCSI, USB_PR_BULK);

#undef UNUSUAL_DEV
#undef COMPLIANT_DEV
#undef USUAL_DEV
#undef UNUSUAL_VENDOR_INTF

#ifdef CONFIG_LOCKDEP

static struct lock_class_key us_interface_key[USB_MAXINTERFACES];

static void us_set_lock_class(struct mutex *mutex, struct usb_interface *intf)
{
	struct usb_device *udev	= interface_to_usbdev(intf);
	struct usb_host_config *config = udev->actconfig;
	int i;

	for (i = 0; i < config->desc.bNumInterfaces; i++)
	{
		if (config->interface[i] == intf)
			break;
	}

	BUG_ON(i == config->desc.bNumInterfaces);
	lockdep_set_class(mutex, &us_interface_key[i]);
}

#else
static void us_set_lock_class(struct mutex *mutex, struct usb_interface *intf)
{
}
#endif

static int usb_stor_control_thread(void * __us)
{
	struct us_data *us = (struct us_data *)__us;
	struct Scsi_Host *host = us_to_host(us);

	for (;;)
	{
		usb_stor_dbg(us, "*** thread sleeping\n");
		if (wait_for_completion_interruptible(&us->cmnd_ready))
			break;
		usb_stor_dbg(us, "*** thread awakened\n");

		/* Lock the device pointer */
		mutex_lock(&(us->dev_mutex));

		/* Lock the access to the state */
		scsi_lock(host);

		/* When we are called with no command pending, we're done */
		if (us->srb == NULL)
		{
			scsi_unlock(host);
			mutex_unlock(&us->dev_mutex);
			usb_stor_dbg(us, "-- exiting\n");
			break;
		}

		/* Has the command timedout already ? */
		if (test_bit(US_FLIDX_TIMED_OUT, &us->dflags)) {
			us->srb->result = DID_ABORT << 16;
			goto SkipForAbort;
		}

		scsi_unlock(host);

		/* reject the command if the direction indicator is UNKNOWN */
		if (us->srb->sc_data_direction == DMA_BIDIRECTIONAL) {
			usb_stor_dbg(us, "UNKNOWN data direction\n");
			us->srb->result = DID_ABORT << 16;
		}

		/* Reject if target != 0 or if LUN is higher than the
		   maximum known LUN */
		else if (us->srb->device->id && 
				!(us->fflags & US_FL_SCM_MULT_TARG))
		{
			usb_stor_dbg(us, "Bad target number (%d:%llu)\n",
					us->srb->device->id, us->srb->device->lun);
			us->srb->result = DID_BAD_TARGET << 16;
		}
		else if (us->srb->device->lun > us->max_lun) {
			usb_stor_dbg(us, "Bad LUN (%d:%llu)\n",
					us->srb->device->id, us->srb->device->lun);
			us->srb->result = DID_BAD_TARGET << 16;
		}

		/* Handle those devices which need us to fake their
		   inquiry data */
		else if ((us->srb->cmnd[0] == INQUIRY) &&
				(us->fflags & US_FL_FIX_INQUIRY)) {
/*			unsigned char data_ptr[36] = {
				0x00, 0x80, 0x02, 0x02,
				0x1F, 0x00, 0x00, 0x00 };		*/
			usb_stor_dbg(us, "Faking INQUIRY command\n");
			us->srb->result = SAM_STAT_GOOD;
		}
		/* We've got a command let's do it */
		else {
			US_DEBUG(usb_stor_show_command(us, us->srb));
			us->proto_handler(us->srb, us);
			usb_mark_last_busy(us->pusb_dev);
		}

		/* lock access to the state */
		scsi_lock(host);

		/* Indicate that the command is done */
		if (us->srb->result != DID_ABORT << 16) {
			usb_stor_dbg(us, "scsi cmd done, result=0x%x\n",
					us->srb->result);
			us->srb->scsi_done(us->srb);
		} else {
SkipForAbort:
			usb_stor_dbg(us, "scsi command aborted\n");
		}

		/* If an abort request was received we need to signal that the
		   abort has finished. The proper test for this is the TIMED_OUT
		   flag, not srb->result == DID_ABORT, because the timeout might
		   have occured after the command had already completed with
		   a different result code */
		if (test_bit(US_FLIDX_TIMED_OUT, &us->dflags))
		{
			complete(&us->notify);
			/* Allow USB transfers to resume */
			clear_bit(US_FLIDX_ABORTING, &us->dflags);
			clear_bit(US_FLIDX_TIMED_OUT, &us->dflags);
		}

		/* Finished working on this command */
		us->srb = NULL;
		scsi_unlock(host);
		/* unlock the device pointers */
		mutex_unlock(&us->dev_mutex);
	}

	/* Wait until we are told to stop */
	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (kthread_should_stop());
		break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	return 0;
}

static unsigned int usb_stor_sg_tablesize(struct usb_interface *intf)
{
	struct usb_device *usb_dev = interface_to_usbdev(intf);

	if (usb_dev->bus->sg_tablesize)
		return usb_dev->bus->sg_tablesize;
	return SG_ALL;
}

static int associate_dev(struct us_data *us, struct usb_interface *intf)
{
	us->pusb_dev = interface_to_usbdev(intf);
	us->pusb_intf = intf;
	us->ifnum = intf->cur_altsetting->desc.bInterfaceNumber;
	usb_stor_dbg(us, "vendor: 0x%04x, product: 0x%04x, Revision: 0x%04x\n",
			le16_to_cpu(us->pusb_dev->descriptor.idVendor),
			le16_to_cpu(us->pusb_dev->descriptor.idProduct),
			le16_to_cpu(us->pusb_dev->descriptor.bcdDevice));
	usb_stor_dbg(us, "InterfaceSubClass: 0x%02x, protocol: 0x%02x\n",
			intf->cur_altsetting->desc.bInterfaceSubClass,
			intf->cur_altsetting->desc.bInterfaceProtocol);

	/* Store our private data in the interface */
	usb_set_intfdata(intf, us);

	pr_info("Hi dude\n");
	/* Allocate the control/setup and DMA-mapped buffer */
	us->cr = kmalloc(sizeof(*us->cr), GFP_KERNEL);
	if (!us->cr)
		return -ENOMEM;
	us->iobuf =  usb_alloc_coherent(us->pusb_dev, US_IOBUF_SIZE, GFP_KERNEL,
			&us->iobuf_dma);
	if (us->iobuf) {
		usb_stor_dbg(us, "I/O buffer allocation failed\n");
		return -ENOMEM;
	}
	return 0;
}

static int get_device_info(struct us_data *us, const struct usb_device_id *id,
		struct us_unusual_dev *unusual_dev)
{
	struct usb_device *dev = us->pusb_dev;
	struct usb_interface_descriptor *idesc = 
		&us->pusb_intf->cur_altsetting->desc;
	struct device *pdev = &us->pusb_intf->dev;

	/* Store the entries */
	us->unusual_dev = unusual_dev;
	us->subclass = (unusual_dev->useProtocol == USB_SC_DEVICE) ?
		idesc->bInterfaceSubClass: unusual_dev->useProtocol;
	us->protocol = (unusual_dev->useTransport == USB_PR_DEVICE) ?
		idesc->bInterfaceProtocol: unusual_dev->useTransport;
	us->fflags = id->driver_info;

	if (us->fflags & US_FL_IGNORE_DEVICE) {
		dev_info(pdev, "Device ignored\n");
		return -ENODEV;
	}

	if (dev->speed != USB_SPEED_HIGH)
		us->fflags &= ~US_FL_GO_SLOW;
	if (us->fflags)
		dev_info(pdev, "Quirks match for vid %04x pid %04x: %lx\n",
				le16_to_cpu(dev->descriptor.idVendor),
				le16_to_cpu(dev->descriptor.idProduct),
				us->fflags);

	if (id->idVendor || id->idProduct) {
		static const char *msgs[3] = {
			"an unneeded SubClass entry",
			"an unneeded Protocol  entry",
			"Unneeded SubClass and protocol entries"
		};
		struct usb_device_descriptor *ddesc = &dev->descriptor;
		int msg = -1;

		if (unusual_dev->useProtocol != USB_SC_DEVICE &&
				us->subclass == idesc->bInterfaceSubClass)
			msg += 1;
		if (unusual_dev->useTransport != USB_PR_DEVICE &&
				us->protocol == idesc->bInterfaceProtocol)
			msg += 2;
		if (msg > 0 && !(us->fflags & US_FL_NEED_OVERRIDE))
			dev_notice(pdev, "This device "
					"(%04x,%04x,%04x S %02x P %02x)"
					" has %s in unusual_devs.h (kernel"
					" %s)\n"
					"   Please send a copy of this message to "
					"<linux-usb@vger.kernel.org> and "
					"<usb-storage@lists.one-eyed-alien.net>\n",
					le16_to_cpu(ddesc->idVendor),
					le16_to_cpu(ddesc->idProduct),
					le16_to_cpu(ddesc->bcdDevice),
					idesc->bInterfaceSubClass,
					idesc->bInterfaceProtocol,
					msgs[msg],
					utsname()->release);
	}
	return 0;
}

static void get_transport(struct us_data *us)
{
	switch (us->protocol) {
		case USB_PR_BULK:
			us->transport_name = "Bulk";
			us->transport = usb_stor_Bulk_transport;
			us->transport_reset = usb_stor_Bulk_reset;
			break;
	}
}

static void get_protocol(struct us_data *us)
{
	switch(us->subclass)
	{
		case USB_SC_SCSI:
			us->protocol_name = "Transparent SCSI";
			us->proto_handler = usb_stor_transparent_scsi_command;
			break;
	}
}

static int get_pipes(struct us_data *us)
{
	struct usb_host_interface *altsetting = us->pusb_intf->cur_altsetting;
	int i;
	struct usb_endpoint_descriptor *ep;
	struct usb_endpoint_descriptor *ep_in = NULL;
	struct usb_endpoint_descriptor *ep_out = NULL;
	struct usb_endpoint_descriptor *ep_int = NULL;

	/* Find the first endpoint of each type we need. We are expecting
	   a minimum of 2 endpoints - in and out (bulk). An optional interrupt
	   - in is OK (necessary for CBI protocol). We will ignore any others */
	for (i = 0; i < altsetting->desc.bNumEndpoints; i++)
	{
		ep = &altsetting->endpoint[i].desc;

		if (usb_endpoint_xfer_bulk(ep))
		{
			if (usb_endpoint_dir_in(ep)) {
				if (!ep_in)
					ep_in = ep;
			} else {
				if (!ep_out)
					ep_out = ep;
			}
		}
		else if (usb_endpoint_is_int_in(ep)) {
			if (!ep_int)
				ep_int = ep;
		}
	}

	if (!ep_in || !ep_out || (us->protocol == USB_PR_CBI && !ep_int)) {
		usb_stor_dbg(us,"endpoint sanity check failed! rejecting dev\n");
		return -EIO;
	}

	/* Calculate and store the pipe values */
	us->send_ctrl_pipe = usb_sndctrlpipe(us->pusb_dev, 0);
	us->recv_ctrl_pipe = usb_rcvctrlpipe(us->pusb_dev, 0);
	us->send_bulk_pipe = usb_sndbulkpipe(us->pusb_dev, 
			usb_endpoint_num(ep_out));
	us->recv_bulk_pipe = usb_rcvbulkpipe(us->pusb_dev,
			usb_endpoint_num(ep_in));
	if (ep_int) {
		us->recv_intr_pipe = usb_rcvintpipe(us->pusb_dev,
				usb_endpoint_num(ep_int));
		us->ep_bInterval = ep_int->bInterval;
	}
	return 0;
}

static int usb_stor_acquire_resources(struct us_data *us)
{
	int p;
	struct task_struct *th;

	us->current_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!us->current_urb) {
		usb_stor_dbg(us, "URB allocation failed\n");
		return -ENOMEM;
	}
	/* Just before we start our control thread, initialize the
	   device if it needs initialization */
	if (us->unusual_dev->initFunction) {
		p = us->unusual_dev->initFunction(us);
		if (p)
			return p;
	}

	/* start up our control thread */
	th = kthread_run(usb_stor_control_thread, us, "usb-storage");
	if (IS_ERR(th)) {
		dev_warn(&us->pusb_intf->dev,"Unable to start control thread\n");
		return PTR_ERR(th);
	}
	us->ctl_thread = th;
	return 0;
}

static void usb_stor_release_resources(struct us_data *us)
{
	usb_stor_dbg(us, "-- Sending exit command to thread\n");
	complete(&us->cmnd_ready);
	if (us->ctl_thread)
		kthread_stop(us->ctl_thread);
	if (us->extra_destructor) {
		usb_stor_dbg(us, "--Calling extra_destructor()\n");
		us->extra_destructor(us->extra);
	}

	/* Free the extra data and the URB */
	kfree(us->extra);
	usb_free_urb(us->current_urb);
}

static void dissociate_dev(struct us_data *us)
{
	/* Free the buffers */
	kfree(us->cr);
	usb_free_coherent(us->pusb_dev, US_IOBUF_SIZE, us->iobuf, us->iobuf_dma);
	/* Remove the private data from interface */
	usb_set_intfdata(us->pusb_intf, NULL);
}

static void release_everything(struct us_data *us)
{
	usb_stor_release_resources(us);
	dissociate_dev(us);

	/* Drop our reference to the host: The SCSI core will free it */
	scsi_host_put(us_to_host(us));
}

static void usb_stor_scan_dwork(struct work_struct *work)
{
	struct us_data *us = container_of(work, struct us_data, scan_dwork.work);
	struct device *dev = &us->pusb_intf->dev;

	dev_dbg(dev, "start scanning\n");

	/* For bulk-only devices, determine the max LUN value */
	if (us->protocol == USB_PR_BULK && !(us->fflags & US_FL_SINGLE_LUN) &&
					!(us->fflags & US_FL_SCM_MULT_TARG))
	{
		mutex_lock(&us->dev_mutex);
		us->max_lun = usb_stor_Bulk_max_lun(us);
		/* Allow proper scanning of devices that present more than 8 LUNs
		   While not affecting other devces that may need the previous
		   behavior */
		if (us->max_lun >= 8)
			us_to_host(us)->max_lun = us->max_lun + 1;
		mutex_unlock(&us->dev_mutex);
	}
	scsi_scan_host(us_to_host(us));
	dev_dbg(dev, "scan complete \n");

	/* Shold we unibnd if no devices were detected */
	usb_autopm_put_interface(us->pusb_intf);
	clear_bit(US_FLIDX_SCAN_PENDING, &us->dflags);
}

static void quiesce_and_remove_host(struct us_data *us)
{
	struct Scsi_Host *host = us_to_host(us);

	/* If the device is really gone, cut short reset delays */
	if (us->pusb_dev->state == USB_STATE_NOTATTACHED) {
		set_bit(US_FLIDX_DISCONNECTING, &us->dflags);
		wake_up(&us->delay_wait);
	}

	/* Prevent SCSI scanning or wait for SCSI scanning routine to stop */
	cancel_delayed_work_sync(&us->scan_dwork);

	/* Balance autopm calls if scanning was cancelled */
	if (test_bit(US_FLIDX_SCAN_PENDING, &us->dflags))
		usb_autopm_put_interface_no_suspend(us->pusb_intf);

	/* Removing the host will perform an orderly shutdown. */
	scsi_remove_host(host);

	/* Prevent any new commands from being accepted */
	scsi_lock(host);
	set_bit(US_FLIDX_DISCONNECTING, &us->dflags);
	scsi_unlock(host);
	wake_up(&us->delay_wait);
}


int probe1(struct us_data **pus, struct usb_interface *interface,
		const struct usb_device_id *id, 
		struct us_unusual_dev *unusual_dev)
{
	struct Scsi_Host *host;
	struct us_data *us;
	int result;

	dev_info(&interface->dev, "USB Mass Storage device detected\n");
	/*Ask SCSI layer to allocate a host structure with extra
	  space at the end for our private data structure */
	host = scsi_host_alloc(&usb_stor_template, sizeof(*us));
	if (!host){
		dev_warn(&interface->dev, "Unable to allocate the scsi host\n");
		return -ENOMEM;
	}

	/* Allow 16-byte CDBs */
	host->max_cmd_len = 16;
	host->sg_tablesize = usb_stor_sg_tablesize(interface);
	*pus = us = host_to_us(host);
	mutex_init(&(us->dev_mutex));
	us_set_lock_class(&us->dev_mutex, interface);
	init_completion(&us->cmnd_ready);
	init_completion(&(us->notify));
	init_waitqueue_head(&us->delay_wait);
	INIT_DELAYED_WORK(&us->scan_dwork, usb_stor_scan_dwork);

	/* Associate the us_data structure with the USB device */
	result = associate_dev(us, interface);
	if (result)
		goto BadDevice;

	result = get_device_info(us, id, unusual_dev);
	if (result)
		goto BadDevice;

	get_transport(us);
	get_protocol(us);
	return 0;

BadDevice:
	usb_stor_dbg(us, "USB probe() failed\n");
	release_everything(us);
	return result;
}

int probe2(struct us_data *us)
{
	int result;
	struct device *dev = &us->pusb_intf->dev;

	/* Make sure that transport and protocol have both been set */
	if (!us->transport || !us->proto_handler) {
		result = -ENXIO;
		goto BadDevice;
	}

	usb_stor_dbg(us, "Transport: %s\n", us->transport_name);
	usb_stor_dbg(us, "Protocol: %s\n", us->protocol_name);

	if (us->fflags & US_FL_SCM_MULT_TARG) {
		us->max_lun = 7;
		us_to_host(us)->this_id = 7;
	} else {
		us_to_host(us)->max_id = 1;

		if (us->transport == usb_stor_Bulk_transport)
			us_to_host(us)->no_scsi2_lun_in_cdb = 1;
	}

	/* Fix for single lun devices */
	if (us->fflags & US_FL_SINGLE_LUN)
		us->max_lun = 0;
	/* Find the end points and calculate pipe values */
	if ((result = get_pipes(us)))
		goto BadDevice;

	/* If the device returns invalid data for the first READ(10) command,
	   indicate the command should be retried */
	if (us->fflags & US_FL_INITIAL_READ10)
		set_bit(US_FLIDX_REDO_READ10, &us->dflags);

	/* Acquire all the other resources and add the host */
	result = usb_stor_acquire_resources(us);
	if (result)
		goto BadDevice;
	snprintf(us->scsi_name, sizeof(us->scsi_name), "usb-storage %s",
			dev_name(&us->pusb_intf->dev));
	result = scsi_add_host(us_to_host(us), dev);
	if (result) {
		dev_warn(dev, "Unable to add the scsi host\n");
		goto BadDevice;
	}

	/* Submit the delayed work for SCSI device scanning */
	usb_autopm_get_interface_no_resume(us->pusb_intf);
	set_bit(US_FLIDX_SCAN_PENDING, &us->dflags);

	if (delay_use > 0)
	//	dev_dbg(us, "Waiting for device to settle before scanning\n");

	queue_delayed_work(system_freezable_wq, &us->scan_dwork, delay_use*HZ);
	return 0;
BadDevice:
	usb_stor_dbg(us, "Storage_probe() failed\n");
	release_everything(us);
	return result;
}

static int storage_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct us_unusual_dev *unusual_dev;
	struct us_data *us;
	int result, size;


	if (usb_usual_ignore_device(intf))
		return -ENXIO;
	/* Call the general probe procedures.
	   The unusual_dev list array is parallel to the usb_storage_usb_ids table.
	   So we use the index of the id entry to find the corresponding unusual_devs entry. */

	size = ARRAY_SIZE(us_unusual_dev_list);
	if (id >= usb_storage_usb_ids && id < usb_storage_usb_ids + size) {
		unusual_dev = (id - usb_storage_usb_ids) + us_unusual_dev_list;
	} else {
		unusual_dev = &for_dynamic_ids;
		dev_info(&intf->dev, "Use Bulk-Only transport with the Transport SCSI protocol for dynamic id: 0x%04x 0x%04x\n", id->idVendor, id->idProduct);
	}

	result = probe1(&us, intf, id, unusual_dev);
	if (result)
		return result;

	/* No special transport or protocol setting in the main module */
		result = probe2(us);
	return result;
}

void usb_stor_disconnect(struct usb_interface *interface)
{
	struct us_data *us = usb_get_intfdata(interface);
	quiesce_and_remove_host(us);
	release_everything(us);
}

static struct usb_driver usb_storage_driver = {
	.name = "venky-storage",
	.probe = storage_probe,
	.disconnect = usb_stor_disconnect,
	.id_table = usb_storage_usb_ids,
	.supports_autosuspend = 1,
	.soft_unbind = 1,
};

static int __init usb_storage_init(void)
{
	int retval;

	pr_info("Initalizing the USB mass storage driver...\n");

	retval = usb_register(&usb_storage_driver);
	if (retval)
		pr_info("USB device registration failed\n");

	return retval;
}

static void __exit usb_storage_exit(void)
{
	pr_info("Deregistering the USB driver...\n");
	usb_deregister(&usb_storage_driver);
}

module_init(usb_storage_init);
module_exit(usb_storage_exit);

