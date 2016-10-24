#include <linux/sched.h>
#include <linux/gfp.h>
#include <linux/errno.h>
#include <linux/export.h>

#include <linux/usb/quirks.h>

#include <scsi/scsi.h>
#include <scsi/scsi_eh.h>
#include <scsi/scsi_device.h>

#include "usb.h"
#include "transport.h"
#include "protocol.h"
#include "scsiglue.h"
#include "debug.h"

#include <linux/blkdev.h>
#include "sd.h"


/* This is the completion handler which will wake us up when the URB completes */
static void usb_stor_blocking_completion(struct urb *urb)
{
	struct completion *urb_done_ptr = urb->context;
	complete(urb_done_ptr);
}

static int usb_stor_msg_common(struct us_data *us, int timeout)
{
	struct completion urb_done;
	long timeleft;
	int status;

	/* Don't submit URBs during abort processing */
	if (test_bit(US_FLIDX_ABORTING, &us->dflags))
		return -EIO;

	/* Set up data structure for the wakeup system */
	init_completion(&urb_done);

	/* Fill the common fields in the URB */
	us->current_urb->context = &urb_done;
	us->current_urb->transfer_flags = 0;

	/* We assume that if transfer buffer isn't us->iobuf then it hasn't
	   been mapped for DMA. Yes, this is clunky, but it's easier than always
	   having the caller tell us whether the transfer buffer has
	   already been mapped */
	if (us->current_urb->transfer_buffer == us->iobuf)
		us->current_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	us->current_urb->transfer_dma = us->iobuf_dma;

	/* Submit the URB */
	status = usb_submit_urb(us->current_urb, GFP_NOIO);
	if (status)
		return status;
	/* Since the URB has been submitting successfully, it's now okay to cancel it */
	set_bit(US_FLIDX_URB_ACTIVE, &us->dflags);

	/* Did an abort occur during the submisssion */
	if (test_bit(US_FLIDX_ABORTING, &us->dflags)) {
		/* Cancel the URB, if it hasn't been cancelled yet */
		if (test_and_clear_bit(US_FLIDX_URB_ACTIVE, &us->dflags))
		{
			usb_stor_dbg(us, "--Cancelling the URB\n");
			usb_unlink_urb(us->current_urb);
		}
	}

	/* Wait for the completion of the URB */
	timeleft = wait_for_completion_interruptible_timeout(
			&urb_done, timeout ? : MAX_SCHEDULE_TIMEOUT);
	clear_bit(US_FLIDX_URB_ACTIVE, &us->dflags);

	if (timeleft <= 0) {
		usb_stor_dbg(us, "%s -- Cancelling URB\n", timeleft == 0 ? "Timeout" : "Signal");
		usb_kill_urb(us->current_urb);
	}

	/* Return the URB status */
	return us->current_urb->status;
}

int usb_stor_control_msg(struct us_data *us, unsigned int pipe, 
		u8 request, u8 requesttype, u16 value,
		u16 index, void *data, u16 size, int timeout)
{
	int status;

	usb_stor_dbg(us, "rq=%02x rqtype=%02x value=%04x index=%02x len=%u\n", 
			request, requesttype, value, index, size);

	/* Setup the data for USB device control request */
	/* Fill in the dev request structure */
	us->cr->bRequestType	=	requesttype;
	us->cr->bRequest	=	request;
	us->cr->wValue		=	cpu_to_le16(value);
	us->cr->wIndex		=	cpu_to_le16(index);
	us->cr->wLength		=	cpu_to_le16(size);

	/* Fill and submit the URB */
	usb_fill_control_urb(us->current_urb, us->pusb_dev,
			pipe, (unsigned char*) us->cr, 
			data, size, usb_stor_blocking_completion, NULL);
	status = usb_stor_msg_common(us, timeout);

	/* Return the actual length of the data transferred if no error */
	if (status == 0)
		status = us->current_urb->actual_length;
	return status;
}


/* Bulk only transport */
/* Determine what the maximum LUN supported is */
int usb_stor_Bulk_max_lun(struct us_data *us)
{
	int result;

	/* Issue the command */
	us->iobuf[0] = 0;
	result = usb_stor_control_msg(us, us->recv_ctrl_pipe,US_BULK_GET_MAX_LUN,
			USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, 
			0, us->ifnum, us->iobuf, 1, 10*HZ);
	usb_stor_dbg(us, "GetMaxLUN command result is %d, data is %d\n",
			result, us->iobuf[0]);
	/*
	 * If we have a successful request, return the result if valid. The
	 * CBW LUN field is 4 bits wide, so the value reported by the device
	 * should fit into that.
	 */
	if (result > 0) { 
		if (us->iobuf[0] < 16) {
			return us->iobuf[0];
		} else {
			dev_info(&us->pusb_intf->dev,
					"Max LUN %d is not valid, using 0 instead", us->iobuf[0]);
		}
	}

	/*
	 * Some devices don't like GetMaxLUN.  They may STALL the control
	 * pipe, they may return a zero-length result, they may do nothing at
	 * all and timeout, or they may fail in even more bizarrely creative
	 * ways.  In these cases the best approach is to use the default
	 * value: only one LUN.
	 */
	return 0;

}

/***************************************************************
 * Transport Routines
 **************************************************************/
/* There are so many devices that report the capacity incorrectly, this routine
   was written to counteract some of the resulting problems */
static void last_sector_hacks(struct us_data *us, struct scsi_cmnd *srb)
{
	struct gendisk *disk;
	struct scsi_disk *sdkp;
	u32 sector;

	/* to report "Medium error: record not found */
	static unsigned char record_not_found[18] = {
		[0]	= 0x70,			/*Current error */
		[2]	= MEDIUM_ERROR,		/* = 0x03 */
		[7]	= 0x0a,			/* Additional length */
		[12]	= 0x14,			/* record not found */
	};

	/* If last-sector problems can't occur, whether because the capacity
	   was already decremented or because the device is known to report
	   the correct capacity, then we don't need to do anything */
	if (!us->use_last_sector_hacks)
		return;
	/* Was this command a READ(10) or a WRITE(10)? */
	if (srb->cmnd[0] != READ_10 && srb->cmnd[0] != WRITE_10)
		goto done;
	/* Did this command access the last sector? */
	sector = (srb->cmnd[2] << 24) | (srb->cmnd[3] << 16) |
		(srb->cmnd[4] << 8) | (srb->cmnd[5]);
	disk = srb->request->rq_disk;
	if (!disk)
		goto done;
	sdkp = scsi_disk(disk);
	if (!sdkp)
		goto done;
	if (sector + 1 != sdkp->capacity)
		goto done;

	if (srb->result == SAM_STAT_GOOD && scsi_get_resid(srb) == 0) {
		us->use_last_sector_hacks = 0;
	} else {
		/* The command failed. Allow upto 3 retries in case this is some
		   normal sort of failure. After that, assume the capacity is
		   wrong and we're trying to access the sector beyone the end.
		   Replace the result code and sense data with values that will
		   cause the SCSI core to fail the command immediately, instead
		   of going into an infinite retry loop.
		 */
		if (++us->last_sector_retries < 3)
			return;
		srb->result = SAM_STAT_CHECK_CONDITION;
		memcpy(srb->sense_buffer, record_not_found, 
				sizeof(record_not_found));
	}
done:
	/* Don't reset the retry counter for TEST UNIT READY commands,
	   because they get issued after device resets which mights be caused
	   by a failed last-sector access
	 */
	if (srb->cmnd[0] != TEST_UNIT_READY)
		us->last_sector_retries = 0;
}


void usb_stor_invoke_transport(struct scsi_cmnd *srb, struct us_data *us)
{
	int need_auto_sense;
	int result;

	/* Send the command to the transport layer */
	scsi_set_resid(srb, 0);
	result = us->transport(srb, us);

	/* If the command gets aborted by the higher layers, we need to
	 * short-circut all other processing
	 */
	if (test_bit(US_FLIDX_TIMED_OUT, &us->dflags)) {
		usb_stor_dbg(us, "-- command was aborted\b");
		srb->result = DID_ABORT << 16;
		goto Handle_Errors;
	}
	/* If there is a transport error, reset and don't auto-sense */
	if (result == USB_STOR_TRANSPORT_ERROR) {
		usb_stor_dbg(us, "--Transport indicates error, resetting \n");
		srb->result = DID_ERROR << 16;
		goto Handle_Errors;
	}

	/* If the transport provided its own sense data, don't auto-sense */
	if (result == USB_STOR_TRANSPORT_NO_SENSE) {
		srb->result = SAM_STAT_CHECK_CONDITION;
		last_sector_hacks(us, srb);
	}

	srb->result = SAM_STAT_GOOD;
	/* Determine if we need to autosense*/
	need_auto_sense = 0;

	if ((us->protocol == USB_PR_CB || us->protocol == USB_PR_DPCM_USB) &&
			srb->sc_data_direction != DMA_FROM_DEVICE) {
		usb_stor_dbg(us, "--CB transport device requiring auto-sense\n");
		need_auto_sense = 1;
	}

	/* if We have a failure we're going to do a REQUEST_SENSE automatically.
	   Note that we differentiate b/w a command "failure" and "error" in
	   the transport mechanism" */
	if (result == USB_STOR_TRANSPORT_FAILED) {
		usb_stor_dbg(us, "-- transport indicates command failure\n");
		need_auto_sense = 1;
	}

	/* Determine if this devcie is SAT by seeing if the command executed
	   successfully. Otherwise we'll have to wait for atleast one
	   CHECK_CONDITION to determine SANE_SENSE support */
	if (unlikely((srb->cmnd[0] == ATA_16 || srb->cmnd[0] == ATA_12) && 
				result == USB_STOR_TRANSPORT_GOOD &&
				!(us->fflags & US_FL_SANE_SENSE) && 
				!(us->fflags & US_FL_BAD_SENSE) &&
				!(srb->cmnd[2] & 0x20))) {
		usb_stor_dbg(us, "--SAT supported, increasing auto-sens\n");
		us->fflags |= US_FL_SANE_SENSE;
	}
	/* A short transfer on a command where we don't expect it is unusual,
	   but it doesn't mean we need to auto-sense */
	if ((scsi_get_resid(srb) > 0) && !((srb->cmnd[0] == REQUEST_SENSE) ||
		(srb->cmnd[0] == INQUIRY) || (srb->cmnd[0] == MODE_SENSE) ||
		(srb->cmnd[0] == LOG_SENSE) || (srb->cmnd[0] == MODE_SENSE_10)))
	{
		usb_stor_dbg(us, "--unexpectedly short transfer\n");
	}
	/* Now if we need to do the auto sense, let's do it */
	if (need_auto_sense)
	{
		int temp_result;
		struct scsi_eh_save ses;
		int sense_size = US_SENSE_SIZE;
		struct scsi_sense_hdr sshdr;
		const u8 *scdd;
		u8 fm_ili;

		/* Device supports and need bigger sense buffer */
		if (us->fflags & US_FL_SANE_SENSE)
			sense_size = ~0;
Retry_Sense:
		usb_stor_dbg(us, "Issuing auto-REQUEST_SENSE\n");
		scsi_eh_prep_cmnd(srb, &ses, NULL, 0, sense_size);
		/* FIXME: we must do the protocol translation here */
		if (us->subclass == USB_SC_RBC || us->subclass == USB_SC_SCSI ||
				us->subclass == USB_SC_CYP_ATACB)
			srb->cmd_len = 6;
		else
			srb->cmd_len = 12;

		/* Issue the auto-sense command */
		scsi_set_resid(srb, 0);
		temp_result = us->transport(us->srb, us);

		/* Let's clean up right away */
		scsi_eh_restore_cmnd(srb, &ses);

		if (test_bit(US_FLIDX_TIMED_OUT, &us->dflags)) {
			usb_stor_dbg(us, "--auto-sense aborted\n");
			srb->result = DID_ABORT << 16;
			/* If SANE_SENSE caused this problem, diable it */
			if (sense_size != US_SENSE_SIZE) {
				us->fflags &= ~US_FL_SANE_SENSE;
				us->fflags |= US_FL_BAD_SENSE;
			}
			goto Handle_Errors;
		}
		/* Some devcies claim to support larger sense but fail when
		   trying to request it. When a transport failure happens using
		   US_FS_SANE_SENSE, we always retry with a standard sense
		   request.. This fixes some USB GSM modems */
		if (temp_result == USB_STOR_TRANSPORT_FAILED && 
				sense_size != US_SENSE_SIZE) {
			usb_stor_dbg(us, "--auto-sense failure, retry small sense \n");
			sense_size = US_SENSE_SIZE;
			us->fflags &= ~US_FL_SANE_SENSE;
			us->fflags |= US_FL_BAD_SENSE;
			goto Retry_Sense;
		}

		/* Other failures */
		if (temp_result != USB_STOR_TRANSPORT_GOOD) {
			usb_stor_dbg(us, "--auto-sense failure\n");
			/* We skip the reset if this happens to be a multi-target
			 * device, since failure of autosense is perfectly valid
			 */
			srb->result = DID_ERROR << 16;
			if (!(us->fflags & US_FL_SCM_MULT_TARG))
				goto Handle_Errors;
			return;
		}

		/* If the sense data returned is larger than 18 bytes then we
		   assume this device supports requesting more in the future.
		   The response code must be 70h through 73h inclusive */
		if (srb->sense_buffer[7] > (US_SENSE_SIZE - 8) &&
				!(us->fflags & US_FL_SANE_SENSE) &&
				!(us->fflags & US_FL_BAD_SENSE) &&
				(srb->sense_buffer[0] & 0x7C) == 0x70) {
			usb_stor_dbg(us,"--Sense data truncated to %i from %i\n",
					US_SENSE_SIZE, srb->sense_buffer[7] + 8);
			srb->sense_buffer[7] = (US_SENSE_SIZE - 8);
		}
		scsi_normalize_sense(srb->sense_buffer, SCSI_SENSE_BUFFERSIZE,
				&sshdr);
		usb_stor_dbg(us, "--Result from auto-sense is %d\n",temp_result);
		usb_stor_dbg(us,"--code: 0x%x, key: 0x%x, ASC: 0x%x, ASCQ: 0x%x",
				sshdr.response_code, sshdr.sense_key,
				sshdr.asc, sshdr.ascq);
#ifdef CONFIG_USB_STORAGE_DEBUG
		usb_stor_show_sense(us, sshdr.sense_key, sshdr.asc, sshdr.ascq);
#endif
		/* Set the result so the higher layers expect this data */
		srb->result = SAM_STAT_CHECK_CONDITION;
		scdd = scsi_sense_desc_find(srb->sense_buffer,
				SCSI_SENSE_BUFFERSIZE, 4);
		fm_ili = (scdd ? scdd[3] : srb->sense_buffer[2]) & 0xA0;
		/* We often get empty sense data. This could indicate that
		   everything worked or that was an unspecified problem
		   We have to decide which */
		if (sshdr.sense_key == 0 && sshdr.asc == 0 && sshdr.ascq == 0 &&
				fm_ili == 0) {
			if (result == USB_STOR_TRANSPORT_GOOD) {
				srb->result = SAM_STAT_GOOD;
				srb->sense_buffer[0] = 0x0;
				/* If there was a problem, report an unspecified hardware
				   error to prevent the higher layers from entering an
				   infinite retry loop.
				 */
			} else {
				srb->result = DID_ERROR << 16;
				if ((sshdr.response_code & 0x72) == 0x72)
					srb->sense_buffer[1] = HARDWARE_ERROR;
				else
					srb->sense_buffer[2] = HARDWARE_ERROR;
			}
		}
	}

	/* Some devices don't work or return incorrect data the first time they
	   get a READ(10) command, or for the first READ(10) after a media change
	   If the INITIAL_READ10 flag is set, keep track of whether READ(10)
	   commands succeed. If the previous one succeded and this one failed,
	   set the REDO_READ10 flag to force a retry
	 */
	if (unlikely((us->fflags & US_FL_INITIAL_READ10) &&
				srb->cmnd[0] == READ_10)) 
	{
		if (srb->result == SAM_STAT_GOOD) {
			set_bit(US_FLIDX_READ10_WORKED, &us->dflags);
		} else if (test_bit(US_FLIDX_READ10_WORKED, &us->dflags)) {
			clear_bit(US_FLIDX_READ10_WORKED, &us->dflags);
			set_bit(US_FLIDX_REDO_READ10, &us->dflags);
		}
		/* Next if the REDO_READ10 flag is set, return a result code
		   that will cause the SCSI core to retry the READ(10) 
		   command immediately
		 */
		if (test_bit(US_FLIDX_REDO_READ10, &us->dflags))
		{
			clear_bit(US_FLIDX_REDO_READ10, &us->dflags);
			srb->result = DID_IMM_RETRY << 16;
			srb->sense_buffer[0] = 0;
		}
	}
	/* Did we transfer less than the minimum amount required */
	if ((srb->result == SAM_STAT_GOOD || srb->sense_buffer[2] == 0) &&
			scsi_bufflen(srb) - scsi_get_resid(srb) < srb->underflow)
		srb->result = DID_ERROR << 16;
	last_sector_hacks(us, srb);
	return;
	/* Error and abort processing: try to resynchronize with the device by
	   issuing a port reset. If that fails, try a class-specifi
	   device reset
	 */
Handle_Errors:
	/* Set the RESETTING bit, and clear the ABORTING bit so that the reset
	   may proceed */
	scsi_lock(us_to_host(us));
	set_bit(US_FLIDX_RESETTING, &us->dflags);
	clear_bit(US_FLIDX_ABORTING, &us->dflags);
	scsi_unlock(us_to_host(us));

	/* We must release the device lock because the pre_reset routine will
	   want to acquire it */
	mutex_unlock(&us->dev_mutex);
	result = usb_stor_port_reset(us);
	mutex_lock(&us->dev_mutex);

	if (result < 0)
	{
		scsi_lock(us_to_host(us));
		usb_stor_report_device_reset(us);
		scsi_unlock(us_to_host(us));
		us->transport_reset(us);
	}
	clear_bit(US_FLIDX_RESETTING, &us->dflags);
	last_sector_hacks(us, srb);
}

/* This is a version of usb_clear_halt() that allows early termination and
   doesn't read the status from the device -- this is because some devices
   crash their internal firmware when the status is requested after a halt.
 */
int usb_stor_clear_halt(struct us_data *us, unsigned int pipe)
{
	int result;
	int endp = usb_pipeendpoint(pipe);

	if (usb_pipein (pipe))
		endp |= USB_DIR_IN;

	result = usb_stor_control_msg(us, us->send_ctrl_pipe,
			USB_REQ_CLEAR_FEATURE, USB_RECIP_ENDPOINT,
			USB_ENDPOINT_HALT, endp, NULL, 0, 3*HZ);
	if (result >= 0)
		usb_reset_endpoint(us->pusb_dev, endp);

	usb_stor_dbg(us, "result = %d\n", result);
	return result;
}


/****************************************************************
 *	Reset routines
 ****************************************************************/
/* This is the common part of the device reset code
 * It's handy that every transport mechanism uses the control endpoint for reset
 * Basically, we send a reset with a 5 second timeout, so we don't get jammed
 * attempting to do the reset
 */
static int usb_stor_reset_common(struct us_data *us, u8 request, u8 requesttype,
		u16 value, u16 index, void *data, u16 size)
{
	int result;
	int result2;

	if (test_bit(US_FLIDX_DISCONNECTING, &us->dflags)) {
		usb_stor_dbg(us, "No reset during disconnect\n");
		return -EIO;
	}

	result = usb_stor_control_msg(us, us->send_ctrl_pipe, request,
			requesttype, value, index, data, size, 5*HZ);
	if (result < 0) {
		usb_stor_dbg(us, "Soft reset failed: %d\n", result);
		return result;
	}

	/* Give the device some time to recover from the reset, but don't
	   delay didconnect processing */
	wait_event_interruptible_timeout(us->delay_wait,
			test_bit(US_FLIDX_DISCONNECTING, &us->dflags), HZ*6);
	if (test_bit(US_FLIDX_DISCONNECTING, &us->dflags)) {
		usb_stor_dbg(us, "Reset interrupted by disconnect\n");
		return -EIO;
	}

	usb_stor_dbg(us, "Soft reset: clearing bulk-in endpoint halt \n");
	result = usb_stor_clear_halt(us, us->recv_bulk_pipe);

	usb_stor_dbg(us, "Soft reset: Clearing bulk-out endPoint half \n");
	result2 = usb_stor_clear_halt(us, us->send_bulk_pipe);

	if (result >= 0)
		result = result2;
	if (result < 0)
		usb_stor_dbg(us, "Soft reset failed\n");
	else
		usb_stor_dbg(us, "Soft reset done \n");
	return result;
}

int usb_stor_Bulk_reset(struct us_data *us)
{
	return usb_stor_reset_common(us, US_BULK_RESET_REQUEST,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			0, us->ifnum, NULL, 0);
}

/* Interpret the results of a USB transfer
 *
 * This function printd appropriate debugging messages, clears halts on non-control
 * endpoints, and translates the status to the corresponding USB_STOR_XFER_xxx return code
 */
static int interpret_urb_result(struct us_data *us, unsigned int pipe,
		unsigned int length, int result, unsigned int partial)
{
	usb_stor_dbg(us, "Status code %d; transferred %u/%u\n", 
			result, partial, length);
	switch(result)
	{
		/* no error code. Did we send all the data ? */
		case 0:
			if (partial != length) {
				usb_stor_dbg(us, "-- short transfer\n");
				return USB_STOR_XFER_SHORT;
			}
			usb_stor_dbg(us, "-- transfer complete\n");
			return USB_STOR_XFER_GOOD;

			/* Stalled */
		case -EPIPE:
			if (usb_pipecontrol(pipe)) {
				usb_stor_dbg(us, "-- stall on control pipe\n");
				return USB_STOR_XFER_STALLED;
			}
			/* For other sort of endpoint, clear the stall */
			usb_stor_dbg(us, "Clearing endpoint halt for pipe 0x%x\n", pipe);
			if (usb_stor_clear_halt(us, pipe) > 0)
				return USB_STOR_XFER_ERROR;
			return USB_STOR_XFER_STALLED;

			/* The device tried to send more than we wanted to read */
		case -EOVERFLOW:
			usb_stor_dbg(us, "--babble\n");
			return USB_STOR_XFER_LONG;

			/* The transfer was cancelled by abort, 
			   disconnect or timeout */
		case -ECONNRESET:
			usb_stor_dbg(us, "-- Transfer cancelled\n");
			return USB_STOR_XFER_ERROR;

			/* Short scatter-gather read transfer */
		case -EREMOTEIO:
			usb_stor_dbg(us, "--Short read transfer\n");
			return USB_STOR_XFER_SHORT;

			/* Abort or disconnect in progress */
		case -EIO:
			usb_stor_dbg(us, "--Abort or disconnect in progress\n");
			return USB_STOR_XFER_ERROR;

			/* Catch all error case */
		default:
			usb_stor_dbg(us, "--Unknown errors\n");
			return USB_STOR_XFER_ERROR;
	}
}


/* This is the common part of the URB message submission code

   All URBs from the usb-storage driver involved in handling a queued scsi
   command must pass through this function for the abort mechanisms
   to work properly
 */
int usb_stor_bulk_transfer_buf(struct us_data *us, unsigned int pipe,
		void *buf, unsigned int length, unsigned int *act_len)
{
	int result;

	usb_stor_dbg(us, "xfer %u bytes\n", length);

	/* Fill and submit the URB */
	usb_fill_bulk_urb(us->current_urb, us->pusb_dev, pipe, buf, length,
			usb_stor_blocking_completion, NULL);
	result = usb_stor_msg_common(us, 0);

	/* Store the actual length of the data transferred */
	if (act_len)
		*act_len = us->current_urb->actual_length;
	return interpret_urb_result(us, pipe, length, result,
			us->current_urb->actual_length);
}

/* Transfer a scatter-gather list via bulk transfer
   This function does basically the same thing as usb_stor_bulk_transfer_buf()
   above, but it uses the usbcore scater-gather library */
static int usb_stor_bulk_transfer_sglist(struct us_data *us, unsigned int pipe,
		struct scatterlist *sg, int num_sg, unsigned int length,
		unsigned int *act_len)
{
	int result;

	/* Don't submit s-g request during abort processing */
	if (test_bit(US_FLIDX_ABORTING, &us->dflags))
		return USB_STOR_XFER_ERROR;
	/* Initialize the scatter-gather request block */
	usb_stor_dbg(us, "xfer %u bytes, %d entries\n", length, num_sg);
	result = usb_sg_init(&us->current_sg, us->pusb_dev, pipe, 0,
			sg, num_sg, length, GFP_NOIO);
	if (result) {
		usb_stor_dbg(us, "usb_sg_init returned %d\n", result);
		return USB_STOR_XFER_ERROR;
	}

	/* Since the block has been initialized successfully, it's now 
	   okay to cancel it */
	set_bit(US_FLIDX_SG_ACTIVE, &us->dflags);

	/* Did an abort occur during the submission? */
	if (test_bit(US_FLIDX_ABORTING, &us->dflags)) {
		/* Cancel the request, if it hasn't been cancelled already */
		if (test_and_clear_bit(US_FLIDX_SG_ACTIVE, &us->dflags)) {
			usb_stor_dbg(us, "-- cancelling sg request \n");
			usb_sg_cancel(&us->current_sg);
		}
	}
	/* Wait for the completion of the transfer */
	usb_sg_wait(&us->current_sg);
	clear_bit(US_FLIDX_SG_ACTIVE, &us->dflags);
	result = us->current_sg.status;
	if (act_len)
		*act_len = us->current_sg.bytes;
	return interpret_urb_result(us, pipe, length, result,
			us->current_sg.bytes);
}

/*
 * Common used function. Transfer a complete command
 * via usb_stor_bulk_transfer_sglist() above. Set cmnd resid
 */
int usb_stor_bulk_srb(struct us_data* us, unsigned int pipe,
		struct scsi_cmnd* srb) 
{
	unsigned int partial;
	int result = usb_stor_bulk_transfer_sglist(us, pipe, scsi_sglist(srb),
			scsi_sg_count(srb), scsi_bufflen(srb),
			&partial);

	scsi_set_resid(srb, scsi_bufflen(srb) - partial);
	return result;
}

int usb_stor_Bulk_transport(struct scsi_cmnd *srb, struct us_data *us)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct bulk_cs_wrap *bcs = (struct bulk_cs_wrap *) us->iobuf;
	unsigned int transfer_length = scsi_bufflen(srb);
	unsigned int residue;
	int result;
	int fake_sense = 0; 
	unsigned int cswlen;
	unsigned int cbwlen = US_BULK_CB_WRAP_LEN;

	/* Take care of BULK32 devices; set extra byte to 0 */
	if (unlikely(us->fflags & US_FL_BULK32)) {
		cbwlen = 32;
		us->iobuf[31] = 0; 
	}

	/* set up the command wrapper */
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = cpu_to_le32(transfer_length);
	bcb->Flags = srb->sc_data_direction == DMA_FROM_DEVICE ?
		US_BULK_FLAG_IN : 0; 
	bcb->Tag = ++us->tag;
	bcb->Lun = srb->device->lun;
	if (us->fflags & US_FL_SCM_MULT_TARG)
		bcb->Lun |= srb->device->id << 4;
	bcb->Length = srb->cmd_len;

	/* copy the command payload */
	memset(bcb->CDB, 0, sizeof(bcb->CDB));
	memcpy(bcb->CDB, srb->cmnd, bcb->Length);

	/* Send it to out endpoint */
	usb_stor_dbg(us, "Bulk command S 0x%x T 0x%x L %d F %d Trg %d LUN %d CL %d\n", 
			le32_to_cpu(bcb->Signature), bcb->Tag, 
			le32_to_cpu(bcb->DataTransferLength), bcb->Flags,
			(bcb->Lun >> 4), (bcb->Lun & 0x0F), bcb->Length);

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe,
			bcb, cbwlen, NULL);
	usb_stor_dbg(us, "Bulk command transfer result=%d\n", result);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;
	/* Data stage */
	/* Send / Receive data payload, if there is any */
	if (unlikely(us->fflags & US_FL_GO_SLOW))
		udelay(125);

	if (transfer_length) {
		unsigned int pipe = srb->sc_data_direction == DMA_FROM_DEVICE ?
			us->recv_bulk_pipe : us->send_bulk_pipe;
		result = usb_stor_bulk_srb(us, pipe, srb);
		usb_stor_dbg(us, "Bulk data transfer result 0x%x\n", result);
		if (result == USB_STOR_TRANSPORT_ERROR)
			return USB_STOR_TRANSPORT_ERROR;
		/* If the device tried to send back more data than the amount
		   requested, the spec requires us to transfer CSW anyway. Since
		   there's no point retrying the command, we'll return fake
		   sense data indicating Illegal request,Invalid field in CDB
		 */
		if (result == USB_STOR_XFER_LONG)
			fake_sense = 1;
		/* Some times device will mistakenly skip the data phase and go
		   directly to the status phase without sending a zero length
		   packet. If we get a 13-byte response here, check whether it
		   really is a CSW
		 */
		if (result == USB_STOR_XFER_SHORT &&
				srb->sc_data_direction == DMA_FROM_DEVICE &&
				transfer_length - scsi_get_resid(srb) ==
				US_BULK_CS_WRAP_LEN) 
		{
			struct scatterlist *sg = NULL;
			unsigned int offset = 0;

			if (usb_stor_access_xfer_buf((unsigned char *)bcs,
						US_BULK_CS_WRAP_LEN, srb, &sg,
						&offset, FROM_XFER_BUF) ==
					US_BULK_CS_WRAP_LEN && bcs->Signature ==
					cpu_to_le32(US_BULK_CS_SIGN))
			{
				usb_stor_dbg(us, "Device skipped data phase\n");
				scsi_set_resid(srb, transfer_length);
				goto skipped_data_phase;
			}
		}
	}

	/* See flow chart on pg 15 of the BUlk only Transport spec for an
	   explanation ofhow this code works */
	/* Get CSW for device status */
	usb_stor_dbg(us, "Attempting to get CSW..\n");
	result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
			bcs, US_BULK_CS_WRAP_LEN, &cswlen);

	/* Some broken devices add necessary zero-length packets to the end of
	   their data transfers. Such packets show up as 0-length CSWs. If we
	   encounter such a thing, try to read CSW again.
	 */
	if (result == USB_STOR_XFER_SHORT && cswlen == 0)
	{
		usb_stor_dbg(us, "Received 0-length CSW; retrying...\n");
		result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
				bcs, US_BULK_CS_WRAP_LEN, &cswlen);
	}

	/* Attempt to read the CSW fail ? */
	if (result == USB_STOR_XFER_STALLED)
	{
		usb_stor_dbg(us, "Attempting to get CSW 2nd try...\n");
		result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
				bcs, US_BULK_CS_WRAP_LEN, NULL);
	}

	/* If we still have a failure at this point, we're in trouble */
	usb_stor_dbg(us, "Bulk status result = %d\n", result);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;    

skipped_data_phase:
	/* Check bulk status */
	residue = le32_to_cpu(bcs->Residue);
	usb_stor_dbg(us, "Bulk status S 0x%x T 0x%x R %u Stat 0x%x\n",
			le32_to_cpu(bcs->Signature), bcs->Tag, residue,
			bcs->Status);
	if (!(bcs->Tag == us->tag || (us->fflags & US_FL_BULK_IGNORE_TAG)) ||
			bcs->Status > US_BULK_STAT_PHASE)
	{
		usb_stor_dbg(us, "Bulk logical error\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* Some broken devices report odd signature, so we do not check them
	   for validity against the spec. We stor the first one we see, and 
	   check subsequent transfer for validity against this signature
	 */
	if (!us->bcs_signature) {
		us->bcs_signature = bcs->Signature;
		if (us->bcs_signature != cpu_to_le32(US_BULK_CS_SIGN))
			usb_stor_dbg(us, "Learnt BCS signature 0x%08X\n",
					le32_to_cpu(us->bcs_signature));
	} else if (bcs->Signature != us->bcs_signature) {
		usb_stor_dbg(us,"Signature mismatch: got %08X, expecting %08X\n",
				le32_to_cpu(bcs->Signature),
				le32_to_cpu(us->bcs_signature));
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* Try to compute the actual residue, based on how much data was
	   really transferred and what the devices tells us */
	if (residue && !(us->fflags & US_FL_IGNORE_RESIDUE)) {
		/* Heuristically detect devices that generate bogus residues
		   by seeing what happens with INQUIRY and READ CAPACITY
		   commands */
		if (bcs->Status == US_BULK_STAT_OK && scsi_get_resid(srb) == 0
			&& ((srb->cmnd[0] == INQUIRY && transfer_length == 36) ||
			(srb->cmnd[0] == READ_CAPACITY && transfer_length == 8)))
		{
			us->fflags |= US_FL_IGNORE_RESIDUE;
		} else {
			residue = min(residue, transfer_length);
			scsi_set_resid(srb, max(scsi_get_resid(srb),
						(int) residue ));
		}
	}

	/* Based on the status code we report good or bad */
	switch(bcs->Status) {
		case US_BULK_STAT_OK:
			/* Device babbled return fake sense data */
			if (fake_sense) {
				memcpy(srb->sense_buffer,
						usb_stor_sense_invalidCDB,
						sizeof(usb_stor_sense_invalidCDB));
				return USB_STOR_TRANSPORT_NO_SENSE;
			}
			/* Command good -- note that data could be short */
			return USB_STOR_TRANSPORT_GOOD;
		case US_BULK_STAT_FAIL:
			/* Command failed */
			return USB_STOR_TRANSPORT_FAILED;
		case US_BULK_STAT_PHASE:
			return USB_STOR_TRANSPORT_ERROR;
	}
	return USB_STOR_TRANSPORT_ERROR;
}

/*Issue a USB port reset to the device. The caller must not hold us->dev_mutex*/
int usb_stor_port_reset(struct us_data *us)
{
	int result;

	/* For these devices we must use the class specific method */
	if (us->pusb_dev->quirks & USB_QUIRK_RESET)
		return -EPERM;

	result = usb_lock_device_for_reset(us->pusb_dev, us->pusb_intf);
	if (result < 0)
		usb_stor_dbg(us, "Unable to lock the device for reset: %d\n",
				result);
	else {
		/* Were we disconnected while waiting for the lock */
		if (test_bit(US_FLIDX_DISCONNECTING, &us->dflags)) {
			result = -EIO;
			usb_stor_dbg(us, "No reset during disconnect \n");
		} else {
			result = usb_reset_device(us->pusb_dev);
			usb_stor_dbg(us,"usb_reset_device returns %d\n",result);
		}
		usb_unlock_device(us->pusb_dev);
	}
	return result;
}

/* Stop the current URB transfer */
void usb_stor_stop_transport(struct us_data *us)
{
	if (test_and_clear_bit(US_FLIDX_URB_ACTIVE, &us->dflags)) {
		usb_unlink_urb(us->current_urb);
		usb_stor_dbg(us, "--Cancelling URB\n");
	}

	if (test_and_clear_bit(US_FLIDX_SG_ACTIVE, &us->dflags)) {
		usb_stor_dbg(us, "--Cancelling sg request\n");
		usb_sg_cancel(&us->current_sg);
	}
}

/* Invoke the transport and basic error-handling/recovery  methds
   This is used by the protocol layer to actually send the message to
   the device and receive the response
 */
