#include <linux/highmem.h>
#include <linux/export.h>
#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>

#include "usb.h"
#include "protocol.h"
#include "transport.h"


void usb_stor_transparent_scsi_command(struct scsi_cmnd *srb, struct us_data *us)
{
	/* Send the command to the transport layer */
	usb_stor_invoke_transport(srb, us);
}

/* Scatter-gather transfer buffer access routines */
/* Copy a buffer of length buflen to/from the srb's transfer buffer. Update
   the **sgptr and *offset variables so that the next copy will pickup from
   where this one left off */
unsigned int usb_stor_access_xfer_buf(unsigned char *buffer,
	unsigned int buflen, struct scsi_cmnd *srb, struct scatterlist **sgptr,
	unsigned int *offset, enum xfer_buf_dir dir)
{
	unsigned int cnt = 0;
	struct scatterlist *sg = *sgptr;
	struct sg_mapping_iter miter;
	unsigned int nents = scsi_sg_count(srb);

	if (sg)
		nents = sg_nents(sg);
	else
		sg = scsi_sglist(srb);

	sg_miter_start(&miter, sg, nents, dir == FROM_XFER_BUF ?
			SG_MITER_FROM_SG : SG_MITER_TO_SG);

	if (!sg_miter_skip(&miter, *offset))
		return cnt;

	while(sg_miter_next(&miter) && cnt < buflen) {
		unsigned int len = min_t(unsigned int, miter.length,
				buflen - cnt);
		if (dir == FROM_XFER_BUF)
			memcpy(buffer + cnt, miter.addr, len);
		else
			memcpy(miter.addr, buffer + cnt, len);
		if (*offset + len < miter.piter.sg->length) {
			*offset += len;
			*sgptr = miter.piter.sg;
		} else {
			*offset = 0;
			*sgptr = sg_next(miter.piter.sg);
		}
		cnt += len;
	}
	sg_miter_stop(&miter);
	return cnt;
}
