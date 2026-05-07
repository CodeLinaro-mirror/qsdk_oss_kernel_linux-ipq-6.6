// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mhi.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include "internal.h"

/**
 * struct mhi_shared_ro - Shared read-only firmware segments
 * @mhi_bufs: Array of RO segment DMA buffers
 * @num_segments: Number of RO segments being shared
 * @refcount: Reference count for tracking endpoint usage
 * @lock: Protects access to this structure during refcount operations
 *
 * When multiple endpoints use the same firmware, RO segments can be
 * shared to reduce memory usage. This structure tracks the shared
 * segments and ensures proper cleanup via refcounting.
 */
struct mhi_shared_ro {
	struct mhi_buf *mhi_bufs;
	u32 num_segments;
	refcount_t refcount;
	struct mutex lock;	/* Protects structure during refcount ops */
};

/* Global shared RO segments - protected by mhi_shared_ro_lock */
static DEFINE_MUTEX(mhi_shared_ro_lock);
static struct mhi_shared_ro *mhi_global_shared_ro;

/* Setup RDDM vector table for RDDM transfer and program RXVEC */
int mhi_rddm_prepare(struct mhi_controller *mhi_cntrl,
		     struct image_info *img_info)
{
	struct mhi_buf *mhi_buf = img_info->mhi_buf;
	struct bhi_vec_entry *bhi_vec = img_info->bhi_vec;
	void __iomem *base = mhi_cntrl->bhie;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	u32 sequence_id;
	unsigned int i;
	int ret;

	for (i = 0; i < img_info->entries - 1; i++, mhi_buf++, bhi_vec++) {
		bhi_vec->dma_addr = cpu_to_le64(mhi_buf->dma_addr);
		bhi_vec->size = cpu_to_le64(mhi_buf->len);
	}

	if (!mhi_cntrl->rddm_prealloc) {
		mhi_buf->dma_addr = dma_map_single(mhi_cntrl->cntrl_dev,
						   mhi_buf->buf, mhi_buf->len,
						   DMA_TO_DEVICE);
		if (dma_mapping_error(mhi_cntrl->cntrl_dev, mhi_buf->dma_addr)) {
			dev_err(dev, "dma mapping failed, Address: %p and len: 0x%zx\n",
				&mhi_buf->dma_addr, mhi_buf->len);
			return -ENOMEM;
		}
	}

	dev_dbg(dev, "BHIe programming for RDDM\n");

	mhi_write_reg(mhi_cntrl, base, BHIE_RXVECADDR_HIGH_OFFS,
		      upper_32_bits(mhi_buf->dma_addr));

	mhi_write_reg(mhi_cntrl, base, BHIE_RXVECADDR_LOW_OFFS,
		      lower_32_bits(mhi_buf->dma_addr));

	mhi_write_reg(mhi_cntrl, base, BHIE_RXVECSIZE_OFFS, mhi_buf->len);
	sequence_id = MHI_RANDOM_U32_NONZERO(BHIE_RXVECSTATUS_SEQNUM_BMSK);

	ret = mhi_write_reg_field(mhi_cntrl, base, BHIE_RXVECDB_OFFS,
				  BHIE_RXVECDB_SEQNUM_BMSK, sequence_id);
	if (ret) {
		dev_err(dev, "Failed to write sequence ID for BHIE_RXVECDB\n");
		return ret;
	}

	dev_dbg(dev, "Address: %p and len: 0x%zx sequence: %u\n",
		&mhi_buf->dma_addr, mhi_buf->len, sequence_id);

	return 0;
}

/* Collect RDDM buffer during kernel panic */
static int __mhi_download_rddm_in_panic(struct mhi_controller *mhi_cntrl)
{
	int ret;
	u32 rx_status;
	enum mhi_ee_type ee;
	const u32 delayus = 2000;
	const u32 soc_reset_delay_ms = 200;
	u32 retry = (mhi_cntrl->timeout_ms * 1000) / delayus;
	const u32 rddm_timeout_us = 400000;
	int rddm_retry = rddm_timeout_us / delayus;
	void __iomem *base = mhi_cntrl->bhie;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;

	dev_dbg(dev, "Entered with pm_state:%s dev_state:%s ee:%s\n",
		to_mhi_pm_state_str(mhi_cntrl->pm_state),
		mhi_state_str(mhi_cntrl->dev_state),
		TO_MHI_EXEC_STR(mhi_cntrl->ee));

	/*
	 * This should only be executing during a kernel panic, we expect all
	 * other cores to shutdown while we're collecting RDDM buffer. After
	 * returning from this function, we expect the device to reset.
	 *
	 * Normaly, we read/write pm_state only after grabbing the
	 * pm_lock, since we're in a panic, skipping it. Also there is no
	 * gurantee that this state change would take effect since
	 * we're setting it w/o grabbing pm_lock
	 */
	mhi_cntrl->pm_state = MHI_PM_LD_ERR_FATAL_DETECT;
	/* update should take the effect immediately */
	smp_wmb();

	/*
	 * Make sure device is not already in RDDM. In case the device asserts
	 * and a kernel panic follows, device will already be in RDDM.
	 * Do not trigger SYS ERR again and proceed with waiting for
	 * image download completion.
	 */
	ee = mhi_get_exec_env(mhi_cntrl);
	if (ee == MHI_EE_MAX)
		goto error_exit_rddm;

	if (ee != MHI_EE_RDDM) {
		dev_dbg(dev, "Trigger device into RDDM mode using SYS ERR\n");
		mhi_set_mhi_state(mhi_cntrl, MHI_STATE_SYS_ERR);

		dev_dbg(dev, "Waiting for device to enter RDDM\n");
		while (rddm_retry--) {
			ee = mhi_get_exec_env(mhi_cntrl);
			if (ee == MHI_EE_RDDM)
				break;

			udelay(delayus);
		}

		if (rddm_retry <= 0) {
			/* Hardware reset so force device to enter RDDM */
			dev_dbg(dev,
				"Did not enter RDDM, do a host req reset\n");
			mhi_debug_reg_dump(mhi_cntrl);
			mhi_soc_reset(mhi_cntrl);
			mdelay(soc_reset_delay_ms);
		}

		ee = mhi_get_exec_env(mhi_cntrl);
		/* If Target still did not swich to RDDM, dump debug registers and bail out */
		if (ee != MHI_EE_RDDM) {
			dev_err(dev, "Failed to switch to RDDM\n");
			mhi_debug_reg_dump(mhi_cntrl);
			return -EIO;
		}
	}

	dev_dbg(dev,
		"Waiting for RDDM image download via BHIe, current EE:%s\n",
		TO_MHI_EXEC_STR(ee));

	while (retry--) {
		ret = mhi_read_reg_field(mhi_cntrl, base, BHIE_RXVECSTATUS_OFFS,
					 BHIE_RXVECSTATUS_STATUS_BMSK, &rx_status);
		if (ret)
			return -EIO;

		if (rx_status == BHIE_RXVECSTATUS_STATUS_XFER_COMPL)
			return 0;

		udelay(delayus);
	}

	ee = mhi_get_exec_env(mhi_cntrl);
	ret = mhi_read_reg(mhi_cntrl, base, BHIE_RXVECSTATUS_OFFS, &rx_status);

	dev_err(dev, "RXVEC_STATUS: 0x%x\n", rx_status);

error_exit_rddm:
	dev_err(dev, "RDDM transfer failed. Current EE: %s\n",
		TO_MHI_EXEC_STR(ee));
	mhi_dump_errdbg_reg(mhi_cntrl);
	return -EIO;
}

/* Download RDDM image from device */
int mhi_download_rddm_image(struct mhi_controller *mhi_cntrl, bool in_panic)
{
	void __iomem *base = mhi_cntrl->bhie;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	rwlock_t *pm_lock = &mhi_cntrl->pm_lock;
	struct mhi_buf *mhi_buf = NULL;
	u32 rx_status;
	int ret;

	/*
	 * Allocate RDDM table if specified, this table is for debugging purpose
	 */
	if (!mhi_cntrl->rddm_prealloc && mhi_cntrl->rddm_size) {
		ret = mhi_alloc_bhie_table(mhi_cntrl, &mhi_cntrl->rddm_image,
					   mhi_cntrl->rddm_size, IMG_TYPE_RDDM);
		if (ret) {
			dev_err(dev, "Failed to allocate RDDM table memory\n");
			return ret;
		}

		/* setup the RX vector table */
		ret = mhi_rddm_prepare(mhi_cntrl, mhi_cntrl->rddm_image);
		if (ret) {
			dev_err(dev, "Failed to prepare RDDM\n");
			mhi_free_bhie_table(mhi_cntrl, mhi_cntrl->rddm_image,
					    IMG_TYPE_RDDM);
			return ret;
		}
	}

	if (in_panic) {
		ret = __mhi_download_rddm_in_panic(mhi_cntrl);
		goto out;
	}

	dev_dbg(dev, "Waiting for RDDM image download via BHIe\n");

	/* Wait for the image download to complete */
	wait_event_timeout(mhi_cntrl->state_event,
			   mhi_read_reg_field(mhi_cntrl, base,
					      BHIE_RXVECSTATUS_OFFS,
					      BHIE_RXVECSTATUS_STATUS_BMSK,
					      &rx_status) || rx_status,
			   msecs_to_jiffies(mhi_cntrl->timeout_ms));

	ret = (rx_status == BHIE_RXVECSTATUS_STATUS_XFER_COMPL) ? 0 : -EIO;

out:
	mhi_buf = &mhi_cntrl->rddm_image->mhi_buf[mhi_cntrl->rddm_image->entries - 1];

	if (!mhi_cntrl->rddm_prealloc)
		dma_unmap_single(mhi_cntrl->cntrl_dev, mhi_buf->dma_addr,
				 mhi_buf->len, DMA_TO_DEVICE);

	if (ret) {
		dev_err(dev, "RDDM transfer failed. RXVEC_STATUS: 0x%x\n",
			rx_status);
		read_lock_bh(pm_lock);
		if (MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state))
			mhi_dump_errdbg_reg(mhi_cntrl);
		read_unlock_bh(pm_lock);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(mhi_download_rddm_image);

static int mhi_fw_load_bhie(struct mhi_controller *mhi_cntrl,
			    const struct mhi_buf *mhi_buf)
{
	void __iomem *base = mhi_cntrl->bhie;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	rwlock_t *pm_lock = &mhi_cntrl->pm_lock;
	u32 tx_status, sequence_id;
	int ret;

	read_lock_bh(pm_lock);
	if (!MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
		read_unlock_bh(pm_lock);
		return -EIO;
	}

	sequence_id = MHI_RANDOM_U32_NONZERO(BHIE_TXVECSTATUS_SEQNUM_BMSK);
	dev_dbg(dev, "Starting image download via BHIe. Sequence ID: %u\n",
		sequence_id);
	mhi_write_reg(mhi_cntrl, base, BHIE_TXVECADDR_HIGH_OFFS,
		      upper_32_bits(mhi_buf->dma_addr));

	mhi_write_reg(mhi_cntrl, base, BHIE_TXVECADDR_LOW_OFFS,
		      lower_32_bits(mhi_buf->dma_addr));

	mhi_write_reg(mhi_cntrl, base, BHIE_TXVECSIZE_OFFS, mhi_buf->len);

	ret = mhi_write_reg_field(mhi_cntrl, base, BHIE_TXVECDB_OFFS,
				  BHIE_TXVECDB_SEQNUM_BMSK, sequence_id);
	read_unlock_bh(pm_lock);

	if (ret)
		return ret;

	/* Wait for the image download to complete */
	ret = wait_event_timeout(mhi_cntrl->state_event,
				 MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state) ||
				 mhi_read_reg_field(mhi_cntrl, base,
						   BHIE_TXVECSTATUS_OFFS,
						   BHIE_TXVECSTATUS_STATUS_BMSK,
						   &tx_status) || tx_status,
				 msecs_to_jiffies(mhi_cntrl->timeout_ms));
	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state) ||
	    tx_status != BHIE_TXVECSTATUS_STATUS_XFER_COMPL) {
		dev_err(dev, "Upper:0x%x Lower:0x%x len:0x%zx sequence:%u\n",
			upper_32_bits(mhi_buf->dma_addr),
			lower_32_bits(mhi_buf->dma_addr),
			mhi_buf->len, sequence_id);

		dev_err(dev, "MHI pm_state: %s tx_status: %d ee: %s\n",
			to_mhi_pm_state_str(mhi_cntrl->pm_state), tx_status,
			TO_MHI_EXEC_STR(mhi_get_exec_env(mhi_cntrl)));

		read_lock_bh(pm_lock);
		if (MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state))
			mhi_dump_errdbg_reg(mhi_cntrl);
		read_unlock_bh(pm_lock);
		return -EIO;
	}

	return (!ret) ? -ETIMEDOUT : 0;
}

static int mhi_fw_load_bhi(struct mhi_controller *mhi_cntrl,
			   dma_addr_t dma_addr,
			   size_t size)
{
	u32 tx_status, session_id;
	int ret;
	void __iomem *base = mhi_cntrl->bhi;
	rwlock_t *pm_lock = &mhi_cntrl->pm_lock;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;

	read_lock_bh(pm_lock);
	if (!MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
		read_unlock_bh(pm_lock);
		goto invalid_pm_state;
	}

	session_id = MHI_RANDOM_U32_NONZERO(BHI_TXDB_SEQNUM_BMSK);
	dev_dbg(dev, "Starting image download via BHI. Session ID: %u\n",
		session_id);
	mhi_write_reg(mhi_cntrl, base, BHI_STATUS, 0);
	mhi_write_reg(mhi_cntrl, base, BHI_IMGADDR_HIGH,
		      upper_32_bits(dma_addr));
	mhi_write_reg(mhi_cntrl, base, BHI_IMGADDR_LOW,
		      lower_32_bits(dma_addr));
	mhi_write_reg(mhi_cntrl, base, BHI_IMGSIZE, size);
	mhi_write_reg(mhi_cntrl, base, BHI_IMGTXDB, session_id);
	read_unlock_bh(pm_lock);

	/* Wait for the image download to complete */
	ret = wait_event_timeout(mhi_cntrl->state_event,
			   MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state) ||
			   mhi_read_reg_field(mhi_cntrl, base, BHI_STATUS,
					      BHI_STATUS_MASK, &tx_status) || tx_status,
			   msecs_to_jiffies(mhi_cntrl->timeout_ms));
	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state))
		goto invalid_pm_state;

	if (tx_status == BHI_STATUS_ERROR) {
		dev_err(dev, "Image transfer failed\n");
		read_lock_bh(pm_lock);
		if (MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state))
			mhi_dump_errdbg_reg(mhi_cntrl);
		read_unlock_bh(pm_lock);
		goto invalid_pm_state;
	}

	return (!ret) ? -ETIMEDOUT : 0;

invalid_pm_state:

	return -EIO;
}

void mhi_free_bhie_table(struct mhi_controller *mhi_cntrl,
			 struct image_info *image_info,
			 enum image_type img_type)
{
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	int i;
	struct mhi_buf *mhi_buf = image_info->mhi_buf;
	u32 num_ro_segments = 0;

	/* Handle shared RO cleanup for FBC images */
	if (img_type == IMG_TYPE_FBC && mhi_cntrl->shared_ro_segments) {
		struct mhi_shared_ro *shared_ro = mhi_cntrl->shared_ro_segments;

		num_ro_segments = shared_ro->num_segments;

		mutex_lock(&mhi_shared_ro_lock);
		if (refcount_dec_and_test(&shared_ro->refcount)) {
			/* Last EP: Free shared RO segments */
			dev_info(dev, "Last EP: Freeing %u shared RO segments\n",
				 num_ro_segments);

			/* Free the actual DMA memory for RO segments */
			for (i = 0; i < num_ro_segments; i++) {
				mhi_fw_free_coherent(mhi_cntrl,
						     shared_ro->mhi_bufs[i].len,
						     shared_ro->mhi_bufs[i].buf,
						     shared_ro->mhi_bufs[i].dma_addr);
			}

			kfree(shared_ro->mhi_bufs);
			kfree(shared_ro);
			mhi_global_shared_ro = NULL;
		} else {
			dev_info(dev, "EP removed: %u RO segments still shared (refcount=%d)\n",
				 num_ro_segments,
				 refcount_read(&shared_ro->refcount));
		}
		mhi_cntrl->shared_ro_segments = NULL;
		mutex_unlock(&mhi_shared_ro_lock);
	}

	/* Free per-EP segments (skip shared RO segments) */
	mhi_buf = &image_info->mhi_buf[num_ro_segments];
	for (i = num_ro_segments; i < image_info->entries; i++, mhi_buf++) {
		if (img_type == IMG_TYPE_RDDM && !mhi_cntrl->rddm_prealloc) {
			if (i == (image_info->entries - 1))
				dma_unmap_single(mhi_cntrl->cntrl_dev,
						 mhi_buf->dma_addr,
						 mhi_buf->len,
						 DMA_FROM_DEVICE);
			kfree(mhi_buf->buf);

		} else {
			mhi_fw_free_coherent(mhi_cntrl, mhi_buf->len,
					     mhi_buf->buf, mhi_buf->dma_addr);
		}
	}

	kfree(image_info->mhi_buf);
	kfree(image_info);
}

/**
 * mhi_alloc_bhie_table_partial - Allocate BHIE table with RO segment reuse
 * @mhi_cntrl: MHI controller
 * @image_info: Pointer to image_info pointer
 * @alloc_size: Total firmware size
 * @rw_size: Size of RW segments to allocate
 * @num_ro_segments: Number of RO segments to reuse
 * @shared_ro_bufs: Shared RO segment buffers to copy
 * @img_type: Image type
 *
 * Allocates BHIE table for subsequent EPs, reusing RO segments from first EP.
 * Only allocates RW segments, copies RO pointers from shared memory.
 */
static int mhi_alloc_bhie_table_partial(struct mhi_controller *mhi_cntrl,
					struct image_info **image_info,
					size_t alloc_size, size_t rw_size,
					u32 num_ro_segments,
					struct mhi_buf *shared_ro_bufs,
					enum image_type img_type)
{
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	size_t seg_size = mhi_cntrl->seg_len;
	int rw_segments, total_segments;
	int i;
	struct image_info *img_info;
	struct mhi_buf *mhi_buf;
	gfp_t gfp = GFP_KERNEL;
	size_t allocated_bytes = 0;

	rw_segments = DIV_ROUND_UP(rw_size, seg_size);
	total_segments = num_ro_segments + rw_segments + 1; /* +1 for vector table */

	dev_dbg(dev, "=== SUBSEQUENT EP: Partial Allocation ===\n");
	dev_dbg(dev, "  Total firmware size: %zu bytes\n", alloc_size);
	dev_dbg(dev, "  RO segments to reuse: %u (skipping %zu bytes)\n",
		num_ro_segments, num_ro_segments * seg_size);
	dev_dbg(dev, "  RW size to allocate: %zu bytes\n", rw_size);
	dev_dbg(dev, "  RW segments: %d\n", rw_segments);
	dev_dbg(dev, "  Total entries: %d (RO:%u + RW:%d + Vec:1)\n",
		total_segments, num_ro_segments, rw_segments);

	img_info = kzalloc(sizeof(*img_info), gfp);
	if (!img_info)
		return -ENOMEM;

	/* Allocate memory for ALL entries (RO + RW + vector) */
	img_info->mhi_buf = kcalloc(total_segments, sizeof(*img_info->mhi_buf), gfp);
	if (!img_info->mhi_buf)
		goto error_alloc_mhi_buf;

	/* Copy RO segment pointers from shared memory */
	memcpy(img_info->mhi_buf, shared_ro_bufs,
	       num_ro_segments * sizeof(struct mhi_buf));
	dev_dbg(dev, "  Copied %u RO segment pointers from shared memory\n",
		num_ro_segments);

	/* Allocate RW segments starting at index num_ro_segments */
	mhi_buf = &img_info->mhi_buf[num_ro_segments];
	for (i = num_ro_segments; i < total_segments; i++, mhi_buf++) {
		size_t vec_size = seg_size;

		/* Vector table is the last entry */
		if (i == total_segments - 1)
			vec_size = sizeof(struct bhi_vec_entry) * i;

		mhi_buf->len = vec_size;
		mhi_buf->buf = mhi_fw_alloc_coherent(mhi_cntrl, vec_size,
						     &mhi_buf->dma_addr,
						     GFP_KERNEL);
		if (!mhi_buf->buf) {
			dev_err(dev, "Failed to allocate RW segment[%d]\n", i);
			goto error_alloc_segment;
		}

		allocated_bytes += vec_size;
		dev_dbg(dev, "  Allocated RW segment[%d]: buf=%p dma=0x%llx len=%zu\n",
			i, mhi_buf->buf, (u64)mhi_buf->dma_addr, mhi_buf->len);
	}

	img_info->bhi_vec = img_info->mhi_buf[total_segments - 1].buf;
	img_info->entries = total_segments;
	*image_info = img_info;

	dev_dbg(dev, "Memory allocation summary:\n");
	dev_dbg(dev, "  RO segments: %u (SHARED, 0 bytes allocated)\n",
		num_ro_segments);
	dev_dbg(dev, "  RW segments: %d (%zu bytes allocated)\n",
		rw_segments + 1, allocated_bytes);
	dev_dbg(dev, "  Total entries: %d\n", total_segments);

	return 0;

error_alloc_segment:
	/* Free only the RW segments we allocated */
	for (--i, --mhi_buf; i >= (int)num_ro_segments; i--, mhi_buf--) {
		mhi_fw_free_coherent(mhi_cntrl, mhi_buf->len,
				     mhi_buf->buf, mhi_buf->dma_addr);
	}

	kfree(img_info->mhi_buf);

error_alloc_mhi_buf:
	kfree(img_info);

	return -ENOMEM;
}

int mhi_alloc_bhie_table(struct mhi_controller *mhi_cntrl,
			 struct image_info **image_info,
			 size_t alloc_size, enum image_type img_type)
{
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	size_t seg_size = mhi_cntrl->seg_len;
	int segments;
	int i;
	struct image_info *img_info;
	struct mhi_buf *mhi_buf;
	/* Maksed __GFP_DIRECT_RECLAIM flag for non-interrupt context
	 * to avoid rcu context sleep issue in kmalloc during panic scenario
	 */
	gfp_t gfp = (in_interrupt() ? GFP_ATOMIC :
		((GFP_KERNEL | __GFP_NORETRY) & ~__GFP_DIRECT_RECLAIM));
	size_t allocated_bytes = 0;

	if (img_type == IMG_TYPE_RDDM)
		seg_size = mhi_cntrl->rddm_seg_len;

	segments = DIV_ROUND_UP(alloc_size, seg_size) + 1;

	dev_dbg(dev, "=== FIRST EP: Full Allocation ===\n");
	dev_dbg(dev, "  Total firmware size: %zu bytes\n", alloc_size);
	dev_dbg(dev, "  Segments to allocate: %d\n", segments);

	img_info = kzalloc(sizeof(*img_info), gfp);
	if (!img_info)
		return -ENOMEM;

	/* Allocate memory for entries */
	img_info->mhi_buf = kcalloc(segments, sizeof(*img_info->mhi_buf),
				    gfp);
	if (!img_info->mhi_buf)
		goto error_alloc_mhi_buf;

	/* Allocate and populate vector table */
	mhi_buf = img_info->mhi_buf;
	for (i = 0; i < segments; i++, mhi_buf++) {
		size_t vec_size = seg_size;

		/* Vector table is the last entry */
		if (i == segments - 1)
			vec_size = sizeof(struct bhi_vec_entry) * i;

		mhi_buf->len = vec_size;

		if (img_type == IMG_TYPE_RDDM && !mhi_cntrl->rddm_prealloc) {
			/* Vector table is the last entry */
			if (i == segments - 1) {
				mhi_buf->buf = kzalloc(PAGE_ALIGN(vec_size),
						       gfp);
				if (!mhi_buf->buf)
					goto error_alloc_segment;

				/* Vector table entry will be dma_mapped during
				 * rddm prepare with DMA_TO_DEVICE and unmapped
				 * once the target completes the RDDM XFER.
				 */
				continue;
			}
			mhi_buf->buf = kmalloc(vec_size, gfp);
			if (!mhi_buf->buf)
				goto error_alloc_segment;

			mhi_buf->dma_addr = dma_map_single(mhi_cntrl->cntrl_dev,
							   mhi_buf->buf,
							   vec_size,
							   DMA_FROM_DEVICE);
			if (dma_mapping_error(mhi_cntrl->cntrl_dev,
					      mhi_buf->dma_addr)) {
				kfree(mhi_buf->buf);
				goto error_alloc_segment;
			}
		} else {
			mhi_buf->buf = mhi_fw_alloc_coherent(mhi_cntrl,
							     vec_size,
							     &mhi_buf->dma_addr,
							     GFP_KERNEL);
			if (!mhi_buf->buf)
				goto error_alloc_segment;
		}

		allocated_bytes += vec_size;
	}

	img_info->bhi_vec = img_info->mhi_buf[segments - 1].buf;
	img_info->entries = segments;
	*image_info = img_info;

	dev_dbg(dev, "Memory allocation summary:\n");
	dev_dbg(dev, "  All segments: %d (%zu bytes allocated)\n",
		segments - 1, allocated_bytes);
	dev_dbg(dev, "  Total entries: %d\n", segments);

	return 0;

error_alloc_segment:
	for (--i, --mhi_buf; i >= 0; i--, mhi_buf--) {
		if (img_type == IMG_TYPE_RDDM && !mhi_cntrl->rddm_prealloc) {
			dma_unmap_single(mhi_cntrl->cntrl_dev,
					 mhi_buf->dma_addr, mhi_buf->len,
					 DMA_FROM_DEVICE);
			kfree(mhi_buf->buf);

		} else {
			mhi_fw_free_coherent(mhi_cntrl, mhi_buf->len,
					     mhi_buf->buf, mhi_buf->dma_addr);
		}
	}

error_alloc_mhi_buf:
	kfree(img_info);

	return -ENOMEM;
}

/**
 * calculate_elf_size - Calculate total size of an ELF image
 * @fw_data: Pointer to ELF data
 * @fw_sz: Size of firmware buffer
 *
 * Parses ELF program headers to find the end of the last segment.
 *
 * Returns: Total ELF size, or 0 on error
 */
static size_t calculate_elf_size(const u8 *fw_data, size_t fw_sz)
{
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdrs;
	size_t max_end = 0;
	int i;

	if (fw_sz < sizeof(struct elf32_hdr))
		return 0;

	ehdr = (struct elf32_hdr *)fw_data;
	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG) != 0)
		return 0;

	phdrs = (struct elf32_phdr *)(fw_data + ehdr->e_phoff);

	/* Find the end of the last program header */
	for (i = 0; i < ehdr->e_phnum; i++) {
		struct elf32_phdr *phdr = &phdrs[i];
		size_t seg_end = phdr->p_offset + phdr->p_filesz;

		if (seg_end > max_end)
			max_end = seg_end;
	}

	return max_end;
}

/**
 * mhi_analyze_elf_for_ro_segments - Analyze ELF to find RO/RW boundary
 * @mhi_cntrl: MHI controller
 * @elf2_data: Pointer to second ELF data
 * @elf2_offset: Offset of second ELF from start of firmware
 * @num_ro_segments: Output - number of RO segments
 *
 * Scans ELF program headers until first RW segment is found.
 * Calculates RO boundary relative to firmware start.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int mhi_analyze_elf_for_ro_segments(struct mhi_controller *mhi_cntrl,
					   const u8 *elf2_data,
					   size_t elf2_offset,
					   u32 *num_ro_segments)
{
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdrs;
	size_t first_rw_offset = 0;
	size_t ro_end;
	size_t seg_len = mhi_cntrl->seg_len;
	int i;
	bool found_rw = false;

	/* Validate ELF header */
	ehdr = (struct elf32_hdr *)elf2_data;
	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG) != 0) {
		dev_err(dev, "Invalid ELF magic in second ELF\n");
		return -EINVAL;
	}

	phdrs = (struct elf32_phdr *)(elf2_data + ehdr->e_phoff);

	/* Scan program headers until first RW segment - STOP there! */
	dev_dbg(dev, "Analyzing second ELF: %u program headers\n", ehdr->e_phnum);
	for (i = 0; i < ehdr->e_phnum; i++) {
		struct elf32_phdr *phdr = &phdrs[i];

		/* Debug: Print all segment info */
		dev_dbg(dev, "  Segment[%d]: offset=0x%x size=0x%x flags=0x%x (%c%c%c)\n",
			i, phdr->p_offset, phdr->p_filesz, phdr->p_flags,
			(phdr->p_flags & PF_R) ? 'R' : '-',
			(phdr->p_flags & PF_W) ? 'W' : '-',
			(phdr->p_flags & PF_X) ? 'X' : '-');

		/* Check if writable (first RW segment) */
		if (phdr->p_flags & PF_W) {
			/* Found first RW segment - STOP HERE */
			first_rw_offset = phdr->p_offset;
			found_rw = true;
			dev_dbg(dev, "First RW at offset 0x%zx in second ELF\n",
				first_rw_offset);
			break;  /* Critical: don't scan further! */
		}
	}

	if (!found_rw) {
		dev_err(dev, "No RW segment found in second ELF\n");
		return -EINVAL;
	}

	/* Calculate RO boundary from start of firmware */
	ro_end = elf2_offset + first_rw_offset - 1;

	/* Round down to seg_len boundary */
	*num_ro_segments = ro_end / seg_len;

	dev_info(dev, "ELF analysis: elf2_offset=0x%zx, first_rw=0x%zx, ro_end=0x%zx, %u RO segments (%zu bytes each)\n",
		 elf2_offset, first_rw_offset, ro_end, *num_ro_segments, seg_len);

	return 0;
}

static void mhi_firmware_copy(struct mhi_controller *mhi_cntrl,
			      const u8 *buf, size_t remainder,
			      struct image_info *img_info,
			      u32 start_segment)
{
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	size_t to_cpy;
	struct mhi_buf *mhi_buf = img_info->mhi_buf;
	struct bhi_vec_entry *bhi_vec = img_info->bhi_vec;
	u32 segment_idx = 0;

	if (start_segment > 0) {
		dev_dbg(dev, "Firmware copy: Skipping first %u RO segments\n", start_segment);
		dev_dbg(dev, "  Copying %zu bytes to RW segments starting at index %u\n",
			remainder, start_segment);

		/* Skip to RW segments */
		mhi_buf += start_segment;
		bhi_vec += start_segment;
		segment_idx = start_segment;
	} else {
		dev_dbg(dev, "Firmware copy: Copying %zu bytes to all segments\n",
			remainder);
	}

	while (remainder) {
		to_cpy = min(remainder, mhi_buf->len);
		memcpy(mhi_buf->buf, buf, to_cpy);
		bhi_vec->dma_addr = cpu_to_le64(mhi_buf->dma_addr);
		bhi_vec->size = cpu_to_le64(to_cpy);

		dev_dbg(dev, "  Copied %zu bytes to segment[%u]: dma=0x%llx\n",
			to_cpy, segment_idx, (u64)mhi_buf->dma_addr);

		buf += to_cpy;
		remainder -= to_cpy;
		bhi_vec++;
		mhi_buf++;
		segment_idx++;
	}

	/* Populate vector table entries for RO segments (if skipped) */
	if (start_segment > 0) {
		mhi_buf = img_info->mhi_buf;
		bhi_vec = img_info->bhi_vec;

		dev_dbg(dev, "Populating vector table for %u RO segments\n", start_segment);

		for (segment_idx = 0; segment_idx < start_segment; segment_idx++) {
			bhi_vec->dma_addr = cpu_to_le64(mhi_buf->dma_addr);
			bhi_vec->size = cpu_to_le64(mhi_buf->len);

			dev_dbg(dev, "  RO segment[%u]: dma=0x%llx size=%llu (shared)\n",
				segment_idx, (u64)mhi_buf->dma_addr,
				(u64)mhi_buf->len);

			bhi_vec++;
			mhi_buf++;
		}
	}
}

void mhi_fw_load_handler(struct mhi_controller *mhi_cntrl)
{
	const struct firmware *firmware = NULL;
	struct mhi_shared_ro *shared_ro = NULL;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	const u8 *original_fw_data, *rw_fw_data;
	enum mhi_pm_state new_state;
	const char *fw_name;
	const u8 *fw_data;
	size_t rw_size, fw_sz, size;
	u32 num_ro_segments = 0;
	dma_addr_t dma_addr;
	void *buf;
	int i, ret;

	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state)) {
		dev_err(dev, "Device MHI is not in valid state\n");
		return;
	}

	/* save hardware info from BHI */
	ret = mhi_read_reg(mhi_cntrl, mhi_cntrl->bhi, BHI_SERIALNU,
			   &mhi_cntrl->serial_number);
	if (ret)
		dev_err(dev, "Could not capture serial number via BHI\n");

	for (i = 0; i < ARRAY_SIZE(mhi_cntrl->oem_pk_hash); i++) {
		ret = mhi_read_reg(mhi_cntrl, mhi_cntrl->bhi, BHI_OEMPKHASH(i),
				   &mhi_cntrl->oem_pk_hash[i]);
		if (ret) {
			dev_err(dev, "Could not capture OEM PK HASH via BHI\n");
			break;
		}
	}

	/* wait for ready on pass through or any other execution environment */
	if (!MHI_FW_LOAD_CAPABLE(mhi_cntrl->ee))
		goto fw_load_ready_state;

	fw_name = (mhi_cntrl->ee == MHI_EE_EDL) ?
		mhi_cntrl->edl_image : mhi_cntrl->fw_image;

	/* check if the driver has already provided the firmware data */
	if (!fw_name && mhi_cntrl->fbc_download &&
	    mhi_cntrl->fw_data && mhi_cntrl->fw_sz) {
		if (!mhi_cntrl->sbl_size) {
			dev_err(dev, "fw_data provided but no sbl_size\n");
			goto error_fw_load;
		}

		size = mhi_cntrl->sbl_size;
		fw_data = mhi_cntrl->fw_data;
		fw_sz = mhi_cntrl->fw_sz;
		goto skip_req_fw;
	}

	if (!fw_name || (mhi_cntrl->fbc_download && (!mhi_cntrl->sbl_size ||
						     !mhi_cntrl->seg_len))) {
		dev_err(dev,
			"No firmware image defined or !sbl_size || !seg_len\n");
		goto error_fw_load;
	}

	ret = request_firmware(&firmware, fw_name, dev);
	if (ret) {
		dev_err(dev, "Error loading firmware: %d\n", ret);
		goto error_fw_load;
	}

	size = (mhi_cntrl->fbc_download) ? mhi_cntrl->sbl_size : firmware->size;

	/* SBL size provided is maximum size, not necessarily the image size */
	if (size > firmware->size)
		size = firmware->size;

	fw_data = firmware->data;
	fw_sz = firmware->size;

skip_req_fw:
	buf = mhi_fw_alloc_coherent(mhi_cntrl, size, &dma_addr, GFP_KERNEL);
	if (!buf) {
		release_firmware(firmware);
		goto error_fw_load;
	}

	/* Download image using BHI */
	memcpy(buf, fw_data, size);
	ret = mhi_fw_load_bhi(mhi_cntrl, dma_addr, size);
	mhi_fw_free_coherent(mhi_cntrl, size, buf, dma_addr);

	/* Error or in EDL mode, we're done */
	if (ret) {
		dev_err(dev, "MHI did not load image over BHI, ret: %d\n", ret);
		release_firmware(firmware);
		goto error_fw_load;
	}

	/* Wait for ready since EDL image was loaded */
	if (fw_name && fw_name == mhi_cntrl->edl_image) {
		release_firmware(firmware);
		goto fw_load_ready_state;
	}

	write_lock_irq(&mhi_cntrl->pm_lock);
	mhi_cntrl->dev_state = MHI_STATE_RESET;
	write_unlock_irq(&mhi_cntrl->pm_lock);

	/*
	 * If we're doing fbc, populate vector tables while
	 * device transitioning into MHI READY state
	 */
	if (mhi_cntrl->fbc_download) {
		original_fw_data = firmware ? firmware->data : mhi_cntrl->fw_data;
		num_ro_segments = 0;
		shared_ro = NULL;

		/*
		 * Some FW combine two separate ELF images (SBL + WLAN FW) in a single
		 * file. Hence, check for the existence of the second ELF header after
		 * SBL. If present, load the second image separately.
		 */
		if (!memcmp(fw_data + mhi_cntrl->sbl_size, ELFMAG, SELFMAG)) {
			fw_data += mhi_cntrl->sbl_size;
			fw_sz -= mhi_cntrl->sbl_size;
		}

		/* Check if we can reuse RO segments BEFORE allocation */
		if (mhi_cntrl->elf_fw_optimization) {
			const u8 *elf2_data;
			size_t elf2_offset, elf1_size, fw_size;

			/* Determine scenario and find second ELF */
			if (fw_data != original_fw_data) {
				/* Dual ELF detected at sbl_size */
				elf2_data = fw_data;
				elf2_offset = mhi_cntrl->sbl_size;
				dev_dbg(dev, "Dual ELF detected at sbl_size\n");
			} else {
				/* Need to find second ELF manually */
				fw_size = firmware ? firmware->size :
						     mhi_cntrl->fw_sz;
				elf1_size = calculate_elf_size(original_fw_data,
							       fw_size);

				if (elf1_size == 0) {
					dev_err(dev, "Failed to calculate first ELF size\n");
					goto skip_optimization;
				}

				/* Verify second ELF exists */
				if (!memcmp(original_fw_data + elf1_size, ELFMAG, SELFMAG)) {
					elf2_data = original_fw_data + elf1_size;
					elf2_offset = elf1_size;
					dev_dbg(dev, "Found second ELF at offset 0x%zx\n",
						elf1_size);
				} else {
					dev_err(dev, "elf_fw_optimization enabled but no second ELF found at 0x%zx\n",
						elf1_size);
					goto skip_optimization;
				}
			}

			/* Analyze second ELF to find RO/RW boundary */
			ret = mhi_analyze_elf_for_ro_segments(mhi_cntrl, elf2_data,
							      elf2_offset, &num_ro_segments);
			if (ret != 0 || num_ro_segments == 0) {
				dev_warn(dev, "ELF analysis failed or no RO segments, skipping optimization\n");
				goto skip_optimization;
			}

			/* Check if we can reuse existing RO segments */
			mutex_lock(&mhi_shared_ro_lock);
			if (mhi_global_shared_ro) {
				/* Subsequent EP: Reuse RO segments */
				shared_ro = mhi_global_shared_ro;
				refcount_inc(&shared_ro->refcount);
				mhi_cntrl->shared_ro_segments = shared_ro;
				dev_info(dev, "Subsequent EP: Reusing %u RO segments (refcount=%d)\n",
					 shared_ro->num_segments,
					 refcount_read(&shared_ro->refcount));
			}
			mutex_unlock(&mhi_shared_ro_lock);
		}

skip_optimization:
		/* Allocate based on whether we have shared RO */
		if (shared_ro) {
			/* Subsequent EP: Allocate only RW segments */
			rw_size = fw_sz - (num_ro_segments * mhi_cntrl->seg_len);
			rw_fw_data = fw_data + (num_ro_segments * mhi_cntrl->seg_len);

			ret = mhi_alloc_bhie_table_partial(mhi_cntrl, &mhi_cntrl->fbc_image,
							   fw_sz, rw_size, num_ro_segments,
							   shared_ro->mhi_bufs, IMG_TYPE_FBC);
			if (ret) {
				release_firmware(firmware);
				goto error_fw_load;
			}

			/* Copy firmware to RW segments only */
			mhi_firmware_copy(mhi_cntrl, rw_fw_data, rw_size,
					  mhi_cntrl->fbc_image, num_ro_segments);
		} else {
			/* First EP: Normal full allocation */
			ret = mhi_alloc_bhie_table(mhi_cntrl, &mhi_cntrl->fbc_image,
						   fw_sz, IMG_TYPE_FBC);
			if (ret) {
				release_firmware(firmware);
				goto error_fw_load;
			}

			/* Copy firmware to all segments */
			mhi_firmware_copy(mhi_cntrl, fw_data, fw_sz,
					  mhi_cntrl->fbc_image, 0);

			/* First EP with optimization: Save RO segments */
			if (mhi_cntrl->elf_fw_optimization && num_ro_segments > 0) {
				struct mhi_shared_ro *new_shared_ro;

				new_shared_ro = kzalloc(sizeof(*new_shared_ro), GFP_KERNEL);
				if (new_shared_ro) {
					new_shared_ro->mhi_bufs = kcalloc(num_ro_segments,
									  sizeof(struct mhi_buf),
									  GFP_KERNEL);
					if (new_shared_ro->mhi_bufs) {
						memcpy(new_shared_ro->mhi_bufs,
						       mhi_cntrl->fbc_image->mhi_buf,
						       num_ro_segments * sizeof(struct mhi_buf));

						new_shared_ro->num_segments = num_ro_segments;
						refcount_set(&new_shared_ro->refcount, 1);
						mutex_init(&new_shared_ro->lock);

						mutex_lock(&mhi_shared_ro_lock);
						mhi_global_shared_ro = new_shared_ro;
						mhi_cntrl->shared_ro_segments = new_shared_ro;
						mutex_unlock(&mhi_shared_ro_lock);

						dev_info(dev, "First EP: Saved %u RO segments for sharing\n",
							 num_ro_segments);
					} else {
						kfree(new_shared_ro);
					}
				}
			}
		}
	}

	release_firmware(firmware);

fw_load_ready_state:
	/* Transitioning into MHI RESET->READY state */
	ret = mhi_ready_state_transition(mhi_cntrl);
	if (ret) {
		dev_err(dev, "MHI did not enter READY state\n");
		goto error_ready_state;
	}

	dev_info(dev, "Wait for device to enter SBL or Mission mode\n");
	return;

error_ready_state:
	if (mhi_cntrl->fbc_download) {
		mhi_free_bhie_table(mhi_cntrl, mhi_cntrl->fbc_image,
				    IMG_TYPE_FBC);
		mhi_cntrl->fbc_image = NULL;
	}

error_fw_load:
	write_lock_irq(&mhi_cntrl->pm_lock);
	new_state = mhi_tryset_pm_state(mhi_cntrl, MHI_PM_FW_DL_ERR);
	write_unlock_irq(&mhi_cntrl->pm_lock);
	if (new_state == MHI_PM_FW_DL_ERR)
		wake_up_all(&mhi_cntrl->state_event);
}

int mhi_download_amss_image(struct mhi_controller *mhi_cntrl)
{
	struct image_info *image_info = mhi_cntrl->fbc_image;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	enum mhi_pm_state new_state;
	int ret;

	if (!image_info)
		return -EIO;

	ret = mhi_handle_boot_args(mhi_cntrl);
	if(ret) {
		dev_err(dev, "Failed to handle the boot-args, ret: %d\n",ret);
		return ret;
	}

	/* Download the License */
	if (mhi_cntrl->device_number == QCN9224_DEVICE_NUM)
		mhi_download_fw_license(mhi_cntrl);

	ret = mhi_fw_load_bhie(mhi_cntrl,
			       /* Vector table is the last entry */
			       &image_info->mhi_buf[image_info->entries - 1]);
	if (ret) {
		dev_err(dev, "MHI did not load AMSS, ret:%d\n", ret);
		write_lock_irq(&mhi_cntrl->pm_lock);
		new_state = mhi_tryset_pm_state(mhi_cntrl, MHI_PM_FW_DL_ERR);
		write_unlock_irq(&mhi_cntrl->pm_lock);
		if (new_state == MHI_PM_FW_DL_ERR)
			wake_up_all(&mhi_cntrl->state_event);
	}

	return ret;
}
