/*
 * Copyright (C) Excito Elektronik i Skåne AB, 2010.
 * Author: Tor Krill <tor@excito.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

/*
 * TODO:
 * Better error recovery
 * No support for using PRDs (Thus max 64KB transfers)
 * No NCQ support
 * No port multiplier support
 * No flush after write
 */

/*
 * Should data structures be malloced?
 */
#define MALLOC_QUEUE 1

#include <common.h>
#include <asm/io.h>
#include <malloc.h>
#include <libata.h>
#include <fis.h>
#include "sata_mv.h"

extern block_dev_desc_t sata_dev_desc[CONFIG_SYS_SATA_MAX_DEVICE];

/* Keep track if hw is initialized or not */
static u32 hw_init = 0;

/* forwards */
static int get_reqip(int port);
static int get_reqop(int port);

struct mv_priv{
	char		name[12];
	u32			link;
	u32			regbase;
	u32			queue_depth;
	u16			pio;
	u16			mwdma;
	u16			udma;

#ifdef MALLOC_QUEUE
	void		*crqb_alloc;
	struct crqb	*request;

	void		*crpb_alloc;
	struct crpb	*response;

#else
	struct crqb request[REQUEST_QUEUE_SIZE] __attribute__ ((aligned(CRQB_ALIGN)));
	struct crpb response[REQUEST_QUEUE_SIZE] __attribute__ ((aligned(CRPB_ALIGN)));
#endif

};

#if 0
static inline void mdelay(unsigned long msec){
	unsigned long i;
	for (i = 0; i < msec; i++)
		udelay(1000);
}
#endif

static int ata_wait_register(volatile unsigned *addr, u32 mask,
                         u32 val, u32 timeout_msec){
	int i;
	u32 temp;

	for (i = 0; (((temp = in_le32(addr)) & mask) != val)
			&& i < timeout_msec; i++)
				mdelay(1);
	return (i < timeout_msec) ? 0 : -1;
}


/* Cut from sata_mv in linux kernel */
static int mv_stop_edma_engine(int port){
	int i;
	struct mv_priv *priv;

	priv = (struct mv_priv *) sata_dev_desc[port].priv;

	/* Disable eDMA.  The disable bit auto clears. */
	out_le32(priv->regbase + EDMA_CMD, EDMA_CMD_DISEDMA);

	/* Wait for the chip to confirm eDMA is off. */
	for (i = 10000; i > 0; i--) {
		u32 reg = in_le32(priv->regbase + EDMA_CMD);
		if (!(reg & EDMA_CMD_ENEDMA)){
			debug("EDMA stop on port %d succesful\n\r",port);
			return 0;
		}
		udelay(10);
	}
	debug("EDMA stop on port %d failed\n\r",port);
	return -1;
}

static int mv_start_edma_engine(int port){
	struct mv_priv *priv =
			(struct mv_priv *) sata_dev_desc[port].priv;
	u32 tmp;

	/* Check preconditions */
	tmp = in_le32(priv->regbase + SIR_SSTATUS);
	if ((tmp & SSTATUS_DET_MASK) != 0x03) {
		printf("Device error on port: %d\n\r",port);
		return -1;
	}

	tmp = in_le32(priv->regbase+PIO_CMD_STATUS);
	if ( tmp & ( ATA_BUSY | ATA_DRQ )){
		printf("Device not ready on port: %d\n\r",port);
		return -1;
	}

	/* Clear interrupt cause */
	out_le32(priv->regbase+EDMA_IECR, 0x0);

	tmp = in_le32( KW_SATAHC_BASE + SATAHC_ICR );
	tmp &= ~( port==0 ? SATAHC_ICR_PORT0 : SATAHC_ICR_PORT1);
	out_le32( KW_SATAHC_BASE + SATAHC_ICR ,tmp);

	/* Configure edma operation */
	tmp=in_le32(priv->regbase+EDMA_CFG);
	tmp &= ~EDMA_CFG_NCQ; /* No NCQ */
	tmp &= ~EDMA_CFG_EQUE; /* Dont queue operations */
	out_le32(priv->regbase+EDMA_CFG,tmp);

	out_le32(priv->regbase+SIR_FIS_IRQ_CAUSE, 0x0);

	/* Configure fis, set all to no-wait for now */
	out_le32(priv->regbase+SIR_FIS_CFG, 0x0);

	/* Setup request queue */
	out_le32(priv->regbase+EDMA_RQBA_HI, 0x0);
#ifdef MALLOC_QUEUE
	out_le32(priv->regbase+EDMA_RQIPR, priv->request);
#else
	out_le32(priv->regbase+EDMA_RQIPR, &priv->request);
#endif
	out_le32(priv->regbase+EDMA_RQOPR, 0x0);

	/* Setup response queue */
	out_le32(priv->regbase+EDMA_RSBA_HI, 0x0);
#ifdef MALLOC_QUEUE
	out_le32(priv->regbase+EDMA_RSOPR, priv->response);
#else
	out_le32(priv->regbase+EDMA_RSOPR, &priv->response);
#endif
	out_le32(priv->regbase+EDMA_RSIPR, 0x0);

	/* Start edma */
	out_le32(priv->regbase+EDMA_CMD, EDMA_CMD_ENEDMA);

	return 0;
}

static int mv_reset_channel(int port){
	struct mv_priv *priv;
	priv = (struct mv_priv *) sata_dev_desc[port].priv;

	/* Make sure edma is stopped  */
	mv_stop_edma_engine(port);

	out_le32(priv->regbase + EDMA_CMD, EDMA_CMD_ATARST);
	udelay(25);     /* allow reset propagation */
	out_le32(priv->regbase + EDMA_CMD, 0);
	mdelay(150);
	return 0;
}

static void mv_reset_port(int port){
	struct mv_priv *priv;
	priv = (struct mv_priv *) sata_dev_desc[port].priv;

	mv_reset_channel(port);

	out_le32(priv->regbase+EDMA_CMD, 0x0);
	out_le32(priv->regbase+EDMA_CFG, 0x101f);
	/* out_le32(priv->regbase+EDMA_TIMER, 0x0);  ???? */
	out_le32(priv->regbase+EDMA_IECR, 0x0);
	out_le32(priv->regbase+EDMA_IEMR, 0x0);
	out_le32(priv->regbase+EDMA_RQBA_HI, 0x0);
	out_le32(priv->regbase+EDMA_RQIPR, 0x0);
	out_le32(priv->regbase+EDMA_RQOPR, 0x0);
	out_le32(priv->regbase+EDMA_RSBA_HI, 0x0);
	out_le32(priv->regbase+EDMA_RSIPR, 0x0);
	out_le32(priv->regbase+EDMA_RSOPR, 0x0);
	/* out_le32(priv->regbase+EDMA_TEST_CTL, 0x0); unknown register */

	out_le32(priv->regbase+EDMA_IORTO,0xfa);

}

static void mv_reset_one_hc(void){
	out_le32(KW_SATAHC_BASE+SATAHC_ICT, 0x00);
	out_le32(KW_SATAHC_BASE+SATAHC_ITT, 0x00);
	out_le32(KW_SATAHC_BASE+SATAHC_ICR, 0x00);
}

static int probe_port(int port){
	u32 tmp;
	int tries, tries2, set15=0;
	struct mv_priv *priv;
	priv = (struct mv_priv *) sata_dev_desc[port].priv;

	debug("Probe port: %d\n\r",port);

	for ( tries=0; tries<2; tries++){

		/* Clear SError */
		out_le32(priv->regbase+SIR_SERROR,0x0);

		/* trigger com-init */
		tmp = in_le32(priv->regbase + SIR_SCONTROL);
		tmp = (tmp & 0x0f0) | 0x300 | SIR_SCONTROL_DETEN;
		out_le32(priv->regbase + SIR_SCONTROL, tmp);

		mdelay(1);

		tmp = in_le32(priv->regbase + SIR_SCONTROL);
		tries2 = 5;
		do{
			tmp = (tmp & 0x0f0) | 0x300;
			out_le32(priv->regbase + SIR_SCONTROL, tmp);
			mdelay(200);
			tmp = in_le32(priv->regbase + SIR_SCONTROL);
		} while ((tmp & 0xf0f) != 0x300 && tries2--);

		mdelay(100);

		for (tries2 = 0; tries2 < 200; tries2++) {

			tmp = in_le32(priv->regbase + SIR_SSTATUS);
			if ((tmp & SSTATUS_DET_MASK) == 0x03) {
				debug("Found device on port\n\r");
				return 0;
			}
			mdelay(1);
		}

		if ((tmp & SSTATUS_DET_MASK) == 0) {
			debug("No device attached on port %d\n\r",port);
			return -1;
		}

		if ( !set15 ){
			/* Try on 1.5Gb/S */
			debug("Try 1.5Gb link\n\r");
			set15=1;
			out_le32(priv->regbase+SIR_SCONTROL, 0x304);

			tmp = in_le32(priv->regbase+SIR_ICFG);
			tmp &= ~SIR_CFG_GEN2EN;
			out_le32(priv->regbase+SIR_ICFG, tmp);

			mv_reset_channel(port);
		}

	}

	debug("Failed to probe port\n\r");
	return -1;
}

/* Get request queue in pointer */
static int get_reqip(int port){
	u32 tmp;
	struct mv_priv *priv = (struct mv_priv *) sata_dev_desc[port].priv;

	tmp = in_le32(priv->regbase+EDMA_RQIPR) & EDMA_RQIPR_IPMASK;
	tmp = tmp >> EDMA_RQIPR_IPSHIFT;

	return tmp;
}

static void set_reqip(int port, int reqin){
	struct mv_priv *priv = (struct mv_priv *) sata_dev_desc[port].priv;
	u32 tmp;
	tmp = in_le32(priv->regbase+EDMA_RQIPR) & ~EDMA_RQIPR_IPMASK;
	tmp |= ((reqin << EDMA_RQIPR_IPSHIFT) & EDMA_RQIPR_IPMASK);
	out_le32(priv->regbase+EDMA_RQIPR, tmp);
}

/* Get next available slot, ignoring possible overwrite */
static int get_next_reqip(int port){
	int slot=get_reqip(port);
	slot = (slot + 1) % REQUEST_QUEUE_SIZE;
	return slot;
}

/* Get request queue out pointer */
static int get_reqop(int port){
	u32 tmp;
	struct mv_priv *priv = (struct mv_priv *) sata_dev_desc[port].priv;

	tmp = in_le32(priv->regbase+EDMA_RQOPR) & EDMA_RQOPR_OPMASK;
	tmp = tmp >> EDMA_RQOPR_OPSHIFT;

	return tmp;
}

/* Get response queue in pointer */
static int get_rspip(int port){
	u32 tmp;
	struct mv_priv *priv = (struct mv_priv *) sata_dev_desc[port].priv;

	tmp = in_le32(priv->regbase+EDMA_RSIPR) & EDMA_RSIPR_IPMASK;
	tmp = tmp >> EDMA_RSIPR_IPSHIFT;

	return tmp;
}

/* Get response queue out pointer */
static int get_rspop(int port){
	u32 tmp;
	struct mv_priv *priv = (struct mv_priv *) sata_dev_desc[port].priv;

	tmp = in_le32(priv->regbase+EDMA_RSOPR) & EDMA_RSOPR_OPMASK;
	tmp = tmp >> EDMA_RSOPR_OPSHIFT;
	return tmp;
}
/* Get next response queue pointer  */
static int get_next_rspop(int port){
	return (get_rspop(port) + 1) % RESPONSE_QUEUE_SIZE;
}

/* Set response queue pointer */
static void set_rspop(int port, int reqin){
	struct mv_priv *priv = (struct mv_priv *) sata_dev_desc[port].priv;
	u32 tmp;

	tmp = in_le32(priv->regbase+EDMA_RSOPR) & ~EDMA_RSOPR_OPMASK;
	tmp |= ((reqin << EDMA_RSOPR_OPSHIFT) & EDMA_RSOPR_OPMASK);

	out_le32(priv->regbase+EDMA_RSOPR, tmp);
}


static int wait_dma_completion(int port, int index, u32 timeout_msec){
	u32 tmp,res;

	tmp = port==0?SATAHC_ICR_PORT0:SATAHC_ICR_PORT1;

	if ( (res = ata_wait_register(
			(u32*)(KW_SATAHC_BASE+SATAHC_ICR), tmp ,tmp, timeout_msec))){
		printf("Failed to wait for completion on port %d\n",port);
	}

	return res;
}

static void process_responses(int port){
	u32 tmp;
	u32 outind = get_rspop(port);
#ifdef DEBUG
	struct mv_priv *priv = (struct mv_priv *) sata_dev_desc[port].priv;
#endif

	/* Ack interrupts */
	tmp = in_le32(KW_SATAHC_BASE+SATAHC_ICR);
	if ( port == 0 ){
		tmp &= ~( 1<<0 | 1<<8);
	}else{
		tmp &= ~(1<<1 | 1<<9);
	}
	tmp &= ~(1<<4);
	out_le32(KW_SATAHC_BASE+SATAHC_ICR,tmp);

	while(get_rspip(port)!=outind){
#ifdef DEBUG
		debug("Response index %d flags %08x on port %d\n",outind,priv->response[outind].flags,port);
#endif
		outind=get_next_rspop(port);
		set_rspop(port,outind);
	}
}

static int mv_ata_exec_ata_cmd(int port, struct sata_fis_h2d *cfis,
				u8 *buffer, u32 len, u32 iswrite){
	struct mv_priv *priv = (struct mv_priv *) sata_dev_desc[port].priv;

	int slot;

	if ( len >= 64*1024 ){
		printf("We only support <64K transfers for now\n");
		return -1;
	}

	/* Initialize request */

	slot = get_reqip(port);

	memset(&priv->request[slot], 0, sizeof(struct crqb));
	priv->request[slot].dtb_low=(u32)buffer;
	/* Dont use PRDs */
	priv->request[slot].control_flags = CRQB_CNTRLFLAGS_PRDMODE;
	priv->request[slot].control_flags |= iswrite?0:CRQB_CNTRLFLAGS_DIR;
	priv->request[slot].control_flags |=
			((cfis->pm_port_c<<CRQB_CNTRLFLAGS_PMPORTSHIFT)
					& CRQB_CNTRLFLAGS_PMPORTMASK);

	priv->request[slot].drb_count=len;

	priv->request[slot].ata_cmd_feat =
			((cfis->command<<CRQB_CMDFEAT_CMDSHIFT) &
					CRQB_CMDFEAT_CMDMASK);
	priv->request[slot].ata_cmd_feat |=
			((cfis->features<<CRQB_CMDFEAT_FEATSHIFT) &
					CRQB_CMDFEAT_FEATMASK);

	priv->request[slot].ata_addr =
			((cfis->lba_low<<CRQB_ADDR_LBA_LOWSHIFT) &
					CRQB_ADDR_LBA_LOWMASK);
	priv->request[slot].ata_addr |=
			((cfis->lba_mid<<CRQB_ADDR_LBA_MIDSHIFT) &
					CRQB_ADDR_LBA_MIDMASK);
	priv->request[slot].ata_addr |=
			((cfis->lba_high<<CRQB_ADDR_LBA_HIGHSHIFT) &
					CRQB_ADDR_LBA_HIGHMASK);
	priv->request[slot].ata_addr |=
			((cfis->device<<CRQB_ADDR_DEVICE_SHIFT) &
					CRQB_ADDR_DEVICE_MASK);

	priv->request[slot].ata_addr_exp =
			((cfis->lba_low_exp<<CRQB_ADDR_LBA_LOW_EXP_SHIFT) &
					CRQB_ADDR_LBA_LOW_EXP_MASK);
	priv->request[slot].ata_addr_exp |=
			((cfis->lba_mid_exp<<CRQB_ADDR_LBA_MID_EXP_SHIFT) &
					CRQB_ADDR_LBA_MID_EXP_MASK);
	priv->request[slot].ata_addr_exp |=
			((cfis->lba_high_exp<<CRQB_ADDR_LBA_HIGH_EXP_SHIFT) &
					CRQB_ADDR_LBA_HIGH_EXP_MASK);
	priv->request[slot].ata_addr_exp |=
			((cfis->features_exp<<CRQB_ADDR_FEATURE_EXP__SHIFT) &
					CRQB_ADDR_FEATURE_EXP__MASK);

	priv->request[slot].ata_sect_count =
			((cfis->sector_count<<CRQB_SECTCOUNT_COUNT_SHIFT) &
					CRQB_SECTCOUNT_COUNT_MASK);
	priv->request[slot].ata_sect_count |=
			((cfis->sector_count_exp<<CRQB_SECTCOUNT_COUNT_EXP_SHIFT) &
					CRQB_SECTCOUNT_COUNT_EXP_MASK);

	/* Trigger operation */
	slot=get_next_reqip(port);
	set_reqip(port, slot);

	/* Wait for completion */
	if( wait_dma_completion(port,slot,10000) ){
		printf("ATA operation timed out\n");
		return -1;
	}

	process_responses(port);

	return len;
}

static u32 mv_sata_rw_cmd_ext(int port, lbaint_t start, u32 blkcnt,
		u8 *buffer, int is_write){

	struct sata_fis_h2d cfis;
	u32 res;
	u64 block;

	block = (u64)start;

	memset(&cfis, 0, sizeof(struct sata_fis_h2d));

	cfis.fis_type = SATA_FIS_TYPE_REGISTER_H2D;

	cfis.command = (is_write) ? ATA_CMD_WRITE_EXT
				 : ATA_CMD_READ_EXT;

	cfis.lba_high_exp = (block >> 40) & 0xff;
	cfis.lba_mid_exp = (block >> 32) & 0xff;
	cfis.lba_low_exp = (block >> 24) & 0xff;
	cfis.lba_high = (block >> 16) & 0xff;
	cfis.lba_mid = (block >> 8) & 0xff;
	cfis.lba_low = block & 0xff;
	cfis.device = ATA_LBA;
	cfis.sector_count_exp = (blkcnt >> 8) & 0xff;
	cfis.sector_count = blkcnt & 0xff;

	res = mv_ata_exec_ata_cmd(
			port,&cfis,buffer,ATA_SECT_SIZE * blkcnt, is_write);

	return res>=0?blkcnt:res;
}


static u32 mv_sata_rw_cmd(int port, lbaint_t start, u32 blkcnt, u8 *buffer
		, int is_write){

	struct sata_fis_h2d cfis;
	lbaint_t block;
	u32 res;

	block = start;

	memset(&cfis, 0, sizeof(struct sata_fis_h2d));

	cfis.fis_type = SATA_FIS_TYPE_REGISTER_H2D;
	cfis.command = (is_write) ? ATA_CMD_WRITE : ATA_CMD_READ;
	cfis.device = ATA_LBA;

	cfis.device |= (block >> 24) & 0xf;
	cfis.lba_high = (block >> 16) & 0xff;
	cfis.lba_mid = (block >> 8) & 0xff;
	cfis.lba_low = block & 0xff;
	cfis.sector_count = (u8)(blkcnt & 0xff);

	res = mv_ata_exec_ata_cmd(
			port,&cfis,buffer,ATA_SECT_SIZE * blkcnt, is_write);

	return res>=0?blkcnt:res;
}

u32 ata_low_level_rw(int dev, lbaint_t blknr, lbaint_t blkcnt, void *buffer,
		int is_write){

	lbaint_t start, blks;
	u8 *addr;
	int max_blks;

	debug("sata_low_level_rw: %lld %lld\n",blknr,blkcnt);

	start = blknr;
	blks = blkcnt;
	addr = (u8 *)buffer;

	max_blks = MV_ATA_MAX_SECTORS;
	do {
		if (blks > max_blks) {
			if(sata_dev_desc[dev].lba48){
				mv_sata_rw_cmd_ext(dev, start, max_blks, addr, is_write);
			}else{
				mv_sata_rw_cmd(dev, start, max_blks, addr, is_write);
			}
			start += max_blks;
			blks -= max_blks;
			addr += ATA_SECT_SIZE * max_blks;
		} else {
			if(sata_dev_desc[dev].lba48){
				mv_sata_rw_cmd_ext(dev, start, blks, addr, is_write);
			}else{
				mv_sata_rw_cmd(dev, start, blks, addr, is_write);
			}
			start += blks;
			blks = 0;
			addr += ATA_SECT_SIZE * blks;
		}
	} while (blks != 0);

	return blkcnt;
}


static int mv_ata_exec_ata_cmd_nondma(int port,
		struct sata_fis_h2d *cfis, u8 *buffer, u32 len, u32 iswrite){
	int i;
	u16 *tp;
	struct mv_priv *priv =
			(struct mv_priv *) sata_dev_desc[port].priv;

	debug("mv_ata_exec_ata_cmd_nondma\n");

	out_le32(priv->regbase+PIO_SECTOR_COUNT,cfis->sector_count);
	out_le32(priv->regbase+PIO_LBA_HI,cfis->lba_high);
	out_le32(priv->regbase+PIO_LBA_MID,cfis->lba_mid);
	out_le32(priv->regbase+PIO_LBA_LOW,cfis->lba_low);
	out_le32(priv->regbase+PIO_ERR_FEATURES,cfis->features);
	out_le32(priv->regbase+PIO_DEVICE,cfis->device);
	out_le32(priv->regbase+PIO_CMD_STATUS,cfis->command);

	mdelay(200);

	if(ata_wait_register((u32 *)(priv->regbase+PIO_CMD_STATUS),ATA_BUSY,0x0,10000)){
		debug("Failed to wait for completion\n");
		return -1;
	}

	if(len>0){
		tp=(u16*)buffer;
		for ( i=0; i<len/2;i++){
			if ( iswrite ){
				out_le16(priv->regbase+PIO_DATA,*tp++);
			}else{
				*tp++ = in_le16(priv->regbase+PIO_DATA);
			}
		}
	}
	return len;
}

static int mv_sata_identify(int port, u16 *id){
	struct sata_fis_h2d h2d;

	memset(&h2d, 0, sizeof(struct sata_fis_h2d));

	h2d.fis_type = SATA_FIS_TYPE_REGISTER_H2D;
	h2d.command = ATA_CMD_ID_ATA;

	/* Give device time to get operational */
	mdelay(150);

	return mv_ata_exec_ata_cmd_nondma(port,&h2d,(u8 *) id, ATA_ID_WORDS * 2, READ_CMD);
}

static void mv_sata_xfer_mode(int port, u16 *id){
	struct mv_priv *priv = (struct mv_priv *) sata_dev_desc[port].priv;

	priv->pio = id[ATA_ID_PIO_MODES];
	priv->mwdma = id[ATA_ID_MWDMA_MODES];
	priv->udma = id[ATA_ID_UDMA_MODES];
	debug("pio %04x, mwdma %04x, udma %04x\n\r"
			,priv->pio, priv->mwdma, priv->udma);
}

static void mv_sata_set_features(int port){
	struct mv_priv *priv =
			(struct mv_priv *) sata_dev_desc[port].priv;
	struct sata_fis_h2d cfis;
	u8 udma_cap;

	memset(&cfis, 0, sizeof(struct sata_fis_h2d));

	cfis.fis_type = SATA_FIS_TYPE_REGISTER_H2D;
	cfis.command = ATA_CMD_SET_FEATURES;
	cfis.features = SETFEATURES_XFER;

	/* First check the device capablity */
	udma_cap = (u8)(priv->udma & 0xff);

	if (udma_cap == ATA_UDMA6)
		cfis.sector_count = XFER_UDMA_6;
	if (udma_cap == ATA_UDMA5)
		cfis.sector_count = XFER_UDMA_5;
	if (udma_cap == ATA_UDMA4)
		cfis.sector_count = XFER_UDMA_4;
	if (udma_cap == ATA_UDMA3)
		cfis.sector_count = XFER_UDMA_3;

	mv_ata_exec_ata_cmd_nondma(port, &cfis, NULL, 0, READ_CMD);
}

int mv_sata_spin_down(int dev){
	struct sata_fis_h2d cfis;
	struct mv_priv *priv =
			(struct mv_priv *) sata_dev_desc[dev].priv;

	if (priv->link == 0){
		debug("No device on port: %d\n",dev);
		return 1;
	}

	memset(&cfis, 0, sizeof(struct sata_fis_h2d));

	cfis.fis_type = SATA_FIS_TYPE_REGISTER_H2D;
	cfis.command = ATA_CMD_STANDBY;

	return mv_ata_exec_ata_cmd_nondma(dev, &cfis, NULL, 0, READ_CMD);
}

int mv_sata_spin_up(int dev){
	struct sata_fis_h2d cfis;
	struct mv_priv *priv =
			(struct mv_priv *) sata_dev_desc[dev].priv;

	if (priv->link == 0){
		debug("No device on port: %d\n",dev);
		return 1;
	}

	memset(&cfis, 0, sizeof(struct sata_fis_h2d));

	cfis.fis_type = SATA_FIS_TYPE_REGISTER_H2D;
	cfis.command = ATA_CMD_IDLE;

	return mv_ata_exec_ata_cmd_nondma(dev, &cfis, NULL, 0, READ_CMD);
}

ulong sata_read(int dev, ulong blknr, lbaint_t blkcnt, void *buffer){

	return ata_low_level_rw(dev, blknr, blkcnt, buffer, READ_CMD);

}

ulong sata_write(int dev, ulong blknr, lbaint_t blkcnt, const void *buffer){

	return ata_low_level_rw(dev, blknr, blkcnt, (void *)buffer, WRITE_CMD);

}

int init_sata(int dev){
	struct mv_priv *priv;

	debug("Initialize sata dev: %d\n\r",dev);

	if ( dev < 0 || dev >= CONFIG_SYS_SATA_MAX_DEVICE) {
		printf("Invalid sata device %d\n\r",dev);
		return -1;
	}

	priv = (struct mv_priv*)  malloc(sizeof(struct mv_priv));
	if ( !priv ){
		printf("Failed to allocate memory for private sata data\n\r");
		return -1;
	}

	memset((void*) priv,0 , sizeof(struct mv_priv));

#ifdef MALLOC_QUEUE
	/* Allocate and align request buffer */
	priv->crqb_alloc=malloc(sizeof(struct crqb)*REQUEST_QUEUE_SIZE
						+CRQB_ALIGN);
	if( !priv->crqb_alloc ){
		printf("Unable to allocate memory for request queue\n\r");
		return -1;
	}
	memset(priv->crqb_alloc ,0 ,sizeof(struct crqb)*REQUEST_QUEUE_SIZE+CRQB_ALIGN);
	priv->request=(struct crqb *) (((u32)priv->crqb_alloc + CRQB_ALIGN)
						& ~(CRQB_ALIGN-1));

	/* Allocate and align response buffer */
	priv->crpb_alloc=malloc(sizeof(struct crpb)*REQUEST_QUEUE_SIZE+CRPB_ALIGN);
	if( !priv->crpb_alloc ){
		printf("Unable to allocate memory for response queue\n\r");
		return -1;
	}
	memset(priv->crpb_alloc ,0 ,sizeof(struct crpb)*REQUEST_QUEUE_SIZE+CRPB_ALIGN);
	priv->response=(struct crpb *) (((u32)priv->crpb_alloc + CRPB_ALIGN)
						& ~(CRPB_ALIGN-1));
#else
	memset(&priv->request,0,sizeof(struct crqb)*REQUEST_QUEUE_SIZE);
	memset(&priv->response,0,sizeof(struct crpb)*REQUEST_QUEUE_SIZE);
#endif
	sata_dev_desc[dev].priv = (void*)priv;

	sprintf(priv->name, "SATA%d", dev);

	priv->regbase = dev==0?KW_SATA0_BASE:KW_SATA1_BASE;

	if ( !hw_init ){
		debug("Initialize sata hw\n\r");
		hw_init=1;
		mv_reset_one_hc();
	}

	mv_reset_port(dev);

	if(probe_port(dev)){
		priv->link=0;
		return -1;
	}
	priv->link=1;

	return 0;
}

int scan_sata(int port){
	unsigned char serial[ATA_ID_SERNO_LEN + 1];
	unsigned char firmware[ATA_ID_FW_REV_LEN + 1];
	unsigned char product[ATA_ID_PROD_LEN + 1];
	u64 n_sectors;
	u16 *id;
	struct mv_priv *priv =
			(struct mv_priv *) sata_dev_desc[port].priv;

	if ( !priv->link ){
		return -1;
	}
	id = (u16 *) malloc(ATA_ID_WORDS*2);
	if ( !id ){
		printf("Failed to malloc id data\n");
		return -1;
	}

	mv_sata_identify(port,id);

	ata_swap_buf_le16(id, ATA_ID_WORDS);
#ifdef DEBUG
	ata_dump_id(id);
#endif

	/* Serial number */
	ata_id_c_string(id, serial, ATA_ID_SERNO, sizeof(serial));
	memcpy(sata_dev_desc[port].product, serial, sizeof(serial));

	/* Firmware version */
	ata_id_c_string(id, firmware, ATA_ID_FW_REV, sizeof(firmware));
	memcpy(sata_dev_desc[port].revision, firmware, sizeof(firmware));

	/* Product model */
	ata_id_c_string(id, product, ATA_ID_PROD, sizeof(product));
	memcpy(sata_dev_desc[port].vendor, product, sizeof(product));

	/* Totoal sectors */
	n_sectors = ata_id_n_sectors(id);
	sata_dev_desc[port].lba = n_sectors;

	/* Check if support LBA48 */
	if (ata_id_has_lba48(id)) {
		sata_dev_desc[port].lba48 = 1;
		debug("Device support LBA48\n\r");
	}

	/* Get the NCQ queue depth from device */
	priv->queue_depth = ata_id_queue_depth(id);

	/* Get the xfer mode from device */
	mv_sata_xfer_mode(port, id);

	/* Set the xfer mode to highest speed */
	mv_sata_set_features(port);

	/* Start up */
	mv_start_edma_engine(port);

	return 0;
}
