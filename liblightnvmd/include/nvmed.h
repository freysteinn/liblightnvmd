/*
 * NVMeDirect Device Driver
 *
 * Copyright (c) 2016 Computer Systems Laboratory, Sungkyunkwan University.
 * http://csl.skku.edu
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the therm and condotions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _NVMED_H
#define _NVMED_H

#include <linux/types.h>

#ifndef MODULE
#include <sys/types.h>
#endif

#define NVMED_IOCTL_NVMED_INFO		_IOW('N', 0x50, struct nvmed_device_info)
#define NVMED_IOCTL_QUEUE_CREATE	_IOR('N', 0x51, unsigned int)
#define NVMED_IOCTL_QUEUE_DELETE	_IOW('N', 0x52, unsigned int)
#define NVMED_IOCTL_GET_BUFFER_ADDR	_IOWR('N', 0x60, struct nvmed_buf)
#define NVMED_IOCTL_GET_USER		_IOWR('N', 0x70, struct nvmed_user_quota)
#define NVMED_IOCTL_SET_USER		_IOWR('N', 0x71, struct nvmed_user_quota)

#define NVMED_CACHE_INIT_NUM_PAGES	2560

#define SQ_SIZE(depth)		(depth * sizeof(struct nvme_command))
#define CQ_SIZE(depth)		(depth * sizeof(struct nvme_completion))

typedef __u64 u64;
typedef __u32 u32;
typedef __u16 u16;
typedef __u8  u8;

typedef enum {
	NVMED_SUCCESS,
	NVMED_FAULT,
	NVMED_NOENTRY,
	NVMED_EXCEEDLIMIT,
	NVMED_NOPERM,
	NVMED_OVERQUOTA,
} NVMED_RESULT;

typedef struct nvmed_buf {
	void* addr;
	unsigned int size;
	u64* pfnList;
} NVMED_BUF;

typedef struct nvmed_device_info {
	int instance;
	int lba_shift;
	unsigned int ns_id;
	int q_depth;
	u64 capacity;
	u32 max_hw_sectors;
	u32 stripe_size;
	u32 db_stride;
	u8	vwc;

	unsigned long start_sect;
	unsigned long nr_sects;
	int part_no;
        int nr_queues;
} NVMED_DEVICE_INFO;

typedef struct nvmed_user_quota {
	uid_t uid;
	unsigned int queue_max;
	unsigned int queue_used;
} NVMED_USER_QUOTA;

#endif /* _NVMED_H */


#ifndef _LIB_NVMED_H
#define _LIB_NVMED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <unistd.h>
#include <linux/types.h>
#include <sys/types.h>
#include <pthread.h>

#define PAGE_SIZE		sysconf(_SC_PAGESIZE)

#define NVMED_BUF_MAGIC	0x4E564DED		//NVM'ED'
#define NVMED_NUM_PREALLOC_PRP	64
#define NVMED_CACHE_FORCE_EVICT_MAX	4
#define NVMED_CACHE_INIT_NUM_PAGES	2560	

#define COMPILER_BARRIER() asm volatile("" ::: "memory")

#define QtoD(queue)		queue->nvmed
#define HtoQ(handle) 	handle->queue
#define HtoD(handle) 	QtoD(HtoQ(handle))

        typedef enum {
                NVMED_FALSE = 0,
                NVMED_TRUE = 1,
        } NVMED_BOOL;

#define FLAG_SET(obj, items) (obj)->flags |= items
#define FLAG_SET_FORCE(obj, items) (obj)->flags = items
#define FLAG_SET_SYNC(obj, items) __sync_or_and_fetch(&obj->flags, items)
#define FLAG_UNSET(obj, items) (obj)->flags &= ~items
#define FLAG_UNSET_SYNC(obj, items) __sync_and_and_fetch(&obj->flags, ~items)
#define __FLAG_ISSET(flags, items) (flags & items)? NVMED_TRUE:NVMED_FALSE
#define __FLAG_ISSET_SYNC(obj , items) __sync_and_and_fetch(&obj->flags, items)? NVMED_TRUE:NVMED_FALSE
#define FLAG_ISSET(obj, items) __FLAG_ISSET((obj)->flags, items)
#define FLAG_ISSET_SYNC(obj, items) __FLAG_ISSET_SYNC(obj, items)

#define INC_SYNC(obj)	__sync_add_and_fetch(&obj, 1);
#define DEC_SYNC(obj)	__sync_sub_and_fetch(&obj, 1);
#define INIT_SYNC(obj)	__sync_and_and_fetch(&obj, 0);

        typedef __u64 u64;
        typedef __u32 u32;
        typedef __u16 u16;
        typedef __u8  u8;

//FLAGS for NVMED
        enum {
                NVMED_NO_CACHE			= 1 << 0,
                NVMED_CACHE_SIZE		= 1	<< 1,
                NVMED_CACHE_LAZY_INIT	= 1	<< 2,

                NVMED_NUM_FLAGS			= 3,
        };

//FLAGS For NVMED_HANDLE
        enum {
                HANDLE_DIRECT_IO		= 1 << 0,
                HANDLE_SYNC_IO			= 1 << 1,

                HANDLE_HINT_DMEM 		= 1 << 2,

                HANDLE_MQ				= 1 << 3,

                HANDLE_NUM_FLAGS		= 4,
        };

//STATUS for Process CQ, IOSCHED thread
        enum {
                TD_STATUS_RUNNING		= 1 << 0,
                TD_STATUS_REQ_STOP		= 1 << 1,
                TD_STATUS_STOP			= 1 << 2,
                TD_STATUS_REQ_SUSPEND	= 1 << 3,
                TD_STATUS_SUSPEND		= 1 << 4,

                TD_STATUS_NUM_FLAGS		= 5,
        };

//FLAGS For NVMED_CACHE
        enum {
                CACHE_UNINIT			= 0,
                CACHE_LOCKED			= 1 << 0,
                CACHE_FREE				= 1 << 1,
                CACHE_LRU				= 1 << 2,
                CACHE_DIRTY				= 1 << 3,
                CACHE_UPTODATE			= 1 << 4,
                CACHE_WRITEBACK			= 1 << 5,

                CACHE_NUM_FLAGS			= 6,
        };

//STATUS FOR IOD
        enum {
                IO_INIT					= 1 << 0,
                IO_COMPLETE				= 1	<< 1,
                IO_ERROR				= 1 << 2,

                IO_NUM_FLAGS			= 3,
        };

//FLAGS FOR AIO
        enum {
                NVMED_AIO_ERROR			= 1 << 0,
                NVMED_AIO_BUSY			= 1	<< 1,
                NVMED_AIO_QUEUED		= 1	<< 2,

                NVMED_AIO_STATUS		= 3,
        };

        enum {
                AIO_INIT				= 0,
                AIO_PROCESS			 	= 1 << 1,
                AIO_COMPLETE			= 1 << 2,

                AIO_NUM_FLAGS			= 3,
        };

        typedef struct nvmed_device_info NVMED_DEVICE_INFO;

        typedef struct nvmed {
                char*	ns_path;
                int 	ns_fd;
                u32 	flags;

                NVMED_DEVICE_INFO *dev_info;

                pthread_spinlock_t 	mngt_lock;

                int numQueue;
                struct nvmed_queue** queue_list;
                int usedQueues;

                pthread_t process_cq_td;
                volatile unsigned int process_cq_status;
                pthread_mutex_t process_cq_mutex;
                pthread_cond_t  process_cq_cond;

        } NVMED;

        typedef struct nvmed_queue {
                NVMED* 	nvmed;
                u32 	flags;

                pthread_spinlock_t 	mngt_lock;
                //pthread_spinlock_t 	sq_lock;
                pthread_spinlock_t 	cq_lock;
                pthread_mutex_t queue_lock;

                u16 qid;
                u32 q_depth;

                int sq_fd;
                int cq_fd;
                int db_fd;
                struct nvme_command 	*sq_cmds;
                volatile struct nvme_completion *cqes;
                volatile struct nvme_completion *cqe;
                void *dbs;
                u32 *sq_db;
                u32 *cq_db;

                u16 sq_head, sq_tail, cq_head;
                u8 cq_phase, cqe_seen;

                struct nvmed_iod* iod_arr;
                unsigned int	  iod_pos;

                u16 nextid;

                unsigned int aio_q_head;
        } NVMED_QUEUE;

        typedef struct nvmed_handle {
                struct nvmed_queue* queue;
                struct nvmed_queue** queue_mq;
                u32	flags;

                ssize_t (*read_func)(struct nvmed_handle*, u8, 
                                     void*, unsigned long, unsigned int, void*);
                ssize_t (*write_func)(struct nvmed_handle*, u8, 
                                      void*, unsigned long, unsigned int, void*);

                off_t	offset;
                off_t bufOffs;

                int num_mq;
                NVMED_QUEUE* (*mq_get_queue)(struct nvmed_handle*, u8, 
                                             unsigned long, unsigned int);

                pthread_spinlock_t prpBuf_lock;
                void** prpBuf;
                u64* pa_prpBuf;
                int prpBuf_size;
                int prpBuf_curr;
                int prpBuf_head;
                int prpBuf_tail;
        } NVMED_HANDLE;

        typedef struct nvmed_aio_ctx {
                NVMED_HANDLE* handle;
                off_t start_lba;
                size_t len;
                void* buf;
                u64* prpList;

                u8 opcode;
                volatile int status;
                int num_init_io;
                int num_complete_io;

                void* private_data;
                void* cb_userdata;
                void (*aio_callback)(const struct nvmed_aio_ctx *context, void *userdata);
        } NVMED_AIO_CTX;

        NVMED* nvmed_open(const char* PATH);
        int nvmed_close(NVMED*);
        int nvmed_feature_get(NVMED* nvmed, int feature);
        int nvmed_feature_set(NVMED* nvmed, int feature, int value);

        void nvmed_queue_create(NVMED*, int);
        int nvmed_queue_destroy(NVMED_QUEUE*);

        NVMED_HANDLE* nvmed_handle_create(NVMED_QUEUE*, int);
        int nvmed_handle_destroy(NVMED_HANDLE*);

        NVMED_HANDLE* nvmed_handle_create_mq(NVMED_QUEUE**, int, int,
                                             NVMED_QUEUE* (*func)(NVMED_HANDLE*, u8, unsigned long, unsigned int));
        int nvmed_handle_destroy_mq(NVMED_HANDLE*);

        int nvmed_handle_feature_get(NVMED_HANDLE*, int);
        int nvmed_handle_feature_set(NVMED_HANDLE*, int, int);

        void* nvmed_get_buffer(NVMED*, unsigned int num_pages);
        void nvmed_put_buffer(void*);

        off_t nvmed_lseek(NVMED_HANDLE*, off_t, int);
        ssize_t nvmed_read(NVMED_HANDLE*, void*, size_t);
        ssize_t nvmed_write(NVMED_HANDLE*, void*, size_t);

        int nvmed_flush(NVMED_HANDLE*);
        int nvmed_discard(NVMED_HANDLE*, unsigned long, unsigned int);

        int nvmed_aio_queue_submit(NVMED_HANDLE*);
        int nvmed_aio_read(NVMED_AIO_CTX*);
        int nvmed_aio_write(NVMED_AIO_CTX*);
        int nvmed_aio_handle_complete(NVMED_HANDLE*);

        int nvmed_set_user_quota(NVMED*, uid_t, unsigned int, 
                                 unsigned int*, unsigned int *);
        int nvmed_get_user_quota(NVMED*, uid_t, 
                                 unsigned int*, unsigned int *);

        int virt_to_phys(NVMED* nvmed, void* addr, u64* paArr, unsigned int num_bytes);

#ifdef __cplusplus
}
#endif

#endif /* _LIB_NVMED_H */

#ifndef _NVME_HDR_H
#define _NVME_HDR_H

#include <linux/types.h>

/* I/O commands */

enum nvme_opcode {
	nvme_cmd_flush		= 0x00,
	nvme_cmd_write		= 0x01,
	nvme_cmd_read		= 0x02,
	nvme_cmd_write_uncor	= 0x04,
	nvme_cmd_compare	= 0x05,
	nvme_cmd_write_zeroes	= 0x08,
	nvme_cmd_dsm		= 0x09,
	nvme_cmd_resv_register	= 0x0d,
	nvme_cmd_resv_report	= 0x0e,
	nvme_cmd_resv_acquire	= 0x11,
	nvme_cmd_resv_release	= 0x15,
};

struct nvme_common_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__le32			cdw2[2];
	__le64			metadata;
	__le64			prp1;
	__le64			prp2;
	__le32			cdw10[6];
};

struct nvme_rw_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2;
	__le64			metadata;
	__le64			prp1;
	__le64			prp2;
	__le64			slba;
	__le16			length;
	__le16			control;
	__le32			dsmgmt;
	__le32			reftag;
	__le16			apptag;
	__le16			appmask;
};

struct nvme_create_cq {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[5];
	__le64			prp1;
	__u64			rsvd8;
	__le16			cqid;
	__le16			qsize;
	__le16			cq_flags;
	__le16			irq_vector;
	__u32			rsvd12[4];
};

struct nvme_create_sq {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[5];
	__le64			prp1;
	__u64			rsvd8;
	__le16			sqid;
	__le16			qsize;
	__le16			sq_flags;
	__le16			cqid;
	__u32			rsvd12[4];
};

struct nvme_delete_queue {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[9];
	__le16			qid;
	__u16			rsvd10;
	__u32			rsvd11[5];
};

struct nvme_dsm_cmd {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[2];
	__le64			prp1;
	__le64			prp2;
	__le32			nr;
	__le32			attributes;
	__u32			rsvd12[4];
};

struct nvme_completion {
	__le32	result;		/* Used by admin commands to return data */
	__u32	rsvd;
	__le16	sq_head;	/* how much of this queue may be reclaimed */
	__le16	sq_id;		/* submission queue that generated this entry */
	__u16	command_id;	/* of the command which completed */
	__le16	status;		/* did the command fail, and if so, why? */
};

struct nvme_command {
	union {
		struct nvme_common_command common;
		struct nvme_rw_command rw;
		struct nvme_create_cq create_cq;
		struct nvme_create_sq create_sq;
		struct nvme_delete_queue delete_queue;
		struct nvme_dsm_cmd dsm;
	};
};

enum {
	NVME_DSMGMT_IDR		= 1 << 0,
	NVME_DSMGMT_IDW		= 1 << 1,
	NVME_DSMGMT_AD		= 1 << 2,
};

struct nvme_dsm_range {
	__le32			cattr;
	__le32			nlb;
	__le64			slba;
};

/* Admin commands */

enum nvme_admin_opcode {
	nvme_admin_delete_sq		= 0x00,
	nvme_admin_create_sq		= 0x01,
	nvme_admin_get_log_page		= 0x02,
	nvme_admin_delete_cq		= 0x04,
	nvme_admin_create_cq		= 0x05,
	nvme_admin_identify		= 0x06,
	nvme_admin_abort_cmd		= 0x08,
	nvme_admin_set_features		= 0x09,
	nvme_admin_get_features		= 0x0a,
	nvme_admin_async_event		= 0x0c,
	nvme_admin_activate_fw		= 0x10,
	nvme_admin_download_fw		= 0x11,
	nvme_admin_format_nvm		= 0x80,
	nvme_admin_security_send	= 0x81,
	nvme_admin_security_recv	= 0x82,
};

enum {
	NVME_QUEUE_PHYS_CONTIG	= (1 << 0),
	NVME_CQ_IRQ_ENABLED	= (1 << 1),
	NVME_SQ_PRIO_URGENT	= (0 << 1),
	NVME_SQ_PRIO_HIGH	= (1 << 1),
	NVME_SQ_PRIO_MEDIUM	= (2 << 1),
	NVME_SQ_PRIO_LOW	= (3 << 1),
	NVME_FEAT_ARBITRATION	= 0x01,
	NVME_FEAT_POWER_MGMT	= 0x02,
	NVME_FEAT_LBA_RANGE	= 0x03,
	NVME_FEAT_TEMP_THRESH	= 0x04,
	NVME_FEAT_ERR_RECOVERY	= 0x05,
	NVME_FEAT_VOLATILE_WC	= 0x06,
	NVME_FEAT_NUM_QUEUES	= 0x07,
	NVME_FEAT_IRQ_COALESCE	= 0x08,
	NVME_FEAT_IRQ_CONFIG	= 0x09,
	NVME_FEAT_WRITE_ATOMIC	= 0x0a,
	NVME_FEAT_ASYNC_EVENT	= 0x0b,
	NVME_FEAT_AUTO_PST	= 0x0c,
	NVME_FEAT_SW_PROGRESS	= 0x80,
	NVME_FEAT_HOST_ID	= 0x81,
	NVME_FEAT_RESV_MASK	= 0x82,
	NVME_FEAT_RESV_PERSIST	= 0x83,
	NVME_LOG_ERROR		= 0x01,
	NVME_LOG_SMART		= 0x02,
	NVME_LOG_FW_SLOT	= 0x03,
	NVME_LOG_RESERVATION	= 0x80,
	NVME_FWACT_REPL		= (0 << 3),
	NVME_FWACT_REPL_ACTV	= (1 << 3),
	NVME_FWACT_ACTV		= (2 << 3),
};

#endif


