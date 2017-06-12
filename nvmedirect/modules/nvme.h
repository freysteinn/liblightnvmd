#ifndef _NVMED_NVME_HEADER_H
#define _NVMED_NVME_HEADER_H

#include <linux/blk-mq.h>

#ifdef KERN_440

struct nvme_dev {
	struct list_head node;
	struct nvme_queue **queues;
	struct request_queue *admin_q;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	int instance;
	unsigned queue_count;
	unsigned online_queues;
	unsigned max_qid;
	int q_depth;
	u32 db_stride;
	u32 ctrl_config;
	struct msix_entry *entry;
	struct nvme_bar __iomem *bar;
	struct list_head namespaces;
	struct kref kref;
	struct device *device;
	struct work_struct reset_work;
	struct work_struct probe_work;
	struct work_struct scan_work;
	char name[12];
	char serial[20];
	char model[40];
	char firmware_rev[8];
	bool subsystem;
	u32 max_hw_sectors;
	u32 stripe_size;
	u32 page_size;
	void __iomem *cmb;
	dma_addr_t cmb_dma_addr;
	u64 cmb_size;
	u32 cmbsz;
	u16 oncs;
	u16 abort_limit;
	u8 event_limit;
	u8 vwc;
};

struct nvme_ns {
	struct list_head list;

	struct nvme_dev *dev;
	struct request_queue *queue;
	struct gendisk *disk;
	struct kref kref;

	unsigned ns_id;
	int lba_shift;
	u16 ms;
	bool ext;
	u8 pi_type;
	int type;
	u64 mode_select_num_blocks;
	u32 mode_select_block_len;
};
#endif //KERN_440

#ifdef KERN_450

struct nvme_ctrl {
	const struct nvme_ctrl_ops *ops;
	struct request_queue *admin_q;
	struct device *dev;
	struct kref kref;
	int instance;
	struct blk_mq_tag_set *tagset;
	struct list_head namespaces;
	struct mutex namespaces_mutex;
	struct device *device;	/* char device */
	struct list_head node;
	struct ida ns_ida;

	char name[12];
	char serial[20];
	char model[40];
	char firmware_rev[8];

	u32 ctrl_config;

	u32 page_size;
	u32 max_hw_sectors;
	u32 stripe_size;
	u16 oncs;
	atomic_t abort_limit;
	u8 event_limit;
	u8 vwc;
	u32 vs;
	bool subsystem;
	unsigned long quirks;
};

struct nvme_ns {
	struct list_head list;

	struct nvme_ctrl *ctrl;
	struct request_queue *queue;
	struct gendisk *disk;
	struct kref kref;
	int instance;

	u8 eui[8];
	u8 uuid[16];

	unsigned ns_id;
	int lba_shift;
	u16 ms;
	bool ext;
	u8 pi_type;
	int type;
	unsigned long flags;

#define NVME_NS_REMOVING 0
#define NVME_NS_DEAD     1

	u64 mode_select_num_blocks;
	u32 mode_select_block_len;
};

struct nvme_dev {
	struct list_head node;
	struct nvme_queue **queues;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	unsigned queue_count;
	unsigned online_queues;
	unsigned max_qid;
	int q_depth;
	u32 db_stride;
	struct msix_entry *entry;
	void __iomem *bar;
	struct work_struct reset_work;
	struct work_struct scan_work;
	struct work_struct remove_work;
	struct mutex shutdown_lock;
	bool subsystem;
	void __iomem *cmb;
	dma_addr_t cmb_dma_addr;
	u64 cmb_size;
	u32 cmbsz;
	unsigned long flags;

#define NVME_CTRL_RESETTING    0
#define NVME_CTRL_REMOVING     1

	struct nvme_ctrl ctrl;
	struct completion ioq_wait;
};

#endif //KERN_450

#ifdef KERN_460
struct nvme_ctrl {
	const struct nvme_ctrl_ops *ops;
	struct request_queue *admin_q;
	struct device *dev;
	struct kref kref;
	int instance;
	struct blk_mq_tag_set *tagset;
	struct list_head namespaces;
	struct mutex namespaces_mutex;
	struct device *device;	/* char device */
	struct list_head node;
	struct ida ns_ida;

	char name[12];
	char serial[20];
	char model[40];
	char firmware_rev[8];
	int cntlid;

	u32 ctrl_config;

	u32 page_size;
	u32 max_hw_sectors;
	u32 stripe_size;
	u16 oncs;
	u16 vid;
	atomic_t abort_limit;
	u8 event_limit;
	u8 vwc;
	u32 vs;
	bool subsystem;
	unsigned long quirks;
};

/*
 * An NVM Express namespace is equivalent to a SCSI LUN
 */
struct nvme_ns {
	struct list_head list;

	struct nvme_ctrl *ctrl;
	struct request_queue *queue;
	struct gendisk *disk;
	struct kref kref;
	int instance;

	u8 eui[8];
	u8 uuid[16];

	unsigned ns_id;
	int lba_shift;
	u16 ms;
	bool ext;
	u8 pi_type;
	int type;
	unsigned long flags;

#define NVME_NS_REMOVING 0
#define NVME_NS_DEAD     1

	u64 mode_select_num_blocks;
	u32 mode_select_block_len;
};

struct nvme_dev {
	struct nvme_queue **queues;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	unsigned queue_count;
	unsigned online_queues;
	unsigned max_qid;
	int q_depth;
	u32 db_stride;
	struct msix_entry *entry;
	void __iomem *bar;
	struct work_struct reset_work;
	struct work_struct scan_work;
	struct work_struct remove_work;
	struct work_struct async_work;
	struct timer_list watchdog_timer;
	struct mutex shutdown_lock;
	bool subsystem;
	void __iomem *cmb;
	dma_addr_t cmb_dma_addr;
	u64 cmb_size;
	u32 cmbsz;
	unsigned long flags;

#define NVME_CTRL_RESETTING    0
#define NVME_CTRL_REMOVING     1

	struct nvme_ctrl ctrl;
	struct completion ioq_wait;
};

#endif //KERN_460

#ifdef KERN_470
struct nvme_ctrl {
	int state;
	spinlock_t lock;
	const struct nvme_ctrl_ops *ops;
	struct request_queue *admin_q;
	struct device *dev;
	struct kref kref;
	int instance;
	struct blk_mq_tag_set *tagset;
	struct list_head namespaces;
	struct mutex namespaces_mutex;
	struct device *device;	/* char device */
	struct list_head node;
	struct ida ns_ida;

	char name[12];
	char serial[20];
	char model[40];
	char firmware_rev[8];
	u16 cntlid;

	u32 ctrl_config;

	u32 page_size;
	u32 max_hw_sectors;
	u32 stripe_size;
	u16 oncs;
	u16 vid;
	atomic_t abort_limit;
	u8 event_limit;
	u8 vwc;
	u32 vs;
	bool subsystem;
	unsigned long quirks;
	struct work_struct scan_work;
	struct work_struct async_event_work;
};

/*
 * An NVM Express namespace is equivalent to a SCSI LUN
 */
struct nvme_ns {
	struct list_head list;

	struct nvme_ctrl *ctrl;
	struct request_queue *queue;
	struct gendisk *disk;
	struct kref kref;
	int instance;

	u8 eui[8];
	u8 uuid[16];

	unsigned ns_id;
	int lba_shift;
	u16 ms;
	bool ext;
	u8 pi_type;
	int type;
	unsigned long flags;

#define NVME_NS_REMOVING 0
#define NVME_NS_DEAD     1

	u64 mode_select_num_blocks;
	u32 mode_select_block_len;
};

struct nvme_dev {
	struct nvme_queue **queues;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	unsigned queue_count;
	unsigned online_queues;
	unsigned max_qid;
	int q_depth;
	u32 db_stride;
	struct msix_entry *entry;
	void __iomem *bar;
	struct work_struct reset_work;
	struct work_struct remove_work;
	struct timer_list watchdog_timer;
	struct mutex shutdown_lock;
	bool subsystem;
	void __iomem *cmb;
	dma_addr_t cmb_dma_addr;
	u64 cmb_size;
	u32 cmbsz;
	struct nvme_ctrl ctrl;
	struct completion ioq_wait;
};

#endif //KERN_470

#ifdef KERN_480
struct nvme_ctrl {
	int state;
	spinlock_t lock;
	const struct nvme_ctrl_ops *ops;
	struct request_queue *admin_q;
	struct request_queue *connect_q;
	struct device *dev;
	struct kref kref;
	int instance;
	struct blk_mq_tag_set *tagset;
	struct list_head namespaces;
	struct mutex namespaces_mutex;
	struct device *device;	/* char device */
	struct list_head node;
	struct ida ns_ida;

	char name[12];
	char serial[20];
	char model[40];
	char firmware_rev[8];
	u16 cntlid;

	u32 ctrl_config;

	u32 page_size;
	u32 max_hw_sectors;
	u32 stripe_size;
	u16 oncs;
	u16 vid;
	atomic_t abort_limit;
	u8 event_limit;
	u8 vwc;
	u32 vs;
	u32 sgls;
	u16 kas;
	unsigned int kato;
	bool subsystem;
	unsigned long quirks;
	struct work_struct scan_work;
	struct work_struct async_event_work;
	struct delayed_work ka_work;

	/* Fabrics only */
	u16 sqsize;
	u32 ioccsz;
	u32 iorcsz;
	u16 icdoff;
	u16 maxcmd;
	struct nvmf_ctrl_options *opts;
};

/*
 * An NVM Express namespace is equivalent to a SCSI LUN
 */
struct nvme_ns {
	struct list_head list;

	struct nvme_ctrl *ctrl;
	struct request_queue *queue;
	struct gendisk *disk;
	struct kref kref;
	int instance;

	u8 eui[8];
	u8 uuid[16];

	unsigned ns_id;
	int lba_shift;
	u16 ms;
	bool ext;
	u8 pi_type;
	int type;
	unsigned long flags;

#define NVME_NS_REMOVING 0
#define NVME_NS_DEAD     1

	u64 mode_select_num_blocks;
	u32 mode_select_block_len;
};

struct nvme_dev {
	struct nvme_queue **queues;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	unsigned queue_count;
	unsigned online_queues;
	unsigned max_qid;
	int q_depth;
	u32 db_stride;
	struct msix_entry *entry;
	void __iomem *bar;
	struct work_struct reset_work;
	struct work_struct remove_work;
	struct timer_list watchdog_timer;
	struct mutex shutdown_lock;
	bool subsystem;
	void __iomem *cmb;
	dma_addr_t cmb_dma_addr;
	u64 cmb_size;
	u32 cmbsz;
	struct nvme_ctrl ctrl;
	struct completion ioq_wait;
};

#endif // KERN_480

#ifdef KERN_490
enum nvme_ctrl_state {
	NVME_CTRL_NEW,
	NVME_CTRL_LIVE,
	NVME_CTRL_RESETTING,
	NVME_CTRL_RECONNECTING,
	NVME_CTRL_DELETING,
	NVME_CTRL_DEAD,
};

struct nvme_ctrl {
	enum nvme_ctrl_state state;
	spinlock_t lock;
	const struct nvme_ctrl_ops *ops;
	struct request_queue *admin_q;
	struct request_queue *connect_q;
	struct device *dev;
	struct kref kref;
	int instance;
	struct blk_mq_tag_set *tagset;
	struct list_head namespaces;
	struct mutex namespaces_mutex;
	struct device *device;	/* char device */
	struct list_head node;
	struct ida ns_ida;

	char name[12];
	char serial[20];
	char model[40];
	char firmware_rev[8];
	u16 cntlid;

	u32 ctrl_config;

	u32 page_size;
	u32 max_hw_sectors;
	u32 stripe_size;
	u16 oncs;
	u16 vid;
	atomic_t abort_limit;
	u8 event_limit;
	u8 vwc;
	u32 vs;
	u32 sgls;
	u16 kas;
	unsigned int kato;
	bool subsystem;
	unsigned long quirks;
	struct work_struct scan_work;
	struct work_struct async_event_work;
	struct delayed_work ka_work;

	/* Fabrics only */
	u16 sqsize;
	u32 ioccsz;
	u32 iorcsz;
	u16 icdoff;
	u16 maxcmd;
	struct nvmf_ctrl_options *opts;
};

/*
 * An NVM Express namespace is equivalent to a SCSI LUN
 */
struct nvme_ns {
	struct list_head list;

	struct nvme_ctrl *ctrl;
	struct request_queue *queue;
	struct gendisk *disk;
	struct nvm_dev *ndev;
	struct kref kref;
	int instance;

	u8 eui[8];
	u8 uuid[16];

	unsigned ns_id;
	int lba_shift;
	u16 ms;
	bool ext;
	u8 pi_type;
	unsigned long flags;

#define NVME_NS_REMOVING 0
#define NVME_NS_DEAD     1

	u64 mode_select_num_blocks;
	u32 mode_select_block_len;
};

/*
 * Represents an NVM Express device.  Each nvme_dev is a PCI function.
 */
struct nvme_dev {
	struct nvme_queue **queues;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	unsigned queue_count;
	unsigned online_queues;
	unsigned max_qid;
	int q_depth;
	u32 db_stride;
	void __iomem *bar;
	struct work_struct reset_work;
	struct work_struct remove_work;
	struct timer_list watchdog_timer;
	struct mutex shutdown_lock;
	bool subsystem;
	void __iomem *cmb;
	dma_addr_t cmb_dma_addr;
	u64 cmb_size;
	u32 cmbsz;
	u32 cmbloc;
	struct nvme_ctrl ctrl;
	struct completion ioq_wait;
};

#endif // KERN_490

#ifdef KERN_4100
enum nvme_ctrl_state {
	NVME_CTRL_NEW,
	NVME_CTRL_LIVE,
	NVME_CTRL_RESETTING,
	NVME_CTRL_RECONNECTING,
	NVME_CTRL_DELETING,
	NVME_CTRL_DEAD,
};

struct nvme_ctrl {
	enum nvme_ctrl_state state;
	spinlock_t lock;
	const struct nvme_ctrl_ops *ops;
	struct request_queue *admin_q;
	struct request_queue *connect_q;
	struct device *dev;
	struct kref kref;
	int instance;
	struct blk_mq_tag_set *tagset;
	struct list_head namespaces;
	struct mutex namespaces_mutex;
	struct device *device;	/* char device */
	struct list_head node;
	struct ida ns_ida;

	char name[12];
	char serial[20];
	char model[40];
	char firmware_rev[8];
	u16 cntlid;

	u32 ctrl_config;

	u32 page_size;
	u32 max_hw_sectors;
	u16 oncs;
	u16 vid;
	atomic_t abort_limit;
	u8 event_limit;
	u8 vwc;
	u32 vs;
	u32 sgls;
	u16 kas;
	unsigned int kato;
	bool subsystem;
	unsigned long quirks;
	struct work_struct scan_work;
	struct work_struct async_event_work;
	struct delayed_work ka_work;

	/* Fabrics only */
	u16 sqsize;
	u32 ioccsz;
	u32 iorcsz;
	u16 icdoff;
	u16 maxcmd;
	struct nvmf_ctrl_options *opts;
};

/*
 * An NVM Express namespace is equivalent to a SCSI LUN
 */
struct nvme_ns {
	struct list_head list;

	struct nvme_ctrl *ctrl;
	struct request_queue *queue;
	struct gendisk *disk;
	struct nvm_dev *ndev;
	struct kref kref;
	int instance;

	u8 eui[8];
	u8 uuid[16];

	unsigned ns_id;
	int lba_shift;
	u16 ms;
	bool ext;
	u8 pi_type;
	unsigned long flags;

#define NVME_NS_REMOVING 0
#define NVME_NS_DEAD     1

	u64 mode_select_num_blocks;
	u32 mode_select_block_len;
};
/*
 * Represents an NVM Express device.  Each nvme_dev is a PCI function.
 */
struct nvme_dev {
	struct nvme_queue **queues;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	unsigned queue_count;
	unsigned online_queues;
	unsigned max_qid;
	int q_depth;
	u32 db_stride;
	void __iomem *bar;
	struct work_struct reset_work;
	struct work_struct remove_work;
	struct timer_list watchdog_timer;
	struct mutex shutdown_lock;
	bool subsystem;
	void __iomem *cmb;
	dma_addr_t cmb_dma_addr;
	u64 cmb_size;
	u32 cmbsz;
	u32 cmbloc;
	struct nvme_ctrl ctrl;
	struct completion ioq_wait;
};

#endif // KERN_4100

#ifdef KERN_4110
enum nvme_ctrl_state {
	NVME_CTRL_NEW,
	NVME_CTRL_LIVE,
	NVME_CTRL_RESETTING,
	NVME_CTRL_RECONNECTING,
	NVME_CTRL_DELETING,
	NVME_CTRL_DEAD,
};

struct nvme_ctrl {
	enum nvme_ctrl_state state;
	bool identified;
	spinlock_t lock;
	const struct nvme_ctrl_ops *ops;
	struct request_queue *admin_q;
	struct request_queue *connect_q;
	struct device *dev;
	struct kref kref;
	int instance;
	struct blk_mq_tag_set *tagset;
	struct list_head namespaces;
	struct mutex namespaces_mutex;
	struct device *device;	/* char device */
	struct list_head node;
	struct ida ns_ida;

	struct opal_dev *opal_dev;

	char name[12];
	char serial[20];
	char model[40];
	char firmware_rev[8];
	u16 cntlid;

	u32 ctrl_config;

	u32 page_size;
	u32 max_hw_sectors;
	u16 oncs;
	u16 vid;
	u16 oacs;
	atomic_t abort_limit;
	u8 event_limit;
	u8 vwc;
	u32 vs;
	u32 sgls;
	u16 kas;
	u8 npss;
	u8 apsta;
	unsigned int kato;
	bool subsystem;
	unsigned long quirks;
	//struct nvme_id_power_state psd[32];
	struct work_struct scan_work;
	struct work_struct async_event_work;
	struct delayed_work ka_work;

	/* Power saving configuration */
	u64 ps_max_latency_us;

	/* Fabrics only */
	u16 sqsize;
	u32 ioccsz;
	u32 iorcsz;
	u16 icdoff;
	u16 maxcmd;
	struct nvmf_ctrl_options *opts;
};

struct nvm_addr_format {
	u8	ch_offset;
	u8	ch_len;
	u8	lun_offset;
	u8	lun_len;
	u8	pln_offset;
	u8	pln_len;
	u8	blk_offset;
	u8	blk_len;
	u8	pg_offset;
	u8	pg_len;
	u8	sect_offset;
	u8	sect_len;
};

/* Device generic information */
struct nvm_geo {
	int nr_chnls;
	int nr_luns;
	int luns_per_chnl; /* -1 if channels are not symmetric */
	int nr_planes;
	int sec_per_pg; /* only sectors for a single page */
	int pgs_per_blk;
	int blks_per_lun;
	int fpg_size;
	int pfpg_size; /* size of buffer if all pages are to be read */
	int sec_size;
	int oob_size;
	int mccap;
	struct nvm_addr_format ppaf;

	/* Calculated/Cached values. These do not reflect the actual usable
	 * blocks at run-time.
	 */
	int max_rq_size;
	int plane_mode; /* drive device in single, double or quad mode */

	int sec_per_pl; /* all sectors across planes */
	int sec_per_blk;
	int sec_per_lun;
};

struct nvm_id_lp_mlc {
	u16	num_pairs;
	u8	pairs[886];
};

struct nvm_id_lp_tbl {
	__u8	id[8];
	struct nvm_id_lp_mlc mlc;
};

struct nvm_id_group {
	u8	mtype;
	u8	fmtype;
	u8	num_ch;
	u8	num_lun;
	u8	num_pln;
	u16	num_blk;
	u16	num_pg;
	u16	fpg_sz;
	u16	csecs;
	u16	sos;
	u32	trdt;
	u32	trdm;
	u32	tprt;
	u32	tprm;
	u32	tbet;
	u32	tbem;
	u32	mpos;
	u32	mccap;
	u16	cpar;

	struct nvm_id_lp_tbl lptbl;
};

struct nvm_id {
	u8	ver_id;
	u8	vmnt;
	u32	cap;
	u32	dom;
	struct nvm_addr_format ppaf;
	struct nvm_id_group grp;
} __packed;

struct nvm_dev {
	struct nvm_dev_ops *ops;

	struct list_head devices;

	/* Device information */
	struct nvm_geo geo;

        /* lower page table */
	int lps_per_blk;
	int *lptbl;

	unsigned long total_secs;

	unsigned long *lun_map;
	void *dma_pool;

	struct nvm_id identity;

	/* Backend device */
	struct request_queue *q;
	char name[DISK_NAME_LEN];
	void *private_data;

	void *rmap;

	struct mutex mlock;
	spinlock_t lock;

	/* target management */
	struct list_head area_list;
	struct list_head targets;
};

/*
 * An NVM Express namespace is equivalent to a SCSI LUN
 */
struct nvme_ns {
	struct list_head list;

	struct nvme_ctrl *ctrl;
	struct request_queue *queue;
	struct gendisk *disk;
	struct nvm_dev *ndev;
	struct kref kref;
	int instance;

	u8 eui[8];
	u8 uuid[16];

	unsigned ns_id;
	int lba_shift;
	u16 ms;
	bool ext;
	u8 pi_type;
	unsigned long flags;

#define NVME_NS_REMOVING 0
#define NVME_NS_DEAD     1

	u64 mode_select_num_blocks;
	u32 mode_select_block_len;
};

/*
 * Represents an NVM Express device.  Each nvme_dev is a PCI function.
 */
struct nvme_dev {
	struct nvme_queue **queues;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	unsigned queue_count;
	unsigned online_queues;
	unsigned max_qid;
	int q_depth;
	u32 db_stride;
	void __iomem *bar;
	struct work_struct reset_work;
	struct work_struct remove_work;
	struct timer_list watchdog_timer;
	struct mutex shutdown_lock;
	bool subsystem;
	void __iomem *cmb;
	dma_addr_t cmb_dma_addr;
	u64 cmb_size;
	u32 cmbsz;
	u32 cmbloc;
	struct nvme_ctrl ctrl;
	struct completion ioq_wait;
};
#endif // KERNL_4110

#endif //_NVMED_NVME_HEADER_H
