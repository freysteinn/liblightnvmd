/*
 * be_ioctl - Backend using the nvmedirect interface
 *
 * Copyright (C) 2017 Frey Alfredsson <frea@itu.dk>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define _GNU_SOURCE
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/lightnvm.h>
#include <linux/nvme_ioctl.h>
#include <liblightnvm.h>
#include <nvm_be.h>
#include <nvm_dev.h>
#include <nvm_debug.h>
#include <nvmed.h>

#include <stdio.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <pthread.h>
#include <endian.h>

/*
 * Create User-space I/O queue and map to User virtual address
 */
void nvmed_queue_create(NVMED* nvmed, int flags)
{
        int ret;
        NVMED_QUEUE* nvmed_queue = 0;
        char pathBase[1024];
        char pathBuf[1024];
        u32 *q_dbs;
        u32	qid;

        pthread_spin_lock(&nvmed->mngt_lock);

        /* Request Create I/O Queues */
        ret = ioctl(nvmed->ns_fd, NVMED_IOCTL_QUEUE_CREATE, &qid);
        if (ret < 0) {
                printf("%s: fail to create I/O queue\n", nvmed->ns_path);
                exit(1);
        }

        nvmed_queue = malloc(sizeof(NVMED_QUEUE));
        nvmed_queue->nvmed = nvmed;
        nvmed_queue->flags = flags;
        nvmed_queue->qid = qid;

        if (nvmed->dev_info->part_no != 0) {
                sprintf(pathBase, "/proc/nvmed/nvme%dn%dp%d/%d",
                        nvmed->dev_info->instance, nvmed->dev_info->ns_id, nvmed->dev_info->part_no, nvmed_queue->qid);
        }
        else {
                sprintf(pathBase, "/proc/nvmed/nvme%dn%d/%d",
                        nvmed->dev_info->instance, nvmed->dev_info->ns_id, nvmed_queue->qid);
        }

        /* Map SQ */
        sprintf(pathBuf, "%s/sq", pathBase);
        nvmed_queue->sq_fd = open(pathBuf, O_RDWR);
        nvmed_queue->sq_cmds = mmap(0, SQ_SIZE(nvmed->dev_info->q_depth), 
                                    PROT_READ | PROT_WRITE, MAP_SHARED, nvmed_queue->sq_fd, 0);

        /* Map CQ */
        sprintf(pathBuf, "%s/cq", pathBase);
        nvmed_queue->cq_fd = open(pathBuf, O_RDWR);
        nvmed_queue->cqes = mmap(0, CQ_SIZE(nvmed->dev_info->q_depth), 
                                 PROT_READ | PROT_WRITE, MAP_SHARED, nvmed_queue->cq_fd, 0);

        /* Map DQ */
        sprintf(pathBuf, "%s/db", pathBase);
        nvmed_queue->db_fd = open(pathBuf, O_RDWR);
        nvmed_queue->dbs = mmap(0, PAGE_SIZE*2, PROT_READ | PROT_WRITE, MAP_SHARED, nvmed_queue->db_fd, 0);

        q_dbs = (void*)((char*)nvmed_queue->dbs + PAGE_SIZE);
        nvmed_queue->sq_db = &q_dbs[nvmed_queue->qid * 2 * nvmed->dev_info->db_stride];
        nvmed_queue->cq_db = &q_dbs[(nvmed_queue->qid * 2 * nvmed->dev_info->db_stride) + nvmed->dev_info->db_stride];
        nvmed_queue->sq_head = 0;
        nvmed_queue->sq_tail = 0;
        nvmed_queue->cq_head = 0;
        nvmed_queue->cq_phase = 1;

        struct nvm_cmd *cmnd = (void*)&nvmed_queue->sq_cmds[nvmed_queue->sq_tail];
        memset(cmnd, 0, 64);

        pthread_spin_init(&nvmed_queue->mngt_lock, 0);
        //pthread_spin_init(&nvmed_queue->sq_lock, 0);
        pthread_spin_init(&nvmed_queue->cq_lock, 0);
        pthread_mutex_init(&nvmed_queue->queue_lock, NULL);

        nvmed->queue_list[nvmed->numQueue++] = nvmed_queue;

        nvmed_queue->nextid = 2 * random() + 1;

        pthread_spin_unlock(&nvmed->mngt_lock);
}

/*
 * Destroy User-space I/O queue
 */
int nvmed_queue_destroy(NVMED_QUEUE* nvmed_queue)
{
        NVMED* nvmed;
        int ret;

        if (nvmed_queue == NULL)
                return -NVMED_NOENTRY;

        nvmed = nvmed_queue->nvmed;

        pthread_spin_lock(&nvmed->mngt_lock);

        munmap(nvmed_queue->dbs, PAGE_SIZE * 2);
        close(nvmed_queue->db_fd);

        munmap((void *)nvmed_queue->cqes, CQ_SIZE(nvmed->dev_info->q_depth));
        close(nvmed_queue->cq_fd);

        munmap(nvmed_queue->sq_cmds, SQ_SIZE(nvmed->dev_info->q_depth));
        close(nvmed_queue->sq_fd);

        pthread_spin_destroy(&nvmed_queue->mngt_lock);
        //pthread_spin_destroy(&nvmed_queue->sq_lock);
        pthread_spin_destroy(&nvmed_queue->cq_lock);
        pthread_mutex_destroy(&nvmed_queue->queue_lock);

        ret = ioctl(nvmed->ns_fd, NVMED_IOCTL_QUEUE_DELETE, &nvmed_queue->qid);
        if (ret == 0) {
                pthread_mutex_lock(&nvmed->process_cq_mutex);
                pthread_cond_signal(&nvmed->process_cq_cond);
                pthread_mutex_unlock(&nvmed->process_cq_mutex);

                nvmed->numQueue--;
        }

        free(nvmed_queue);

        pthread_spin_unlock(&nvmed->mngt_lock);

        return NVMED_SUCCESS;
}

/*
 * Translate /dev/nvmeXnY -> /proc/nvmed/nvmeXnY/admin
 *							or /proc/nvmed/nvmeXnYpZ/admin
 */
int get_path_from_blkdev(const char* blkdev, char** admin_path)
{
        char temp_path[16];
        char *proc_path;
        int path_len;

        strcpy(temp_path, blkdev+9);
        path_len = 23;
        path_len+= strlen(temp_path);

        proc_path = malloc(sizeof(char) * path_len);
        sprintf(proc_path, "/proc/nvmed/nvme%s/admin", temp_path);

        if (access(proc_path, F_OK) < 0) {
                free(proc_path);
                return -NVMED_NOENTRY;
        }

        *admin_path = proc_path;
        return NVMED_SUCCESS;
}

/*
 * Open NVMe device
 */
NVMED* nvmed_open(const char* path)
{
        char* admin_path;
        int result;
        NVMED_DEVICE_INFO *dev_info;
        NVMED* nvmed;
        int adminfd;
        int ret;

        result = get_path_from_blkdev(path, &admin_path);
        if (result < 0) {
                printf("%s: fail to open nvme device file\n", path);
                return NULL;
        }

        adminfd = open(admin_path, 0);
        if (adminfd < 0) {
                printf("%s: fail to open nvme device file\n", admin_path);
                return NULL;
        }

        //IOCTL - Get Device Info
        dev_info = malloc(sizeof(*dev_info));
        ret = ioctl(adminfd, NVMED_IOCTL_NVMED_INFO, dev_info);
        if (ret < 0) {
                close(adminfd);
                return NULL;
        }

        //nvmed = malloc(sizeof(NVMED));
        nvmed = malloc(sizeof(*nvmed));
        if (nvmed == NULL) {
                free(admin_path);
                free(dev_info);
                close(adminfd);
                printf("%s: fail to allocation nvmed buffer\n", admin_path);
                return NULL;
        }

        // Getting NVMe Device Info
        nvmed->ns_path = admin_path;
        nvmed->ns_fd = adminfd;
        nvmed->dev_info = dev_info;

        // QUEUE
        nvmed->numQueue = 0;
        nvmed->usedQueues = 0;

        pthread_spin_init(&nvmed->mngt_lock, 0);

        pthread_cond_init(&nvmed->process_cq_cond, NULL);
        pthread_mutex_init(&nvmed->process_cq_mutex, NULL);

        return nvmed;
}

/*
 * Translate virtual memory address to physical memory address
 */
int virt_to_phys(NVMED* nvmed, void* addr, u64* paArr, unsigned int num_pages)
{
        struct nvmed_buf nvmed_buf;
        int ret = 0;

        nvmed_buf.addr = addr;
        nvmed_buf.size = num_pages;
        nvmed_buf.pfnList = paArr;

        ret = ioctl(nvmed->ns_fd, NVMED_IOCTL_GET_BUFFER_ADDR, &nvmed_buf);
        if (ret < 0)
                return -1;

        return 0;
}

static inline int nvme_is_write(struct nvme_command *cmd)
{
        return cmd->common.opcode & 1;
}

struct nvmed_dma {
        void* buffer_dma;
        u64* buffer_phy;

        void* prp2_dma;
        u64* prp2_phy;

        void* ppa_dma;
        u64* ppa_phy;

        void* meta_dma;
        u64* meta_phy;

        int buffer_page_count;
        int prp2_page_count;
        int ppa_page_count;
        int meta_page_count;

        NVMED_QUEUE* nvmed_queue;
};

static pthread_key_t dma_key;
static pthread_once_t dma_key_init = PTHREAD_ONCE_INIT;
void nvmed_dma_cleanup(void *arg)
{
        struct nvmed_dma* dma = arg;

        munmap(dma->buffer_dma, dma->buffer_page_count * PAGE_SIZE);
        free(dma->buffer_phy);

        munmap(dma->prp2_dma, dma->prp2_page_count * PAGE_SIZE);
        free(dma->prp2_phy);

        munmap(dma->meta_dma, dma->meta_page_count * PAGE_SIZE);
        free(dma->meta_phy);

        munmap(dma->ppa_dma, dma->ppa_page_count * PAGE_SIZE);
        free(dma->ppa_phy);

        free(dma);
}

void init_thread_dma()
{
        pthread_key_create(&dma_key, nvmed_dma_cleanup);
}

void* get_dma_mmap(int page_count)
{
        return mmap(NULL, page_count * PAGE_SIZE, PROT_READ | PROT_WRITE,
                    MAP_ANONYMOUS | MAP_LOCKED | MAP_SHARED, -1, 0);
}

struct nvmed_dma* get_dma(struct nvm_dev *dev)
{
        const int MAX_PAGES = 64;
        NVMED* nvmed = dev->nvmed;
        struct nvmed_dma* dma;
        int queue = 0;

        pthread_once(&dma_key_init, init_thread_dma);
        if ((dma = pthread_getspecific(dma_key)) == NULL) {
                dma = malloc(sizeof(*dma));

                // Handle buffer DMA
                dma->buffer_page_count = MAX_PAGES;
                dma->buffer_dma = get_dma_mmap(dma->buffer_page_count);
                dma->buffer_phy = malloc(sizeof(u64) * dma->buffer_page_count);

                // Handle prp2 DMA
                dma->prp2_page_count = (sizeof(u64) * (MAX_PAGES - 1) + PAGE_SIZE - 1) / PAGE_SIZE;
                dma->prp2_dma = get_dma_mmap(dma->prp2_page_count);
                dma->prp2_phy = malloc(sizeof(u64) * dma->prp2_page_count);

                // Handle meta DMA
                dma->meta_page_count = sizeof(u64) * (dev->geo.meta_nbytes + PAGE_SIZE - 1) / PAGE_SIZE;
                dma->meta_dma = get_dma_mmap(dma->meta_page_count);
                dma->meta_phy = malloc(sizeof(u64) * dma->meta_page_count);

                // Handle ppa DMA
                dma->ppa_page_count = MAX_PAGES;
                dma->ppa_dma = get_dma_mmap(dma->ppa_page_count);
                dma->ppa_phy = malloc(sizeof(u64) * dma->buffer_page_count);

                // Map virtual to physical addresses
                virt_to_phys(nvmed, dma->buffer_dma, dma->buffer_phy, dma->buffer_page_count);
                virt_to_phys(nvmed, dma->prp2_dma, dma->prp2_phy, dma->prp2_page_count);
                virt_to_phys(nvmed, dma->meta_dma, dma->meta_phy, dma->meta_page_count);
                virt_to_phys(nvmed, dma->ppa_dma, dma->ppa_phy, dma->ppa_page_count);
                memcpy(dma->prp2_dma, ((u64*)dma->buffer_phy) + 1, sizeof(u64) * (dma->buffer_page_count - 1));

                queue = nvmed->usedQueues % nvmed->dev_info->nr_queues;
                dma->nvmed_queue = nvmed->queue_list[queue];
                nvmed->usedQueues++;

                pthread_setspecific(dma_key, dma);
        }
        return dma;
}

/*
 * Send I/O to submission queue and ring SQ Doorbell
 */
int nvm_be_send_cmd(struct nvm_dev *dev, struct nvm_cmd *vcmd,
                    struct nvm_ret *result)
{
        struct nvmed_dma* dma = get_dma(dev);
        NVMED* nvmed = dev->nvmed;
        NVMED_QUEUE* nvmed_queue = dma->nvmed_queue;
        volatile struct nvme_completion *cqe;
        struct nvme_command *c;
        u16 head, phase;
        int buffer_size = (vcmd->vuser.nppas + 1) * PAGE_SIZE;

        if (vcmd->vuser.flags) {
                return -1;
        }

        // Submission
        pthread_spin_lock(&nvmed_queue->cq_lock);
        //pthread_spin_lock(&nvmed_queue->sq_lock);

        //pthread_mutex_lock(&nvmed_queue->queue_lock);

        c = &nvmed_queue->sq_cmds[nvmed_queue->sq_tail];

        /* NVM */
        memset(c, 0, sizeof(*c));
        c->common.opcode = vcmd->vuser.opcode;
        c->common.nsid = htole32(nvmed->dev_info->ns_id);
        ((struct nvm_cmd*)c)->vadmin.nppas = htole16(vcmd->vuser.nppas);
        ((struct nvm_cmd*)c)->vadmin.control = htole16(vcmd->vuser.control);

        c->rw.command_id = nvmed_queue->nextid;
        nvmed_queue->nextid = nvmed_queue->nextid + 2;

        // nvm_cmd_vuser_pr(vcmd);

        // TODO: Error if flags is non zero
        switch (vcmd->vuser.opcode) {
        case 0x90: // Erase
                break;
        case 0x91: // Write
                // Handle metadata
                if (vcmd->vuser.metadata && vcmd->vuser.metadata_len) {
                        memcpy(dma->meta_dma, (void*)vcmd->vuser.metadata, vcmd->vuser.metadata_len);
                }

                // Handle buffer
                if (vcmd->vuser.addr) {
                        memcpy(dma->buffer_dma, (void*)vcmd->vuser.addr, buffer_size);
                }
        case 0x92: // Read
                // Handle metadata
                if (vcmd->vuser.metadata && vcmd->vuser.metadata_len) {
                        c->rw.metadata = htole64(dma->meta_phy[0]);
                }

                break;
        default:
                printf("NVMeDirect: Unknown command: %hhx\n", vcmd->vuser.opcode);
                exit(1);
        }
        // Handle dma buffer
        c->rw.slba = dma->ppa_phy[0];
        memcpy(dma->ppa_dma, (void*)vcmd->vuser.ppa_list, sizeof(u64) * (vcmd->vuser.nppas + 1));

        // Handle ppa list
        if (vcmd->vuser.ppa_list == 0) {
                c->rw.prp1 = dma->buffer_phy[0];
                c->rw.prp2 = 0;
        }
        else if (vcmd->vuser.ppa_list == 1) {
                c->rw.prp1 = dma->buffer_phy[0];
                c->rw.prp2 = dma->buffer_phy[1];
                // Handle dma buffer
                c->rw.slba = dma->ppa_phy[0];
                memcpy(dma->ppa_dma, (void*)vcmd->vuser.ppa_list, sizeof(u64) * (vcmd->vuser.nppas + 1));
        }
        else {
                c->rw.prp1 = dma->buffer_phy[0];
                c->rw.prp2 = dma->prp2_phy[0];
        }
        // End NVM

        if (++nvmed_queue->sq_tail == nvmed->dev_info->q_depth)
                nvmed_queue->sq_tail = 0;

        COMPILER_BARRIER();
        *(volatile u32 *)nvmed_queue->sq_db = nvmed_queue->sq_tail;


        // Completion
        head = nvmed_queue->cq_head;
        phase = nvmed_queue->cq_phase;
        cqe = (volatile struct nvme_completion *)&nvmed_queue->cqes[head];
        for (;;) {
                if ((cqe->status & 1) == nvmed_queue->cq_phase)
                        break;
        }

        if (++head == nvmed->dev_info->q_depth) {
                head = 0;
                phase = !phase;
        }
        COMPILER_BARRIER();
        *(volatile u32 *)nvmed_queue->cq_db = head;
        nvmed_queue->cq_head = head;
        nvmed_queue->cq_phase = phase;

        pthread_spin_unlock(&nvmed_queue->cq_lock);
        //pthread_mutex_unlock(&nvmed_queue->queue_lock);

        if (vcmd->vuser.opcode == 0x92) {
                if (vcmd->vuser.metadata && vcmd->vuser.metadata_len) {
                        memcpy((void*)vcmd->vuser.metadata, dma->meta_dma, vcmd->vuser.metadata_len);
                }
                memcpy((void*)vcmd->vuser.addr, dma->buffer_dma, buffer_size);
        }

        return 0;
}

static inline uint64_t _ilog2(uint64_t x)
{
        uint64_t val = 0;

        while (x >>= 1)
                val++;

        return val;
}

int nvm_be_nvmed_vuser(struct nvm_dev *dev, struct nvm_cmd *cmd,
                       struct nvm_ret *ret)
{

        int err = nvm_be_send_cmd(dev, cmd, ret);

        if (ret) {
                ret->result = 0;
                ret->status = 0;
                /* ret->result = cmd->vuser.result; */
                /* ret->status = cmd->vuser.status; */
        }

        if (err || cmd->vuser.result) {
                errno = EIO;
                return -1;
        }
        return err;
}

int nvm_be_nvmed_vadmin(struct nvm_dev *dev, struct nvm_cmd *cmd,
                        struct nvm_ret *ret)
{
        const int err = ioctl(dev->fd, NVME_NVM_IOCTL_ADMIN_VIO, cmd);

        if (ret) {
                ret->result = cmd->vadmin.result;
                ret->status = cmd->vadmin.status;
        }

        if (err || cmd->vadmin.result) {
                errno = EIO;
                return -1;
        }

        return 0;
}

int nvm_be_nvmed_user(struct nvm_dev *dev, struct nvm_cmd *cmd,
                      struct nvm_ret *ret)
{
        /*
          TODO: This function is not needed for our current tests, therefore
          it has not been written.
        */
        exit(1);
        const int err = ioctl(dev->fd, NVME_IOCTL_SUBMIT_IO, cmd);
        if (ret) {
                ret->result = 0x0;
                ret->status = 0x0;
        }
        if (err) {
                errno = EIO;
                return -1;
        }

        return 0;
}

int nvm_be_nvmed_admin(struct nvm_dev *dev, struct nvm_cmd *cmd,
                       struct nvm_ret *ret)
{
        const int err = ioctl(dev->fd, NVME_IOCTL_ADMIN_CMD, cmd);
        if (ret) {
                ret->result = cmd->admin.result;
                ret->status = 0x0;
        }
        if (err) {
                errno = EIO;
                return -1;
        }

        return 0;
}

static inline void _construct_ppaf_mask(struct nvm_spec_ppaf_nand *ppaf,
                                        struct nvm_spec_ppaf_nand_mask *mask)
{
        for (int i = 0 ; i < 12; ++i) {
                if ((i % 2)) {
                        // i-1 = offset
                        // i = width
                        mask->a[i/2] = (((uint64_t)1<< ppaf->a[i])-1) << ppaf->a[i-1];
                }
        }
}

static inline int _ioctl_fill_geo(struct nvm_dev *dev, struct nvm_ret *ret)
{
        struct nvm_cmd cmd = {.cdw={0}};
        struct nvm_spec_identify *idf = 0;
        int err = 0;

        err = posix_memalign((void **)&idf, 4096, sizeof(*idf));
        if (err) {
                errno = ENOMEM;
                return -1;
        }

        cmd.vadmin.opcode = NVM_S12_OPC_IDF;	// Setup command
        cmd.vadmin.addr = (uint64_t)idf;
        cmd.vadmin.data_len = sizeof(*idf);

        err = nvm_be_nvmed_vadmin(dev, &cmd, ret);
        if (err) {
                free(idf);
                return -1;			// NOTE: Propagate errno
        }

        switch (idf->s.verid) {
        case NVM_SPEC_VERID_12:
                dev->geo.page_nbytes = idf->s12.grp[0].fpg_sz;
                dev->geo.sector_nbytes = idf->s12.grp[0].csecs;
                dev->geo.meta_nbytes = idf->s12.grp[0].sos;

                dev->geo.nchannels = idf->s12.grp[0].num_ch;
                dev->geo.nluns = idf->s12.grp[0].num_lun;
                dev->geo.nplanes = idf->s12.grp[0].num_pln;
                dev->geo.nblocks = idf->s12.grp[0].num_blk;
                dev->geo.npages = idf->s12.grp[0].num_pg;

                dev->ppaf = idf->s12.ppaf;
                dev->mccap = idf->s12.grp[0].mccap;
                break;

        case NVM_SPEC_VERID_20:
                dev->geo.sector_nbytes = idf->s20.csecs;
                dev->geo.meta_nbytes = idf->s20.sos;
                dev->geo.page_nbytes = idf->s20.mw_min * dev->geo.sector_nbytes;

                dev->geo.nchannels = idf->s20.num_ch;
                dev->geo.nluns = idf->s20.num_lun;
                dev->geo.nplanes = idf->s20.mw_opt / idf->s20.mw_min;
                dev->geo.nblocks = idf->s20.num_chk;
                dev->geo.npages = ((idf->s20.clba * idf->s20.csecs) / dev->geo.page_nbytes) / dev->geo.nplanes;
                dev->geo.nsectors = dev->geo.page_nbytes / dev->geo.sector_nbytes;

                dev->ppaf = idf->s20.ppaf;
                dev->mccap = idf->s20.mccap;
                break;

        default:
                NVM_DEBUG("Unsupported Version ID(%d)", idf->s.verid);
                errno = ENOSYS;
                free(idf);
                return -1;
        }

        dev->verid = idf->s.verid;
        _construct_ppaf_mask(&dev->ppaf, &dev->mask);

        // WARN: HOTFIX for reports of unrealisticly large OOB area
        if (dev->geo.meta_nbytes > 100) {
                dev->geo.meta_nbytes = 16;	// Naively hope this is right
        }
        dev->geo.nsectors = dev->geo.page_nbytes / dev->geo.sector_nbytes;

        /* Derive total number of bytes on device */
        dev->geo.tbytes = dev->geo.nchannels * dev->geo.nluns * \
                dev->geo.nplanes * dev->geo.nblocks * \
                dev->geo.npages * dev->geo.nsectors * \
                dev->geo.sector_nbytes;

        /* Derive the sector-shift-width for LBA mapping */
        dev->ssw = _ilog2(dev->geo.sector_nbytes);

        /* Derive a default plane mode */
        switch(dev->geo.nplanes) {
        case 4:
                dev->pmode = NVM_FLAG_PMODE_QUAD;
                break;
        case 2:
                dev->pmode = NVM_FLAG_PMODE_DUAL;
                break;
        case 1:
                dev->pmode = NVM_FLAG_PMODE_SNGL;
                break;
        default:
                errno = EINVAL;
                free(idf);
                return -1;
        }

        dev->erase_naddrs_max = NVM_NADDR_MAX;
        dev->write_naddrs_max = NVM_NADDR_MAX;
        dev->read_naddrs_max = NVM_NADDR_MAX;

        dev->meta_mode = NVM_META_MODE_NONE;

        free(idf);

        return 0;
}

struct nvm_dev *nvm_be_nvmed_open(const char *dev_path)
{
        struct nvm_dev *dev = NULL;
        struct nvm_ret ret = {0,0};
        int err;

        NVMED* nvmed;

        if (strlen(dev_path) > NVM_DEV_PATH_LEN) {
                NVM_DEBUG("FAILED: Device path too long\n");
                errno = EINVAL;
                return NULL;
        }

        dev = malloc(sizeof(*dev));
        if (!dev) {
                NVM_DEBUG("FAILED: allocating 'struct nvm_dev'\n");
                return NULL;	// Propagate errno from malloc
        }

        strncpy(dev->path, dev_path, NVM_DEV_PATH_LEN);
        strncpy(dev->name, dev_path+5, NVM_DEV_NAME_LEN);

        dev->fd = open(dev->path, O_RDWR | O_DIRECT);
        if (dev->fd < 0) {
                NVM_DEBUG("FAILED: open(dev->path(%s)), dev->fd(%d)\n",
                          dev->path, dev->fd);

                free(dev);

                return NULL;	// Propagate errno from open
        }

        err = _ioctl_fill_geo(dev, &ret);
        if (err) {
                NVM_DEBUG("FAILED: _ioctl_fill_geo, err(%d)", err);

                close(dev->fd);
                free(dev);

                return NULL;
        }

        nvmed = nvmed_open(dev_path);
        nvmed->queue_list = calloc(nvmed->dev_info->nr_queues, sizeof(struct nvmed_queue));
        for (int i = 0; i < nvmed->dev_info->nr_queues; i++) {
                nvmed_queue_create(nvmed, 0);
        }
        dev->nvmed = nvmed;

        return dev;
}

void nvm_be_nvmed_close(struct nvm_dev *dev)
{
        NVMED* nvmed = dev->nvmed;
        void *status;

        for (int i = 0; i < nvmed->dev_info->nr_queues; i++) {
                nvmed_queue_destroy(nvmed->queue_list[i]);
        }
        free(nvmed->queue_list);

        if (nvmed == NULL || nvmed->ns_fd == 0)
                return; // Error

        close(nvmed->ns_fd);
        free(nvmed->ns_path);

        pthread_spin_destroy(&nvmed->mngt_lock);

        free(nvmed->dev_info);
        free(nvmed);
        close(dev->fd);
}

struct nvm_be nvm_be_nvmed = {
        .id = NVM_BE_IOCTL,

        .open = nvm_be_nvmed_open,
        .close = nvm_be_nvmed_close,

        .user = nvm_be_nvmed_user,
        .admin = nvm_be_nvmed_admin,

        .vuser = nvm_be_nvmed_vuser,
        .vadmin = nvm_be_nvmed_vadmin,
};
