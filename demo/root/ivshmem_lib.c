// ivshmem_lib.c  （完整文件，替换原来的）
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <libgen.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/signalfd.h>
#include <poll.h>
#include <limits.h>
#include <assert.h>


struct ivshm_regs {
    uint32_t id;
    uint32_t max_peers;
    uint32_t int_control;
    uint32_t doorbell;
    uint32_t state;
};

typedef struct {
    int fd;
    char uio_devname[64];
    volatile struct ivshm_regs *regs;
    volatile uint32_t *state;
    volatile uint32_t *rw;
    volatile uint32_t *in;
    volatile uint32_t *out;
    unsigned int id;
    unsigned int max_peers;
    int has_msix;
    int pagesize;
    int target;

    /* 保存每个 map 的 size 和 offset，便于 munmap */
    size_t map_size[5];
    off_t  map_offset[5];
} ivshm_handle_t;

static int read_sysfs_hex(const char *path, unsigned long *val)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -errno;
    char buf[64];
    ssize_t r = read(fd, buf, sizeof(buf)-1);

    // if (read(fd, buf, sizeof(buf)-1) < 0) {
    //     fprintf(stderr, "ivshmem: read(%s) failed: %s (%d)\n", path, strerror(errno), errno);
    // }

    close(fd);
    if (r <= 0) return -EINVAL;
    buf[r] = '\0';
    if (sscanf(buf, "0x%lx", val) != 1) return -EINVAL;
    return 0;
}

static int uio_read_map_info(const char *uio_devname, int idx, unsigned long *out_size, unsigned long *out_offset)
{
    char path[256];
    int ret;

    snprintf(path, sizeof(path), "/sys/class/uio/%s/maps/map%d/size", uio_devname, idx);
    ret = read_sysfs_hex(path, out_size);
    if (ret) return ret;

    snprintf(path, sizeof(path), "/sys/class/uio/%s/maps/map%d/offset", uio_devname, idx);
    ret = read_sysfs_hex(path, out_offset);
    if (ret) return ret;

    return 0;
}




/* helper to map UIO map n safely */
static void *map_uio_region(int fd, int map_index, size_t map_size, unsigned long map_offset_from_sysfs)
{
    size_t pgsize = (size_t) getpagesize(); /* or sysconf(_SC_PAGESIZE) */
    off_t mmap_off;

    /* Prefer using map_index * page_size for /dev/uioN mapping.
       Keep the original sysfs offset for debug only. */
    mmap_off = (off_t)map_index * (off_t)pgsize;

    fprintf(stderr, "ivshmem: map%d: size=0x%zx sysfs_off=0x%lx mmap_off=0x%lx pgsize=%zu\n",
            map_index, map_size, (unsigned long)map_offset_from_sysfs, (unsigned long)mmap_off, pgsize);

    if (map_size == 0) {
        return NULL;
    }

    /* try mmap with computed mmap_off */
    void *m = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mmap_off);
    if (m == MAP_FAILED) {
        /* If it fails, try fallback: try to align sysfs offset down to page boundary */
        off_t alt_off = (off_t)(map_offset_from_sysfs & ~(pgsize - 1));
        fprintf(stderr, "ivshmem: mmap(map%d) with mmap_off failed: %s (%d). trying alt_off=0x%lx\n",
                map_index, strerror(errno), errno, (unsigned long)alt_off);
        m = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, alt_off);
        if (m == MAP_FAILED) {
            fprintf(stderr, "ivshmem: mmap(map%d) alt_off failed: %s (%d)\n", map_index, strerror(errno), errno);
            return NULL;
        }
    }
    return m;
}

/* Open & mmap all maps 0..4 (if present). target_override <0 -> auto */
void *ivshmem_open_shared(const char *devpath, int target_override)
{
    if (!devpath) return NULL;
    ivshm_handle_t *h = calloc(1, sizeof(*h));
    if (!h) return NULL;

    h->fd = open(devpath, O_RDWR | O_SYNC);
    if (h->fd < 0) {
        fprintf(stderr, "ivshmem: open('%s') failed: %s (%d)\n",
                devpath ? devpath : "(null)", strerror(errno), errno);
        free(h);
        return NULL;
    }
    fprintf(stderr, "ivshmem: opened %s -> fd=%d\n", devpath, h->fd);

    char *dup = strdup(devpath);
    if (!dup) { close(h->fd); free(h); return NULL; }
    strncpy(h->uio_devname, basename(dup), sizeof(h->uio_devname)-1);
    free(dup);

    h->pagesize = getpagesize();

    /* for maps 0..4, read size/offset; if read fails for a map, treat as 0 */
    for (int i = 0; i < 5; ++i) {
        unsigned long sz = 0, off = 0;
        int r = uio_read_map_info(h->uio_devname, i, &sz, &off);
        if (r == 0) {
            h->map_size[i] = (size_t)sz;
            h->map_offset[i] = (off_t)off;
        } else {
            h->map_size[i] = 0;
            h->map_offset[i] = 0;
        }
    }

    /* map0: regs */
    if (h->map_size[0] == 0) { close(h->fd); free(h); return NULL; }
    void *m0 = map_uio_region(h->fd, 0, h->map_size[0], h->map_offset[0]);
    if (!m0) { close(h->fd); free(h); return NULL; }
    h->regs = (volatile struct ivshm_regs *)m0;

    /* read id/max_peers */
    h->id = (unsigned int)h->regs->id;
    h->max_peers = (unsigned int)h->regs->max_peers;

    /* map1: state (read) */
    if (h->map_size[1]) {
        void *m1 = map_uio_region(h->fd, 1, h->map_size[1], h->map_offset[1]);
        if (!m1) { close(h->fd); free(h); return NULL; }
        h->state = (volatile uint32_t *)m1;
    }

    /* map2: rw (rw) */
    if (h->map_size[2]) {
        void *m2 = map_uio_region(h->fd, 2, h->map_size[2], h->map_offset[2]);
        if (!m2) { close(h->fd); free(h); return NULL; }
        h->rw = (volatile uint32_t *)m2;
    }

    /* map3: in */
    if (h->map_size[3]) {
        void *m3 = map_uio_region(h->fd, 3, h->map_size[3], h->map_offset[3]);
        if (!m3) { close(h->fd); free(h); return NULL; }
        h->in = (volatile uint32_t *)m3;
    }

    /* map4: out */
    if (h->map_size[4]) {
        void *m4 = map_uio_region(h->fd, 4, h->map_size[4], h->map_offset[4]);
        if (!m4) { close(h->fd); free(h); return NULL; }
        h->out = (volatile uint32_t *)m4;
    }

    /* detect msix via sysfs */
    char sysfs_path[128];
    snprintf(sysfs_path, sizeof(sysfs_path), "/sys/class/uio/%s/device/msi_irqs", h->uio_devname);
    h->has_msix = (access(sysfs_path, R_OK) == 0) ? 1 : 0;

    /* set initial values */
    h->regs->state = h->id + 1;
    if (h->rw) h->rw[h->id] = 0;
    if (h->out) h->out[0] = 0;

    h->regs->int_control = 1;

    if (target_override < 0) h->target = (h->id + 1) % (h->max_peers ? h->max_peers : 1);
    else h->target = target_override;

    return (void*)h;
}

/* close */
int ivshmem_close_shared(void *handle)
{
    if (!handle) return -EINVAL;
    ivshm_handle_t *h = (ivshm_handle_t*)handle;
    if (h->out && h->map_size[4]) munmap((void*)h->out, h->map_size[4]);
    if (h->in && h->map_size[3]) munmap((void*)h->in, h->map_size[3]);
    if (h->rw && h->map_size[2]) munmap((void*)h->rw, h->map_size[2]);
    if (h->state && h->map_size[1]) munmap((void*)h->state, h->map_size[1]);
    if (h->regs && h->map_size[0]) munmap((void*)h->regs, h->map_size[0]);
    if (h->fd >= 0) close(h->fd);
    free(h);
    return 0;
}

/* send irq */
int ivshmem_send_irq_shared(void *handle)
{
    if (!handle) return -EINVAL;
    ivshm_handle_t *h = (ivshm_handle_t*)handle;
    h->regs->int_control = 1;
    uint32_t int_no = h->has_msix ? (h->id + 1) : 0;
    h->regs->doorbell = int_no | (h->target << 16);
    return 0;
}

/* write out */
int ivshmem_write_out_shared(void *handle, unsigned int start_idx, 
    const int32_t *values, unsigned int count)
{
    if (!handle || !values) return -EINVAL;
    ivshm_handle_t *h = (ivshm_handle_t*)handle;
    if (!h->out) return -EFAULT;

    /* 计算字节偏移并检查边界 */
    size_t start_offset = start_idx * sizeof(uint32_t);
    size_t total_size = count * sizeof(uint32_t);

    if ((start_offset >= h->map_size[4]) || 
    ((start_offset + total_size) > h->map_size[4])) {
    fprintf(stderr, "Error: Out of bounds (start=%u, count=%u, max_size=%zu)\n",
    start_idx, count, h->map_size[4]/sizeof(uint32_t));
    return -ERANGE;
    }

    // 逐个元素写入
    for (unsigned int i = 0; i < count; i++) {
        h->out[start_idx + i] = values[i];  // 直接按顺序写入
        __sync_synchronize();  // 确保写入可见性
    }

    return 0;
}



/* read state */
uint32_t ivshmem_read_state_shared(void *handle, unsigned int idx)
{
    if (!handle) return 0;
    ivshm_handle_t *h = (ivshm_handle_t*)handle;
    if (!h->state) return 0;
    if ((size_t)(idx * sizeof(uint32_t)) >= h->map_size[1]) return 0;
    return h->state[idx];
}


uint32_t ivshmem_read_rw_shared(void *handle, unsigned int idx)
{
    if (!handle) return 0;
    ivshm_handle_t *h = (ivshm_handle_t*)handle;
    if (!h->rw) return 0;
    if ((size_t)(idx * sizeof(uint32_t)) >= h->map_size[2]) return 0;
    return h->rw[idx];
}

uint32_t ivshmem_read_in_shared(void *handle, unsigned int idx)
{
    if (!handle) return 0;
    ivshm_handle_t *h = (ivshm_handle_t*)handle;
    if (!h->in) return 0;
    if ((size_t)(idx * sizeof(uint32_t)) >= h->map_size[3]) return 0;
    return h->in[idx];
}

/* wait irq - blocking read on uio fd */
int ivshmem_wait_irq_shared(void *handle, uint32_t *out_cnt)
{
    if (!handle) return -EINVAL;
    ivshm_handle_t *h = (ivshm_handle_t*)handle;
    uint32_t cnt;
    ssize_t r = read(h->fd, &cnt, sizeof(cnt));
    if (r != sizeof(cnt)) return -errno;
    if (out_cnt) *out_cnt = cnt;
    return 0;
}

int ivshmem_ack_irq_shared(void *handle)
{
    if (!handle) return -EINVAL;
    ivshm_handle_t *h = (ivshm_handle_t*)handle;

    uint32_t one = 1;
    ssize_t w = write(h->fd, &one, sizeof(one));
    if (w != sizeof(one)) {
        return -errno;
    }
    return 0;
}


/* small helpers to expose id/max_peers */
int ivshmem_get_id_shared(void *handle) { if (!handle) return -1; return ((ivshm_handle_t*)handle)->id; }
int ivshmem_get_max_peers_shared(void *handle) { if (!handle) return -1; return ((ivshm_handle_t*)handle)->max_peers; }

/* export symbols visibility */
__attribute__((visibility("default"))) void *ivshmem_open_shared_sym(const char *devpath, int target_override) { return ivshmem_open_shared(devpath, target_override); }
__attribute__((visibility("default"))) int ivshmem_close_shared_sym(void *h) { return ivshmem_close_shared(h); }
__attribute__((visibility("default"))) int ivshmem_send_irq_shared_sym(void *h) { return ivshmem_send_irq_shared(h); }
__attribute__((visibility("default"))) int ivshmem_write_out_shared_sym(void *h, unsigned int start_idx, 
    const int32_t *values, unsigned int count) { return ivshmem_write_out_shared(h, start_idx, values, count); }
__attribute__((visibility("default"))) uint32_t ivshmem_read_state_shared_sym(void *h, unsigned int idx) { return ivshmem_read_state_shared(h, idx); }
__attribute__((visibility("default"))) uint32_t ivshmem_read_rw_shared_sym(void *h, unsigned int idx) { return ivshmem_read_rw_shared(h, idx); }
__attribute__((visibility("default"))) uint32_t ivshmem_read_in_shared_sym(void *h, unsigned int idx) { return ivshmem_read_in_shared(h, idx); }
__attribute__((visibility("default"))) int ivshmem_wait_irq_shared_sym(void *h, uint32_t *cnt) { return ivshmem_wait_irq_shared(h, cnt); }
__attribute__((visibility("default"))) int ivshmem_get_id_shared_sym(void *h) { return ivshmem_get_id_shared(h); }
__attribute__((visibility("default"))) int ivshmem_get_max_peers_shared_sym(void *h) { return ivshmem_get_max_peers_shared(h); }
__attribute__((visibility("default"))) int ivshmem_ack_irq_shared_sym(void *h) { return ivshmem_ack_irq_shared(h); }