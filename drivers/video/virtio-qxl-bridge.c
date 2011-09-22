/*
 * Virtio QXL Device
 *
 *
 * Authors:
 *  Erlon R. Cruz <erlon.cruz@br.flextronics.com>
 *  Rafael F. Santos <Rafael.Santos@fit-tecnologia.org.br>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/virtio.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_bridge.h>
#include <linux/mm.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>

#define DRIVER_STRING "virtio-qxl-bridge"
#define SG_ELEMENTS 128

#define DEBUG_ERROR 0
#define DEBUG_INFO 0
#define DEBUG_VM 0
#define DEBUG_SG 0
#define DEBUG_PUSH_AREA 0
#define DEBUG_PULL_AREA 0
#define DEBUG_IOCTL 0
#define DEBUG_IOPWRITE 0

#define dprintk(_level, _fmt, ...)				\
    do {							\
        if (_level) {						\
            printk(_fmt,## __VA_ARGS__);			\
        }							\
    } while (0)

static inline void printHexa(void *buf, int len)
{
	uint8_t *cur, *ubuf;
	ubuf = (uint8_t *) buf;

	for (cur = ubuf; (cur - ubuf) < len; cur++)
#ifdef __KERNEL__
		printk("%02X", *cur);
#else
		ErrorF("%02X", *cur);
#endif

}

int devindex = 0, cdev_major;
struct semaphore idxsem;
static struct class *virtio_qxl_class;

struct qxl_memory {
	u8 *ram;
	u8 *vram;
	u8 *rom;
};

struct virtio_qxl_bridge {
	spinlock_t lock;
	struct virtio_device *vdev;
	struct virtqueue *vq;
	struct scatterlist sg[128];
	struct scatterlist *vgasg;
	struct cdev cdev;
	struct virtioqxl_config config;
	struct semaphore sem;
	u8 *driver_mem;
	struct qxl_memory mem_desc;
};

struct vbr_req {
	struct list_head list;
	struct vbr_proto_hdr hdr;
	u8 status;
};

static int vmalloc_fill_sg(struct scatterlist *sg, unsigned char *virt,
			   int length)
{
	struct page *pg;
	int sg_entries = 0;
	int tmp_off;
	uint8_t *start, *end;

	start = virt;
	end = start + length;

	/* unaligned */
	if ((ulong) virt & ~PAGE_MASK) {
		int gap = 0, cplen;
		// Start inside the page
		tmp_off = ((ulong) virt & ~PAGE_MASK);
		// Length from start to end of the page
		gap = PAGE_SIZE - ((ulong) virt & ~PAGE_MASK);
		if (gap > length)
			cplen = length;
		else
			cplen = gap;
		pg = vmalloc_to_page(start);
		sg_set_page(&sg[sg_entries++], pg, cplen, tmp_off);
		start += cplen;
		dprintk(DEBUG_SG,
			"%s: Unaligned buffer: tt len %d, offset %d, gap %d,"
			" unaligned head %d", __func__, length, tmp_off, gap,
			cplen);
	} else {
		dprintk(DEBUG_SG, "%s: Aligned buffer tt len %d", __func__,
			length);
	}

	// Now start is aligned rigth? hooope so
	while (end - start >= PAGE_SIZE) {
		pg = vmalloc_to_page(start);
		sg_set_page(&sg[sg_entries++], pg, PAGE_SIZE, 0);
		start += PAGE_SIZE;
	}

	if (end - start > 0) {
		pg = vmalloc_to_page(start);
		sg_set_page(&sg[sg_entries++], pg, end - start, 0);
		dprintk(DEBUG_SG, " unaligned tail %d", (int)(end - start));
		start += end - start;
	}

	if (start != end)
		panic("Anomalous behaviour when filling SG\n");

	dprintk(DEBUG_SG, " %d SG entries\n", sg_entries);
	return sg_entries;
}

static int send_packet(struct virtio_qxl_bridge *devdata,
		       uint what, void *buffer, int len, int flags)
{
	int in, out;
	unsigned int readlen;
	struct vbr_req *rq;
	struct scatterlist *sg;
	in = out = 0;

	rq = kmalloc(sizeof(*rq), GFP_KERNEL);
	memset(rq, 0x0, sizeof(*rq));
	rq->hdr.flags |= flags;
	rq->hdr.function = what;
	rq->hdr.param = (u8 *) buffer - devdata->mem_desc.ram;
	rq->hdr.len = len;

	if (what == VIRTIOQXL_GETCFG)
		sg = devdata->sg;
	else
		sg = devdata->vgasg;

	sg_set_buf(&sg[out++], &rq->hdr, sizeof(rq->hdr));

	if (flags & CONFIG_READ) {
		if (is_vmalloc_addr(buffer))
			in += vmalloc_fill_sg(&sg[out], buffer, len);
		else
			sg_set_buf(&sg[out + in++], buffer, len);
	} else if (is_vmalloc_addr(buffer)) {
		out += vmalloc_fill_sg(&sg[out], buffer, len);
	} else {
		sg_set_buf(&sg[out++], buffer, len);
	}

	sg_set_buf(&sg[out + in++], &rq->status, sizeof(rq->status));

	if (virtqueue_add_buf(devdata->vq, sg, out, in, rq, GFP_KERNEL) < 0) {
		dprintk(DEBUG_ERROR, "%s: error adding buffer\n",
			DRIVER_STRING);
		return -1;
	}

	virtqueue_kick(devdata->vq);
	while (!virtqueue_get_buf(devdata->vq, &readlen))
		cpu_relax();

	return 0;
}

static int get_from_host(struct virtio_qxl_bridge *devdata, uint what,
			 void *bufto, int len)
{
	return send_packet(devdata, what, bufto, len, CONFIG_READ);
}

static int set_on_host(struct virtio_qxl_bridge *devdata, uint what,
		       void *buffrom, int len)
{
	return send_packet(devdata, what, buffrom, len, CONFIG_WRITE);
}

static int device_open(struct inode *inode, struct file *file)
{
	struct virtio_qxl_bridge *virtiodata;
	dprintk(DEBUG_INFO, "%s: entering %s\n", DRIVER_STRING, __func__);

	virtiodata =
	    container_of(inode->i_cdev, struct virtio_qxl_bridge, cdev);
	file->private_data = virtiodata;

	return 0;
}

static long device_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct virtio_qxl_bridge *brdev;
	brdev = file->private_data;

	dprintk(DEBUG_IOCTL, "virtio_qxl_bridge: %s: %s\n",
		__func__, io_port_string[_IOC_NR(cmd)]);
	switch (cmd) {
	case QXL_IOCTL_UPDATE_ROM:
		get_from_host(brdev, VIRTIOQXL_GET_ROM, brdev->mem_desc.rom,
			      brdev->config.romsize);
		break;
	case QXL_IOCTL_QXL_IO_PUSH_AREA: {
		struct qxl_ram_area area;
		u8 *start;
#if DEBUG_PUSH_AREA
		u8 *startp, *endp;
#endif

		copy_from_user(&area, (void __user *)arg,
			       sizeof(struct qxl_ram_area));
		start = (u8 *) (brdev->mem_desc.ram + area.offset);
		set_on_host(brdev, VIRTIOQXL_SET_RAM, start, area.len);

#if DEBUG_PUSH_AREA
		dprintk(DEBUG_PUSH_AREA,
			"%s: pushing area [%d - %d], %d bytes: [",
			DRIVER_STRING, area.offset,
			area.offset + area.len, area.len);

		startp = start;
		endp = start + area.len;

		/* unaligned */
		if ((ulong) start & ~PAGE_MASK) {
			int gaptonpage = (PAGE_SIZE - ((ulong) start &
						~PAGE_MASK));
			if (gaptonpage < area.len) {
				printHexa(startp, gaptonpage);
				startp += gaptonpage;
			} else {
				printHexa(startp, area.len);
				startp += area.len;
			}

			if ((endp - startp) > 0)
				printk(" [EOP] ");
		}

		while (endp - startp >= PAGE_SIZE) {
			printHexa(startp, PAGE_SIZE);
			startp += PAGE_SIZE;
			if ((endp - startp) > 0)
				printk(" [EOP] ");
		}

		if (endp - startp < 0) {
			startp -= PAGE_SIZE;
			printHexa(startp, endp - startp);
		} else if (endp - startp > 0) {
			printHexa(startp, endp - startp);
		}
		dprintk(DEBUG_PUSH_AREA, "]\n\n");
#endif
		break;
	}
	case QXL_IOCTL_QXL_IO_PULL_AREA:{
		struct qxl_ram_area area;
		u8 *start;

		copy_from_user(&area, (void __user *)arg,
			       sizeof(struct qxl_ram_area));
		start = (u8 *) (brdev->mem_desc.ram + area.offset);
		dprintk(DEBUG_PULL_AREA,
			"%s: pulling area[%d->%d], %d bytes: [",
			DRIVER_STRING, area.offset,
			area.offset + area.len, area.len);
		if (DEBUG_PULL_AREA)
			printHexa(start, area.len);
		dprintk(DEBUG_PULL_AREA, "]\n\n");

		get_from_host(brdev, VIRTIOQXL_GET_RAM, start,
			      area.len);
		break;
	}
	case QXL_IOCTL_NOTIFY_CMD:
	case QXL_IOCTL_NOTIFY_CURSOR:
	case QXL_IOCTL_UPDATE_AREA:
	case QXL_IOCTL_UPDATE_IRQ:
	case QXL_IOCTL_NOTIFY_OOM:
	case QXL_IOCTL_RESET:
	case QXL_IOCTL_SET_MODE:
	case QXL_IOCTL_LOG:
	case QXL_IOCTL_MEMSLOT_ADD:
	case QXL_IOCTL_MEMSLOT_DEL:
	case QXL_IOCTL_DETACH_PRIMARY:
	case QXL_IOCTL_ATTACH_PRIMARY:
	case QXL_IOCTL_CREATE_PRIMARY:
	case QXL_IOCTL_DESTROY_PRIMARY:
	case QXL_IOCTL_DESTROY_SURFACE_WAIT:
	case QXL_IOCTL_DESTROY_ALL_SURFACES:
	case QXL_IOCTL_UPDATE_AREA_ASYNC:
	case QXL_IOCTL_MEMSLOT_ADD_ASYNC:
	case QXL_IOCTL_CREATE_PRIMARY_ASYNC:
	case QXL_IOCTL_DESTROY_PRIMARY_ASYNC:
	case QXL_IOCTL_DESTROY_SURFACE_ASYNC:
	case QXL_IOCTL_DESTROY_ALL_SURFACES_ASYNC:
	case QXL_IOCTL_FLUSH_SURFACES_ASYNC:
	case QXL_IOCTL_FLUSH_RELEASE: {
		struct iowrite_cmd *iocmd =
			kmalloc(sizeof(*iocmd), GFP_KERNEL);
		iocmd->port = _IOC_NR(cmd);
		iocmd->arg = arg;
		dprintk(DEBUG_IOPWRITE, " port %d, arg %d\n",
			iocmd->port, iocmd->arg);
		set_on_host(brdev, VIRTIOQXL_IOPORT_WRITE, iocmd,
			    sizeof(*iocmd));
		kfree(iocmd);
		break;
	}
	default:
		return -ENOTTY;
	}

	dprintk(DEBUG_IOCTL, "%s: returning\n", __func__);
	return 0;
}

static ssize_t device_read(struct file *file, char __user * buf, size_t count,
			   loff_t * f_pos)
{
	struct virtio_qxl_bridge *virtiodata;
	struct virtioqxl_config *config;
	int ret = sizeof(*config);

	dprintk(DEBUG_INFO, "%s: virtio_qxl_bridge: %s: reading %d bytes\n",
		DRIVER_STRING, __func__, (int)count);

	virtiodata = file->private_data;
	config = &virtiodata->config;

	if (copy_to_user(buf, config, ret)) {
		*f_pos += ret;
		return ret;
	}
	return 0;
}

static ssize_t device_write(struct file *file, const char __user * buf,
			    size_t count, loff_t * f_pos)
{
	return 0;
}

/**************************** Commom VMA ops **************************/
static void vma_open(struct vm_area_struct *vma)
{
	dprintk(DEBUG_VM, "virtio_vios: mapped virt %lx, phys %lx\n",
		vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static void vma_close(struct vm_area_struct *vma)
{
	dprintk(DEBUG_VM, "%s: %s\n", __func__, DRIVER_STRING);
}

static int vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	unsigned long offset;
	struct virtio_qxl_bridge *dev = vma->vm_private_data;
	struct page *page;

	down(&dev->sem);

	offset = vmf->pgoff << PAGE_SHIFT;

	if (vmf->flags & FAULT_FLAG_WRITE)
		dprintk(DEBUG_VM, "%s: write: offset: %lu\n", __func__,
			offset);
	else
		dprintk(DEBUG_VM, "%s: read: offset: %lu\n", __func__,
			offset);

	page = vmalloc_to_page((void *) dev->driver_mem + offset);
	get_page(page);
	vmf->page = page;

	up(&dev->sem);

	if (!page)
		return VM_FAULT_SIGBUS;

	return 0;
}

struct vm_operations_struct remap_vm_ops = {
	.open = vma_open,
	.close = vma_close,
	.fault = vma_fault
};

static int device_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct virtio_qxl_bridge *virtiodata;
	int ret = 0;

	dprintk(DEBUG_VM, "%s:mapping\n", __func__);

	virtiodata = (struct virtio_qxl_bridge *)file->private_data;
	vma->vm_private_data = file->private_data;

	vma->vm_flags |= VM_RESERVED | VM_IO;
	vma->vm_ops = &remap_vm_ops;

	return ret;
}

int device_release(struct inode *inode, struct file *file)
{
	dprintk(DEBUG_INFO, "%s: %s\n", DRIVER_STRING, __func__);
	return 0;
}

struct file_operations device_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = device_ioctl,
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.mmap = device_mmap,
	.release = device_release
};

static void setup_cdev(struct virtio_qxl_bridge *virtiodata)
{
	int err, devno, minor;
	struct device *dev;

	down(&idxsem);
	minor = devindex;
	devindex++;
	up(&idxsem);

	devno = MKDEV(cdev_major, minor);

	dprintk(DEBUG_INFO, "%s: %s: adding char device %d/%d\n",
		DRIVER_STRING, __func__, cdev_major, minor);

	/* Print driver port string */
	cdev_init(&virtiodata->cdev, &device_fops);
	virtiodata->cdev.owner = THIS_MODULE;
	virtiodata->cdev.ops = &device_fops;
	err = cdev_add(&virtiodata->cdev, devno, 1);

	/* Fail gracefully if need be */
	if (err)
		dprintk(DEBUG_INFO, "%s: error %d adding char device %d",
			DRIVER_STRING, err, devindex);
	/* Create a sysfs class entry */
	dev =
	    device_create(virtio_qxl_class, NULL, devno, NULL, "virtioqxl%u",
			  minor);
	if (IS_ERR(dev))
		dprintk(DEBUG_INFO, "%s: error %li creating device %d\n",
			DRIVER_STRING, PTR_ERR(dev), devindex);
}

static int __devinit qxl_bridge_probe(struct virtio_device *vdev)
{
	int err, memlen, sg_elements;
	struct virtio_qxl_bridge *brdev;
	struct virtioqxl_config *cfg;

	dprintk(DEBUG_INFO, "%s: probing\n", DRIVER_STRING);
	brdev = kmalloc(sizeof(struct virtio_qxl_bridge), GFP_KERNEL);
	if (!brdev)
		return -ENOMEM;
	dprintk(DEBUG_INFO, "%s: allocated %lu bytes as virtio_data\n",
		DRIVER_STRING, sizeof(struct virtio_qxl_bridge));

	brdev->vq = virtio_find_single_vq(vdev, NULL, "requests");
	if (IS_ERR(brdev->vq)) {
		err = PTR_ERR(brdev->vq);
		dprintk(DEBUG_ERROR, "%s: error finding vq\n", DRIVER_STRING);
		goto out_find;
	}

	cfg = &brdev->config;
	brdev->vdev = vdev;
	brdev->vdev->priv = brdev;
	spin_lock_init(&brdev->lock);
	sg_init_table(brdev->sg, SG_ELEMENTS);
	sema_init(&brdev->sem, 1);

	setup_cdev(brdev);

	get_from_host(brdev, VIRTIOQXL_GETCFG, cfg, sizeof(*cfg));
	memlen = cfg->ramsize + cfg->vramsize + cfg->romsize;
	dprintk(DEBUG_INFO,
		"%s: got config information from host: ram size %d, "
		"vram size %d, rom size %d\n", DRIVER_STRING, cfg->ramsize,
		cfg->vramsize, cfg->romsize);

	brdev->driver_mem = vmalloc(memlen);
	if (!brdev->driver_mem) {
		dprintk(DEBUG_ERROR, "%s: error allocating video memory\n",
			DRIVER_STRING);
		err = -ENOMEM;
		goto out_find;
	}
	memset(brdev->driver_mem, 0x0, memlen);
	sg_elements = memlen / PAGE_SIZE + 1 + 2;	//Memmory + alignment + head/tail
	brdev->vgasg =
	    kmalloc(sizeof(struct scatterlist) * sg_elements, GFP_KERNEL);
	if (!brdev->vgasg) {
		dprintk(DEBUG_ERROR, "%s: error allocating SG memory\n",
			DRIVER_STRING);
		err = -ENOMEM;
		goto out_sgmem;
	}
	dprintk(DEBUG_INFO, "%s: allocated %d bytes, %d SG elements\n",
		DRIVER_STRING, memlen, sg_elements);

	brdev->mem_desc.ram = brdev->driver_mem;
	brdev->mem_desc.vram = brdev->driver_mem + brdev->config.ramsize;
	brdev->mem_desc.rom = brdev->driver_mem + brdev->config.ramsize +
	    brdev->config.vramsize;
	sg_init_table(brdev->vgasg, sg_elements);

	return 0;

out_sgmem:
	vfree(brdev->driver_mem);
out_find:
	kfree(brdev);

	return err;
}

static void __devexit qxl_bridge_remove(struct virtio_device *vdev)
{
	struct virtio_qxl_bridge *brdata = vdev->priv;

	kfree(brdata->vgasg);
	vfree(brdata->driver_mem);
	cdev_del(&brdata->cdev);
	vdev->config->reset(vdev);
	vdev->config->del_vqs(vdev);
	dprintk(DEBUG_INFO, "%s: removing\n", DRIVER_STRING);
	kfree(brdata);
}

static const struct virtio_device_id id_table[] = {
	{6, VIRTIO_DEV_ANY_ID},
	{0},
};

static struct virtio_driver __refdata virtio_qxl = {
	.driver.name = KBUILD_MODNAME,
	.driver.owner = THIS_MODULE,
	.id_table = id_table,
	.probe = qxl_bridge_probe,
	.remove = __devexit_p(qxl_bridge_remove),
};

static int __init init(void)
{
	int error;
	int first_minor, ret;
	dev_t dev = 0;

	dprintk(DEBUG_INFO, "%s: init\n", DRIVER_STRING);

	first_minor = 0;

	ret = alloc_chrdev_region(&dev, first_minor, 1, "virtio-qxl-bridge");
	cdev_major = MAJOR(dev);

	if (ret < 0) {
		dprintk(DEBUG_INFO, "%s: can't get major %d\n", DRIVER_STRING,
			cdev_major);
		return ret;
	}

	sema_init(&idxsem, 1);

	virtio_qxl_class = class_create(THIS_MODULE, "virtioqxl");
	if (IS_ERR(virtio_qxl_class)) {
		dprintk(DEBUG_INFO, "%s: error creating driver class.\n",
			DRIVER_STRING);
		return PTR_ERR(virtio_qxl_class);
	}
	error = register_virtio_driver(&virtio_qxl);

	return error;
}

static void __exit fini(void)
{
	class_destroy(virtio_qxl_class);
	unregister_virtio_driver(&virtio_qxl);
	unregister_chrdev_region(cdev_major, 1);

	dprintk(DEBUG_INFO, "%s: ...exit.\n", DRIVER_STRING);
}

module_init(init);
module_exit(fini);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("VirtIO QXL bridge");
MODULE_LICENSE("GPL");
