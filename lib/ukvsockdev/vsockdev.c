#define _GNU_SOURCE /* for asprintf() */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <uk/alloc.h>
#include <uk/assert.h>
#include <uk/bitops.h>
#include <uk/print.h>
#include <uk/ctors.h>
#include <uk/arch/atomic.h>
#include <uk/vsockdev.h>

/*
 * Vsock device is different from the network device where we could have
 * multiple virtualized network interface cards on a machine. It does
 * not make sense to have multiple vsock devices on a machine, therefore
 * there is no list of devices.
 */
struct uk_vsockdev *uk_vskdev = NULL;

int uk_vsockdev_drv_register(struct uk_vsockdev *dev)
{
	UK_ASSERT(dev);

	// Drv already registered
	UK_ASSERT(uk_vskdev != NULL);
	/* Data must be unallocated. */
	// UK_ASSERT(PTRISERR(dev->_data));
	/* Assert mandatory configuration. */
	// UK_ASSERT(dev->dev_ops);
	// UK_ASSERT(dev->dev_ops->dev_configure);
	// UK_ASSERT(dev->dev_ops->dev_start);
	// UK_ASSERT(dev->dev_ops->queue_configure);
	// UK_ASSERT(dev->dev_ops->get_info);
	// UK_ASSERT(dev->dev_ops->queue_get_info);
	// UK_ASSERT(dev->submit_one);
	// UK_ASSERT(dev->finish_reqs);
	// UK_ASSERT((dev->dev_ops->queue_intr_enable &&
	// 			dev->dev_ops->queue_intr_disable)
	// 		|| (!dev->dev_ops->queue_intr_enable
	// 			&& !dev->dev_ops->queue_intr_disable));

	// dev->_data = _alloc_data(a, blkdev_count,  drv_name);
	// if (!dev->_data)
		// return -ENOMEM;


	uk_vskdev = dev;
	uk_pr_info("Registered vsockdev: %p\n", dev);
	// dev->_data->state = UK_BLKDEV_UNCONFIGURED;

	return 1;
}

struct uk_vsockkdev *uk_vsockdev_get(unsigned int id)
{
	return uk_vskdev;
}

void uk_vsockdev_drv_unregister(struct uk_vsockdev *dev)
{
	UK_ASSERT(dev != NULL);

	uk_vskdev = NULL;

	uk_pr_info("Unregistered vsockdev: %p\n", dev);
}
