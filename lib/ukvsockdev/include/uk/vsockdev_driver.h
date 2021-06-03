#ifndef __UK_VSOCKDEV_DRIVER__
#define __UK_VSOCKDEV_DRIVER__

#include <uk/vsockdev_core.h>
#include <uk/assert.h>

/**
 * Unikraft block driver API.
 *
 * This header contains all API functions that are supposed to be called
 * by a block device driver.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Assign Unikraft vsock device.
 * This should be called whenever a driver adds a new device.
 *
 * @param dev
 *	Struct to unikraft vsock device that shall be registered
 * @return
 *	- (-ENOMEM): Allocation of private
 *	- (>=0): Vsock device ID on success
 */
int uk_vsockdev_drv_register(struct uk_vsockdev *dev);

/**
 * Frees the data allocated for the Unikraft Block Device.
 * Removes the block device from the list.
 *
 * @param dev
 *	Unikraft block device
 */
void uk_blkdev_drv_unregister();

#ifdef __cplusplus
}
#endif

#endif /* __UK_VSOCKDEV_DRIVER__ */
