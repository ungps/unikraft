#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uk/print.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <virtio/virtio_bus.h>
#include <virtio/virtio_ids.h>
#include <uk/vsockdev.h>
#include <virtio/virtio_vsock.h>
#include <uk/sglist.h>
#include <uk/vsockdev_driver.h>

#define DRIVER_NAME		"virtio-vsock"

#define	VTVSOCK_INTR_EN				(1 << 0)
#define	VTVSOCK_INTR_EN_MASK		(1)
#define	VTVSOCK_INTR_USR_EN			(1 << 1)
#define	VTVSOCK_INTR_USR_EN_MASK	(2)

/**
 * Define max possible fragments for the vsock packets.
 */
#define VSOCK_MAX_FRAGMENTS    ((__U16_MAX >> __PAGE_SHIFT) + 2)

#define to_virtiovsockdev(vdev) \
	__containerof(vdev, struct virtio_vsock_device, vsockdev)

/* No features at the moment
 **/
#define VIRTIO_VSOCK_DRV_FEATURES(features) \
	(VIRTIO_FEATURES_UPDATE(features, 0))

typedef enum {
	VSOCK_RX = 0,
	VSOCK_TX = 1,
	VSOCK_EV = 2,
} virtq_type_t;

static struct uk_alloc *a;
static const char *drv_name = DRIVER_NAME;

struct virtio_vsock_device {
	/* Pointer to Unikraft Vscock Device */
	struct uk_vsockdev vsockdev;
	/* The vsock device identifier */
	__u16 uid;
	/* The vsock context identifier */
	__u16 cid;
	/* Virtio Device */
	struct virtio_dev *vdev;
	/* List of all the virtqueue in the pci device */
	struct virtqueue *vq;
	/* Event queue */
	struct   uk_vsockdev_ev_queue *evq;
	/* Receive queue */
	struct   uk_vsockdev_rx_queue *rxq;
	/* Transmit queue */
	struct   uk_vsockdev_tx_queue *txq;
};


/**
 * @internal structure to represent the transmit queue.
 */
struct uk_vsockdev_tx_queue {
	/* The virtqueue reference */
	struct virtqueue *vq;
	/* The hw queue identifier */
	uint16_t hwvq_id;
	/* The user queue identifier */
	uint16_t lqueue_id;
	/* The nr. of descriptor limit */
	uint16_t max_nb_desc;
	/* The nr. of descriptor user configured */
	uint16_t nb_desc;
	/* The flag to interrupt on the transmit queue */
	uint8_t intr_enabled;
	/* Reference to the uk_vsockdev */
	struct uk_vsockdev *vskdev;
	/* The scatter list and its associated fragements */
	struct uk_sglist sg;
	struct uk_sglist_seg *sgsegs;
};

/**
 * @internal structure to represent the receive queue.
 */
struct uk_vsockdev_rx_queue {
	/* The virtqueue reference */
	struct virtqueue *vq;
	/* The virtqueue hw identifier */
	uint16_t hwvq_id;
	/* The libukvsock queue identifier */
	uint16_t lqueue_id;
	/* The nr. of descriptor limit */
	uint16_t max_nb_desc;
	/* The nr. of descriptor user configured */
	uint16_t nb_desc;
	/* The flag to interrupt on the transmit queue */
	uint8_t intr_enabled;
	/* Reference to the uk_vsockdev */
	struct uk_vsockdev *vskdev;
	/* The scatter list and its associated fragements */
	struct uk_sglist sg;
	struct uk_sglist_seg *sgsegs;
};


/**
 * @internal structure to represent the event queue.
 */
struct uk_vsockdev_ev_queue {
	/* The virtqueue reference */
	struct virtqueue *vq;
	/* The hw queue identifier */
	uint16_t hwvq_id;
	/* The user queue identifier */
	uint16_t lqueue_id;
	/* The nr. of descriptor limit */
	uint16_t max_nb_desc;
	/* The nr. of descriptor user configured */
	uint16_t nb_desc;
	/* The flag to interrupt on the event queue */
	uint8_t intr_enabled;
	/* Reference to the uk_vsockdev */
	struct uk_vsockdev *vskdev;
	/* The scatter list and its associated fragements */
	struct uk_sglist sg;
	struct uk_sglist_seg *sgsegs;
};

static int virtio_vsockdev_rxq_enqueue(struct uk_vsockdev_rx_queue *rxq,
		struct virtio_vsock_packet *pkt)
{
	__u8 *start;
	size_t len = 0;
	int rc = 0;
	struct uk_sglist *sg;

	UK_ASSERT(rxq);
	UK_ASSERT(pkt);

	if (virtqueue_is_full(rxq->vq)) {
		uk_pr_debug("The virtqueue is full\n");
		return -ENOSPC;
	}

	start = pkt->data;
	len = pkt->hdr.len;

	sg = &rxq->sg;
	uk_sglist_reset(sg);

	/* Appending the header buffer to the sglist */
	uk_sglist_append(sg, &pkt->hdr, sizeof(struct virtio_vsock_hdr));

	/* Appending the data buffer to the sglist */
	uk_sglist_append(sg, start, len);

	rc = virtqueue_buffer_enqueue(rxq->vq, pkt, &rxq->sg,
				       0, sg->sg_nseg);

	return rc;
}

static int virtio_vsockdev_rxq_fillup(struct uk_vsockdev_rx_queue *rxq,
				   __u16 nb_desc,
				   int notify)
{
	struct virtio_vsock_packet *pkt;
	int rc = 0;
	__u16 i;

	for (i = 0; i < nb_desc; i++) {
		pkt = uk_calloc(a, 1, sizeof(*pkt));
		if (unlikely(!pkt)) {
			uk_pr_info("Couldn't alloc rx packet\n");
			break;
		}

		pkt->data = uk_calloc(a, VIRTIO_VSOCK_RX_DATA_SIZE, sizeof(*pkt->data));
		if (unlikely(!pkt->data)) {
			uk_pr_info("Couldn't alloc rx data\n");
			break;
		}

		rc = virtio_vsockdev_rxq_enqueue(rxq, pkt);
		if (unlikely(rc < 0)) {
			uk_pr_err("Failed to add a buffer to receive virtqueue %p: %d\n",
				rxq, rc);
			uk_free(a, pkt);
			break;
		}
	}

out:
	uk_pr_debug("Programmed %d receive vsock to receive virtqueue %p\n", i, rxq);

	/**
	 * Notify the host, when we submit new descriptor(s).
	 */
	if (notify && i)
		virtqueue_host_notify(rxq->vq);

	return 0;
}

static int virtio_vsockdev_evq_enqueue(struct uk_vsockdev_ev_queue *evq,
		struct virtio_vsock_event *ev)
{
	int rc = 0;
	struct uk_sglist *sg;

	UK_ASSERT(evq);
	UK_ASSERT(ev);

	if (virtqueue_is_full(evq->vq)) {
		uk_pr_debug("The virtqueue is full\n");
		return -ENOSPC;
	}

	sg = &evq->sg;
	uk_sglist_reset(sg);

	/* Appending the event to the sglist */
	uk_sglist_append(sg, &ev, sizeof(struct virtio_vsock_event));

	rc = virtqueue_buffer_enqueue(evq->vq, ev, &evq->sg,
				       0, sg->sg_nseg);

	return rc;
}

static int virtio_vsockdev_evq_fillup(struct uk_vsockdev_ev_queue *evq,
				   __u16 nb_desc,
				   int notify)
{
	struct virtio_vsock_event *ev;
	int rc = 0;
	__u16 i;

	for (i = 0; i < nb_desc; i++) {
		ev = uk_calloc(a, 1, sizeof(*ev));
		if (unlikely(!ev)) {
			uk_pr_info("Couldn't alloc event buffer\n");
			break;
		}

		rc = virtio_vsockdev_evq_enqueue(evq, ev);
		if (unlikely(rc < 0)) {
			uk_pr_err("Failed to add a buffer to receive virtqueue %p: %d\n",
				evq, rc);
			uk_free(a, ev);
			break;
		}
	}

out:
	uk_pr_debug("Programmed %d receive vsock to receive virtqueue %p\n", i, evq);

	/**
	 * Notify the host, when we submit new descriptor(s).
	 */
	if (notify && i)
		virtqueue_host_notify(evq->vq);

	return 0;
}


// static int virtio_blkdev_queue_dequeue(struct uk_blkdev_queue *queue,
// 		struct uk_blkreq **req)
// {
// 	int ret = 0;
// 	__u32 len;
// 	struct virtio_blkdev_request *response_req;

// 	UK_ASSERT(req);
// 	*req = NULL;

// 	ret = virtqueue_buffer_dequeue(queue->vq, (void **) &response_req,
// 			&len);
// 	if (ret < 0) {
// 		uk_pr_info("No data available in the queue\n");
// 		return 0;
// 	}

// 	/* We need at least one byte for the result status */
// 	if (unlikely(len < 1)) {
// 		uk_pr_err("Received invalid response size: %u\n", len);
// 		ret = -EINVAL;
// 		goto out;
// 	}

// 	*req = response_req->req;
// 	(*req)->result = -response_req->status;

// out:
// 	uk_free(a, response_req);
// 	return ret;
// }

// static int virtio_blkdev_complete_reqs(struct uk_blkdev *dev,
// 		struct uk_blkdev_queue *queue)
// {
// 	struct uk_blkreq *req;
// 	int rc = 0;

// 	UK_ASSERT(dev);

// 	/* Queue interrupts have to be off when calling receive */
// 	UK_ASSERT(!(queue->intr_enabled & VTBLK_INTR_EN));

// moretodo:
// 	for (;;) {
// 		rc = virtio_blkdev_queue_dequeue(queue, &req);
// 		if (unlikely(rc < 0)) {
// 			uk_pr_err("Failed to dequeue the request: %d\n", rc);
// 			goto err_exit;
// 		}

// 		if (!req)
// 			break;

// 		uk_blkreq_finished(req);
// 		if (req->cb)
// 			req->cb(req, req->cb_cookie);
// 	}

// 	/* Enable interrupt only when user had previously enabled it */
// 	if (queue->intr_enabled & VTBLK_INTR_USR_EN_MASK) {
// 		rc = virtqueue_intr_enable(queue->vq);
// 		if (rc == 1)
// 			goto moretodo;
// 	}

// 	return 0;

// err_exit:
// 	return rc;
// }

static int virtio_vsockdev_recv_done(struct virtqueue *vq, void *priv)
{
	struct uk_vsockdev_rx_queue *queue = NULL;

	UK_ASSERT(vq && priv);

	queue = (struct uk_vsockdev_rx_queue *) priv;

	/* Disable the interrupt for the ring */
	virtqueue_intr_disable(vq);
	queue->intr_enabled &= ~(VTVSOCK_INTR_EN);

	// TODO
	// uk_vsockdev_drv_queue_event(&queue->vskdev->vsockdev, queue->lqueue_id);

	return 1;
}

// static int virtio_blkdev_queue_intr_enable(struct uk_blkdev *dev,
// 		struct uk_blkdev_queue *queue)
// {
// 	int rc = 0;

// 	UK_ASSERT(dev);

// 	/* If the interrupt is enabled */
// 	if (queue->intr_enabled & VTBLK_INTR_EN)
// 		return 0;

// 	/**
// 	 * Enable the user configuration bit. This would cause the interrupt to
// 	 * be enabled automatically, if the interrupt could not be enabled now
// 	 * due to data in the queue.
// 	 */
// 	queue->intr_enabled = VTBLK_INTR_USR_EN;
// 	rc = virtqueue_intr_enable(queue->vq);
// 	if (!rc)
// 		queue->intr_enabled |= VTBLK_INTR_EN;

// 	return rc;
// }

// static int virtio_blkdev_queue_intr_disable(struct uk_blkdev *dev,
// 		struct uk_blkdev_queue *queue)
// {
// 	UK_ASSERT(dev);
// 	UK_ASSERT(queue);

// 	virtqueue_intr_disable(queue->vq);
// 	queue->intr_enabled &= ~(VTBLK_INTR_USR_EN | VTBLK_INTR_EN);

// 	return 0;
// }

/**
 * This function setup the vring infrastructure.
 */
static int virtio_vsockdev_vqueue_setup(struct virtio_vsock_device *vskdev,
		uint16_t queue_id, uint16_t nr_desc, virtq_type_t queue_type,
		struct uk_alloc *a)
{
	int rc = 0;
	int id = 0;
	virtqueue_callback_t callback;
	uint16_t max_desc, hwvq_id;
	struct virtqueue *vq;

	if (queue_type == VSOCK_RX) {
		id = VSOCK_RX;
		callback = virtio_vsockdev_recv_done;
		max_desc = vskdev->rxq->max_nb_desc;
		hwvq_id = vskdev->rxq->hwvq_id;
	} else if (queue_type == VSOCK_TX) {
		id = VSOCK_TX;
		/* We don't support the callback from the txqueue yet */
		callback = NULL;
		max_desc = vskdev->txq->max_nb_desc;
		hwvq_id = vskdev->txq->hwvq_id;
	} else {
		id = VSOCK_EV;
		/* We don't support the callback from the evqueue yet */
		callback = NULL;
		max_desc = vskdev->evq->max_nb_desc;
		hwvq_id = vskdev->evq->hwvq_id;
	}

	if (unlikely(max_desc < nr_desc)) {
		uk_pr_err("Max allowed desc: %"__PRIu16" Requested desc:%"__PRIu16"\n",
			  max_desc, nr_desc);
		return -ENOBUFS;
	}
	nr_desc = (nr_desc != 0) ? nr_desc : max_desc;

	uk_pr_debug("Configuring the %d descriptors\n", nr_desc);
	/* Check if the descriptor is a power of 2 */
	if (unlikely(nr_desc & (nr_desc - 1))) {
		uk_pr_err("Expect descriptor count as a power 2\n");
		return -EINVAL;
	}

	vq = virtio_vqueue_setup(vskdev->vdev, hwvq_id, nr_desc, callback, a);
	if (unlikely(PTRISERR(vq))) {
		uk_pr_err("Failed to set up virtqueue %"__PRIu16"\n",
			  queue_id);
		rc = PTR2ERR(vq);
		return rc;
	}

	if (queue_type == VSOCK_RX) {
		vq->priv = &vskdev->rxq;
		vskdev->rxq->vskdev = &vskdev->vsockdev;
		vskdev->rxq->vq = vq;
		vskdev->rxq->nb_desc = nr_desc;
		vskdev->rxq->lqueue_id = queue_id;
	} else if (queue_type == VSOCK_TX) {
		vskdev->txq->vskdev = &vskdev->vsockdev;
		vskdev->txq->vq = vq;
		vskdev->txq->nb_desc = nr_desc;
		vskdev->txq->lqueue_id = queue_id;
	} else {
		vskdev->txq->vskdev = &vskdev->vsockdev;
		vskdev->txq->vq = vq;
		vskdev->txq->nb_desc = nr_desc;
		vskdev->txq->lqueue_id = queue_id;
	}

	return id;
}

static struct uk_vsockdev_rx_queue *virtio_vsockdev_rx_queue_setup(
				struct uk_vsockdev *dev, uint16_t queue_id,
				uint16_t nb_desc,
				struct uk_vsockdev_rxqueue_conf *conf)
{
	struct virtio_vsock_device *vskdev;
	struct uk_vsockdev_rx_queue *rxq = NULL;
	int rc;

	UK_ASSERT(dev);
	UK_ASSERT(conf);

	vskdev = to_virtiovsockdev(dev);
	if (queue_id >= 3) {
		uk_pr_err("Invalid virtqueue identifier: %"__PRIu16"\n",
			  queue_id);
		rc = -EINVAL;
		goto err_exit;
	}
	/* Setup the virtqueue with the descriptor */
	uk_pr_info("queue_id: %d, nb_desc: %d, queue_type %d\n", queue_id, nb_desc, VSOCK_RX);
	rc = virtio_vsockdev_vqueue_setup(vskdev, queue_id, nb_desc, VSOCK_RX,
					conf->a);
	if (rc < 0) {
		uk_pr_err("Failed to set up virtqueue %"__PRIu16": %d\n",
			  queue_id, rc);
		goto err_exit;
	}
	rxq  = vskdev->rxq;

	/* Allocate receive buffers for this queue */
	virtio_vsockdev_rxq_fillup(rxq, rxq->nb_desc, 0);

exit:
	return rxq;

err_exit:
	rxq = ERR2PTR(rc);
	goto exit;
}

static struct uk_vsockdev_rx_queue *virtio_vsockdev_ev_queue_setup(
				struct uk_vsockdev *dev, uint16_t queue_id,
				uint16_t nb_desc,
				struct uk_vsockdev_evqueue_conf *conf)
{
	struct virtio_vsock_device *vskdev;
	struct uk_vsockdev_ev_queue *evq = NULL;
	int rc;

	UK_ASSERT(dev);
	UK_ASSERT(conf);

	vskdev = to_virtiovsockdev(dev);
	if (queue_id >= 3) {
		uk_pr_err("Invalid virtqueue identifier: %"__PRIu16"\n",
			  queue_id);
		rc = -EINVAL;
		goto err_exit;
	}
	/* Setup the virtqueue with the descriptor */
	rc = virtio_vsockdev_vqueue_setup(vskdev, queue_id, nb_desc, VSOCK_EV,
					conf->a);
	if (rc < 0) {
		uk_pr_err("Failed to set up virtqueue %"__PRIu16": %d\n",
			  queue_id, rc);
		goto err_exit;
	}
	evq = vskdev->evq;

	/* Allocate receive buffers for this queue */
	virtio_vsockdev_evq_fillup(evq, evq->nb_desc, 0);

exit:
	return evq;

err_exit:
	evq = ERR2PTR(rc);
	goto exit;
}

static struct uk_vsockdev_tx_queue *virtio_vsockdev_tx_queue_setup(struct uk_vsockdev *dev,
		uint16_t queue_id,
		uint16_t nb_desc,
		const struct uk_vsockdev_queue_conf *queue_conf)
{
	struct virtio_vsock_device *vskdev;
	int rc = 0;
	struct uk_vsockdev_tx_queue *queue;

	UK_ASSERT(dev != NULL);
	UK_ASSERT(queue_conf != NULL);

	vskdev = to_virtiovsockdev(dev);
	if (unlikely(queue_id >= 3)) {
		uk_pr_err("Invalid queue_id %"__PRIu16"\n", queue_id);
		rc = -EINVAL;
		goto err_exit;
	}

	/* Setup the virtqueue */
	rc = virtio_vsockdev_vqueue_setup(vskdev, queue_id, nb_desc, VSOCK_TX,
				queue_conf->a);
	if (rc < 0) {
		uk_pr_err("Failed to set up virtqueue %"__PRIu16": %d\n",
			  queue_id, rc);
		goto err_exit;
	}

exit:
	return queue;
err_exit:
	queue = ERR2PTR(rc);
	goto exit;
}

static int virtio_vsockdev_queues_alloc(struct virtio_vsock_device *vskdev)
{
	int rc = 0;
	uint16_t i = 0;
	int vq_avail = 0;
	int total_queues = 3; // one for rx, one for tx, one for ev
	__u16 qdesc_size[total_queues];

	vskdev->rxq = uk_calloc(a, 1, sizeof(*vskdev->rxq));
	vskdev->txq = uk_calloc(a, 1, sizeof(*vskdev->txq));
	vskdev->evq = uk_calloc(a, 1, sizeof(*vskdev->evq));

	if (unlikely(vskdev->rxq == NULL) || unlikely(vskdev->txq == NULL) ||
		unlikely(vskdev->evq == NULL)) {
		uk_pr_err("Failed to allocate memory for queue management\n");
		rc = -ENOMEM;
		goto exit;
	}

	vq_avail = virtio_find_vqs(vskdev->vdev, total_queues, qdesc_size);
	if (unlikely(vq_avail != total_queues)) {
		uk_pr_err("Expected: %d queues, Found: %d queues\n",
				total_queues, vq_avail);
		rc = -ENOMEM;
		goto err_free_txrxev;
	}

	/**
	 * Initialize the received queue with the information received
	 * from the device.
	 */
	vskdev->rxq->hwvq_id = 0;
	vskdev->rxq->max_nb_desc = qdesc_size[vskdev->rxq->hwvq_id];
	uk_sglist_init(&vskdev->rxq->sg,
				(sizeof(vskdev->rxq->sgsegs) /
			sizeof(vskdev->rxq->sgsegs[0])),
				&vskdev->rxq->sgsegs[0]);

	/**
	 * Initialize the transmit queue with the information received
	 * from the device.
	 */
	vskdev->txq->hwvq_id = 1;
	vskdev->txq->max_nb_desc = qdesc_size[vskdev->txq->hwvq_id];
	uk_sglist_init(&vskdev->txq->sg,
				(sizeof(vskdev->txq->sgsegs) /
			sizeof(vskdev->txq->sgsegs[0])),
				&vskdev->txq->sgsegs[0]);

	/**
	 * Initialize the event queue with the information received
	 * from the device.
	 */
	vskdev->evq->hwvq_id = 2;
	vskdev->evq->max_nb_desc = qdesc_size[vskdev->evq->hwvq_id];
	uk_sglist_init(&vskdev->evq->sg,
				(sizeof(vskdev->evq->sgsegs) /
			sizeof(vskdev->evq->sgsegs[0])),
				&vskdev->evq->sgsegs[0]);

exit:
	return rc;

err_free_txrxev:
	if (!vskdev->rxq)
		uk_free(a, vskdev->rxq);
	if (!vskdev->txq)
		uk_free(a, vskdev->txq);
	if (!vskdev->txq)
		uk_free(a, vskdev->evq);
	goto exit;
}

static int virtio_vsockdev_start(struct uk_vsockdev *dev)
{
	struct virtio_vsock_device *d;

	UK_ASSERT(dev != NULL);

	d = to_virtiovsockdev(dev);
	virtio_dev_drv_up(d->vdev);

	uk_pr_info(DRIVER_NAME": %"__PRIu16" started\n", d->uid);

	return 0;
}

// /* If one queue has unconsumed responses it returns -EBUSY
//  * TODO restart doesn't work
//  **/
static int virtio_blkdev_stop(struct uk_vsockdev *dev)
{
	struct virtio_vsock_device *d;
	uint16_t q_id;
	int rc = 0;

	UK_ASSERT(dev != NULL);

	d = to_virtiovsockdev(dev);
	if (virtqueue_hasdata(d->rxq->vq)) {
		uk_pr_err("Queue:%"__PRIu16" has unconsumed responses\n",
				q_id);
		return -EBUSY;
	}
	if (virtqueue_hasdata(d->txq->vq)) {
		uk_pr_err("Queue:%"__PRIu16" has unconsumed responses\n",
				q_id);
		return -EBUSY;
	}
	if (virtqueue_hasdata(d->evq->vq)) {
		uk_pr_err("Queue:%"__PRIu16" has unconsumed responses\n",
				q_id);
		return -EBUSY;
	}

	rc = virtio_dev_reset(d->vdev);
	if (rc) {
		uk_pr_info(DRIVER_NAME":%"__PRIu16" stopped", d->uid);
		goto out;
	}

	uk_pr_warn(DRIVER_NAME":%"__PRIu16" Start is not allowed!!!", d->uid);

out:
	return rc;
}

static int virtio_vsockdev_unconfigure(struct uk_vsockdev *dev)
{
	struct virtio_vsock_device *d;

	UK_ASSERT(dev != NULL);
	d = to_virtiovsockdev(dev);
	uk_free(a, d->rxq);
	uk_free(a, d->txq);
	uk_free(a, d->evq);

	return 0;
}

static int virtio_vsockdev_feature_negotiate(struct virtio_vsock_device *vskdev)
{
	int bytes_to_read;
	int guest_cid;
	__u16 num_queues;
	__u32 max_segments;
	__u32 max_size_segment;
	int rc = 0;

	UK_ASSERT(vskdev);

	/* Get size of device */
	bytes_to_read = virtio_config_get(vskdev->vdev,
			__offsetof(struct virtio_vsock_config, guest_cid),
			&guest_cid,
			sizeof(guest_cid),
			1);
	if (bytes_to_read != sizeof(guest_cid))  {
		uk_pr_err("Failed to get cid from device %d\n", rc);
		rc = -EAGAIN;
		goto exit;
	}
	if (guest_cid < 3) {
		uk_pr_err("Invalid context id for vsock (%d) device\n", guest_cid);
		rc = -EAGAIN;
		goto exit;
	}
	
	vskdev->cid = guest_cid;

exit:
	return rc;
}

static inline void virtio_vsockdev_feature_set(struct virtio_vsock_device *vskdev)
{
	vskdev->vdev->features = 0;

	/* Setting the feature the driver support */
	VIRTIO_VSOCK_DRV_FEATURES(vskdev->vdev->features);
}

static const struct uk_vsockdev_ops virtio_vsockdev_ops = {
//  	.dev_configure = virtio_vsockdev_configure,
//  	.txq_configure = virtio_vsockdev_tx_queue_setup,
// 		.rxq_intr_enable = virtio_vsockdev_rxq_intr_enable,
// 		.rxq_intr_disable = virtio_vsockdev_rxq_intr_disable,
// 		.txq_intr_enable = virtio_vsockdev_txq_intr_enable,
// 		.txq_intr_disable = virtio_vsockdev_txq_intr_disable,
// 		.evq_intr_enable = virtio_vsockdev_evq_intr_enable,
// 		.evq_intr_disable = virtio_vsockdev_evq_intr_disable,

// 		.dev_start = virtio_blkdev_start,
// 		.dev_stop = virtio_blkdev_stop,
// 		.queue_unconfigure = virtio_blkdev_queue_release,
// 		.dev_unconfigure = virtio_blkdev_unconfigure,
};

static int virtio_vsock_add_dev(struct virtio_dev *vdev)
{
	struct virtio_vsock_device *vskdev;
	struct uk_vsockdev_rxqueue_conf rx_conf;
	struct uk_vsockdev_evqueue_conf ev_conf;
	int rc = 0;

	UK_ASSERT(vdev != NULL);

	vskdev = uk_calloc(a, 1, sizeof(*vskdev));
	if (!vskdev)
		return -ENOMEM;

	vskdev->vdev = vdev;
	vskdev->vsockdev.dev_ops = &virtio_vsockdev_ops;

	rc = uk_vsockdev_drv_register(&vskdev->vsockdev);
	if (rc < 0) {
		uk_pr_err("Failed to register virtio_vsock device: %d\n", rc);
		goto err_out;
	}

	vskdev->uid = rc;
	virtio_vsockdev_feature_set(vskdev);
	rc = virtio_vsockdev_feature_negotiate(vskdev);
	if (rc) {
		uk_pr_err("Failed to negotiate the device feature %d\n", rc);
		goto err_negotiate_feature;
	}

	uk_pr_info("Virtio-vsock device with cid %d registered with libukvsockdev\n", vskdev->cid);

	rc = virtio_vsockdev_queues_alloc(vskdev);
	rx_conf.a = a;
	rc = virtio_vsockdev_rx_queue_setup(vskdev, VSOCK_RX, 0, &rx_conf);
	ev_conf.a = a;
	rc = virtio_vsockdev_ev_queue_setup(vskdev, VSOCK_EV, 0, &ev_conf);

out:
	return rc;
err_negotiate_feature:
	virtio_dev_status_update(vskdev->vdev, VIRTIO_CONFIG_STATUS_FAIL);
err_out:
	uk_free(a, vskdev);
	goto out;
}

static int virtio_vsock_drv_init(struct uk_alloc *drv_allocator)
{
	/* driver initialization */
	if (!drv_allocator)
		return -EINVAL;

	a = drv_allocator;
	return 0;
}

static const struct virtio_dev_id vvsock_dev_id[] = {
	{VIRTIO_ID_VSOCK},
	{VIRTIO_ID_INVALID} /* List Terminator */
};

static struct virtio_driver vsock_drv = {
	.dev_ids = vvsock_dev_id,
	.init    = virtio_vsock_drv_init,
	.add_dev = virtio_vsock_add_dev
};
VIRTIO_BUS_REGISTER_DRIVER(&vsock_drv);
