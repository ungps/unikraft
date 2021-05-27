#ifndef __PLAT_DRV_VIRTIO_VSOCK_H
#define __PLAT_DRV_VIRTIO_VSOCK_H
#include <virtio/virtio_ids.h>
#include <virtio/virtio_config.h>
#include <virtio/virtio_types.h>

struct virtio_vsock_config {
	__u64 guest_cid;
} __packed;

enum virtio_vsock_event_id {
	VIRTIO_VSOCK_EVENT_TRANSPORT_RESET = 0,
};

struct virtio_vsock_event {
	__u32 id;
} __packed;

struct virtio_vsock_hdr {
	__u64	src_cid;
	__u64	dst_cid;
	__u32	src_port;
	__u32	dst_port;
	__u32	len;
	__u16	type;
	__u16	op;
	__u32	flags;
	__u32	buf_alloc;
	__u32	fwd_cnt;
} __packed;

enum virtio_vsock_type {
	VIRTIO_VSOCK_TYPE_STREAM = 1,
};

enum virtio_vsock_op {
	VIRTIO_VSOCK_OP_INVALID = 0,

	/* Connect operations */
	VIRTIO_VSOCK_OP_REQUEST = 1,
	VIRTIO_VSOCK_OP_RESPONSE = 2,
	VIRTIO_VSOCK_OP_RST = 3,
	VIRTIO_VSOCK_OP_SHUTDOWN = 4,

	/* To send payload */
	VIRTIO_VSOCK_OP_RW = 5,

	/* Tell the peer our credit info */
	VIRTIO_VSOCK_OP_CREDIT_UPDATE = 6,
	/* Request the peer to send the credit info to us */
	VIRTIO_VSOCK_OP_CREDIT_REQUEST = 7,
};

/* VIRTIO_VSOCK_OP_SHUTDOWN flags values */
enum virtio_vsock_shutdown {
	VIRTIO_VSOCK_SHUTDOWN_RCV = 1,
	VIRTIO_VSOCK_SHUTDOWN_SEND = 2,
};

#endif /* __PLAT_DRV_VIRTIO_VSOCK_H */
