/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Roxana Nicolescu <nicolescu.roxana1996@gmail.com>
 *
 * Copyright (c) 2019, University Politehnica of Bucharest.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/* This is derived from uknetdev because of consistency reasons */
#ifndef __UK_VSOCKDEV_CORE__
#define __UK_VSOCKDEV_CORE__

#include <uk/list.h>
#include <uk/config.h>
#include <fcntl.h>
#if defined(CONFIG_LIBUKVSOCKDEV_DISPATCHERTHREADS) || \
		defined(CONFIG_LIBUKVSOCKDEV_SYNC_IO_BLOCKED_WAITING)
#include <uk/sched.h>
#include <uk/semaphore.h>
#endif

/**
 * Unikraft vsock API common declarations.
 *
 * This header contains all API data types. Some of them are part of the
 * public API and some are part of the internal API.
 *
 * The device data and operations are separated. This split allows the
 * function pointer and driver data to be per-process, while the actual
 * configuration data for the device is shared.
 */

#ifdef __cplusplus
extern "C" {
#endif

struct uk_vsockdev;


/**
 * Function type used for queue event callbacks.
 *
 * @param dev
 *   The Unikraft Vsock Device.
 * @param queue
 *   The queue on the Unikraft vsock device on which the event happened.
 * @param argp
 *   Extra argument that can be defined on callback registration.
 */
typedef void (*uk_vsockdev_queue_event_t)(struct uk_vsockdev *dev,
					uint16_t queue_id, void *argp);

/**
 * User callback used by the driver to allocate vsockbufs
 * that are used to setup receive descriptors.
 *
 * @param argp
 *   User-provided argument.
 * @param pkts
 *   Array for vsockbuf pointers that the function should allocate.
 * @param count
 *   Number of vsockbufs requested (equal to length of pkts).
 * @return
 *   Number of successful allocated vsockbufs,
 *   has to be in range [0, count].
 *   References to allocated packets are placed to pkts[0]...pkts[count -1].
 */
typedef uint16_t (*uk_vsockdev_alloc_rxpkts)(void *argp,
					   struct virtio_vsock_packet *pkts[],
					   uint16_t count);


/**
 * A structure used to configure an Unikraft vsock device RX queue.
 */
struct uk_vsockdev_rxqueue_conf {
	uk_vsockdev_queue_event_t callback; /**< Event callback function. */
	void *callback_cookie;            /**< Argument pointer for callback. */

	struct uk_alloc *a;               /**< Allocator for descriptors. */

	uk_vsockdev_alloc_rxpkts alloc_rxpkts; /**< Allocator for rx vsockbufs */
	void *alloc_rxpkts_argp;             /**< Argument for alloc_rxpkts */
#ifdef CONFIG_LIBUKNETDEV_DISPATCHERTHREADS
	struct uk_sched *s;               /**< Scheduler for dispatcher. */
#endif
};

/**
 * A structure used to configure an Unikraft vsock device TX queue.
 */
struct uk_vsockdev_txqueue_conf {
	struct uk_alloc *a;               /* Allocator for descriptors. */
};

/** Driver callback type to set up a RX queue of an Unikraft vsock device. */
typedef struct uk_vsockdev_tx_queue * (*uk_vsockdev_txq_configure_t)(
	struct uk_vsockdev *dev, uint16_t queue_id, uint16_t nb_desc,
	struct uk_vsockdev_txqueue_conf *tx_conf);

/** Driver callback type to set up a TX queue of an Unikraft vsock device. */
typedef struct uk_vsockdev_rx_queue * (*uk_vsockdev_rxq_configure_t)(
	struct uk_vsockdev *dev, uint16_t queue_id, uint16_t nb_desc,
	struct uk_vsockdev_rxqueue_conf *rx_conf);

/** Driver callback type to set up a EV queue of an Unikraft vsock device. */
typedef struct uk_vsockdev_ev_queue * (*uk_vsockdev_evq_configure_t)(
	struct uk_vsockdev *dev, uint16_t queue_id, uint16_t nb_desc,
	struct uk_vsockdev_evqueue_conf *ev_conf);

struct uk_vsockdev_ops {
	uk_vsockdev_txq_configure_t			txq_configure;
	uk_vsockdev_rxq_configure_t			rxq_configure;
	uk_vsockdev_evq_configure_t			evq_configure;
	// uk_blkdev_queue_intr_enable_t			queue_intr_enable;
	// uk_blkdev_queue_intr_disable_t			queue_intr_disable;
	// uk_blkdev_queue_unconfigure_t			queue_unconfigure;
	// uk_blkdev_unconfigure_t				dev_unconfigure;
};

struct uk_vsockdev {
	/* Pointer to submit request function */
	// uk_blkdev_queue_submit_one_t submit_one;
	/* Pointer to handle_responses function */
	// uk_blkdev_queue_finish_reqs_t finish_reqs;
	/* Pointer to API-internal state data. */
	// struct uk_blkdev_data *_data;
	/* Functions callbacks by driver. */
	const struct uk_vsockdev_ops *dev_ops;
	/* Pointers to queues (API-private) */
	// struct uk_blkdev_queue *_queue[CONFIG_LIBUKBLKDEV_MAXNBQUEUES];
	/* Entry for list of block devices */
	// UK_TAILQ_ENTRY(struct uk_vsockdev) _list;
};

/**
 * Function type used for queue event callbacks.
 *
 * @param dev
 *   The Unikraft Vsock Device.
 * @param queue
 *   The queue on the Unikraft vsock device on which the event happened.
 * @param argp
 *   Extra argument that can be defined on callback registration.
 *
 * Note: This should call dev->finish_reqs function in order to process the
 *   received responses.
 */
typedef void (*uk_vsockdev_queue_event_t)(struct uk_vsockdev *dev,
		uint16_t queue_id, void *argp);

/**
 * Structure used to configure an Unikraft vsock device queue.
 *
 */
struct uk_vsockdev_queue_conf {
	/* Allocator used for descriptor rings */
	struct uk_alloc *a;
	/* Event callback function */
	uk_vsockdev_queue_event_t callback;
	/* Argument pointer for callback*/
	void *callback_cookie;

#if CONFIG_LIBUKVSOCKDEV_DISPATCHERTHREADS
	/* Scheduler for dispatcher. */
	struct uk_sched *s;
#endif
};

#ifdef __cplusplus
}
#endif

#endif /* __UK_VSOCKDEV_CORE__ */
