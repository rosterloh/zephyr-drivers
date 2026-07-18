/*
 * Copyright (c) 2026 Richard Osterloh <richard.osterloh@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-device callback slist. Storage lives in the backend's data struct;
 * the subsystem locates it via an offset stored in the backend's config.
 *
 * Backends declare:
 *
 *   struct my_data {
 *       struct actuator_common_data common;
 *       struct actuator_cb_storage cb_storage;
 *       struct actuator_cb_node cb_pool[CONFIG_ACTUATOR_MAX_CALLBACKS_PER_DEVICE];
 *       ...
 *   };
 *
 *   struct my_config {
 *       struct actuator_cb_offsets cb_offsets;  // MUST be first member
 *       ...
 *   };
 *
 * The offset (set via offsetof at DEVICE_DT_INST_DEFINE time) tells this
 * file where to find cb_storage inside the data struct.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>
#include <zephyr/actuator/actuator.h>

static struct actuator_cb_storage *storage_for(const struct device *dev)
{
	const struct actuator_cb_offsets *off = (const struct actuator_cb_offsets *)dev->config;
	return (struct actuator_cb_storage *)((char *)dev->data + off->storage_offset);
}

static int register_cb(const struct device *dev, int kind, void *fn, void *ud)
{
	struct actuator_cb_storage *s = storage_for(dev);

	atomic_val_t prev = atomic_inc(&s->used);
	if ((size_t)prev >= s->pool_n) {
		atomic_dec(&s->used);
		return -ENOMEM;
	}

	struct actuator_cb_node *n = &s->pool[prev];
	n->kind = kind;
	if (kind == ACTUATOR_CB_KIND_STATE) {
		n->fn.state = (actuator_state_cb_t)fn;
	} else {
		n->fn.feedback = (actuator_feedback_cb_t)fn;
	}
	n->user_data = ud;
	sys_slist_append(&s->list, &n->node);
	return 0;
}

int actuator_register_state_cb(const struct device *dev, actuator_state_cb_t cb, void *user_data)
{
	return register_cb(dev, ACTUATOR_CB_KIND_STATE, (void *)cb, user_data);
}

int actuator_register_feedback_cb(const struct device *dev, actuator_feedback_cb_t cb,
				  void *user_data)
{
	return register_cb(dev, ACTUATOR_CB_KIND_FEEDBACK, (void *)cb, user_data);
}

void actuator_callbacks_fire_state(const struct device *dev, enum actuator_state new_state)
{
	struct actuator_cb_storage *s = storage_for(dev);
	struct actuator_cb_node *n;
	SYS_SLIST_FOR_EACH_CONTAINER(&s->list, n, node) {
		if (n->kind == ACTUATOR_CB_KIND_STATE) {
			n->fn.state(dev, new_state, n->user_data);
		}
	}
}

void actuator_callbacks_fire_feedback(const struct device *dev, const struct actuator_feedback *fb)
{
	struct actuator_cb_storage *s = storage_for(dev);
	struct actuator_cb_node *n;
	SYS_SLIST_FOR_EACH_CONTAINER(&s->list, n, node) {
		if (n->kind == ACTUATOR_CB_KIND_FEEDBACK) {
			n->fn.feedback(dev, fb, n->user_data);
		}
	}
}
