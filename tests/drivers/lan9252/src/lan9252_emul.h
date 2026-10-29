/*
 * Copyright (c) 2026 Richard Osterloh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LAN9252_EMUL_H_
#define LAN9252_EMUL_H_

#include <stdbool.h>

#include <zephyr/drivers/emul.h>

/* Keep ECAT_CSR_CMD.BUSY set after the next command; clearing also clears BUSY. */
void lan9252_emul_set_csr_stuck_busy(const struct emul *target, bool stuck);

#endif /* LAN9252_EMUL_H_ */
