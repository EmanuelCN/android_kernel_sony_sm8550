/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */
#ifndef __SPI_GENI_QCOM_HEADER___
#define __SPI_GENI_QCOM_HEADER___
#include <linux/types.h>

struct spi_device;
int geni_spi_get_master_irq(struct spi_device *spi_slv);

#endif /*__SPI_GENI_QCOM_HEADER___*/
