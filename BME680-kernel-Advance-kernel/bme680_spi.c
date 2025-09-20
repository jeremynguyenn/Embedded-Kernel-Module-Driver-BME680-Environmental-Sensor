// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include "bme680.h"

static int bme680_spi_probe(struct spi_device *spi) {
    struct regmap *regmap = devm_regmap_init_spi(spi, &bme680_regmap_config);
    if (IS_ERR(regmap)) return PTR_ERR(regmap);
    spi_set_drvdata(spi, regmap);
    return bme680_core_probe(&spi->dev, regmap, spi_get_device_id(spi)->name, spi);
}

static int bme680_spi_remove(struct spi_device *spi) {
    return bme680_core_remove(&spi->dev);
}

static const struct spi_device_id bme680_spi_id[] = {
    { "bme680", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, bme680_spi_id);

static struct spi_driver bme680_spi_driver = {
    .driver = {
        .name = "bme680_spi",
        .of_match_table = bme680_of_match,
    },
    .probe = bme680_spi_probe,
    .remove = bme680_spi_remove,
    .id_table = bme680_spi_id,
};
module_spi_driver(bme680_spi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("BME680 SPI Driver");
MODULE_VERSION("3.2");