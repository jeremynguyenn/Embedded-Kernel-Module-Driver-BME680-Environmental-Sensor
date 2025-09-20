// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include "bme680.h"

static int bme680_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct regmap *regmap = devm_regmap_init_i2c(client, &bme680_regmap_config);
    if (IS_ERR(regmap)) return PTR_ERR(regmap);
    i2c_set_clientdata(client, regmap);
    return bme680_core_probe(&client->dev, regmap, id->name, client);
}

static int bme680_i2c_remove(struct i2c_client *client) {
    return bme680_core_remove(&client->dev);
}

static const struct i2c_device_id bme680_id[] = {
    { "bme680", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bme680_id);

static const struct of_device_id bme680_of_match[] = {
    { .compatible = "bosch,bme680" },
    { }
};
MODULE_DEVICE_TABLE(of, bme680_of_match);

static struct i2c_driver bme680_i2c_driver = {
    .driver = {
        .name = "bme680_i2c",
        .of_match_table = bme680_of_match,
    },
    .probe = bme680_i2c_probe,
    .remove = bme680_i2c_remove,
    .id_table = bme680_id,
};
module_i2c_driver(bme680_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("BME680 I2C Driver");
MODULE_VERSION("3.2");