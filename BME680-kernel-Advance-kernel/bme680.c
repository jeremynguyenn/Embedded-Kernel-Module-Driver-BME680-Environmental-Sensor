// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/hwmon.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/events.h>
#include <linux/vmalloc.h>
#include <linux/rcupdate.h>
#include "bme680.h"

static int heater_temp = 300;
module_param(heater_temp, int, 0644);
MODULE_PARM_DESC(heater_temp, "Heater temperature in °C");

static dev_t bme680_dev_num;
static struct cdev bme680_cdev;
static struct class *bme680_class;
static struct device *bme680_device;

const struct regmap_config bme680_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0xEF,
    .cache_type = REGCACHE_RBTREE,
    .volatile_table = &bme680_volatile_table,
};

static int bme680_compensate_temperature(const struct bme680_calib *calib, int32_t adc_temp, int32_t *temp, int32_t *t_fine) {
    int64_t var1, var2;
    var1 = ((int64_t)adc_temp) - ((int64_t)calib->par_t1 << 1);
    var2 = (var1 * ((int64_t)calib->par_t2)) >> 11;
    var2 += (((var1 * var1) * ((int64_t)calib->par_t3)) >> 13);
    *t_fine = (int32_t)var2;
    *temp = (int32_t)((var2 * 5 + 128) >> 8);
    return 0;
}

static int bme680_compensate_pressure(const struct bme680_calib *calib, int32_t adc_press, int32_t t_fine, uint32_t *press) {
    int64_t var1, var2, var3;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1;
    var2 = (var2 * (int64_t)calib->par_p6) >> 17;
    var2 += ((var1 * (int64_t)calib->par_p5) << 2);
    var2 = (var2 + ((int64_t)calib->par_p4 << 35)) >> 35;
    var3 = ((int64_t)(var1 * var1) * (int64_t)var1) * ((int64_t)calib->par_p3) >> 63;
    var1 = (((int64_t)calib->par_p2 * var1) >> 2) + (var2 + var3);
    var2 = (int64_t)adc_press * 262144;
    var3 = ((int64_t)var1 * var1) >> 13;
    var1 = ((int64_t)var3 * (int64_t)calib->par_p1) >> 35;
    var2 = ((int64_t)var2 - var1) * 128;
    var1 = (calib->par_p10 * var3) >> 35;
    var3 = ((int64_t)var2 * 100) >> 47;
    *press = (uint32_t)((var3 + var1 + 256) >> 8);
    return 0;
}

static int bme680_compensate_humidity(const struct bme680_calib *calib, int32_t adc_hum, int32_t t_fine, uint32_t *hum) {
    int64_t var1, var2, var3, var4, var5, var6;
    var1 = (int64_t)t_fine - (int64_t)76800;
    var2 = ((((int64_t)adc_hum << 14) - (((int64_t)calib->par_h4) << 20) - (((int64_t)calib->par_h5) * var1)) + ((int64_t)16384)) >> 15;
    var3 = var2 * ((((int64_t)var1 * ((int64_t)calib->par_h6)) >> 9) * (((int64_t)var1 * ((int64_t)calib->par_h6)) >> 9) >> 12) >> 10;
    var4 = ((int64_t)calib->par_h2) * 128;
    var4 = (var3 + var4) >> 15;
    var5 = (((int64_t)var2 * ((int64_t)var2)) >> 8) * ((int64_t)calib->par_h1);
    var6 = (var5 >> 4) + ((int64_t)calib->par_h3 * ((int64_t)var1 * ((int64_t)var1 >> 8) * ((int64_t)var1 * ((int64_t)var1) >> 8)) >> 12;
    var6 = ((var6 >> 3) + ((int64_t)4194304)) >> 12;
    var5 = ((var4 * var6) >> 10);
    *hum = (uint32_t)((var5 >> 10) << 12);
    return 0;
}

static int bme680_compensate_gas_resistance(const struct bme680_calib *calib, uint16_t adc_gas_res, uint8_t gas_range, uint32_t *gas_res) {
    int64_t var1;
    uint64_t var2;
    int64_t var3;
    uint32_t lookup_k1_range[16], lookup_k2_range[16];
    for (int i = 0; i < 16; i++) {
        lookup_k1_range[i] = (uint32_t)(bme680_lookup_k1_range[i] * 100);
        lookup_k2_range[i] = (uint32_t)(bme680_lookup_k2_range[i] * 100);
    }
    var1 = (int64_t)((1340 + (5 * (int64_t)calib->range_sw_err)) * ((int64_t)lookup_k1_range[gas_range]));
    var1 >>= 16;
    var2 = (((int64_t)adc_gas_res << 15) - (int64_t)(16777216)) + var1;
    var3 = (((int64_t)lookup_k2_range[gas_range] * (int64_t)var1)) >> 9;
    *gas_res = (uint32_t)(var2 / var3);
    return 0;
}

static int bme680_read_calib(struct bme680_data *data) {
    uint8_t coeff_array[BME680_LEN_COEFF_ALL];
    int ret;
    ret = regmap_bulk_read(data->regmap, BME680_REG_COEFF1, coeff_array, BME680_LEN_COEFF1);
    if (ret < 0) return ret;
    ret = regmap_bulk_read(data->regmap, BME680_REG_COEFF2, &coeff_array[BME680_LEN_COEFF1], BME680_LEN_COEFF2);
    if (ret < 0) return ret;
    ret = regmap_bulk_read(data->regmap, BME680_REG_COEFF3, &coeff_array[BME680_LEN_COEFF1 + BME680_LEN_COEFF2], BME680_LEN_COEFF3);
    if (ret < 0) return ret;
    rcu_read_lock();
    data->calib.par_t1 = BME680_CONCAT_BYTES(coeff_array[BME680_IDX_T1_MSB], coeff_array[BME680_IDX_T1_LSB]);
    data->calib.par_t2 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_T2_MSB], coeff_array[BME680_IDX_T2_LSB]));
    data->calib.par_t3 = (int8_t)coeff_array[BME680_IDX_T3_MSB];
    data->calib.par_p1 = BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P1_MSB], coeff_array[BME680_IDX_P1_LSB]);
    data->calib.par_p2 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P2_MSB], coeff_array[BME680_IDX_P2_LSB]));
    data->calib.par_p3 = (int8_t)coeff_array[BME680_IDX_P3];
    data->calib.par_p4 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P4_MSB], coeff_array[BME680_IDX_P4_LSB]));
    data->calib.par_p5 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P5_MSB], coeff_array[BME680_IDX_P5_LSB]));
    data->calib.par_p6 = (int8_t)coeff_array[BME680_IDX_P6];
    data->calib.par_p7 = (int8_t)coeff_array[BME680_IDX_P7];
    data->calib.par_p8 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P8_MSB], coeff_array[BME680_IDX_P8_LSB]));
    data->calib.par_p9 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P9_MSB], coeff_array[BME680_IDX_P9_LSB]));
    data->calib.par_p10 = (uint8_t)coeff_array[BME680_IDX_P10];
    data->calib.par_h2 = (uint16_t)(((uint16_t)coeff_array[BME680_IDX_H2_MSB]) << 4 | (coeff_array[BME680_IDX_H2_LSB] & BME680_BIT_H1_DATA_MSK));
    data->calib.par_h1 = (uint16_t)((((uint16_t)coeff_array[BME680_IDX_H1_MSB]) << 4) | ((coeff_array[BME680_IDX_H1_LSB]) >> 4));
    data->calib.par_h3 = (int8_t)coeff_array[BME680_IDX_H3];
    data->calib.par_h4 = (int8_t)coeff_array[BME680_IDX_H4];
    data->calib.par_h5 = (int8_t)coeff_array[BME680_IDX_H5];
    data->calib.par_h6 = (uint8_t)coeff_array[BME680_IDX_H6];
    data->calib.par_h7 = (int8_t)coeff_array[BME680_IDX_H7];
    data->calib.par_gh1 = (int8_t)coeff_array[BME680_IDX_GH1];
    data->calib.par_gh2 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_GH2_MSB], coeff_array[BME680_IDX_GH2_LSB]));
    data->calib.par_gh3 = (int8_t)coeff_array[BME680_IDX_GH3];
    data->calib.res_heat_val = (int8_t)coeff_array[BME680_IDX_RES_HEAT_VAL];
    data->calib.res_heat_range = (coeff_array[BME680_IDX_RES_HEAT_RANGE] & BME680_RHRANGE_MSK) >> 6;
    data->calib.range_sw_err = (coeff_array[BME680_IDX_RANGE_SW_ERR] & BME680_RSERROR_MSK);
    rcu_read_unlock();
    return 0;
}

int bme680_read_data(struct bme680_data *data, struct bme680_fifo_data *fdata) {
    uint8_t buf[BME680_NUM_BULK_READ_REGS];
    int32_t adc_temp, adc_press, adc_hum;
    uint16_t adc_gas_res;
    uint8_t gas_range;
    int ret;
    mutex_lock(&data->lock);
    ret = regmap_bulk_read(data->regmap, BME680_REG_MEAS_STAT_0, buf, BME680_NUM_BULK_READ_REGS);
    if (ret < 0) {
        mutex_unlock(&data->lock);
        return ret;
    }
    adc_temp = (buf[4] << 12) | (buf[5] << 4) | (buf[6] >> 4);
    adc_press = (buf[1] << 12) | (buf[2] << 4) | (buf[3] >> 4);
    adc_hum = (buf[7] << 8) | buf[8];
    adc_gas_res = (buf[13] << 2) | (buf[14] >> 6);
    gas_range = buf[14] & BME680_GAS_RANGE_MASK;
    fdata->heat_stable = buf[14] & BME680_GAS_STAB_BIT;
    rcu_read_lock();
    bme680_compensate_temperature(&data->calib, adc_temp, &fdata->temperature, &data->t_fine);
    bme680_compensate_pressure(&data->calib, adc_press, data->t_fine, &fdata->pressure);
    bme680_compensate_humidity(&data->calib, adc_hum, data->t_fine, &fdata->humidity);
    bme680_compensate_gas_resistance(&data->calib, adc_gas_res, gas_range, &fdata->gas_resistance);
    rcu_read_unlock();
    fdata->timestamp = ktime_get_ns();
    bme680_calculate_iaq(data, &fdata->iaq_index);
    kfifo_put(&data->data_fifo, *fdata);
    wake_up_interruptible(&data->wait_data);
    mutex_unlock(&data->lock);
    return 0;
}

int bme680_calculate_iaq(struct bme680_data *data, uint32_t *iaq) {
    float iaq_score = 25.0 * log(data->fifo_data.gas_resistance / 100000.0) - (data->fifo_data.humidity / 1000.0) + 50.0;
    *iaq = iaq_score > 100 ? IAQ_GOOD : (iaq_score > 50 ? IAQ_MODERATE : IAQ_POOR);
    return 0;
}

static int bme680_trigger_handler(struct iio_trigger *trig, struct iio_poll_func *pf) {
    struct iio_dev *indio_dev = pf->indio_dev;
    struct bme680_data *data = iio_priv(indio_dev);
    struct bme680_fifo_data fdata[16];
    int i;
    mutex_lock(&data->lock);
    if (atomic_inc_and_test(&data->read_count)) {
        printk(KERN_INFO "Read count incremented\n");
    }
    if (atomic_dec_and_test(&data->error_count)) {
        printk(KERN_INFO "Error count zeroed\n");
    }
    int old_val = atomic_read(&data->read_count);
    atomic_cmpxchg(&data->read_count, old_val, old_val + 1);
    for (i = 0; i < 16; i++) {
        bme680_read_data(data, &fdata[i]);
        iio_push_to_buffers_with_timestamp(indio_dev, &fdata[i], fdata[i].timestamp);
    }
    mutex_unlock(&data->lock);
    iio_trigger_notify_done(indio_dev->trig);
    return IRQ_HANDLED;
}

static const struct file_operations bme680_fops = {
    .owner = THIS_MODULE,
    .open = bme680_open,
    .release = bme680_release,
    .read = bme680_read,
    .unlocked_ioctl = bme680_ioctl,
};

static const struct iio_info bme680_iio_info = {
    .driver_module = THIS_MODULE,
    .read_raw = bme680_read_raw,
};

int bme680_core_probe(struct device *dev, struct regmap *regmap, const char *name, void *bus_data) {
    struct bme680_data *data;
    struct iio_dev *indio_dev;
    u8 chip_id;
    int ret;
    indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
    if (!indio_dev) return -ENOMEM;
    data = iio_priv(indio_dev);
    data->indio_dev = indio_dev;
    data->regmap = regmap;
    data->dev = dev;
    data->mode = BME680_MODE_FORCED;
    mutex_init(&data->lock);
    spin_lock_init(&data->reg_lock);
    rwlock_init(&data->calib_lock);
    init_waitqueue_head(&data->wait_data);
    sema_init(&data->fifo_sem, 1);
    atomic_set(&data->read_count, 0);
    atomic_set(&data->error_count, 0);
    data->start_time = jiffies;
    ret = kfifo_alloc(&data->data_fifo, 64 * sizeof(struct bme680_fifo_data), GFP_KERNEL);
    if (ret) return ret;
    data->shm_size = PAGE_SIZE * 4;
    data->shm_buffer = vmalloc(data->shm_size);
    if (!data->shm_buffer) goto err_kfifo;
    memset(data->shm_buffer, 0, data->shm_size);
    memcpy(data->shm_buffer, &data->calib, sizeof(data->calib));
    ret = regmap_read(regmap, BME680_REG_CHIP_ID, &chip_id);
    if (ret || chip_id != BME680_CHIP_ID_VAL) {
        ret = ret ? ret : -ENODEV;
        goto err_vmalloc;
    }
    ret = bme680_read_calib(data);
    if (ret < 0) goto err_vmalloc;
    regmap_write(regmap, BME680_REG_SOFT_RESET, BME680_CMD_SOFTRESET);
    msleep(10);
    data->oversampling_temp = BME680_OVERSAMPLING_X1;
    data->oversampling_press = BME680_OVERSAMPLING_X1;
    data->oversampling_humid = BME680_OVERSAMPLING_X1;
    data->filter_coeff = BME680_FILTER_OFF;
    data->heater_temp = heater_temp;
    data->heater_dur = 150;
    data->preheat_curr_ma = 10;
    data->gas_enable = true;
    data->gas_threshold = 100000;
    ret = alloc_chrdev_region(&bme680_dev_num, 0, 1, "bme680");
    if (ret) goto err_vmalloc;
    cdev_init(&bme680_cdev, &bme680_fops);
    data->cdev = bme680_cdev;
    ret = cdev_add(&bme680_cdev, bme680_dev_num, 1);
    if (ret) goto err_chrdev;
    bme680_class = class_create(THIS_MODULE, "bme680");
    if (IS_ERR(bme680_class)) {
        ret = PTR_ERR(bme680_class);
        goto err_cdev;
    }
    bme680_device = device_create(bme680_class, dev, bme680_dev_num, data, "bme680");
    if (IS_ERR(bme680_device)) {
        ret = PTR_ERR(bme680_device);
        goto err_class;
    }
    indio_dev->name = name ? name : "bme680";
    indio_dev->channels = bme680_channels;
    indio_dev->num_channels = ARRAY_SIZE(bme680_channels);
    indio_dev->info = &bme680_iio_info;
    indio_dev->modes = INDIO_DIRECT_MODE;
    ret = iio_triggered_buffer_setup(indio_dev, NULL, bme680_trigger_handler, NULL);
    if (ret) goto err_device;
    ret = iio_device_register(indio_dev);
    if (ret) goto err_buffer;
    ret = bme680_ip glimc_init(data);
    if (ret) goto err_iio;
    data->poll_thread = kthread_run(bme680_poll_thread, data, "bme680_poll");
    if (IS_ERR(data->poll_thread)) {
        ret = PTR_ERR(data->poll_thread);
        goto err_ipc;
    }
    data->debugfs_dir = debugfs_create_dir("bme680", NULL);
    if (!IS_ERR(data->debugfs_dir))
        debugfs_create_file("stats", 0444, data->debugfs_dir, data, &bme680_debugfs_fops);
    data->hwmon_dev = hwmon_device_register_with_info(dev, "bme680", data, &bme680_hwmon_chip_info, NULL);
    if (IS_ERR(data->hwmon_dev)) {
        ret = PTR_ERR(data->hwmon_dev);
        goto err_thread;
    }
    indio_dev->buffer = devm_iio_kfifo_allocate(indio_dev);
    if (!indio_dev->buffer) goto err_hwmon;
    iio_device_attach_buffer(indio_dev, indio_dev->buffer);
    pm_runtime_set_autosuspend_delay(dev, 1000);
    pm_runtime_use_autosuspend(dev);
    pm_runtime_enable(dev);
    dev_info(dev, "BME680 Core Probed\n");
    return 0;
err_hwmon:
    hwmon_device_unregister(data->hwmon_dev);
err_thread:
    kthread_stop(data->poll_thread);
err_ipc:
    bme680_ipc_cleanup(data);
err_iio:
    iio_device_unregister(indio_dev);
err_buffer:
    iio_triggered_buffer_cleanup(indio_dev);
err_device:
    device_destroy(bme680_class, bme680_dev_num);
err_class:
    class_destroy(bme680_class);
err_cdev:
    cdev_del(&bme680_cdev);
err_chrdev:
    unregister_chrdev_region(bme680_dev_num, 1);
err_vmalloc:
    vfree(data->shm_buffer);
err_kfifo:
    kfifo_free(&data->data_fifo);
    return ret;
}

int bme680_core_remove(struct device *dev) {
    struct iio_dev *indio_dev = dev_get_drvdata(dev);
    struct bme680_data *data = iio_priv(indio_dev);
    pm_runtime_disable(dev);
    bme680_ipc_cleanup(data);
    kthread_stop(data->poll_thread);
    debugfs_remove_recursive(data->debugfs_dir);
    iio_device_unregister(indio_dev);
    iio_triggered_buffer_cleanup(indio_dev);
    hwmon_device_unregister(data->hwmon_dev);
    device_destroy(bme680_class, bme680_dev_num);
    class_destroy(bme680_class);
    cdev_del(&bme680_cdev);
    unregister_chrdev_region(bme680_dev_num, 1);
    kfifo_free(&data->data_fifo);
    vfree(data->shm_buffer);
    dev_info(dev, "BME680 Core Removed\n");
    return 0;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("BME680 Core Driver");
MODULE_VERSION("3.2");