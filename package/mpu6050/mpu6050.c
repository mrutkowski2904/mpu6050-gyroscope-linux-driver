/**
 * Author:    Maciej Rutkowski
 * Created:   19.02.2023
 *
 * Driver for MPU6050 accelerometer/gyroscope.
 * Tested on Beaglebone Black rev C.
 **/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include "mpu6050.h"

/* --------------------- FUNCTION DECLARATIONS -------------------- */

static int device_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int device_remove(struct i2c_client *client);
static int device_thread(void *data);

/* operations for sysfs device */
ssize_t gyroscope_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t acceleration_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t probe_time_ms_show(struct device *dev, struct device_attribute *attr, char *buf);

/* --------------------------- VARIABLES -------------------------- */

struct driver_private_data driver_data;

/* dev_attr_gyroscope */
static DEVICE_ATTR(gyroscope, S_IRUGO, gyroscope_show, NULL);
/* dev_attr_acceleration */
static DEVICE_ATTR(acceleration, S_IRUGO, acceleration_show, NULL);
/* dev_attr_probe_time_ms*/
static DEVICE_ATTR(probe_time_ms, S_IRUGO, probe_time_ms_show, NULL);

struct attribute *mpu6050_sysfs_attrs[] = {
    &dev_attr_gyroscope.attr,
    &dev_attr_acceleration.attr,
    &dev_attr_probe_time_ms.attr,
    NULL,
};

const struct attribute_group mpu6050_sysfs_attrs_group = {
    .attrs = mpu6050_sysfs_attrs,
};

/* device tree match for this driver */
struct of_device_id mpu6050_device_match[] = {
    {.compatible = "org,mrmpu"},
    {},
};
MODULE_DEVICE_TABLE(of, mpu6050_device_match);

struct i2c_device_id mpu6050_ids[] = {
    {"mrmpu", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, mpu6050_ids);

struct i2c_driver mrmpu6050_platform_driver = {
    .probe = device_probe,
    .remove = device_remove,
    .id_table = mpu6050_ids,
    .driver = {
        .name = "mrmpu",
        .of_match_table = of_match_ptr(mpu6050_device_match),
    },
};

/* --------------------- FUNCTION DEFINITIONS --------------------- */

ssize_t gyroscope_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    struct device_private_data *dev_data;

    dev_data = dev_get_drvdata(dev);
    read_lock(&dev_data->data_rwlock);
    count = sprintf(buf, "%d,%d,%d\n", dev_data->gyroscope[0], dev_data->gyroscope[1], dev_data->gyroscope[2]);
    read_unlock(&dev_data->data_rwlock);
    return count;
}

ssize_t acceleration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    struct device_private_data *dev_data;

    dev_data = dev_get_drvdata(dev);
    read_lock(&dev_data->data_rwlock);
    count = sprintf(buf, "%d,%d,%d\n", dev_data->acceleration[0], dev_data->acceleration[1], dev_data->acceleration[2]);
    read_unlock(&dev_data->data_rwlock);
    return count;
}

ssize_t probe_time_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", MPU6050_DEFAULT_SAMPLE_TIME_MS);
}

static int device_thread(void *data)
{
    int status;
    u8 error_counter;
    s32 tmp_a_x, tmp_a_y, tmp_a_z;
    s32 tmp_g_x, tmp_g_y, tmp_g_z;
    struct device_private_data *dev_data;

    dev_data = (struct device_private_data *)data;
    dev_data->operational = true;
    error_counter = 0;
    status = i2c_smbus_write_byte_data(dev_data->i2c_client, MPU6050_REG_ADDR_POWER_MGMT, MPU6050_REG_INIT_POWER_MGMT);
    if (status)
    {
        dev_err(dev_data->device, "error while configuring device\n");
        dev_data->operational = false;
    }

    status = i2c_smbus_write_byte_data(dev_data->i2c_client, MPU6050_REG_ADDR_GYRO_CFG, MPU6050_REG_INIT_GYRO_CFG);
    if (status)
    {
        dev_err(dev_data->device, "error while configuring gyro feature\n");
        dev_data->operational = false;
    }
    status = i2c_smbus_write_byte_data(dev_data->i2c_client, MPU6050_REG_ADDR_ACCEL_CFG, MPU6050_REG_INIT_ACCEL_CFG);
    if (status)
    {
        dev_err(dev_data->device, "error while configuring accelerometer feature\n");
        dev_data->operational = false;
    }
    while (!kthread_should_stop())
    {
        if (!dev_data->operational)
        {
            dev_err(dev_data->device, "MPU6050 is not working!\n");
            return -ECOMM;
        }
        msleep(MPU6050_DEFAULT_SAMPLE_TIME_MS);

        /* acceleration */
        tmp_a_x = i2c_smbus_read_word_data(dev_data->i2c_client, MPU6050_REG_ADDR_ACCEL_X);
        tmp_a_y = i2c_smbus_read_word_data(dev_data->i2c_client, MPU6050_REG_ADDR_ACCEL_Y);
        tmp_a_z = i2c_smbus_read_word_data(dev_data->i2c_client, MPU6050_REG_ADDR_ACCEL_Z);

        if (IS_ERR_VALUE(tmp_a_x) || IS_ERR_VALUE(tmp_a_y) || IS_ERR_VALUE(tmp_a_z))
            error_counter++;
        else
            error_counter = 0;

        /* gyroscope */
        tmp_g_x = i2c_smbus_read_word_data(dev_data->i2c_client, MPU6050_REG_ADDR_GYRO_X);
        tmp_g_y = i2c_smbus_read_word_data(dev_data->i2c_client, MPU6050_REG_ADDR_GYRO_Y);
        tmp_g_z = i2c_smbus_read_word_data(dev_data->i2c_client, MPU6050_REG_ADDR_GYRO_Z);

        if (IS_ERR_VALUE(tmp_g_x) || IS_ERR_VALUE(tmp_g_y) || IS_ERR_VALUE(tmp_g_z))
            error_counter++;
        else
            error_counter = 0;

        if (error_counter == 10)
        {
            dev_data->operational = false;
            continue;
        }

        write_lock(&dev_data->data_rwlock);
        dev_data->acceleration[0] = be16_to_cpu(tmp_a_x);
        dev_data->acceleration[1] = be16_to_cpu(tmp_a_y);
        dev_data->acceleration[2] = be16_to_cpu(tmp_a_z);

        dev_data->gyroscope[0] = be16_to_cpu(tmp_g_x);
        dev_data->gyroscope[1] = be16_to_cpu(tmp_g_y);
        dev_data->gyroscope[2] = be16_to_cpu(tmp_g_z);
        write_unlock(&dev_data->data_rwlock);
    }

    return 0;
}

int device_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int status;
    struct device_private_data *dev_data;

    dev_info(&client->dev, "mpu6050 probe called\n");
    dev_data = devm_kzalloc(&client->dev, sizeof(struct device_private_data), GFP_KERNEL);
    if (!dev_data)
    {
        dev_err(&client->dev, "error while allocating device private data\n");
        return -ENOMEM;
    }
    dev_data->i2c_client = client;
    i2c_set_clientdata(client, dev_data);

    rwlock_init(&dev_data->data_rwlock);

    /* create device in /sys/class/mpu6050 */
    mutex_lock(&driver_data.data_mutex);
    dev_data->device = device_create(driver_data.sysfs_class, NULL, client->dev.devt, dev_data, "mpu6050_%d", driver_data.device_no);
    if (IS_ERR(dev_data->device))
    {
        mutex_unlock(&driver_data.data_mutex);
        status = PTR_ERR(dev_data->device);
        dev_err(&client->dev, "error while creating device\n");
        return status;
    }
    driver_data.device_no++;
    mutex_unlock(&driver_data.data_mutex);

    /* add attrs for this device */
    status = sysfs_create_group(&dev_data->device->kobj, &mpu6050_sysfs_attrs_group);
    if (status)
    {
        dev_err(&client->dev, "error while creating device attributes\n");
        device_destroy(driver_data.sysfs_class, dev_data->device->devt);
        return status;
    }

    dev_data->device_thread = kthread_create(device_thread, dev_data, "mpu6050_kthread");
    if (!dev_data->device_thread)
    {
        dev_err(&client->dev, "error while creating device thread\n");
        return -ECHILD;
    }

    dev_info(&client->dev, "mpu6050 probe finished\n");
    wake_up_process(dev_data->device_thread);

    return 0;
}

int device_remove(struct i2c_client *client)
{
    struct device_private_data *dev_data;

    dev_info(&client->dev, "mpu6050 remove called\n");
    dev_data = i2c_get_clientdata(client);

    if (dev_data->operational)
        kthread_stop(dev_data->device_thread);

    mutex_lock(&driver_data.data_mutex);
    device_destroy(driver_data.sysfs_class, dev_data->device->devt);
    mutex_unlock(&driver_data.data_mutex);

    dev_info(&client->dev, "mpu6050 removed\n");

    return 0;
}

static int __init mpu6050_driver_init(void)
{
    int status = 0;

    driver_data.device_no = 0;
    mutex_init(&driver_data.data_mutex);

    driver_data.sysfs_class = class_create(THIS_MODULE, "mpu6050");
    if (IS_ERR_OR_NULL(driver_data.sysfs_class))
    {
        pr_err("error while creating mpu6050 class\n");
        return PTR_ERR(driver_data.sysfs_class);
    }

    /* driver's probe will be called after this function call */
    status = i2c_add_driver(&mrmpu6050_platform_driver);
    if (status)
    {
        class_destroy(driver_data.sysfs_class);
        pr_err("error while adding i2c_driver\n");
        return status;
    }

    pr_info("mpu6050 driver initialised");
    return status;
}

static void __exit mpu6050_driver_exit(void)
{
    i2c_del_driver(&mrmpu6050_platform_driver);
    class_destroy(driver_data.sysfs_class);
}

module_init(mpu6050_driver_init);
module_exit(mpu6050_driver_exit);

MODULE_LICENSE("GPL");