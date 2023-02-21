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

/* --------------------------- VARIABLES -------------------------- */

struct driver_private_data driver_data;

/* dev_attr_gyroscope */
static DEVICE_ATTR(gyroscope, S_IRUGO, gyroscope_show, NULL);
/* dev_attr_acceleration */
static DEVICE_ATTR(acceleration, S_IRUGO, acceleration_show, NULL);

struct attribute *mpu6050_sysfs_attrs[] = {
    &dev_attr_gyroscope.attr,
    &dev_attr_acceleration.attr,
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
    struct device_private_data *dev_data;
    dev_data = dev_get_drvdata(dev);

    dev_info(dev, "test: %d, %d, %d\n", dev_data->gyroscope[0], dev_data->gyroscope[1], dev_data->gyroscope[2]);

    /* WIP */
    return 0;
}

ssize_t acceleration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct device_private_data *dev_data;
    dev_data = dev_get_drvdata(dev);

    dev_info(dev, "test: %d, %d, %d\n", dev_data->acceleration[0], dev_data->acceleration[1], dev_data->acceleration[2]);

    /* WIP */
    return 0;
}

static int device_thread(void *data)
{
    struct device_private_data *dev_data;

    dev_data = (struct device_private_data *)data;

    while (!kthread_should_stop())
    {
        msleep(1000);
        dev_info(dev_data->device, "kthread running v2 :)\n");
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

    /* create device in /sys/class/mpu6050 */
    spin_lock(&driver_data.driver_data_lock);
    dev_data->device = device_create(driver_data.sysfs_class, NULL, client->dev.devt, dev_data, "mpu6050_%d", driver_data.device_no);
    if (IS_ERR(dev_data->device))
    {
        spin_unlock(&driver_data.driver_data_lock);
        status = PTR_ERR(dev_data->device);
        dev_err(&client->dev, "error while creating device\n");
        return status;
    }
    driver_data.device_no++;
    spin_unlock(&driver_data.driver_data_lock);

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

    kthread_stop(dev_data->device_thread);
    spin_lock(&driver_data.driver_data_lock);
    device_destroy(driver_data.sysfs_class, dev_data->device->devt);
    spin_unlock(&driver_data.driver_data_lock);

    dev_info(&client->dev, "mpu6050 removed\n");

    return 0;
}

static int __init mpu6050_driver_init(void)
{
    int status = 0;

    driver_data.device_no = 0;
    spin_lock_init(&driver_data.driver_data_lock);

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