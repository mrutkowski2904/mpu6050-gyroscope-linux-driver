#ifndef _MR_MPU6050_H
#define _MR_MPU6050_H

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/wait.h>
#include <linux/kthread.h>

struct driver_private_data
{
    spinlock_t driver_data_lock;
    struct class *sysfs_class;
    int device_no;
};

struct device_private_data
{
    struct i2c_client *i2c_client;
    struct device *device;
    struct task_struct *device_thread;

    u16 acceleration[3];
    u16 gyroscope[3];
};

#endif /* _MR_MPU6050_H */