#ifndef _MR_MPU6050_H
#define _MR_MPU6050_H

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/rwlock.h>

#define MPU6050_DEFAULT_SAMPLE_TIME_MS 200

/* 8 bit wide registers */
#define MPU6050_REG_ADDR_POWER_MGMT 0x6b
#define MPU6050_REG_ADDR_GYRO_CFG 0x1b
#define MPU6050_REG_ADDR_ACCEL_CFG 0x1c

/* 16 bit wide registers */
#define MPU6050_REG_ADDR_ACCEL_X 0x3b
#define MPU6050_REG_ADDR_ACCEL_Y 0x3d
#define MPU6050_REG_ADDR_ACCEL_Z 0x3f
#define MPU6050_REG_ADDR_GYRO_X 0x43
#define MPU6050_REG_ADDR_GYRO_Y 0x45
#define MPU6050_REG_ADDR_GYRO_Z 0x47

/* init register values */
/* power on the device */
#define MPU6050_REG_INIT_POWER_MGMT 0x00
/* 1000 deg/s */
#define MPU6050_REG_INIT_GYRO_CFG 0x10
/* 8g */
#define MPU6050_REG_INIT_ACCEL_CFG 0x10

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
    bool operational;

    rwlock_t data_rwlock;
    u16 acceleration[3];
    u16 gyroscope[3];
};

#endif /* _MR_MPU6050_H */