MPU6050_VERSION = 1.0
MPU6050_SITE = $(BR2_EXTERNAL)/package/mpu6050
MPU6050_SITE_METHOD = local
$(eval $(kernel-module))
$(eval $(generic-package))