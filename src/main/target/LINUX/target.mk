SITL_TARGETS += $(TARGET)
FEATURES       += #SDCARD_SPI VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_iio.c \
            drivers/barometer/barometer_fake.c \
            drivers/compass/compass_fake.c \
            drivers/serial_tcp.c
