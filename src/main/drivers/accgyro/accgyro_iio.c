/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

#include "platform.h"

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <stdio.h>
#include <pthread.h>
#endif

#if defined(USE_IIO_GYRO) && defined(USE_IIO_ACC)
#include <iio.h>

static struct iio_context *ctx = NULL;
static struct iio_buffer *imubuf = NULL;
static struct iio_device *imu = NULL;

static bool enable_failed;

#define GRAVITY 9.805f
#define SCALE 10.0f

static bool get_imu_dev(struct iio_context *ctx, struct iio_device **dev)
{
    *dev = iio_context_find_device(ctx, IIO_ACCGYRO_NAME);
    if (!*dev)
    {
        printf("Could not find MPU6050\n");
        return false;
    }

    return true;
}

static void enable_dev_channel(struct iio_device *dev, char *name)
{
    struct iio_channel *ch;

    if (enable_failed)
        return;

    ch = iio_device_find_channel(dev, name, false);
    if (ch == NULL)
    {
        enable_failed = true;
        printf("Enabling channel %s failed!\n", name);
        return;
    }
    printf("Enabling channel %s\n", name);
    iio_channel_enable(ch);
}

static void disable_imu_channel(struct iio_context *ctx, char *channel)
{
    struct iio_device *dev = NULL;
    struct iio_channel *ch = NULL;

    get_imu_dev(ctx, &dev);

    if (!dev || !channel)
    {
        printf("Disabling IMU channel failed\n");
        return;
    }

    ch = iio_device_find_channel(dev, channel, false);
    if (!ch)
    {
        printf("Disabling IMU channel, could not find channel\n");
        return;
    }
    iio_channel_disable(ch);
}

#endif

#ifdef USE_IIO_GYRO

#include "build/build_config.h"

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_iio.h"

#define ANGVL_X "anglvel_x"
#define ANGVL_Y "anglvel_y"
#define ANGVL_Z "anglvel_z"

bool iioGyroRead(gyroDev_t *gyro)
{
    ssize_t rxn;
    char *data;
    struct iio_channel *ch;
    ptrdiff_t inc;

    assert(imubuf != NULL);
    rxn = iio_buffer_refill(imubuf);
    if (rxn < 0)
    {
        printf("Error filling up IMU buffer\n");
        return false;
    }
    ch = iio_device_find_channel(imu, ANGVL_X, false);
    data = iio_buffer_first(imubuf, ch);
    inc = iio_buffer_step(imubuf);
    gyro->gyroADCRaw[X] = ((int16_t *)data)[0];
    data += inc;
    gyro->gyroADCRaw[Y] = ((int16_t *)data)[1];
    data += inc;
    gyro->gyroADCRaw[Z] = ((int16_t *)data)[2];
    return true;
}

static void iioGyroInit(gyroDev_t *gyro)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&gyro->lock, NULL) != 0)
    {
        printf("Create gyro lock error!\n");
    }
#endif

    if (ctx == NULL)
    {
        ctx = iio_create_default_context();
    }

    if (ctx == NULL)
    {
        printf("failed to acquire default iio context!\n");
    }

    if (!iio_context_get_devices_count(ctx))
    {
        printf("No IIO devices found!\n");
    }

    if (imu == NULL)
    {
        if (!get_imu_dev(ctx, &imu))
        {
            printf("Could not find IMU device!\n");
        }
    }

    enable_dev_channel(imu, ANGVL_X);
    enable_dev_channel(imu, ANGVL_Y);
    enable_dev_channel(imu, ANGVL_Z);

    if (enable_failed)
    {
        printf("Exiting since enabling one of the IMU channels failed\n");
    }

    if (imubuf == NULL)
    {
        imubuf = iio_device_create_buffer(imu, 16, false);
        if (!imubuf)
        {
            printf("Enabling IMU buffers failed!\n");
        }
    }
}

bool iioGyroDetect(gyroDev_t *gyro)
{
    gyro->initFn = iioGyroInit;
    gyro->readFn = iioGyroRead;
    return true;
}

#endif // USE_IIO_GYRO

#ifdef USE_IIO_ACC

#define ACCEL_X "accel_x"
#define ACCEL_Y "accel_y"
#define ACCEL_Z "accel_z"

bool iioAccRead(accDev_t *acc)
{
    ssize_t rxn;
    char *data;
    struct iio_channel *ch;
    ptrdiff_t inc;

    assert(imubuf != NULL);
    rxn = iio_buffer_refill(imubuf);
    if (rxn < 0)
    {
        printf("Error filling up IMU buffer\n");
        return false;
    }
    ch = iio_device_find_channel(imu, ACCEL_X, false);
    data = iio_buffer_first(imubuf, ch);
    inc = iio_buffer_step(imubuf);
    acc->ADCRaw[X] = ((int16_t *)data)[0];
    data += inc;
    acc->ADCRaw[Y] = ((int16_t *)data)[1];
    data += inc;
    acc->ADCRaw[Z] = ((int16_t *)data)[2];
    return true;
}

static void iioAccInit(accDev_t *acc)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&acc->lock, NULL) != 0)
    {
        printf("Create acc lock error!\n");
    }
#endif

    if (ctx == NULL)
    {
        ctx = iio_create_default_context();
    }

    if (ctx == NULL)
    {
        printf("failed to acquire default iio context!\n");
    }
    if (imu == NULL)
    {
        if (!get_imu_dev(ctx, &imu))
        {
            printf("Could not find IMU device!\n");
        }
    }
    enable_dev_channel(imu, ACCEL_X);
    enable_dev_channel(imu, ACCEL_Y);
    enable_dev_channel(imu, ACCEL_Z);

    if (imubuf == NULL)
    {
        imubuf = iio_device_create_buffer(imu, 16, false);
        if (!imubuf)
        {
            printf("Enabling IMU buffers failed!\n");
        }
    }

    if (enable_failed)
    {
        printf("Exiting since enabling one of the IMU channels failed\n");
    }
}

bool iioAccDetect(accDev_t *acc)
{
    acc->initFn = iioAccInit;
    acc->readFn = iioAccRead;
    acc->revisionCode = 0;
    return true;
}

#endif // USE_IIO_ACC
