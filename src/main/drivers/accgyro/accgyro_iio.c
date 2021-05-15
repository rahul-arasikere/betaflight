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

#if defined(USE_IIO_GYRO) || defined(USE_IIO_ACC)
#include <iio.h>

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_iio.h"
#include "build/build_config.h"

#include "common/axis.h"
#include "common/utils.h"

#define ANGLVEL_X "anglvel_x"
#define ANGLVEL_Y "anglvel_y"
#define ANGLVEL_Z "anglvel_z"

#define ACCEL_X "accel_x"
#define ACCEL_Y "accel_y"
#define ACCEL_Z "accel_z"

static struct iio_context *context;

static struct iio_device *accgyro_device;
static struct iio_buffer *accgyro_buffer;

static struct iio_channel *gyro_anglvel_x;
static struct iio_channel *gyro_anglvel_y;
static struct iio_channel *gyro_anglvel_z;

static struct iio_channel *accel_x;
static struct iio_channel *accel_y;
static struct iio_channel *accel_z;

bool setup_iio_structures()
{
    if (context == NULL)
    {
        context = iio_create_default_context();
        if (context == NULL)
        {
            perror("Failed to acquire default context!\n");
            return false;
        }
        accgyro_device = iio_context_find_device(context, IIO_GYRO_NAME);
        if (accgyro_device == NULL)
        {
            perror("Failed find gyro device!\n");
            return false;
        }

#if defined(USE_IIO_GYRO)
        gyro_anglvel_x = iio_device_find_channel(accgyro_device, ANGLVEL_X, false);
        if (gyro_anglvel_x == NULL)
        {
            perror("Failed to get anglvel_x channel...\n");
            return false;
        }
        gyro_anglvel_y = iio_device_find_channel(accgyro_device, ANGLVEL_Y, false);
        if (gyro_anglvel_y == NULL)
        {
            perror("Failed to get anglvel_y channel...\n");
            return false;
        }
        gyro_anglvel_z = iio_device_find_channel(accgyro_device, ANGLVEL_Z, false);
        if (gyro_anglvel_z == NULL)
        {
            perror("Failed to get anglvel_z channel...\n");
            return false;
        }
        iio_channel_enable(gyro_anglvel_x);
        iio_channel_enable(gyro_anglvel_y);
        iio_channel_enable(gyro_anglvel_z);
#endif

#if defined(USE_IIO_ACC)
        accel_x = iio_device_find_channel(accgyro_device, ACCEL_X, false);
        if (accel_x == NULL)
        {
            perror("Failed to get accel_x channel...\n");
            return false;
        }
        accel_y = iio_device_find_channel(accgyro_device, ACCEL_Y, false);
        if (accel_x == NULL)
        {
            perror("Failed to get accel_y channel...\n");
            return false;
        }
        accel_z = iio_device_find_channel(accgyro_device, ACCEL_Z, false);
        if (accel_x == NULL)
        {
            perror("Failed to get accel_z channel...\n");
            return false;
        }
        iio_channel_enable(accel_x);
        iio_channel_enable(accel_y);
        iio_channel_enable(accel_z);
#endif

        accgyro_buffer = iio_device_create_buffer(accgyro_device, 1, false);
        if (accgyro_buffer == NULL)
        {
            perror("Failed to create gyro buffer!\n");
            return false;
        }
    }
    return true;
}

#endif

#ifdef USE_IIO_GYRO

bool iioGyroRead(gyroDev_t *gyro)
{
    gyro->dataReady = false;
    if (iio_buffer_refill(accgyro_buffer) < 0)
    {
        return false; // no data ready yet
    }
    if (iio_channel_read_raw(gyro_anglvel_x, accgyro_buffer, &(gyro->gyroADCRaw[X]), sizeof(int16_t)) == 0)
    {
        return false;
    }
    if (iio_channel_read_raw(gyro_anglvel_y, accgyro_buffer, &(gyro->gyroADCRaw[Y]), sizeof(int16_t)) == 0)
    {
        return false;
    }
    if (iio_channel_read_raw(gyro_anglvel_z, accgyro_buffer, &(gyro->gyroADCRaw[Z]), sizeof(int16_t)) == 0)
    {
        return false;
    }
    gyro->dataReady = true;
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
    iio_buffer_set_blocking_mode(accgyro_buffer, true);
}

bool iioGyroDetect(gyroDev_t *gyro)
{
#if defined(USE_IIO_GYRO)
    if (setup_iio_structures())
    {
        gyro->initFn = iioGyroInit;
        gyro->readFn = iioGyroRead;
        gyro->scale = GYRO_SCALE_2000DPS;
        return true;
    }
#endif
    return false;
}

#endif // USE_IIO_GYRO

#ifdef USE_IIO_ACC

bool iioAccRead(accDev_t *acc)
{
    acc->dataReady = false;
    if (iio_buffer_refill(accgyro_buffer) < 0)
    {
        return false; // no data ready yet
    }
    if (iio_channel_read_raw(accel_x, accgyro_buffer, &(acc->ADCRaw[X]), sizeof(int16_t)) == 0)
    {
        return false;
    }
    if (iio_channel_read_raw(accel_y, accgyro_buffer, &(acc->ADCRaw[Y]), sizeof(int16_t)) == 0)
    {
        return false;
    }
    if (iio_channel_read_raw(accel_z, accgyro_buffer, &(acc->ADCRaw[Z]), sizeof(int16_t)) == 0)
    {
        return false;
    }
    acc->dataReady = true;
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
    iio_buffer_set_blocking_mode(accgyro_buffer, true);
}

bool iioAccDetect(accDev_t *acc)
{
#if defined(USE_IIO_ACC)
    if (setup_iio_structures())
    {
        acc->initFn = iioAccInit;
        acc->readFn = iioAccRead;
        acc->revisionCode = 0;
        return true;
    }
#endif
    return false;
}

#endif // USE_IIO_ACC
