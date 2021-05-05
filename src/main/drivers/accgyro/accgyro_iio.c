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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <stdio.h>
#include <pthread.h>
#endif

#if defined(USE_IIO_GYRO) && defined(USE_IIO_ACC)
#include <iio.h>

static struct iio_context *ctx = NULL;
static struct iio_buffer *imubuf = NULL;

#define GRAVITY 9.805f
#define SCALE 10.0f

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
}

bool iioGyroDetect(gyroDev_t *gyro)
{
}

#endif // USE_IIO_GYRO

#ifdef USE_IIO_ACC

#define ACCEL_X "accel_x"
#define ACCEL_Y "accel_y"
#define ACCEL_Z "accel_z"

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
}

bool iioAccDetect(accDev_t *acc)
{
}

#endif // USE_IIO_ACC
