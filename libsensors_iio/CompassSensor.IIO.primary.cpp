/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_NDEBUG 0

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>
#include <linux/input.h>
#include <string.h>

#include "CompassSensor.IIO.primary.h"
#include "sensors.h"
#include "MPLSupport.h"
#include "sensor_params.h"
#include "ml_sysfs_helper.h"
#include "local_log_def.h"

#define COMPASS_MAX_SYSFS_ATTRB sizeof(compassSysFs) / sizeof(char*)

#warning "Invensense compass cal with AMI306 IIO on primary bus"
#define USE_MPL_COMPASS_HAL          (1)
#define COMPASS_NAME                 "ami306"

/*****************************************************************************/

CompassSensor::CompassSensor()
                  : SensorBase(COMPASS_NAME, NULL),
                    mCompassTimestamp(0),
                    mCompassInputReader(8)
{
    FILE *fptr;

    VFUNC_LOG;

    strncpy(dev_full_name, dev_name,
            sizeof(dev_full_name) / sizeof(dev_full_name[0]));

    if (inv_init_sysfs_attributes()) {
        LOGE("Error Instantiating Compass\n");
        return;
    }

    if (!strcmp(dev_full_name, "INV_COMPASS") || !strcmp(dev_full_name, "ami306")) {
        mI2CBus = COMPASS_BUS_SECONDARY;
    } else {
        mI2CBus = COMPASS_BUS_PRIMARY;
    }

    memset(mCachedCompassData, 0, sizeof(mCachedCompassData));

    if (!isIntegrated()) {
        enable(ID_M, 0);
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:compass name: %s", dev_full_name);
    enable_iio_sysfs();

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:cat %s (%lld)",
            compassSysFs.compass_orient, getTimestamp());
    fptr = fopen(compassSysFs.compass_orient, "r");
    if (fptr != NULL) {
        int om[9];
        fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d", 
               &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
               &om[6], &om[7], &om[8]);
        fclose(fptr);

        LOGV_IF(EXTRA_VERBOSE,
                "HAL:compass mounting matrix: "
                "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                om[0], om[1], om[2], om[3], om[4], om[5], om[6], om[7], om[8]);

        mCompassOrientation[0] = om[0];
        mCompassOrientation[1] = om[1];
        mCompassOrientation[2] = om[2];
        mCompassOrientation[3] = om[3];
        mCompassOrientation[4] = om[4];
        mCompassOrientation[5] = om[5];
        mCompassOrientation[6] = om[6];
        mCompassOrientation[7] = om[7];
        mCompassOrientation[8] = om[8];
    } else {
        LOGE("HAL:Couldn't read compass mounting matrix");
    }
}

void CompassSensor::enable_iio_sysfs()
{
    VFUNC_LOG;

    int tempFd = 0;
    char iio_trigger_name[MAX_CHIP_ID_LEN], iio_device_node[MAX_CHIP_ID_LEN];
    FILE *tempFp = NULL;
    const char* compass = dev_full_name;

    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:echo 1 > %s (%lld)",
            compassSysFs.in_timestamp_en, getTimestamp());
    tempFd = open(compassSysFs.in_timestamp_en, O_RDWR);
    if(tempFd < 0) {
        LOGE("HAL:could not open %s timestamp enable", compass);
    } else if(enable_sysfs_sensor(tempFd, 1) < 0) {
        LOGE("HAL:could not enable timestamp");
    }

    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)",
            compassSysFs.trigger_name, getTimestamp());
    tempFp = fopen(compassSysFs.trigger_name, "r");
    if (tempFp == NULL) {
        LOGE("HAL:could not open %s trigger name", compass);
    } else {
        if (fscanf(tempFp, "%s", iio_trigger_name) < 0) {
            LOGE("HAL:could not read trigger name");
        }
        fclose(tempFp);
    }

    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:echo %s > %s (%lld)",
            iio_trigger_name, compassSysFs.current_trigger, getTimestamp());
    tempFp = fopen(compassSysFs.current_trigger, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open current trigger");
    } else {
        if (fprintf(tempFp, "%s", iio_trigger_name) < 0) {
            LOGE("HAL:could not write current trigger");
        }
        fclose(tempFp);
    }

    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:echo %d > %s (%lld)",
            IIO_BUFFER_LENGTH, compassSysFs.buffer_length, getTimestamp());
    tempFp = fopen(compassSysFs.buffer_length, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open buffer length");
    } else {
        if (fprintf(tempFp, "%d", IIO_BUFFER_LENGTH) < 0) {
            LOGE("HAL:could not write buffer length");
        }
        fclose(tempFp);
    }

    sprintf(iio_device_node, "%s%d", "/dev/iio:device",
            find_type_by_name(compass, "iio:device"));
    compass_fd = open(iio_device_node, O_RDONLY);
    int res = errno;
    if (compass_fd < 0) {
        LOGE("HAL:could not open '%s' iio device node in path '%s' - "
             "error '%s' (%d)",
             compass, iio_device_node, strerror(res), res);
    } else {
        LOGV_IF(EXTRA_VERBOSE,
                "HAL:iio %s, compass_fd opened : %d", compass, compass_fd);
    }
}

CompassSensor::~CompassSensor()
{
    VFUNC_LOG;

    free(pathP);
    if( compass_fd > 0)
        close(compass_fd);
}

int CompassSensor::getFd() const
{
    VHANDLER_LOG;
    return compass_fd;
}

/**
 *  @brief        This function will enable/disable sensor.
 *  @param[in]    handle
 *                  which sensor to enable/disable.
 *  @param[in]    en
 *                  en=1, enable; 
 *                  en=0, disable
 *  @return       if the operation is successful.
 */
int CompassSensor::enable(int32_t /*handle*/, int en) 
{
    VFUNC_LOG;

    mEnable = en;
    int res;

    /* reset master enable */
    res = write_sysfs_int(compassSysFs.chip_enable, 0);
    if (res < 0) {
        return res;
    }

    if (en) {
        write_sysfs_int(compassSysFs.compass_x_fifo_enable, en);
        write_sysfs_int(compassSysFs.compass_y_fifo_enable, en);
        write_sysfs_int(compassSysFs.compass_z_fifo_enable, en);

        res = write_sysfs_int(compassSysFs.chip_enable, en);
        LOGE_IF(res < 0, "HAL: enable compass failed");
    }

    return res;
}

int CompassSensor::setDelay(int32_t /*handle*/, int64_t ns) 
{
    VFUNC_LOG;
    int tempFd;
    int res;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %.0f > %s (%lld)", 
            1000000000.f / ns, compassSysFs.compass_rate, getTimestamp());
    mDelay = ns;
    if (ns == 0)
        return -1;
    tempFd = open(compassSysFs.compass_rate, O_RDWR);
    res = write_attribute_sensor(tempFd, 1000000000.f / ns);
    if(res < 0) {
        LOGE("HAL:Compass update delay error");
    }
    return res;
}


/**
    @brief      This function will return the state of the sensor.
    @return     1=enabled; 0=disabled
**/
int CompassSensor::getEnable(int32_t /*handle*/)
{
    VFUNC_LOG;
    return mEnable;
}

/* use for Invensense compass calibration */
#define COMPASS_EVENT_DEBUG (0)
void CompassSensor::processCompassEvent(const input_event *event)
{
    VHANDLER_LOG;

    switch (event->code) {
    case EVENT_TYPE_ICOMPASS_X:
        LOGV_IF(COMPASS_EVENT_DEBUG, "EVENT_TYPE_ICOMPASS_X\n");
        mCachedCompassData[0] = event->value;
        break;
    case EVENT_TYPE_ICOMPASS_Y:
        LOGV_IF(COMPASS_EVENT_DEBUG, "EVENT_TYPE_ICOMPASS_Y\n");
        mCachedCompassData[1] = event->value;
        break;
    case EVENT_TYPE_ICOMPASS_Z:
        LOGV_IF(COMPASS_EVENT_DEBUG, "EVENT_TYPE_ICOMPASS_Z\n");
        mCachedCompassData[2] = event->value;
        break;
    }
    
    mCompassTimestamp = 
        (int64_t)event->time.tv_sec * 1000000000L + event->time.tv_usec * 1000L;
}

void CompassSensor::getOrientationMatrix(signed char *orient)
{
    VFUNC_LOG;
    memcpy(orient, mCompassOrientation, sizeof(mCompassOrientation));
}

long CompassSensor::getSensitivity()
{
    VFUNC_LOG;

    long sensitivity;
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:cat %s (%lld)", 
            compassSysFs.compass_scale, getTimestamp());
    inv_read_data(compassSysFs.compass_scale, &sensitivity);
    return sensitivity;
}

/**
    @brief         This function is called by sensors_mpl.cpp
                   to read sensor data from the driver.
    @param[out]    data      sensor data is stored in this variable. Scaled such that
                             1 uT = 2^16
    @para[in]      timestamp data's timestamp
    @return        1, if 1   sample read, 0, if not, negative if error
 */
int CompassSensor::readSample(long *data, int64_t *timestamp) {
    VFUNC_LOG;

    int i;
    char *rdata = mIIOBuffer;

    size_t rsize = read(compass_fd, rdata, (8 * mEnable + 8) * 1);

    if (!mEnable) {
        rsize = read(compass_fd, rdata, (8 + 8) * IIO_BUFFER_LENGTH);
        // LOGI("clear buffer with size: %d", rsize);
    }
/*
    LOGI("get one sample of AMI IIO data with size: %d", rsize);
    LOGI_IF(mEnable, "compass x/y/z: %d/%d/%d", *((short *) (rdata + 0)),
        *((short *) (rdata + 2)), *((short *) (rdata + 4)));
*/
    if (mEnable) {
        for (i = 0; i < 3; i++) {
            data[i] = *((short *) (rdata + i * 2));
        }
        *timestamp = *((long long *) (rdata + 8 * mEnable));
    }

    return mEnable;
}

/**
 *  @brief  This function will return the current delay for this sensor.
 *  @return delay in nanoseconds. 
 */
int64_t CompassSensor::getDelay(int32_t /*handle*/)
{
    VFUNC_LOG;
    return mDelay;
}

void CompassSensor::fillList(struct sensor_t *list)
{
    VFUNC_LOG;

    list->maxRange = COMPASS_AMI306_RANGE;
    list->resolution = COMPASS_AMI306_RESOLUTION;
    list->power = COMPASS_AMI306_POWER;
    list->minDelay = COMPASS_AMI306_MINDELAY;
}

int CompassSensor::inv_init_sysfs_attributes(void)
{
    VFUNC_LOG;

    unsigned char i = 0;
    char sysfs_path[MAX_SYSFS_NAME_LEN], iio_trigger_path[MAX_SYSFS_NAME_LEN], tbuf[2];
    char *sptr;
    char **dptr;
    int num;
    const char* compass = dev_full_name;

    pathP = (char*)malloc(
                    sizeof(char[COMPASS_MAX_SYSFS_ATTRB][MAX_SYSFS_NAME_LEN]));
    sptr = pathP;
    dptr = (char**)&compassSysFs;
    if (sptr == NULL)
        return -1;

    do {
        *dptr++ = sptr;
        sptr += sizeof(char[MAX_SYSFS_NAME_LEN]);
    } while (++i < COMPASS_MAX_SYSFS_ATTRB);

    // get proper (in absolute/relative) IIO path & build sysfs paths
    sprintf(sysfs_path, "%s%d", "/sys/bus/iio/devices/iio:device",
        find_type_by_name(compass, "iio:device"));
    sprintf(iio_trigger_path, "%s%d", "/sys/bus/iio/devices/trigger",
        find_type_by_name(compass, "iio:device"));

    sprintf(compassSysFs.chip_enable, "%s%s", sysfs_path, "/buffer/enable");
    sprintf(compassSysFs.in_timestamp_en, "%s%s", sysfs_path, "/scan_elements/in_timestamp_en");
    sprintf(compassSysFs.trigger_name, "%s%s", iio_trigger_path, "/name");
    sprintf(compassSysFs.current_trigger, "%s%s", sysfs_path, "/trigger/current_trigger");
    sprintf(compassSysFs.buffer_length, "%s%s", sysfs_path, "/buffer/length");

    sprintf(compassSysFs.compass_x_fifo_enable, "%s%s", sysfs_path, "/scan_elements/in_magn_x_en");
    sprintf(compassSysFs.compass_y_fifo_enable, "%s%s", sysfs_path, "/scan_elements/in_magn_y_en");
    sprintf(compassSysFs.compass_z_fifo_enable, "%s%s", sysfs_path, "/scan_elements/in_magn_z_en");
    sprintf(compassSysFs.compass_rate, "%s%s", sysfs_path, "/sampling_frequency");
    sprintf(compassSysFs.compass_scale, "%s%s", sysfs_path, "/in_magn_scale");
    sprintf(compassSysFs.compass_orient, "%s%s", sysfs_path, "/compass_matrix");

#if SYSFS_VERBOSE
    // test print sysfs paths
    dptr = (char**)&compassSysFs;
    LOGI("sysfs path base: %s", sysfs_path);
    LOGI("trigger sysfs path base: %s", iio_trigger_path);
    for (i = 0; i < COMPASS_MAX_SYSFS_ATTRB; i++) {
        LOGE("HAL:sysfs path: %s", *dptr++);
    }
#endif
    return 0;
}
