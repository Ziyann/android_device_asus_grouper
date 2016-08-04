/*
* Copyright (C) 2012 Invensense, Inc.
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

#ifndef INV_SENSOR_PARAMS_H
#define INV_SENSOR_PARAMS_H

/* Physical parameters of the sensors supported by Invensense MPL */
#define SENSORS_ROTATION_VECTOR_HANDLE    (ID_RV)
#define SENSORS_LINEAR_ACCEL_HANDLE       (ID_LA)
#define SENSORS_GRAVITY_HANDLE            (ID_GR)
#define SENSORS_GYROSCOPE_HANDLE          (ID_GY)
#define SENSORS_RAW_GYROSCOPE_HANDLE      (ID_RG)
#define SENSORS_ACCELERATION_HANDLE       (ID_A)
#define SENSORS_MAGNETIC_FIELD_HANDLE     (ID_M)
#define SENSORS_ORIENTATION_HANDLE        (ID_O)
#define SENSORS_SCREEN_ORIENTATION_HANDLE (ID_SO)

/******************************************/
//COMPASS_ID_AMI306
#define COMPASS_AMI306_RANGE            (5461.f)
#define COMPASS_AMI306_RESOLUTION       (0.9f)
#define COMPASS_AMI306_POWER            (0.15f)
#define COMPASS_AMI306_MINDELAY         (10000)
/*******************************************/
//ACCEL_ID_MPU6050
#define ACCEL_MPU6050_RANGE             (2.f * GRAVITY_EARTH)
#define ACCEL_MPU6050_RESOLUTION        (0.004f * GRAVITY_EARTH)
#define ACCEL_MPU6050_POWER             (0.5f)
#define ACCEL_MPU6050_MINDELAY          (1000)
/******************************************/
#define RAD_P_DEG                       (3.14159f / 180.f)
//GYRO MPU6050
#define GYRO_MPU6050_RANGE              (2000.f * RAD_P_DEG)
#define GYRO_MPU6050_RESOLUTION         (2000.f / 32768.f * RAD_P_DEG)
#define GYRO_MPU6050_POWER              (5.5f)
#define GYRO_MPU6050_MINDELAY           (1000)

#endif  /* INV_SENSOR_PARAMS_H */

