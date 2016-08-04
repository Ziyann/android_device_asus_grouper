# Copyright (C) 2016 The Android Open Source Project
# Copyright (C) 2016 Dániel Járai
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


LOCAL_PATH := $(call my-dir)

# HAL module implemenation stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)

LOCAL_MODULE := sensors.grouper

LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := optional
LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\"
LOCAL_C_INCLUDES += device/asus/grouper/libsensors_iio
LOCAL_SRC_FILES := \
    sensors.cpp \
    InputEventReader.cpp \
    LightSensor.cpp \
    SensorBase.cpp

LOCAL_SHARED_LIBRARIES := libinvensense_hal liblog libutils libdl

include $(BUILD_SHARED_LIBRARY)

