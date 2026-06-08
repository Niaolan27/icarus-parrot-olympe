LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# 1. Module Identification
LOCAL_MODULE := ncnn
LOCAL_CATEGORY_PATH := airsdk/missions/dependencies

# 2. Exporting Paths for Compile-Time Linking
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_EXPORT_LDLIBS := -L$(LOCAL_PATH)/lib -lncnn

# 3. Packaging Instruction
include $(BUILD_PREBUILT)