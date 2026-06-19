LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# 1. Module Identification
LOCAL_MODULE := ncnn
LOCAL_CATEGORY_PATH := airsdk/missions/dependencies

# 2. Exporting Paths for Compile-Time Linking
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_EXPORT_LDLIBS := -L$(LOCAL_PATH)/lib -lncnn
LOCAL_COPY_FILES := \
	$(LOCAL_PATH)/lib/libncnn.so:usr/lib/libncnn.so \
	$(LOCAL_PATH)/lib/libncnn.so:usr/lib/libncnn.so.1 \
	/opt/hisi-linux/x86-arm/aarch64-himix100-linux/aarch64-linux-gnu/lib64/libgomp.so.1.0.0:usr/lib/libgomp.so.1

# 3. Packaging Instruction
include $(BUILD_PREBUILT)
