LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# 1. Module Identification
LOCAL_MODULE := ncnn
LOCAL_CATEGORY_PATH := airsdk/missions/dependencies

NCNN_LIB_DIR := $(LOCAL_PATH)/lib
NCNN_COPY_FILES :=

ifeq ("$(TARGET_ARCH)","x64")
  NCNN_LIB_DIR := $(LOCAL_PATH)/lib/x64
endif

ifeq ("$(TARGET_ARCH)","aarch64")
  NCNN_COPY_FILES += \
	/opt/hisi-linux/x86-arm/aarch64-himix100-linux/aarch64-linux-gnu/lib64/libgomp.so.1.0.0:usr/lib/libgomp.so.1
endif

ifeq ("$(wildcard $(NCNN_LIB_DIR)/libncnn.so)","")
  $(error Missing ncnn prebuilt for TARGET_ARCH=$(TARGET_ARCH): expected $(NCNN_LIB_DIR)/libncnn.so)
endif

# 2. Exporting Paths for Compile-Time Linking
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_EXPORT_LDLIBS := -L$(NCNN_LIB_DIR) -lncnn
LOCAL_COPY_FILES := \
	$(NCNN_LIB_DIR)/libncnn.so:usr/lib/libncnn.so \
	$(NCNN_LIB_DIR)/libncnn.so:usr/lib/libncnn.so.1 \
	$(NCNN_COPY_FILES)

# 3. Packaging Instruction
include $(BUILD_PREBUILT)
