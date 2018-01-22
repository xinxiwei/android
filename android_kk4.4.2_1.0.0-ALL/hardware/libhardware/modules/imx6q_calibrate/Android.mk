LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog 
LOCAL_SRC_FILES := imx6q_calibrate.c \
				   imx6q_slavespi.c \
				   imx6q_gpio.c \
				   imx6q_masterspi.c \
				   imx6q_vibrate_calibrate.c \
				   imx6q_vibrate_calibrate_config.c \
				   imx6q_walg_tda.c \
				   
				   
				   
LOCAL_MODULE := imx6q_calibrate.default
include $(BUILD_SHARED_LIBRARY)
