LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog 
LOCAL_SRC_FILES := imx6q_vibrate.c \
				   imx6q_slavespi.c \
				   imx6q_gpio.c \
				   imx6q_masterspi.c \
				   imx6q_spi_vibrate.c \
				   imx6q_vibrate_config.c \
				   imx6q_walg_tda.c \
				   imx6q_iir_coeffs.c \
				   imx6q_spectrum.c \
				   imx6q_iir_filter.c \
				   imx6q_fir_filter.c \				   
				   
				   
				   
LOCAL_MODULE := imx6q_vibrate.default
include $(BUILD_SHARED_LIBRARY)
