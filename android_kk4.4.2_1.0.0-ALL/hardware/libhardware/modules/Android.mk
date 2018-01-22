hardware_modules := gralloc hwcomposer audio nfc nfc-nci local_time \
	power usbaudio audio_remote_submix camera consumerir  imx6q_debug imx6q_vibrate imx6q_calibrate imx6q_press imx6q_beep imx6q-light imx6q-backlight
include $(call all-named-subdir-makefiles,$(hardware_modules))
