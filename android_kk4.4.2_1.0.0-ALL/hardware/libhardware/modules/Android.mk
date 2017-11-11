hardware_modules := gralloc hwcomposer audio nfc nfc-nci local_time \
	power usbaudio audio_remote_submix camera consumerir  imx6q-ctr imx6q_spi imx6q_keysound imx6q_light
include $(call all-named-subdir-makefiles,$(hardware_modules))
