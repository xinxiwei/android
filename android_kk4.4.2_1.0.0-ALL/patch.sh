#!/bin/bash

echo
echo
echo "===========开始编译新平台版本软件,打工业级芯片patch ==========="
echo
echo

myFile1="/bootable/bootloader/uboot-imx/cpu/arm_cortexa8/mx6/generic.c"
myFile2="/kernel_imx/drivers/mmc/core/mmc.o"
myFile3="/kernel_imx/drivers/mxc/thermal/thermal.o"

source env.sh
source build/envsetup.sh
lunch 16


set -x
cp -r device/patch/new_version_patch/bootable/bootloader/uboot-imx/cpu/arm_cortexa8/mx6/generic.c     bootable/bootloader/uboot-imx/cpu/arm_cortexa8/mx6 
cp -r device/patch/new_version_patch/kernel_imx/drivers/mmc/core/mmc.c      kernel_imx/drivers/mmc/core/mmc.c
cp -r device/patch/new_version_patch/kernel_imx/drivers/mxc/thermal/thermal.c    kernel_imx/drivers/mxc/thermal/thermal.c

cd  bootable/bootloader/uboot-imx/cpu/arm_cortexa8/mx6
if [ -f "$myFile1" ]
then
 rm "$myFile1"
else
   echo "$myFile1 not exist"
fi
cd -

cd  kernel_imx/drivers/mmc/core/
if [ -f "$myFile2" ]
then
 rm "$myFile2"
else
   echo "$myFile2 not exist"
fi
cd -
	
cd kernel_imx/drivers/mxc/thermal/
if [ -f "$myFile3" ]
then
 rm "$myFile3"
else
   echo "$myFile3 not exist"
fi
cd -
	
rm -rf out/target/product/sabresd_6dq/u-boot*
rm -rf out/target/product/sabresd_6dq/*.img
rm -rf out/target/product/sabresd_6dq/recovery
rm -rf out/target/product/sabresd_6dq/root
rm -rf out/target/product/sabresd_6dq/system/app
rm -rf out/target/product/sabresd_6dq/system/priv-app
rm -rf out/target/product/sabresd_6dq/system/lib/hw
rm -rf out/target/product/sabresd_6dq/system/framework/framework*.jar
rm -rf out/target/product/sabresd_6dq/system/framework/services.jar
rm -rf out/target/product/sabresd_6dq/system/build.prop

set +x

mmm frameworks/base
mmm frameworks/base/services/java
mmm frameworks/base/core/jni
mmm hardware/libhardware

./mk.sh uboot distclean

make -j8 2>&1 | tee build.log

