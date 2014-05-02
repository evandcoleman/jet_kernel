export KERNEL_PATH=`pwd`
export JET_PATH=`pwd`/../..
export MYDROID=${JET_PATH}/mydroid
export PATH=$PATH:${MYDROID}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/
export CROSS_COMPILE=${MYDROID}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
export PATH=${MYDROID}/../u-boot/tools:$PATH

#make ARCH=arm distclean
#make ARCH=arm jet_defconfig
make -j$(egrep '^processor' /proc/cpuinfo | wc -l) ARCH=arm $kernel_config uImage 2>&1 | tee ${JET_PATH}/logs/kernel_make.out
cd ${KERNEL_PATH}

