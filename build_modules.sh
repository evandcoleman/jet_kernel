export KERNEL_PATH=`pwd`
export JET_PATH=`pwd`/../..
export MYDROID=${JET_PATH}/mydroid
export PATH=$PATH:${MYDROID}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/
export CROSS_COMPILE=${MYDROID}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
export PATH=${MYDROID}/../u-boot/tools:$PATH

make -j$(egrep '^processor' /proc/cpuinfo | wc -l) ARCH=arm modules 2>&1 | tee ${JET_PATH}/logs/kernel_modules.out

