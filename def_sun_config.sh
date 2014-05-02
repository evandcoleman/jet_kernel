export MYDROID=`pwd`/../../mydroid
export PATH=$PATH:${MYDROID}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/
export CROSS_COMPILE=${MYDROID}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
export PATH=${MYDROID}/../u-boot/tools:$PATH

def_config=jet_sun_defconfig

echo -e "\033[1;32m------ Creating a .config file based on ${def_config} ------\033[m"
make ARCH=arm $def_config
echo -e "\033[1;32m------ DONE Creating a .config file based on ${def_config} ------\033[m"

