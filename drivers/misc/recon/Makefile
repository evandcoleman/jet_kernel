#
# Makefile for reconinstruments.
#




obj-$(CONFIG_JET_PROXMUX)		+= proxmux.o
proxmux-objs := jet_proxmux.o \
			    mux_queue.o

obj-$(CONFIG_JET_SENSORS)		+= sensors.o
sensors-objs := i2c_transfer.o \
			    jet_lsm9ds0.o \
			    jet_lps25h.o \
			    jet_sensors.o
obj-$(CONFIG_JET_TMP103_FLEX)	+=jet_tmp103_flex.o

obj-$(CONFIG_MFI)		+= mfi.o

obj-$(CONFIG_APDS9900)		+= apds9900.o

obj-$(CONFIG_TFA98xx)		+= tfa98xx.o

obj-$(CONFIG_JET_CAMERA_MT9M114)		+= jet_mt9m114.o