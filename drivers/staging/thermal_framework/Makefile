#
# Makefile for sensor chip drivers.
#
ccflags-$(CONFIG_THERMAL_DEBUG) := -DDEBUG
obj-$(CONFIG_THERMAL_FRAMEWORK) += thermal_framework.o \
				    governor/ \
				    sensor/
obj-$(CONFIG_OMAP4_DUTY_CYCLE)	 += omap4_duty_cycle.o
