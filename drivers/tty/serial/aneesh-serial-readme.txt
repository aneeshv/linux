Loading the module:
*******************
In rowboat-android/device/ti/beaglebone/init.am335xevm.rc

 on boot

+    insmod /system/lib/modules/aneesh-serial.ko
+
 # Default Read Ahead value for sdcards
     write /sys/block/mmcblk0/queue/read_ahead_kb 2048

Passing Module params
*********************
-    insmod /system/lib/modules/aneesh-serial.ko
+    insmod /system/lib/modules/aneesh-serial.ko aneesh_ser_debug=1

Devices are listed under driver after successful probing
*******************************************************
Before successful probing:
**************************
$ adb shell ls /sys/bus/platform/drivers/omap_uart
bind
uevent
unbind
After successful probing:
*************************
$ adb shell ls /sys/bus/platform/drivers/omap_uart
bind
omap_uart.0
omap_uart.1
omap_uart.2
omap_uart.3
omap_uart.4
omap_uart.5
uevent
unbind
