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
