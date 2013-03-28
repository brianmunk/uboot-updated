uboot-updated
=============

A updated branch of u-boot (November 2012), includes b3 modifcations



compiling
---------

Only compile on the target device, cross compiling regulary fails due to 
missalignment.

```
make bubba3_config
make -j2
```
