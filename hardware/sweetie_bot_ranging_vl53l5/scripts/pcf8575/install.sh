#!/bin/bash
sudo rm -f /boot/overlays/gpio-pcf857x.dtbo
dtc -I dts -O dtb -o gpio-pcf857x.dtb gpio-pcf857x.dts
sudo cp gpio-pcf857x.dtb /boot/overlays/gpio-pcf857x.dtbo

