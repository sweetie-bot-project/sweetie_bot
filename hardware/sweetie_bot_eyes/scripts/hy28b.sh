#!/bin/sh
sudo dtc -@ -I dts -O dtb -o /boot/overlays/hy28b2-overlay.dtb hy28b2-overlay.dts
