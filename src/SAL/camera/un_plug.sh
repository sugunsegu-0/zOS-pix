#!/bin/bash

bind_usb() {
	echo "$1" > /sys/bus/usb/drivers/usb/bind
}

unbind_usb() {
	echo "$1" > /sys/bus/usb/drivers/usb/unbind
}


for i in "usb4" "usb6" "usb9"
do
	unbind_usb "$i"
	bind_usb "$i"
done
