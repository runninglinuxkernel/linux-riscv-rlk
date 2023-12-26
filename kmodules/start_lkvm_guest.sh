#!/bin/bash

export BLK_BDF="0000:00:04.0"
export BLK_DID="1b36 0010"

echo "$BLK_BDF" > /sys/bus/pci/devices/$BLK_BDF/driver/unbind
echo "$BLK_DID" > /sys/bus/pci/drivers/vfio-pci/new_id

./lkvm-static run -m 128 -p 'nokaslr console=ttyS0 root=/dev/nvme0n1' -k /mnt/Image --vfio-pci=0000:00:04.0 --debug
