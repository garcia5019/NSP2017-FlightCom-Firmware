#elup.sh - Electron Upload: Compile in the cloud and flash over USB with DFU-util
#!/bin/bash

modem=`ls -1 /dev/cu.* | grep -vi bluetooth | tail -1`

stty -f $modem 19200

rm -rf electron_firmware*
echo '----------------------------------------------------'
particle compile electron $1
echo '----------------------------------------------------'
particle flash --usb electron_firmware*
echo '----------------------------------------------------'

