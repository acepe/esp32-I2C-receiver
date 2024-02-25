This project is part of my bluetooth keyboard to USB converter and is designed to work alongside this other project (which still  needs a lot of cleaning up, but works good enough) https://github.com/acepe/esp32-bt2ps2.
It acts as a I2C receiver for HID-Codes which are then resend over an USB-connection to the host PC.

The goal of both projects is to make a bluetooth keyboard useable for editing the BIOS or entering full-disk-encryption passphrases, when the bluetooth-stack isn't available yet. 

This project is only necessary if you have a bluetooth classic keyboard, because (as far as I understand) the only ESP32 that supports bluetooth-classic 
is the "normal" ESP32 (in my case ESP32-WROOM-32 Dev Kit), not the newer S2/S3/.. variants. Sadly this model doesn't support USB-OTG/-HID...
The newer S2/S3 variants support USB, but only bluetooth-BLE, which in turn doesn't work with my keyboard (Keychron K1 Pro)...

So in that case we need two ESPs. The first one (ESP32-WROOM-32) does the bluetooth-stuff and a newer ESP32-S3 does the USB-stuff. 
They communicate over I2C, the first ESP acting as master, the second as slave. 
Power is coming from USB. The 3V3 and GND pins of both ESPs are connected, so both share the power.

This is the code for the second ESP that receives the keyboard HID Codes over I2C and resends them over the USB connection.
The first ESP needs code from here: https://github.com/acepe/esp32-bt2ps2  -- Branch: "usb-over-i2c"

If your keyboard uses Bluetooth-BLE (like for example my other Logitech MX Keys) you dont need this project. You only need a single ESP32 (WROOM-32) and this other project: https://github.com/acepe/esp32-bt2ps2  -- Branch: "ble-only"
