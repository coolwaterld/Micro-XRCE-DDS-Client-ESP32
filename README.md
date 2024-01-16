# Micro-XRCE-DDS-Client-ESP32

This is a ESP_IDF project(vscode + idf_explorer). I tested on ESP32S3 (but also works for ESP32, ESP32S2,ESP32C3...).

It must work with https://github.com/coolwaterld/Micro-XRCE-DDS-Agent.

I learned component structues from https://github.com/raveious/esp-xrce-dds-client and https://github.com/raveious/esp-microcdr, but updated to latest Micro-DDS-XRCE.

I learn application structure from  exmaples of esp-idf/examples/protocols/sockets/udp_client and Micro-XRCE-DDS-Client/examples/BinaryEntityCreation.

It runs on wifi/udp4. It can communicate with RTI DDS applications. (Note firewall!!!)

## Set up ESP_IDF environment 

Reference https://www.waveshare.net/wiki/ESP32-S3-GEEK

## HW debuging

https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/api-guides/jtag-debugging/using-debugger.html

https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/DEBUGGING.md

## how to compile and deploy proiject

1. Open project with vsc(with idf_explorer)
2. Select port to use. (Make sure connect ESP32S3 and give right to ports )
3. Select device target
4. start menuconfig to set SSID and password for wifi, change IP and port of microAgent (You should start it on PC: ./MicroXRCEAgent udp4 --port 8888)
5. Build project
6. Select flash method
7. Flash device
8. Monitor
9. or start OpenOCD server and start gdb


