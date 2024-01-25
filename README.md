
# OTA template

UNDER CONSTRUCTION

Template for ESP32 apps that can be updated over the air

The binaries for the app and for the spiffs embedded webpages (if any) are hosted on a https sever, in my case on github pages.
The folder containg the binairies must als contain textfiles with the version number of the (new) software.
These files (binaryVersion.txt for the app and storageVersion.txt for the website) are read.
If the version differ from the present version:
In case of the spiffs: The new image is flashed into the spiffs partition
For the app: the processor must be reset to invoke the ESP32_OTAbootloader_1Partition. (see this github repo)
Of course this bootloader must be present.
Use partionsOTA_4M ( or 8M).csv for both projects.
The wifiSettings_t wifiSettings must also be the same.

The update URLs are provided by the app and written to the wifiSettings. The bootloader reads these values.
So the bootloader can be generic for multiple projects.

Therefore it is needed to flash and run first both app and bootloader to write the correct URLS.

   
 
      





