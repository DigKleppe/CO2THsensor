
# OTA template

Template for ESP32 apps that can be updated over the air

The binaries for the app and for the spiffs embedded webpages (if any) are hosted on a https sever, in my case on github pages.
The folder containg the binairies must als contain textfiles with the version number of the (new) software.
These files (binaryVersion.txt for the app and storageVersion.txt for the website) are read.
If the version differ from the present version:
In case of the spiffs: The new image is flashed into the spiffs partition
Use partionsOTA_4M ( or 8M).csv.



   
 
      





