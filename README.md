<p align="center"><img src="https://github.com/Rainboooom/test/blob/main/QIDI.png" height="240" alt="QIDI's logo" /></p>
<p align="center"><a href="/LICENSE"><img alt="GPL-V3.0 License" src="https://github.com/Rainboooom/test/blob/main/qidi.svg"></a></p>

# Document Instructions
The 3D printers of QIDI are based on Klipper.Based on the Klipper open source project, we have made some modifications to its source code to meet some of the user's needs.At the same time, we have also made modifications to Moonraker, so that the screens we set can correspond to the operations on the page.
Thanks to the developers and maintainers of these open source projects.Please consider using or supporting these powerful projects.
- **Klipper**
- **Moonraker**

This document contains the updated files and source code for each version of the QIDI product, as well as two update methods:

1. Use the recommended packaging file to update the system through the machine's automatic update function  
2. Use the source code we provide to compile and update the source code through SSH connection to the microcontroller  

The detailed operation procedures for these two update methods will be written in this file, but let us know the directory composition of the document first, so that you can quickly know which folder you can find the files you need.

## Introduction to directory structure

There are only two main folders here, one of which has a fixed name called *QD_Update*, the other folder is named in the form of a model suffix version number,such as *smart3_V1.0.22*

*QD_Update* directory contains three files:
- ***printer.cfg***:Basic and necessary configuration files, including 3D printer configuration parameters, leveling parameters, etc
- ***QD_Smart3_UI4.3***:The update file for the machine screen and the version of the UI update file need to be consistent with the version of the SOC file, otherwise there may be errors in UI mismatch with the system.
- ***QD_Smart_SOC***:Including firmware information and all changes we made to Klipper, Moonraker, Fluid, etc

*\<Printer model\>_\<VERSION\>* directory contains four main software:
- ***fluidd***
- ***klipper***
- ***moonraker***
- ***xindi***

## Detailed update process
**Use the recommended packaging file**
1. Prepare a blank named USB drive
2. Create a new folder on the USB drive and name it *QD_ Update*
3. Download all files in the *QD_Update* folder and save them to the folder with the same name on your USB drive. <a href="https://github.com/QIDITECH/QIDI_SMART3/tree/main/QD_Update"  download="QIDI.zip">Files:Click here</a>
4. Insert the USB flash disk directly into the USB interface of the machine. If your operation is correct, you can see the update prompt button when you enter the version information interface. When you press the update button, a prompt will appear. Please press the shutdown button after the prompt appears, and wait 20 seconds before restarting the machine.
5. After restarting, an automatic update will occur, which will take about forty minutes minutes. Please be patient and ensure that the power is connected to avoid any issues with the update. After the update is completed, there will be an update completion prompt.

**Update by replacing files**</br>
*This method carries great risks, as errors can lead to many unexpected errors, which will affect our after-sales service to continue providing you with continuous service*</br>
1. Connect your printer device through SSH.
2. Confirm which software you need to replace.Download the corresponding file and replace the software through SSH connection.The following are the paths of each software within the system.
  
  Software|Directory
  ---|---
  fluidd|/home/mks/
  klipper|/home/mks/
  moonraker|/home/mks/
  
3. If there is no need to update Xindi, simply replace it. For example, if I replace the klipper folder, save it and restart it.
4. If you want to recompile and install xindi through source code
> ```shell
> #enter these command to set up xindi
> cd /root
> rm -rf xindi
> git clone https://github.com/Rainboooom/QIDISmart3.git
> mv QIDISmart3 xindi
> cd /root/xindi/build
> cmake ..
> make
> ```
5. The complete process takes about ten minutes, when you see that the compilation progress has reached 100%, it indicates that you have completed the replacement. Just restart the machine.











  
