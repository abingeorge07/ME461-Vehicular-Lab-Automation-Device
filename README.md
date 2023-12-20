# ME461-Vehicular-Lab-Automation-Device
<b>Boston University ENG 2023: Senior Capstone</b> 

<p align="left" width="100%">
    <img width="33%" src="Assets/VLAD.png"> 
</p>


**Team VLAD Members:**

- Adam Bahlous-Boldi
- Miguel Ianus-Valdivia
- Kyle Fieleke
- Vlad Pyltsov
- Abin Binoy George

## Table of Contents

1. [Motivation and Goals](#motivation-and-goals)
2. [Peripherals](#peripherals)
3. [Folder Description](#folder-description)
4. [Final Report](#final-report)
5. [Video Demo](#final-report)
6. [Software Documentation](#software-documentation)
7. [Next Steps](#next-steps)


## Motivation and Goals
This was our senior design project at Boston University, with the main objective of showcasing our skills as an engineer. This project was proposed by Dr. Keith Brown.

Huge thanks to Dr. Brown and Professor Anthony Linn for their support.

## Peripherals

**Microcontroller:** ESP32 
**Peripheral used:**
- DC Motor
- Infrared Sensor
- Encoder
- Servo Motor
- IMU


## Folder Description
- **Assets**: Images
- **CAD Files**: All the CAD files.
- **Documents**: Final report and instruction report.
- **Individual Peripheral Testing**: Test scripts for each peripheral.
- **Main Scripts**: Main build scripts for the microcontroller and the PC sending instructions to the controller.
- **Datasheet**: Datasheet for some of the peripherals. 

## Final Report
More information on the project details and our solution can be found in our [final report.](Documents/Team%2021%20Vlad%20ME461%20Final%20Report.pdf) 

## Video Demo
Video demo can be found on [Google Drive.](https://drive.google.com/drive/folders/1fOim3Kx6D7g6Yu-gywjHp_sshHIz1DwE?usp=share_link)


## Software Documentation 
Information on how to build and run the software scripts can be found in this [document.](Documents/Documentation.pdf) Following is just a brief overview of this document.

### Espressif Installation 
Espressif IDE can be installed [here.](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#installation-step-by-step)

### Build Project
```
idf.py set-target esp32
idf.py menuconfig
idf.py build
```

### Flash onto Microcontroller
```
idf.py -p <port> flash
```

### Monitor output from Microcontroller
```
idf.py -p <port> monitor
```
## Next Steps
While we have provided a good starting point for any laboratory to begin automating its
experiments, there are still a few tasks that can be done to further advance this project.

If you face any bugs, please create an issue on this GitHub repository.


