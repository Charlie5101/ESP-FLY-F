# _Sample project_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This is the simplest buildable example. The example is used by command `idf.py create-project`
that copies the project to user specified path and set it's name. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project)



## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.

```
├── Top
|   ├── main
|   |   └── Task Create
|   └── 
├── Mid
|   ├── sensor
|   ├── black box
|   ├── indicator
|   ├── buzzer
|   ├── bat_adc
|   ├── motor
|   ├── GPS
|   ├── RGB LED
|   ├── Receiver
|   ├── PID
|	└── DSHOT
└── ground
    ├── INT
    |   └── Control
    ├── SPI
    ├── UART
    ├── IIC
    ├── PWM
    ├── ADC
    └── GPIO
```
### System Struct
```
└── main.c
    ├── main
    |	├── Task Create
    |	└── Task
 	└── Task MAIN
    |	└── Task
    └── Task sensor
    └── Task black box
    └── Task indicator
    |	└── indicator
    |    	└── SPI/RMT
    └── Task buzzer
    └── Task bat adc
    └── Task motor
    └── Task GPS
    └── Task RGB LED
    └── Task Uppermonitor
    └── Task Wifi Rec
```

### File Call Struct
```
├── main.c
|   ├── main
|   |	├── Task Create
|   |	└── Task
|	└── Task MAIN
|   |	└── Task
|   └── Task sensor
|   └── Task black box
|   └── Task indicator
|   |	└── SPI/RMT
|   └── Task buzzer
|   └── Task bat adc
|   └── Task motor
|   └── Task GPS
|   └── Task RGB LED
|   └── Task Uppermonitor
|   └── Task Wifi Rec
├── Mid
|   ├── sensor
|   ├── black box
|   ├── indicator
|   ├── buzzer
|   ├── bat_adc
|   ├── motor
|   ├── GPS
|   ├── RGB LED
|   ├── Receiver
|   └── PID
└── ground
    ├── INT
    |   └── Control
    ├── SPI
    ├── UART
    ├── IIC
    ├── PWM
    ├── ADC
    └── GPIO
```
