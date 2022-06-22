# Verisense Matlab API 
Currently the Verisense Matlab Examples are released as a Pre-Alpha release

Please refer to our [support policy](https://shimmersensing.com/support/wireless-sensor-networks-documentation/) on Pre-Alpha/Alpha Releases

## Introduction
These examples is to help users get started with using the Verisense device. The Verisense [datarecordingexample](https://github.com/ShimmerEngineering/Shimmer-MATLAB-ID/blob/master/VerisenseTools/datarecordingexample.m) is the best example to start with.


## Hardware Supported
- Verisense IMU
- Verisense Pulse+ (_upcoming_)

## Minimum Matlab Version Tested On
- Matlab 12 (Windows)
_Note:_ The provided Verisense Matlab examples uses the c# API executable example provided [here](https://github.com/ShimmerEngineering/Shimmer-C-API/tree/master/ShimmerBLE/ConsoleTools/VerisenseConfigureAndSyncConsole) to communicate with the Verisense device via BLE. While the example does support linux (tested on Ubuntu) we've not tested it with Matlab running on linux. 

## Functionality Supported
- Configuring via BLE, we currently only support a limited amount of [default configurations](https://github.com/ShimmerEngineering/Shimmer-C-API/wiki/Verisense-Default-Configurations), do get in touch with us if you require a different set of configurations
- Logged Sensor Data Syncing via BLE
_note: syncing is the retrieval of logged sensor data in a binary file fomat. **this binary file can be [parsed using this Java app](https://github.com/ShimmerEngineering/Shimmer-MATLAB-ID/tree/master/VerisenseTools/FileParser), however users should be aware that we do not provide file parsing source code in C#**_

## VerisenseConfigureAndSyncConsoleApp
The console app is used to communicate with the Verisense device. It can be used to configure the Verisense device and run data sync. For more details regarding the console app, can refer to [VerisenseConfigureAndSyncConsoleApp](https://github.com/ShimmerEngineering/Shimmer-C-API/blob/master/ShimmerBLE/ConsoleTools/README.md)

## FileParser
The file parser is used to parsed the binary files generated after data sync. For more details regarding the file parser and parsed file, can refer to [file parser](https://github.com/ShimmerEngineering/Shimmer-C-API/blob/master/ShimmerBLE/FileParser/README.md)

## syncandparseexample
This example is used to synchronize and parse the data of a Verisense device via Bluetooth

HOW TO USE:
- Open the repository in matlab
- Run the syncandparseexample function with the following parameters
    - uuid of the Verisense device, e.g. '00000000-0000-0000-0000-d02b463da2bb'
    - the path of the binary files, e.g. 'C:\\\\Users\\\\VerisenseUser\\\\Desktop'
    - the trial name, e.g. 'trialA'
    - the participant ID, e.g. 'participantB'

## configureexample
This example is used to set the Verisense device to default configuration settings. Note that the RTC clock is being set while connected

HOW TO USE:
- Open the repository in matlab
- Run the configureexample function with the following parameters
    - uuid of the Verisense device, e.g. '00000000-0000-0000-0000-d02b463da2bb'
    - the default operational configuration (ACCEL1/ACCEL2_GYRO/GSR_BATT_ACCEL1/GSR_BATT/PPG).

NOTE: The RTC clock is being set while connecting.

## disableloggingexample
This example is used to disable logging

HOW TO USE:
- Open the repository in matlab
- Run the disableloggingexample function with the following parameter
    - uuid of the Verisense device, e.g. '00000000-0000-0000-0000-d02b463da2bb'

## erasedataexample
This example is used to erase the sensor data

HOW TO USE:
- Open the repository in matlab
- Run the erasedataexample function with the following parameters
    - uuid of the Verisense device, e.g. '00000000-0000-0000-0000-d02b463da2bb'


## datarecordingexample
This example is used to perform a data collection. Erase data on the device, set default operational config, collect data throughout the duration specified, sync the Verisense device, parse the data, plot some of the parsed data and disable logging.

HOW TO USE:
- Open the repository in matlab
- Run the syncandparseexample function with the following parameters
    - uuid of the Verisense device, e.g. '00000000-0000-0000-0000-d02b463da2bb'
    - duration of the data collection
    - the default operational configuration (ACCEL1/ACCEL2_GYRO/GSR_BATT_ACCEL1/GSR_BATT/PPG).
    - the path of the binary files, e.g. 'C:\\\\Users\\\\WeiWentan\\\\Desktop'
    - the trial name, e.g. 'trialA'
    - the participant ID, e.g. 'participantB'
