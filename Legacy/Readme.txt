
Readme.txt file for Shimmer Matlab Instrument Driver  
----------------------------------------------------

The Shimmer Matlab Instrument Driver package is a set of Matlab .m files designed to assist users of the 
Shimmer2, Shimmer2r and Shimmer3 in the development of Shimmer based applications in Matlab.

The driver package contains the following items:
 
 - Shimmer Matlab Instrument Driver User Manual 
 - Sampling Rate Table.txt (in resources directory)
 - Readme.txt (this file)
 - ShimmerBiophysicalProcessingLibrary_Rev_X_Y.jar
 - ReadmeShimmerBiophysicalProcessingLibrary.txt

 - Shimmer Matlab Instrument Driver   
    ShimmerHandleClass.m
    
 - Examples and supporting Matlab scripts   
    plotandwriteexample.m
    twoshimmerexample.m
    orientation3Dexample.m
    plotandwriteecgexample.m
    plotandwriteemgexample.m
    plotandwriteexgtestsignalexample.m
    plotandwriteecgleadoffdetectionexample
    plotandwriteemgleadoffdetectionexample
    ppgtoheartrateexample.m
    ecgtoheartrateexample.m
    SetEnabledSensorsMacrosClass.m
    FilterClass.m
          
    in quaterion directory:
     quat2angleaxis.m                                 
     quatconjugate.m
     quatmultiply.m
     quatrotate.m
     
To begin using the Shimmer Matlab Instrument Driver please refer to the
Shimmer Matlab Instrument Driver User Manual.

#############################################################

Rev0.1 is the original beta release. 
Release date 02/02/2012.

#############################################################

Rev0.2 is the original beta release with bug fixes and minor 
modifications. 
Release date 17/02/2012.


Changes since Rev 0.1
=====================

Known Bugs
----------

Bug ID. 0.1.1 
Bug Description: Trying to disconnect when already disconnected generates an error.
Bug Status: Fixed.

Bug ID. 0.1.2 
Bug Description: Issue with boolean() variable in Matlab 2011 and earlier.
Bug Status: Fixed.

Bug ID. 0.1.3 
Bug Description: Wait for acknowledgment timing out on some systems. Changed ShimmerHandleClass.m -> DEFAULT_TIMEOUT value from 5 secs to 8 secs.
Bug Status: Fixed.


Modifications
-------------

ShimmerHandleClass.m -> waitforack
Added tic/toc to improve accuracy of timeout functionality


Minor updates and corrections to Matlab Library User Manual


#############################################################

Rev0.3 is the original beta release with bug fixes and minor 
modifications. 
Release date 27/02/2012.


Changes since Rev 0.2
=====================

Known Bugs
----------

Bug ID. 0.2.1 
Bug Description: Error check failed to isolate corrupt packets.
Bug Status: Fixed.

Bug ID. 0.2.2 
Bug Description: Byte sequence 13 10 was being interpreted as 'carriage return' 'newline' and 13 was being automatically removed.
Bug Status: Fixed.


Modifications
-------------

plotandwriteexample.m 
Data is now written in tab delimited format as opposed to comma delimited format.


Minor updates and corrections to Matlab Library User Manual


#############################################################

Rev0.4 is the original beta release with bug fixes and minor 
modifications. 
Release date 29/02/2012.


Changes since Rev 0.3
=====================

Modifications
-------------
Name change from Shimmer Matlab Library to Shimmer Matlab Instrument Driver.

Appropriate changes to User Manual to reflect name change.


#############################################################

Rev0.5 is the original beta release with a bug fix 

Release date 24/04/2012.


Changes since Rev 0.4
=====================

Known Bugs
----------

Bug ID. 0.4.1 
Bug Description: Magnetometer signal erroneous when expected value is negative.
Bug Status: Fixed.


#############################################################


Rev1.0 is the first full release. It is based on the original beta release 
with significant additions to the functionality. The additions include the
ability to retrieve both uncalibrated and calibrated data and a method
to check for any dropped packets.

Release date 03/07/2012.


Changes since Rev 0.5
=====================

Known Bugs
----------

Bug ID. 0.5.1 
Bug Description: Sensors are all enabled when the Heart Rate sensor is enabled. 
Bug Status: Fixed.

Added the following functions:-

getcalibrationparameters
getacceldata
calculatetwoscomplement
getgyrodata
getmagdata
gettimestampdata
getecgdata
getemgdata
getexpboarddata
getstraingaugedata
getgsrdata
getheartratedata
getenabledsensors
calibrategsrdata
calibrateinertialsensor
calibratetimestampdata
getpercentageofpacketsreceived
getdata


#############################################################



Rev1.1

Release date 13/08/2012.


Changes since Rev 1.0
=====================

Known Bugs
----------

Bug ID. 1.0.1 
Bug Description: Any sampling rate above 255Hz is stored as 255. 
Bug Status: Fixed.

#############################################################

Rev1.2

Release date 13/08/2012.


Changes since Rev 1.1
=====================

Known Bugs
----------

Bug ID. 1.1.1 
Bug Description: Any sampling rate above 255Hz is stored as 255 when an inquiry is done. 
Bug Status: Fixed.

#############################################################

Rev1.3

Release date 21/02/2013.


Changes since Rev 1.2
=====================

Modifications
-------------

The command setpmux has been set to a private function, the pmux is set automatically now depending 
on whether battery voltage monitoring or Exp Board is enabled. To enabled batt voltage monitoring the following methods were updated/made
- getdata
- getenabledsignalnames
- getbattvoltdata
- setexternalboard
- setenabledsensors
- updated get calibration methods, now returns the com port when a shimmer device is not calibrated
- the gsr calibration method has been updated
- accel calibration parameters have been updated
- default accel calibration parameters for all ranges have been added
- default alignment for accel has been updated
- warning msgs concerning crosstalk when using accel + exp board a7 added
- warning msg concerning calibration parameters when setting a different accel range

Known Bugs
----------

Bug ID. 1.2.1 
Bug Description: setpmux and setfivevoltreg command not working.
Bug Status: Fixed.


#############################################################

Rev1.4

Release date 20/03/2013.


Changes since Rev 1.3
=====================

Modifications
-------------

- Battery monitoring
- Low Battery notification (only usable with BTStream)
- get ecg,emg,all calibration parameters
- write ecg and emg calibration parameters
- Firmware version now supported (Boiletplate 0.1 and BTStream 1.0), note that Boilerplate is now considered legacy firmware
- Updates to Heart Rate to support BTStream 1.0 which now sends a 2 byte value, indicating when the pulse occured and the time duration between the curent pulse and the previous one
- ECG and EMG units are mVolts* when default calibration parameters are used	


#############################################################

Rev1.5

Release date 27/06/2013.


Changes since Rev 1.4
=====================

Modifications
-------------

- Magnetometer configuration (range and data rate).
- 3D orientation estimation (in quaternion format); note that accel, gyro and mag must be enabled.
- New example: orientation3Dexample.m.
- Gyro in-use calibration method to update offset bias vector.	
- Added function to set buffer size; this is called from shimmer.connect().
- Fixed FirmwareVersion to take a non-integer value.


#############################################################

Rev1.6

Release date 27/06/2013.


Changes since Rev 1.5
=====================

Modifications
-------------

- Updated to work with Shimmer3


#############################################################

Rev1.7

Release date 11/10/2013.


Changes since Rev 1.6
=====================

Modifications
-------------

- Updated to work with BTStream and Boilerplate 
- Fixed twos complement method


#############################################################

Rev1.8

Release date 14/10/2013.


Changes since Rev 1.7
=====================

Modifications
-------------

- Added pressure sensor support for Shimmer3


#############################################################

Rev1.9

Release date xx/xx/2013.


Changes since Rev 1.8
=====================

Modifications
-------------

- Updated twos complement method because current use of BITCMP(x,n) is being deprecated
- Updated code to support Low Noise Accel and Wide Range Accel
- Updated getdata method to check whether there is any data read from the buffer, before processsing the data
- Added GSR support for Shimmer3
- Added minor support for EXG for internal testing, users should wait for official release


#############################################################

Rev2.0

Release date 29/01/2014.


Changes since Rev 1.9
=====================

Modifications
-------------

- Removed old folders (fw) and old scripts which dont belong


#############################################################

Rev2.1 

Release date 21/03/2014.


Changes since Rev 2.0
=====================

Modifications
-------------

- ExG support for Shimmer3 (including examples)
- Add battlimitwarning for Shimmer3
- Revised comments and warnings


#############################################################

Rev2.2 

Release date 01/07/2014.


Changes since Rev 2.1
=====================

Modifications
-------------

- Bridge Amplifier support for Shimmer3 (Shimmer2/2r equivalent is known as Strain Gauge)
- Improved filter implementation (using FilterClass.m)
- Functions for setting wide-range accelerometer high resolution and low power modes for Shimmer3  
- Compatibility change signal names accelerometers (User Manual section 'Differences between Shimmer2r and Shimmer3')
- Use of macros for enabling sensors 
- Function specific warnings
- Revised comments 
- pppgtoheartrateexample (Heart Rate from Photo Plethysmograph) 
- Improved existing examples
- Firmware compatibility codes


#############################################################

Rev2.3
 
Release date 13/10/2015.


Errata
------
- Version number in ShimmerHandleClass was not updated (v2.2)

Changes since Rev 2.2
=====================

Modifications
-------------

- Added SetEnabledSensorsMacrosClass, disableallsensors() - for easier enabling/disabling sensors.
- Improved existing examples. 
- orientation3Dexample - Changed orientation of Shimmer visualisation for Shimmer3.
- Added small description for all lower level functions.
- New getdata(), deprecated old function to depricatedgetdata()
- ExG lead-off support
- Get Expansion Board ID support
- LogAndStream FW support
- Configurable Baudrate support
- ecgtoheartrateexample (Heart Rate from ECG)
- plotandwriteecgleadoffdetectionexample
- plotandwriteemgleadoffdetectionexample

Fixed Bugs
----------
- 16BIT ExG is parsed incorrectly - interpretdatapacketformat() 
- Strain Gauge data incorrectly calibrated; gain was multiplied by a factor 2.8, while this should be a factor 1.
- Bridge Amplifier+ data incorrectly calibrated; gain was multiplied by a factor 2.8, while this should be a factor 1.
- Incorrect default calibration parameters Shimmer3 magnetometer. (Minor impact: for a few ranges test values were used instead of default values from datasheet, alignment matrix was multiplied by -1.)


#############################################################

Rev2.4 

Release date 06/03/2015.


Changes since Rev 2.3
=====================

Modifications
-------------

- Added resettodefaultconfiguration() - resets Shimmer default configuration for Shimmer3.
- Class properties are now updated when setconfigbytes() / setconfigbyte0() is called.
- Autoset data rates of Wide Range Accel, Gyro, Mag and ExG when Shimmer sampling rate is set.   

Fixed Bugs
----------
- getorientation3D() - wrong variablename internalBoard.
- parseinquiryresponse() - class property ConfigByte1 incorrectly updated.


#############################################################

Rev2.5 

Release date 12/05/2015.


Changes since Rev 2.4
=====================

Modifications
-------------
- Changed recommended filter cut-off frequency to 5Hz in plotandwriteemgexample
- Update: ShimmerBiophysicalProcessingLibrary_Rev_0_8.jar
- getgsrdata() -> reverted back to Shimmmer2r values for Shimmer3
- Modified getpercentageofpacketsreceived() to accommodate for minor possible variations in internal Shimmer clock


Fixed Bugs
----------
- Updated readfirmwareversion to be compatible with versions > 0.6.0/0.4.0 of BtStream/LogAndStream.
- Fixed bug in estimategyrooffset() for the case when GyroBuffer = [0 0 0];
- DAccelCalParametersOV / DAccelCalParameterSM / DAccelCalParametersOVAM had Shimmer2r values -> changed to Shimmer3 values (2g range).
- Added missing case for getaccelrange==0 in getcalibrationparameters().


#############################################################

Rev2.6 

Release date dd/mm/yyyy.


Changes since Rev 2.5
=====================

Modifications
-------------
- Updated/corrected help comments for getdeprecateddata()/getdata().
- Updated ShimmerBiophysicalProcessingLibrary + Readme to Rev_0_10. 
- Updated ecgtoheartrateexample for new version of ShimmerBiophysicalProcessingLibrary - with improved ECGtoHR algorithm.
- Updated and renamed LogAndStream statuses. 
- For firmware compatibility code >= 6:
  - Three byte timestamp.
  - Added get battery voltage command (for 'Connected' state - new instream command response).
    - For LogAndStream FW:
      - Added get/set/read/write functions for Real Word Clock (RWC) on Shimmer.  
      - Added new Action Methods for start/stop 'logging only'; stop 'logging+streaming' -> 'stop' now only stops streaming.
- For firmware compatibility code >= 7:
  - Disable periodic streaming of Battery Voltage; not supported.  
- Added enabletimestampunix() - enable to:
   - get system (PC) timestamp for last sample in each serial buffer.
   - add 'Time Stamp Unix' channel to data after 'Time Stamp' - 'Nan' between consecutive system timestamps.    
- Added convertUnixTimeMillisecondsToMatlabTime() / convertMatlabTimeToUnixTimeMilliseconds()


#############################################################

Rev2.7 

Release date 14/08/2017.


Changes since Rev 2.6
=====================

Modifications
-------------
- Added support for new IMU/BMP sensors. Based on 'HardwareCompatibilityCode', see determinehwcompcode().
- Added Pressure/temperature example - BMP180/BMP280.
- Minor comment fixes in examples.   
- Updated getexpboardid() - for SR31 and unified boards: SR47-49.
- setaccelrate() - added rate setting '9' - 1344Hz for LSM303DLHC.

#############################################################

Rev2.8

Release date 19/09/2017.


Changes since Rev 2.7
=====================

Modifications
-------------
- Bugfix -> HardwareCompatibilityCode for Shimmer3 IMU (SR31).
- Minor other bugfixes.
- Examples now also write signal format/units to file.