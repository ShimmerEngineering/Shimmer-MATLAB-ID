%  * Rev_2.8
%  * 
%  * Copyright (c) 2010 - 2015, Shimmer Research, Ltd. 
%  * All rights reserved
%  *
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions are
%  * met:
%  *
%  *    * Redistributions of source code must retain the above copyright
%  *      notice, this list of conditions and the following disclaimer.
%  *    * Redistributions in binary form must reproduce the above
%  *      copyright notice, this list of conditions and the following
%  *      disclaimer in the documentation and/or other materials provided
%  *      with the distribution.
%  *    * Neither the name of Shimmer Research, Ltd. nor the names of its
%  *      contributors may be used to endorse or promote products derived
%  *      from this software without specific prior written permission.
%  *    * You may not use or distribute this Software or any derivative works
%  *      in any form for commercial purposes with the exception of commercial
%  *      purposes when used in conjunction with Shimmer products purchased 
%  *      from Shimmer or their designated agent or with permission from 
%  *      Shimmer. 
%  *      Examples of commercial purposes would be running business 
%  *      operations, licensing, leasing, or selling the Software, or 
%  *      distributing the Software for use with commercial products.
%  *
%  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
%  * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
%  * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
%  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
%  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
%  * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
%  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%  *
%  * @version 2.0
%  * @author Jong Chern Lim
%  * @date   October, 2013
%  * 
%  * @version 2.1
%  * @author Jong Chern Lim, Ruud Stolk, Niamh O'Mahony
%  * @date   March, 2014
%  *
%  * @version 2.2
%  * @author Ruud Stolk, Niamh O'Mahony
%  * @date   June, 2014
%  *
%  * @version 2.3
%  * @author Ruud Stolk
%  * @date   October, 2015
%  *
%  * @version 2.4
%  * @author Ruud Stolk
%  * @date   March, 2015
%  *
%  * @version 2.5
%  * @author Ruud Stolk
%  * @date   May, 2015
% 
%  * @version 2.6
%  * @author Ruud Stolk
%  * @date   Oct, 2015
%
%  * @version 2.7
%  * @author Ruud Stolk
%  * @date   Aug, 2017
%
%  * @version 2.8
%  * @author Ruud Stolk
%  * @date   Sep, 2017



classdef ShimmerHandleClass < handle   % Inherit from super class 'handle' 
    %ShimmerHandleClass   a class to interface with the Shimmer wireless sensor
    %                     platform.
    %
    %% view ShimmerHandleClass properties
    % properties ShimmerHandleClass
    
    %% view ShimmerHandleClass methods
    % methodview ShimmerHandleClass;
     
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties (SetAccess='protected', GetAccess='public')
        
        ComPort='Nan';                                                     % String value defining the numerical identifier of the of the COM Port paired with the Shimmer
        State='Disconnected';                                              % String value defining the state of the Shimmer
        InternalBoard={'None'};                                            % Cell value defining the internal daughter board attached to the Shimmer
        ExternalBoard={'None'};                                            % Cell value defining the external daughter board attached to the Shimmer
        SamplingRate='Nan';                                                % Numerical value defining the sampling rate of the Shimmer
        AccelRange='Nan';                                                  % Numerical value defining the accelerometer range of the Shimmer
        AccelWideRangeDataRate='Nan';
        AccelWideRangeHRMode = 'Nan';                                      % High Resolution mode LSM303DLHC/LSM303AHTR
        AccelWideRangeLPMode = 'Nan';                                      % Low Power mode LSM303DLHC/LSM303AHTR 
        FirmwareIdentifier='Nan';
        FirmwareInternal='Nan';
        FirmwareMajorVersion='Nan';
        FirmwareMinorVersion='Nan';
        FullFirmwareName='Nan';
        FirmwareCompatibilityCode = 'Nan';                                 % Firmware compatibility code, see readfirmwareversion for details
        GsrRange='Nan';                                                    % Numerical value defining the gsr range of the Shimmer
        MagRange='Nan';                                                    % Numerical value defining the mag range of the Shimmer
        GyroRange='Nan';                                                   % Numerical value defining the gyro range of the Shimmer
        MagRate='Nan';                                                     % Numerical value defining the mag rate of the Shimmer
        InternalExpPower='Nan';                                            % Numerical value defining the internal exp power for the Shimmer3
        GyroRate='Nan';
        PressureResolution='Nan';
        ShimmerVersion='Nan';
        BlinkLED='Nan';
        UsingDefaultAccelCalParams=1;
        HardwareCompatibilityCode = 'Nan';                                 % Numerical value identifying which sensors are onboard the connected Shimmer3                                   
        
        % LogAndStream firmware
        SdCardDirectoryName = 'Nan';                                       % Name of SD Log Directory in use
        IsSensing = 'Nan';                                                 % Status: Sensing/Not Sensing
        IsDocked = 'Nan';                                                  % Status: Docked/Not docked
        IsSDLogging = 'Nan';                                               % Status: SDLogging/Not SDlogging
        IsStreaming = 'Nan';                                               % Status: Streaming/Not Streaming

        
        % ACCEL Calibration
        DefaultAccelCalibrationParameters=true;
        DefaultDAccelCalibrationParameters=true;
        AccelCalParametersOV=[2048;2048;2048];                             
        AccelCalParametersSM=[101 0 0; 0 101 0; 0 0 101];
        AccelCalParametersAM=[-1 0 0; 0 -1 0; 0 0 1];   
        % Needed to store calibration values of Digital Accel
        DAccelCalParametersOV=[0;0;0];                           
        DAccelCalParametersSM=[1631 0 0; 0 1631 0; 0 0 1631]; 
        DAccelCalParametersAM=[-1 0 0; 0 1 0; 0 0 -1];   
        
        % Accel Calibration Default Values for Shimmer 2
        AccelCalParametersSM1p5gShimmer2=[101 0 0; 0 101 0; 0 0 101];      % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (1.5g))
        AccelCalParametersSM2gShimmer2=[76 0 0; 0 76 0; 0 0 76];           % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (2g))
        AccelCalParametersSM4gShimmer2=[38 0 0; 0 38 0; 0 0 38];           % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (4g))
        AccelCalParametersSM6gShimmer2=[25 0 0; 0 25 0; 0 0 25];           % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (6g))
        AccelCalParametersOVShimmer2=[2048;2048;2048]; 
        AccelCalParametersAMShimmer2=[-1 0 0; 0 -1 0; 0 0 1];  
        % Accel Calibration Default Values for Shimmer3
        AccelLowNoiseCalParametersSM2gShimmer3=[83 0 0; 0 83 0; 0 0 83];           % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (2g))  - KXRB5-2042
        AccelLowNoiseCalParametersSM2gShimmer3_2=[92 0 0; 0 92 0; 0 0 92];         % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (2g))  - KXTC9-2050
        AccelWideRangeCalParametersSM2gShimmer3=[1631 0 0; 0 1631 0; 0 0 1631];    % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (2g))  - LSM303DLHC
        AccelWideRangeCalParametersSM4gShimmer3=[815 0 0; 0 815 0; 0 0 815];       % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (4g))  - LSM303DLHC
        AccelWideRangeCalParametersSM8gShimmer3=[408 0 0; 0 408 0; 0 0 408];       % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (8g))  - LSM303DLHC
        AccelWideRangeCalParametersSM16gShimmer3=[135 0 0; 0 135 0; 0 0 135];      % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (16g)) - LSM303DLHC
        AccelWideRangeCalParametersSM2gShimmer3_2=[1671 0 0; 0 1671 0; 0 0 1671];  % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (2g))  - LSM303AHTR
        AccelWideRangeCalParametersSM4gShimmer3_2=[836 0 0; 0 836 0; 0 0 836];     % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (4g))  - LSM303AHTR
        AccelWideRangeCalParametersSM8gShimmer3_2=[418 0 0; 0 418 0; 0 0 418];     % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (8g))  - LSM303AHTR
        AccelWideRangeCalParametersSM16gShimmer3_2=[209 0 0; 0 209 0; 0 0 209];    % Default Calibration Parameters for Accelerometer (Sensitivity Matrix (16g)) - LSM303AHTR
        AccelLowNoiseCalParametersOVShimmer3=[2047;2047;2047];                     % KXRB5-2042
        AccelLowNoiseCalParametersOVShimmer3_2=[2253;2253;2253];                   % KXTC9-2050
        AccelWideRangeCalParametersOVShimmer3=[0;0;0];                             % LSM303DLHC/LSM303AHTR
        AccelLowNoiseCalParametersAMShimmer3=[0 -1 0; -1 0 0; 0 0 -1];             % KXRB5-2042/KXTC9-2050
        AccelWideRangeCalParametersAMShimmer3=[-1 0 0; 0 1 0; 0 0 -1];             % LSM303DLHC
        AccelWideRangeCalParametersAMShimmer3_2=[0 -1 0; 1 0 0; 0 0 -1];           % LSM303AHTR
        
        
        % GYRO Calibration
        DefaultGyroCalibrationParameters=true;
        GyroCalParametersOV=[0;0;0];                              
        GyroCalParametersSM=[2.73 0 0; 0 2.73 0; 0 0 2.73];                
        GyroCalParametersAM=[0 -1 0; -1 0 0; 0 0 -1];                      
        % Gyro Calibration Default Values for Shimmer 2
        GyroCalParametersOVShimmer2=[1843;1843;1843];                      % Default Calibration Parameters for Gyroscope (Offset Vector)
        GyroCalParametersSMShimmer2=[2.73 0 0; 0 2.73 0; 0 0 2.73];        % Default Calibration Parameters for Gyroscope (Sensitivity Matrix)
        GyroCalParametersAMShimmer2=[0 -1 0; -1 0 0; 0 0 -1];              % Default Calibration Parameters for Gyroscope (Alignment Matrix)
        % Gyro Calibration Default Values for Shimmer3
        GyroCalParametersOVShimmer3=[0;0;0];                               % Default Calibration Parameters for Gyroscope (Offset Vector)
        GyroCalParametersSM2000dpsShimmer3=[16.4 0 0; 0 16.4 0; 0 0 16.4]; % Default Calibration Parameters for Gyroscope (Sensitivity Matrix)
        GyroCalParametersSM1000dpsShimmer3=[32.8 0 0; 0 32.8 0; 0 0 32.8]; % Default Calibration Parameters for Gyroscope (Sensitivity Matrix)
        GyroCalParametersSM500dpsShimmer3=[65.5 0 0; 0 65.5 0; 0 0 65.5];  % Default Calibration Parameters for Gyroscope (Sensitivity Matrix)
        GyroCalParametersSM250dpsShimmer3=[131 0 0; 0 131 0; 0 0 131];     % Default Calibration Parameters for Gyroscope (Sensitivity Matrix)
        GyroCalParametersAMShimmer3=[0 -1 0; -1 0 0; 0 0 -1];              % Default Calibration Parameters for Gyroscope (Alignment Matrix)
        
        %MAG Calibration
        DefaultMagneCalibrationParameters=true;
        MagneCalParametersOV=[0;0;0];                                      
        MagneCalParametersSM=[580 0 0;0 580 0;0 0 580];                    
        MagneCalParametersAM=[1 0 0; 0 1 0; 0 0 -1];                       
        % Mag Calibration Default Values for Shimmer 2
        MagneCalParametersOVShimmer2=[0;0;0];                              % Default Calibration Parameters for Magnetometer (Offset Vector)
        MagneCalParametersSMShimmer2=[580 0 0;0 580 0;0 0 580];            % Default Calibration Parameters for Magnetometer (Sensitivity Matrix)
        MagneCalParametersAMShimmer2=[1 0 0; 0 1 0; 0 0 -1];               % Default Calibration Parameters for Magnetometer (Alignment Matrix)
        % Mag Calibration Default Values for Shimmer3
        MagneCalParametersOVShimmer3=[0;0;0];                              % Default Calibration Parameters for Magnetometer (Offset Vector)      - LSM303DLHC/LSM303AHTR
        MagneCalParametersSM1_3gaussShimmer3=[1100 0 0;0 1100 0;0 0 980];  % Default Calibration Parameters for Magnetometer (Sensitivity Matrix) - LSM303DLHC
        MagneCalParametersSM1_9gaussShimmer3=[855 0 0;0 855 0;0 0 760];    % Default Calibration Parameters for Magnetometer (Sensitivity Matrix) - LSM303DLHC
        MagneCalParametersSM2_5gaussShimmer3=[670 0 0;0 670 0;0 0 600];    % Default Calibration Parameters for Magnetometer (Sensitivity Matrix) - LSM303DLHC
        MagneCalParametersSM4_0gaussShimmer3=[450 0 0;0 450 0;0 0 400];    % Default Calibration Parameters for Magnetometer (Sensitivity Matrix) - LSM303DLHC
        MagneCalParametersSM4_7gaussShimmer3=[400 0 0;0 400 0;0 0 355];    % Default Calibration Parameters for Magnetometer (Sensitivity Matrix) - LSM303DLHC
        MagneCalParametersSM5_6gaussShimmer3=[330 0 0;0 330 0;0 0 295];    % Default Calibration Parameters for Magnetometer (Sensitivity Matrix) - LSM303DLHC
        MagneCalParametersSM8_1gaussShimmer3=[230 0 0;0 230 0;0 0 205];    % Default Calibration Parameters for Magnetometer (Sensitivity Matrix) - LSM303DLHC
        MagneCalParametersSM49_2gaussShimmer3=[667 0 0;0 667 0;0 0 667];   % Default Calibration Parameters for Magnetometer (Sensitivity Matrix) - LSM303DLHC

        MagneCalParametersAMShimmer3=[-1 0 0; 0 1 0; 0 0 -1];              % Default Calibration Parameters for Magnetometer (Alignment Matrix)   - LSM303DLHC
        MagneCalParametersAMShimmer3_2=[0 -1 0; 1 0 0; 0 0 -1];            % Default Calibration Parameters for Magnetometer (Alignment Matrix)   - LSM303AHTR
        
        DefaultECGCalibrationParameters=true;
        DefaultEMGCalibrationParameters=true;
        DefaultPressureCalibrationParameters=true;
        DefaultEXGCalibrationParameters=true;
        
        GyroInUseCalibration = 0;                                          % Enable/disable gyro in-use calibration
        GyroBufferSize = 100;                                              % Buffer size (samples)
        GyroBuffer;                                                        % Buffer for gyro in-use calibration
        GyroMotionThreshold = 1.2;                                         % Threshold for detecting motion
        
        Orientation3D = 0;                                                 % Enable/disable 3D orientation, i.e. get quaternions in getdata
                
        %Shimmer 2/2r 
        EMGOffset = 2060;
        EMGGain = 750;
        ECGRALLOffset = 2060;
        ECGRALLGain = 175;
        ECGLALLOffset = 2060;
        ECGLALLGain = 175;
        BattVoltLimit = 3.4;
              
        WriteLEDBLINKWAITFORACK=0;
               
        % Pressure Sensor Coefficients - BMP180
        AC1 = 'Nan';
        AC2 = 'Nan';
        AC3 = 'Nan';
        AC4 = 'Nan';
        AC5 = 'Nan';
        AC6 = 'Nan';
        B1 = 'Nan';
        B2 = 'Nan';
        MB = 'Nan';
        MC = 'Nan';
        MD = 'Nan';
                
        % Pressure Sensor Coefficients - BMP280
        DIG_T1 = 'Nan';
        DIG_T2 = 'Nan';
        DIG_T3 = 'Nan';
        DIG_P1 = 'Nan';
        DIG_P2 = 'Nan';
        DIG_P3 = 'Nan';
        DIG_P4 = 'Nan';
        DIG_P5 = 'Nan';
        DIG_P6 = 'Nan';
        DIG_P7 = 'Nan';
        DIG_P8 = 'Nan';
        DIG_P9 = 'Nan';
       
        %SHIMMER3 ExG Configurations
        %ExG chip1
        EXG1Config1 = 'Nan';
        EXG1Config2 = 'Nan';
        EXG1Loff = 'Nan';
        EXG1Ch1Set = 'Nan';
        EXG1Ch2Set = 'Nan';
        EXG1RLD_Sens = 'Nan';
        EXG1LOFF_Sens = 'Nan';
        EXG1LOFF_Stat = 'Nan';
        EXG1Resp1 = 'Nan';
        EXG1Resp2 = 'Nan';
        EXG1CH1Gain = 'Nan';
        EXG1CH2Gain = 'Nan';
        EXG1Rate = 'Nan'; 
        EXG1PDB_LOFF_COMP = 'Nan'; % Lead-off comparator power-down
        EXG1FLEAD_OFF = 'Nan'; % Lead-off frequency 
        EXG1RLD_LOFF_SENSE = 'Nan'; % RLD lead-off sense function
        EXG1LOFF2N = 'Nan'; % Channel 2 lead-off detection negative inputs
        EXG1LOFF2P = 'Nan'; % Channel 2 lead-off detection positive inputs
        EXG1LOFF1N = 'Nan'; % Channel 1 lead-off detection negative inputs
        EXG1LOFF1P = 'Nan'; % Channel 1 lead-off detection positive inputs
        EXG1PD1 = 'Nan'; % Channel 1 power-down
        EXG1PD2 = 'Nan'; % Channel 2 power-down
        EXG1ILEAD_OFF = 'Nan'; % Lead-off current magnitude
        EXG1COMP_TH = 'Nan'; % Lead-off comparator threshold
        

        
        %ExG chip2
        EXG2Config1 = 'Nan';
        EXG2Config2 = 'Nan';
        EXG2Loff = 'Nan';
        EXG2Ch1Set = 'Nan';
        EXG2Ch2Set = 'Nan';
        EXG2RLD_Sens = 'Nan';
        EXG2LOFF_Sens = 'Nan';
        EXG2LOFF_Stat = 'Nan';
        EXG2Resp1 = 'Nan';
        EXG2Resp2 = 'Nan';
        EXG2CH1Gain = 'Nan';
        EXG2CH2Gain = 'Nan';
        EXG2Rate = 'Nan';  
        EXG2PDB_LOFF_COMP = 'Nan'; % Lead-off comparator power-down
        EXG2FLEAD_OFF = 'Nan'; % Lead-off frequency 
        EXG2RLD_LOFF_SENSE = 'Nan'; % RLD lead-off sense function
        EXG2LOFF2N = 'Nan'; % Channel 2 lead-off detection negative inputs
        EXG2LOFF2P = 'Nan'; % Channel 2 lead-off detection positive inputs
        EXG2LOFF1N = 'Nan'; % Channel 1 lead-off detection negative inputs
        EXG2LOFF1P = 'Nan'; % Channel 1 lead-off detection positive inputs
        EXG2PD1 = 'Nan'; % Channel 1 power-down
        EXG2PD2 = 'Nan'; % Channel 2 power-down
        EXG2ILEAD_OFF = 'Nan'; % Lead-off current magnitude
        EXG2COMP_TH = 'Nan'; % Lead-off comparator threshold

        EXGLeadOffDetectionMode = 'Nan';
        EXGReferenceElectrodeConfiguration = 'Nan';
        EMGflag = false; % flag is true if EMG, EMG 16BIT or EMG 24BIT is enabled
        ECGflag = false; % flag is true if ECG, ECG 16BIT or ECG 24BIT is enabled
        
        %Daughter Card ID
        IDByteArray = 'Nan'; 
        
        %Baud Rate
        BaudRate = 'Nan';
        
        % Battery Voltage - (FW comp. code > 6)
        LatestBatteryVoltageReading = 'Nan';  
               
        % Enable PC Timestamps
        EnableTimestampUnix = 0;
        LastSampleSystemTimeStamp = 0;
        
        % Real Time Clock on Shimmer3 - (LogAndStream FW with FW comp. code > 6)
        RealTimeClockMilliseconds = 'Nan';
        RealTimeClockTicks = 'Nan';
        
    end
    
    properties (Access='protected')
        
        Hrealterm;                                                         % Handle to realterm server
        FilePointer=0;                                                     % Pointer to current file read location in realterm buffer file
        
        BufferSize=1;                                                      % Shimmer data buffersize, currently not used
        
        SignalNameArray;                                                   % Cell array contain the names of the enabled sensor signals in string format
        SignalDataTypeArray;                                               % Cell array contain the names of the sensor signal datatypes in string format
        nBytesDataPacket;                                                  % Unsigned integer value containing the size of a data packet for the Shimmer in its current setting
        
       
        
        ConfigByte0;                                                       % The current value of the config byte0 setting on the Shimmer
        ConfigByte1;                                                       % The current value of the config byte1 setting on the Shimmer3
        ConfigByte2;                                                       % The current value of the config byte2 setting on the Shimmer3
        ConfigByte3;                                                       % The current value of the config byte3 setting on the Shimmer3
        EnabledSensors;                                                    % Bitmap for the enable pins for the different Shimmer sensors currently enabled
        % Shimmer2/2r:
        SENSOR_ACCEL          = 8;   % 0x0080
        SENSOR_GYRO           = 7;   % 0x0040
        SENSOR_MAG            = 6;   % 0x0020
        SENSOR_ECG            = 5;   % 0x0010
        SENSOR_EMG            = 4;   % 0x0008
        SENSOR_GSR            = 3;   % 0x0004
        SENSOR_EXT_A7         = 2;   % 0x0002
        SENSOR_EXT_A0         = 1;   % 0x0001
        SENSOR_STRAIN         = 16;  % 0x8000
        SENSOR_HEART          = 15;  % 0x4000
        
        % Shimmer3:
        SENSOR_A_ACCEL            = 8;   % 0x000080
        % SENSOR_GYRO               = 7;   % 0x000040 (same for Shimmer2/2r)
        % SENSOR_MAG                = 6;   % 0x000020 (same for Shimmer2/2r)
        SENSOR_EXG1_24BIT         = 5;   % 0x000010
        SENSOR_EXG2_24BIT         = 4;   % 0x000008
        % SENSOR_GSR                = 3;   % 0x000004 (same for Shimmer2/2r)
        % SENSOR_EXT_A7             = 2;   % 0x000002 (same for Shimmer2/2r)
        SENSOR_BRIDGE_AMP         = 16;  % 0x8000   
        SENSOR_EXT_A6             = 1;   % 0x000001
        SENSOR_VBATT              = 14;  % 0x002000
        SENSOR_WR_ACCEL           = 13;  % 0x001000
        SENSOR_EXT_A15            = 12;  % 0x000800
        SENSOR_INT_A1             = 11;  % 0x000400
        SENSOR_INT_A12            = 10;  % 0x000200
        SENSOR_INT_A13            = 9;   % 0x000100
        SENSOR_INT_A14            = 24;  % 0x800000
        SENSOR_MPU9150_ACCEL      = 23;  % 0x400000
        SENSOR_MPU9150_MAG        = 22;  % 0x200000
        SENSOR_EXG1_16BIT         = 21;  % 0x100000
        SENSOR_EXG2_16BIT         = 20;  % 0x080000
        SENSOR_BMP180_PRESSURE    = 19;  % 0x040000
        SENSOR_BMP180_TEMPERATURE = 18;  % 0x020000        
        
        SerialDataOverflow = [];                                           % Feedback buffer used by the framepackets method
        nFramePacketErrors = 0;                                            % Packet error count used by the framepackets method
   
        nClockOverflows = 0;                                               % count number of clock overflows for time stamp calibration
        LastUncalibratedLoopTimeStamp = 0;                                 % Last received uncalibrated looped time stamp data
        
        HRLastKnown=0;
        LastQuaternion=[0.5, 0.5, 0.5, 0.5];                               % Last estimated quaternion value, used to incrementally update quaternion.
        
        WarningGetUncalibratedData = 0;                                    % Warning flag     
        WarningGetDeprecatedGetData = 0;                                   % Warning flag    
        GetADCFlag = 0;                                                    % Flag for getdata/getadcdata
        
    end
       
    properties (Access='protected', Constant=true)
        ACK_RESPONSE = 255;                                                % The acknowledgemnt byte is the first byte value returned from the Shimmer after each command the Shimmer receives
        DATA_PACKET_START_BYTE = 0;                                        % First byte value in a Data Packet
        INQUIRY_COMMAND = char(1);                                         % Inquiry command value sent to the shimmer in order to receive an inquiry response
        INQUIRY_RESPONSE = char(2);                                        % First byte value in the Inquiry Response packet
        GET_SAMPLING_RATE_COMMAND = char(3);                               % Get sampling rate command sent to the shimmer in order to receive the sampling rate response
        SAMPLING_RATE_RESPONSE = char(4);                                  % First byte value received from the shimmer in the sampling rate response, it is followed by the byte value defining the setting
        SET_SAMPLING_RATE_COMMAND = char(5);                               % First byte sent to the shimmer when implementing a set sampling rate operation, it is followed by the byte value defining the setting
        TOGGLE_LED_COMMAND = char(6);                                      % This byte value is sent in order to toggle the red LED
        START_STREAMING_COMMAND = char(7);                                 % This byte value is sent in order to start data streaming from the Shimmer
        SET_SENSORS_COMMAND = char (8);                                    % First byte sent to the shimmer when implementing a set enabled sensors operation, it is followed by the 2 byte values defining the setting
        SET_ACCEL_RANGE_COMMAND = char(9);                                 % First byte sent to the shimmer when implementing a set accelerometer range operation, it is followed by the byte value defining the setting
        ACCEL_RANGE_RESPONSE = char(10);                                   % First byte value received from the shimmer in the accel range response, it is followed by the byte value defining the setting
        GET_ACCEL_RANGE_COMMAND = char(11);                                % Get accelerometer range command sent to the shimmer in order to receive the accelerometer range response
        SET_5V_REG_COMMAND = char(12);                                     % First byte sent to the shimmer when implementing a set 5 volt Regulator operation, it is followed by the bit value defining the setting
        SET_PMUX_COMMAND = char(13);                                       % First byte sent to the shimmer when implementing a set PMux operation, it is followed by the bit value defining the setting
        SET_CONFIG_BYTE0_COMMAND = char(14);                               % First byte sent to the shimmer when implementing a set config byte0 operation, it is followed by the byte value defining the setting
        CONFIG_BYTE0_RESPONSE = char(15);                                  % First byte value received from the shimmer in the config byte0 response, it is followed by the byte value defining the setting
        GET_CONFIG_BYTE0_COMMAND = char(16);                               % Get config byte0 command sent to the shimmer in order to receive the config byte0 response (Get config bytes byte0, byte1, byte2, byte3 For Shimmer3.) 
        STOP_STREAMING_COMMAND = char(32);                                 % This byte value is sent in order to stop data streaming from the Shimmer
        SET_GSR_RANGE_COMMAND = char(33);                                  % First byte sent to the shimmer when implementing a set gsr range operation, it is followed by the byte value defining the setting
        GSR_RANGE_RESPONSE = char(34);                                     % First byte value received from the shimmer in the config byte0 response, it is followed by the byte value defining the setting
        GET_GSR_RANGE_COMMAND = char(35);                                  % Get gsr range command sent to the shimmer in order to receive the gsr range response
        GET_SHIMMER_VERSION_COMMAND = char(36);                            % Get shimmer version command sent to the shimmer in order to receive the shimmer version response
        SHIMMER_VERSION_RESPONSE = char(37);                               % First byte value received from the shimmer in the shimmer version response, it is followed by the byte value defining the setting
        SET_EMG_CALIBRATION_COMMAND      = char(38);
        EMG_CALIBRATION_RESPONSE         = char(39);
        GET_EMG_CALIBRATION_COMMAND      = char(40);
        SET_ECG_CALIBRATION_COMMAND      = char(41);
        ECG_CALIBRATION_RESPONSE         = char(42);
        GET_ECG_CALIBRATION_COMMAND      = char(43);
        GET_ALL_CALIBRATION_COMMAND      = char(44);
        ALL_CALIBRATION_RESPONSE         = char(45);
        GET_FW_VERSION_COMMAND           = char(46);
        FW_VERSION_RESPONSE              = char(47);
        SET_BLINK_LED                    = char(48);
        BLINK_LED_RESPONSE               = char(49);
        GET_BLINK_LED                    = char(50);
        SET_GYRO_TEMP_VREF_COMMAND       = char(51);
        SET_BUFFER_SIZE_COMMAND          = char(52);
        BUFFER_SIZE_RESPONSE             = char(53);
        GET_BUFFER_SIZE_COMMAND          = char(54);
        SET_MAG_GAIN_COMMAND             = char(hex2dec('37'));
        MAG_GAIN_RESPONSE                = char(hex2dec('38'));
        GET_MAG_GAIN_COMMAND             = char(hex2dec('39'));
        SET_MAG_SAMPLING_RATE_COMMAND    = char(hex2dec('3A'));
        MAG_SAMPLING_RATE_RESPONSE       = char(hex2dec('3B'));
        GET_MAG_SAMPLING_RATE_COMMAND    = char(hex2dec('3C')); 
        GET_SHIMMER_VERSION_COMMAND_NEW  = char(hex2dec('3F')); 
        SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND = char(hex2dec('40'));  % Only available for Shimmer3 
        LSM303DLHC_ACCEL_SAMPLING_RATE_RESPONSE	= char(hex2dec('41'));     % Only available for Shimmer3
        GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND	= char(hex2dec('42')); % Only available for Shimmer3
        SET_LSM303DLHC_ACCEL_LPMODE_COMMAND	= char(hex2dec('43'));         % Only available for Shimmer3
        LSM303DLHC_ACCEL_LPMODE_RESPONSE = char(hex2dec('44'));            % Only available for Shimmer3
        GET_LSM303DLHC_ACCEL_LPMODE_COMMAND	= char(hex2dec('45'));         % Only available for Shimmer3
        SET_LSM303DLHC_ACCEL_HRMODE_COMMAND	= char(hex2dec('46'));         % Only available for Shimmer3
        LSM303DLHC_ACCEL_HRMODE_RESPONSE	= char(hex2dec('47'));         % Only available for Shimmer3
        GET_LSM303DLHC_ACCEL_HRMODE_COMMAND	= char(hex2dec('48'));         % Only available for Shimmer3
        SET_MPU9150_GYRO_RANGE_COMMAND	= char(hex2dec('49'));
        MPU9150_GYRO_RANGE_RESPONSE = char(hex2dec('4A'));
        GET_MPU9150_GYRO_RANGE_COMMAND  = char(hex2dec('4B'));
        SET_MPU9150_SAMPLING_RATE_COMMAND = char(hex2dec('4C'));
        MPU9150_SAMPLING_RATE_RESPONSE = char(hex2dec('4D'));
        GET_MPU9150_SAMPLING_RATE_COMMAND = char(hex2dec('4E'));

        SET_BMP180_PRES_RESOLUTION_COMMAND = char(hex2dec('52'));
        BMP180_PRES_RESOLUTION_RESPONSE = char(hex2dec('53'));
        GET_BMP180_PRES_RESOLUTION_COMMAND = char(hex2dec('54'));
        %SET_BMP180_PRES_CALIBRATION_COMMAND	= char(hex2dec('55'));
        %BMP180_PRES_CALIBRATION_RESPONSE = char(hex2dec('56'));
        %GET_BMP180_PRES_CALIBRATION_COMMAND = char(hex2dec('57'));
        BMP180_CALIBRATION_COEFFICIENTS_RESPONSE = char(hex2dec('58'));
        GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND = char(hex2dec('59'));
        BMP280_CALIBRATION_COEFFICIENTS_RESPONSE = char(hex2dec('9F'));
        GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND = char(hex2dec('A0'));
        RESET_TO_DEFAULT_CONFIGURATION_COMMAND = char(hex2dec('5A'));
        RESET_CALIBRATION_VALUE_COMMAND = char(hex2dec('5B'));
        MPU9150_MAG_SENS_ADJ_VALS_RESPONSE = char(hex2dec('5C'));
        GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND = char(hex2dec('5D'));
        SET_INTERNAL_EXP_POWER_ENABLE_COMMAND = char(hex2dec('5E'));
        INTERNAL_EXP_POWER_ENABLE_RESPONSE = char(hex2dec('5F'));
        GET_INTERNAL_EXP_POWER_ENABLE_COMMAND = char(hex2dec('60'));
        SET_EXG_REGS_COMMAND = char(hex2dec('61'));
        EXG_REGS_RESPONSE = char(hex2dec('62'));
        GET_EXG_REGS_COMMAND = char(hex2dec('63'));
        DAUGHTER_CARD_ID_RESPONSE = char(hex2dec('65'));
        GET_DAUGHTER_CARD_ID_COMMAND = char(hex2dec('66'));
        SET_BT_COMMS_BAUD_RATE = char(hex2dec('6A'));
        BT_COMMS_BAUD_RATE_RESPONSE = char(hex2dec('6B'));
        GET_BT_COMMS_BAUD_RATE = char(hex2dec('6C'));
        
        START_SDBT_COMMAND = char(hex2dec('70'));
        STATUS_RESPONSE = char(hex2dec('71'));
        GET_STATUS_COMMAND = char(hex2dec('72'));
        DIR_RESPONSE = char(hex2dec('88'));
        GET_DIR_COMMAND = char(hex2dec('89'));
        INSTREAM_CMD_RESPONSE = char(hex2dec('8A'));
        
        SET_RWC_COMMAND = char(hex2dec('8F'));
        GET_RWC_COMMAND = char(hex2dec('91'));
        RWC_RESPONSE = char(hex2dec('90')); 
        
        START_LOGGING_ONLY_COMMAND = char(hex2dec('92'));
        STOP_LOGGING_ONLY_COMMAND = char(hex2dec('93'));
        STOP_SDBT_COMMAND = char(hex2dec('97'));

        GET_VBATT_COMMAND = char(hex2dec('95'));
        VBATT_RESPONSE = char(hex2dec('94')); 
        
        SET_VBATT_FREQ_COMMAND = char(hex2dec('98'));
                 

        
        % Shimmer versions
        SHIMMER_2 = 1;
        SHIMMER_2R = 2;
        SHIMMER_3 = 3;
        
        INTERNAL_BOARDS_AVAILABLE = [cellstr('None'); cellstr('Gyro'); cellstr('Mag'); cellstr('9DOF'); cellstr('EXG'); cellstr('ECG'); cellstr('EMG'); cellstr('GSR'); cellstr('Strain Gauge'); cellstr('Bridge Amplifier')]; % Cell array of string constants conatining names of valid internal daughter boards
                
        EXTERNAL_BOARDS_AVAILABLE = [cellstr('None'); cellstr('ExpBoard'); cellstr('External'); cellstr('Heart Rate')]; % Cell array of string constants containing names of valid external daughter boards
        
        DEFAULT_TIMEOUT = 8;                                               % Default timeout for Wait for Acknowledgement response
        
        GET_ACCEL_CALIBRATION_PARAMETERS_COMMAND = char(19);               % Command to get accelerometer calibration parameters from the shimmer device
        GET_GYRO_CALIBRATION_PARAMETERS_COMMAND = char(22);                % Command to get gyroscope calibration parameters from the shimmer device
        GET_MAGNE_CALIBRATION_PARAMETERS_COMMAND = char(25);               % Command to get magnetometer calibration parameters from the shimmer device
               
                
    end
    

    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Constructor Method
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        
        function thisShimmer = ShimmerHandleClass(comPort)
            
            %SHIMMERHANDLECLASS - Constructor for the class
            %
            %   THIsSHIMMER = SHIMMERHANDLECLASS(COMPORT) creates a handle to an
            %   instance of the ShimmerHandleClass. The instance is set to be
            %   associated with the Shimmer that is paired with the com port
            %   defined in the input COMPORT.
            %
            %   SYNOPSIS: thisShimmer = ShimmerHandleClass(comPort)
            %
            %   INPUT: comPort - string value defining the numeric value of the
            %                    desired Com Port
            %
            %   OUTPUT: thisShimmer - handle to the instance of the
            %                         ShimmerHandleClass
            %
            %   EXAMPLE: shimmer1 = ShimmerHandleClass('7');
            %
            %            
            %
            thisShimmer.ComPort = comPort;
            
        end % function ShimmerHandleClass  
        
    end % methods
    
       
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    methods
        
        function isSet = resettodefaultconfiguration(thisShimmer) % function resettodefaultconfiguration
            %RESETTODEFAULTCONFIGURATION - Reset the Shimmer configuration to default
            %
            %   RESETTODEFAULTCONFIGURATION sets the configuration of
            %   the Shimmer back to its default values.The function
            %   will return a 1 if the operation was successful otherwise
            %   it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.resettodefaultconfiguration
            %
            %   OUTPUT: isSet -   Boolean value which indicates if the
            %                     operation was successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.resettodefaultconfiguration;
            %
            %
            if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                disp('Warning: resettodefaultconfiguration - Command only supported for Shimmer3.')
            elseif ~(strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                fprintf(strcat('Warning: resettodefaultconfiguration - Cannot set sampling range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            else
                isSet = writeresettodefaultconfiguration(thisShimmer);
                
                % Update class properties 
                if (isSet)
                    thisShimmer.inquiry;
                    
                    if (thisShimmer.FirmwareCompatibilityCode >= 3)
                        thisShimmer.readexgconfiguration(1);
                        thisShimmer.readexgconfiguration(2);
                    end                    
                end
            end
            
        end % function resettodefaultconfiguration

        function samplingRate = setsamplingrate(thisShimmer, samplingRate)
            %SETSAMPLINGRATE - Sets the sampling rate of the Shimmer
            %
            %   SAMPLINGRATE = SETSAMPLINGRATE(SAMPLINGRATE) sets the
            %   sampling rate of the Shimmer to the closest available value to
            %   the input value SAMPLINGRATE. There are a limited number of
            %   sampling rates available, these are defined in the
            %   'Sampling Rate Table.txt'. The actual sampling rate setting
            %   after the operation is returned from the function.
            %
            %   SYNOPSIS: samplingRate = thisShimmer.setsamplingrate(samplingRate)
            %
            %   INPUT: samplingRate - Numeric value defining the desired
            %                         sampling rate in Hertz.
            %
            %   OUTPUT: samplingRate - Numeric value defining the actual sampling
            %                          rate setting (in Hertz) after the operation.
            %
            %   EXAMPLE: samplingRate = shimmer1.setsamplingrate(51.2);
            %
            %   See also getsamplingrate
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                
                isWritten = writesamplingrate(thisShimmer,samplingRate);   % Write samplingRate value to the Shimmer
                
                if (isWritten)
                    isRead = readsamplingrate(thisShimmer);                % Following a succesful write, call the readsamplingrate function which updates the SamplingRate property with the current Shimmer sampling rate setting
                    if (isRead)
                        samplingRate = thisShimmer.SamplingRate;           % Following a successful write and successful read, set the return value (samplingRate) to value stored in the SamplingRate property
                        disp(['setsamplingrate - Shimmer Sampling Rate is set to ' num2str(samplingRate) 'Hz']);
                        
                        % set ExG rate as close as possible to Shimmer sampling rate; but never lower
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            if (thisShimmer.SamplingRate <= 125)
                                thisShimmer.setexgrate(0,1);               % set data rate to 125Hz for SENSOR_EXG1
                                thisShimmer.setexgrate(0,2);               % set data rate to 125Hz for SENSOR_EXG2
                                disp('setsamplingrate - ExG Rate is set to 125Hz');
                            elseif (thisShimmer.SamplingRate <= 250)
                                thisShimmer.setexgrate(1,1);               % set data rate to 250Hz for SENSOR_EXG1
                                thisShimmer.setexgrate(1,2);               % set data rate to 250Hz for SENSOR_EXG2
                                disp('setsamplingrate - ExG Rate is set to 250Hz');
                            elseif (thisShimmer.SamplingRate <= 500)
                                thisShimmer.setexgrate(2,1);               % set data rate to 500Hz for SENSOR_EXG1
                                thisShimmer.setexgrate(2,2);               % set data rate to 500Hz for SENSOR_EXG2
                                disp('setsamplingrate - ExG Rate is set to 500Hz');
                            elseif (thisShimmer.SamplingRate <= 1000)
                                thisShimmer.setexgrate(3,1);               % set data rate to 1000Hz for SENSOR_EXG1
                                thisShimmer.setexgrate(3,2);               % set data rate to 1000Hz for SENSOR_EXG2
                                disp('setsamplingrate - ExG Rate is set to 1000Hz');
                            elseif (thisShimmer.SamplingRate <= 2000)
                                thisShimmer.setexgrate(4,1);               % set data rate to 2000Hz for SENSOR_EXG1
                                thisShimmer.setexgrate(4,2);               % set data rate to 2000Hz for SENSOR_EXG2
                                disp('setsamplingrate - ExG Rate is set to 2000Hz');
                            elseif (thisShimmer.SamplingRate <= 4000)
                                thisShimmer.setexgrate(5,1);               % set data rate to 4000Hz for SENSOR_EXG1
                                thisShimmer.setexgrate(5,2);               % set data rate to 4000Hz for SENSOR_EXG2
                                disp('setsamplingrate - ExG Rate is set to 4000Hz');
                            elseif (thisShimmer.SamplingRate <= 32768)
                                thisShimmer.setexgrate(6,1);               % set data rate to 8000Hz for SENSOR_EXG1
                                thisShimmer.setexgrate(6,2);               % set data rate to 8000Hz for SENSOR_EXG2
                                disp('setsamplingrate - ExG Rate is set to 8000Hz');
                            end
                        end
                        
                        
                        % set WR Accel data rate as close as possible to Shimmer sampling rate; but never lower
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.HardwareCompatibilityCode < 2)
                            if (thisShimmer.SamplingRate <= 1)
                                thisShimmer.setaccelrate(1);                   % set data rate to 1Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 1Hz');
                            elseif (thisShimmer.SamplingRate <= 10)
                                thisShimmer.setaccelrate(2);                   % set data rate to 10Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 10Hz');
                            elseif (thisShimmer.SamplingRate <= 25)
                                thisShimmer.setaccelrate(3);                   % set data rate to 25Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 25Hz');
                            elseif (thisShimmer.SamplingRate <= 50)
                                thisShimmer.setaccelrate(4);                   % set data rate to 50Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 50Hz');
                            elseif (thisShimmer.SamplingRate <= 100)
                                thisShimmer.setaccelrate(5);                   % set data rate to 100Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 100Hz');
                            elseif (thisShimmer.SamplingRate <= 200)
                                thisShimmer.setaccelrate(6);                   % set data rate to 200Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 200Hz');
                            elseif (thisShimmer.SamplingRate <= 400)
                                thisShimmer.setaccelrate(7);                   % set data rate to 400Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 400Hz');
                            elseif (thisShimmer.SamplingRate <= 32768)
                                thisShimmer.setaccelrate(9);                   % set data rate to 1344Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 1344Hz');
                            end
                        elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.HardwareCompatibilityCode >= 2)
                            if (thisShimmer.SamplingRate <= 12.5)
                                thisShimmer.setaccelrate(1);                   % set data rate to 12.5Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 12.5Hz');
                            elseif (thisShimmer.SamplingRate <= 25)
                                thisShimmer.setaccelrate(2);                   % set data rate to 25Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 25Hz');
                            elseif (thisShimmer.SamplingRate <= 50)
                                thisShimmer.setaccelrate(3);                   % set data rate to 50Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 50Hz');
                            elseif (thisShimmer.SamplingRate <= 100)
                                thisShimmer.setaccelrate(4);                   % set data rate to 100Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 100Hz');
                            elseif (thisShimmer.SamplingRate <= 200)
                                thisShimmer.setaccelrate(5);                   % set data rate to 200Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 200Hz');
                            elseif (thisShimmer.SamplingRate <= 400)
                                thisShimmer.setaccelrate(6);                   % set data rate to 400Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 400Hz');
                            elseif (thisShimmer.SamplingRate <= 800)
                                thisShimmer.setaccelrate(7);                   % set data rate to 800Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 800Hz');
                            elseif (thisShimmer.SamplingRate <= 1600)
                                thisShimmer.setaccelrate(8);                   % set data rate to 1600Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 1600Hz');
                            elseif (thisShimmer.SamplingRate <= 3200)
                                thisShimmer.setaccelrate(9);                   % set data rate to 3200Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 3200Hz');
                            elseif (thisShimmer.SamplingRate <= 32768)
                                thisShimmer.setaccelrate(10);                  % set data rate to 6400Hz for WR accel
                                disp('setsamplingrate - WR Accel Rate is set to 6400Hz');
                            end
                        end
                        
                        % set Gyro data rate as close as possible to Shimmer sampling rate; but never lower
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.SamplingRate <= 32768)
                            gyroRate = min(255, floor(8000/thisShimmer.SamplingRate - 1));                      % gyro rate to send to Shimmer -> programmable from 4..8000Hz)
                            if (gyroRate>=0)
                                thisShimmer.setgyrorate(gyroRate);
                                actualRate = 8000/(1+gyroRate);                                                 % actual gyro rate
                            else
                                thisShimmer.setgyrorate(0);
                                actualRate = 8000;
                            end
                            fprintf(['setsamplingrate - Gyro Rate is set to' ' ' num2str(actualRate) 'Hz.\n']);
                        end
                                                
                        % set Mag data rate as close as possible to Shimmer sampling rate; but never lower
                        if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            if (thisShimmer.SamplingRate <= 0.5)
                                thisShimmer.setmagrate(0);                     % set data rate to 0.50Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 0.50Hz');
                            elseif (thisShimmer.SamplingRate <= 1)
                                thisShimmer.setmagrate(1);                     % set data rate to 1.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 1.00Hz');
                            elseif (thisShimmer.SamplingRate <= 2)
                                thisShimmer.setmagrate(2);                     % set data rate to 2.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 2.00Hz');
                            elseif (thisShimmer.SamplingRate <= 5)
                                thisShimmer.setmagrate(3);                     % set data rate to 5.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 5.00Hz');
                            elseif (thisShimmer.SamplingRate <= 10)
                                thisShimmer.setmagrate(4);                     % set data rate to 10.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 10.00Hz');
                            elseif (thisShimmer.SamplingRate <= 20)
                                thisShimmer.setmagrate(5);                     % set data rate to 20.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 20.00Hz');
                            else
                                thisShimmer.setmagrate(6);                     % set data rate to 50.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 50.00Hz');
                            end
                        elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.HardwareCompatibilityCode < 2)
                            if (thisShimmer.SamplingRate <= 0.75)
                                thisShimmer.setmagrate(0);                     % set data rate to 0.75Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 0.75Hz');
                            elseif (thisShimmer.SamplingRate <= 1.50)
                                thisShimmer.setmagrate(1);                     % set data rate to 1.50Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 1.50Hz');
                            elseif (thisShimmer.SamplingRate <= 3)
                                thisShimmer.setmagrate(2);                     % set data rate to 3.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 3.00Hz');
                            elseif (thisShimmer.SamplingRate <= 7.5)
                                thisShimmer.setmagrate(3);                     % set data rate to 7.50Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 7.50Hz');
                            elseif (thisShimmer.SamplingRate <= 15)
                                thisShimmer.setmagrate(4);                     % set data rate to 15.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 15.00Hz');
                            elseif (thisShimmer.SamplingRate <= 30)
                                thisShimmer.setmagrate(5);                     % set data rate to 30.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 30.00Hz');
                            elseif (thisShimmer.SamplingRate <= 75)
                                thisShimmer.setmagrate(6);                     % set data rate to 75.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 75.00Hz');
                            elseif (thisShimmer.SamplingRate <= 32768)
                                thisShimmer.setmagrate(7);                     % set data rate to 220.00Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 220.00Hz');
                            end
                        elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.HardwareCompatibilityCode >= 2)
                            if (thisShimmer.SamplingRate <= 10.0)
                                thisShimmer.setmagrate(0);                     % set data rate to 10.0Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 10.0Hz');
                            elseif (thisShimmer.SamplingRate <= 20.0)
                                thisShimmer.setmagrate(1);                     % set data rate to 20.0Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 20.0Hz');
                            elseif (thisShimmer.SamplingRate <= 50.0)
                                thisShimmer.setmagrate(2);                     % set data rate to 50.0Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 50.0Hz');
                            elseif (thisShimmer.SamplingRate <= 32768)
                                thisShimmer.setmagrate(3);                     % set data rate to 100.0Hz for Mag
                                disp('setsamplingrate - Mag Rate is set to 100.0Hz');
                            end
                        end  
                    else
                        samplingRate = 'Nan';                              % Following a successful write but failed read, set the return value (samplingRate) to 'Nan' signifying unknown
                    end
                else
                    samplingRate = thisShimmer.SamplingRate;               % Following a failed write, set the return value (samplingRate) to value stored in the SamplingRate property
                end
            else
                fprintf(strcat('Warning: setsamplingrate - Cannot set sampling range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                samplingRate = 'Nan';
            end
            
        end % function setsamplingrate
        
        function isSet = setbaudrate(thisShimmer, baudRate)
            %SETBAUDRATE - Sets the baud rate of the Shimmer
            %
            %   BAUDRATEOUT = SETBAUDRATE(BAUDRATE) sets the
            %   baud rate of the Shimmer.
            %
            %   SYNOPSIS: isSet = thisShimmer.setbaudrate(baudRate)
            %
            %   INPUT: baudRate - Numeric value defining the desired
            %                     baud rate in kB/s. Valid values are:
            %                     0 (115200 - default), 1 (1200), 2 (2400), 
            %                     3 (4800), 4 (9600), 5 (19200), 6 (38400),
            %                     7 (57600), 8 (230400), 9 (460800) and 10 (921600). 
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setbaudrate(4);
            %
            %   See also getbaudrate
            if ~(strcmp(thisShimmer.State,'Connected'))                % Shimmer must be in a Connected state
                fprintf(strcat('Warning: setbaudrate - Cannot set Baud Rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: setbaudrate - This function is only supported for Shimmer3.');
                isSet = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 5) 
                disp('Warning: setbaudrate - This function is not supported for this firmware version, please update firmware.');
                isSet = false;
            elseif ~(baudRate == 0 || baudRate == 1 || baudRate == 2 || baudRate == 3 ...
                    || baudRate == 4 || baudRate == 5 || baudRate == 6 || baudRate == 7 ...
                    || baudRate == 8 || baudRate == 9 || baudRate == 10)
                disp('Warning: setbaudrate - Invalid Baud Rate selected.');
                isSet = false;
            else
                isWritten = writebaudrate(thisShimmer,baudRate);           % Write baudRate value to the Shimmer
                
                if (isWritten)
                    isRead = readbaudrate(thisShimmer);                    % Following a succesful write, call the readbaudrate function which updates the BaudRate property with the current Shimmer baud rate setting
                    if (isRead)
                        isSet = (baudRate == thisShimmer.BaudRate);        % Successful write and successful read
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end
            end
        end % function setbaudrate

        
        function isSet = setenabledsensors(thisShimmer, varargin)
            %SETENABLEDSENSORS - Enables/disables the sensors on the Shimmer
            %
            %   SETENABLEDSENSORS(VARARGIN) enables/disables the sensors on
            %   the shimmer based on values defined in the input VARARGIN.
            %   The function will return a 1 if the operation was
            %   successful otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setenabledsensors(varargin)
            %
            %   INPUT: varargin - Multi-variable value defining the name of
            %                     each sensor in string format followed by
            %                     a boolean value (1=Enable, 0=Disable).
            %                     Valid values for SENSORNAME are listed
            %                     in SetEnabeldSensorsMacrosClass          
            %                     If more than one of the following:
            %                     'ECG' (or 'ECG 24BIT'),
            %                     'EMG' (or 'EMG 24BIT'), 'ECG 16BIT', 
            %                     'EMG 16BIT', 'EXG1' (or 'EXG1 24BIT'),
            %                     'EXG2' (or 'EXG2 24BIT'), 'EXG1 16BIT',
            %                     'EXG2 16BIT' is set, only the last of
            %                     these sensors in varargin is actually set.
            %                     Exceptions are 'EXG1' (or 'EXG1 24BIT'),
            %                     'EXG2' (or 'EXG2 24BIT') resp. 'EXG1
            %                     16BIT', 'EXG2 16BIT'. Both 'EXG1', 'EXG2'
            %                     (or 'EXG1 24BIT', 'EXG2 24BIT') or 'EXG1
            %                     16BIT', 'EXG2 16BIT' can be set. When
            %                     using Shimmer3 use 'Pressure' to enable
            %                     the pressure sensor. By default 'Accel'
            %                     enables the low noise accelerometer on
            %                     Shimmer3. 
            % 
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setenabledsensors('Accel',1,'Gyro',0,'Mag',1));
            %
            %   See also  getenabledsignalnames getsignalname
            %   disableallsensors getsignalindex SetEnabledSensorsMacrosClass
                        
            if (strcmp(thisShimmer.State,'Connected'))                                 % Shimmer must be in a Connected state                
                enabledSensors = determineenabledsensorsbytes(thisShimmer, varargin);
                isWritten = writeenabledsensors(thisShimmer, uint32(enabledSensors));                
                if (isWritten)
                    isRead = readenabledsensors(thisShimmer);                           % Following a succesful write, call the readenabledsensors function which updates the enabledSensors property with the current Shimmer enabled sensors setting                    
                    if (isRead)
                        isSet = (enabledSensors == thisShimmer.EnabledSensors);          % isSet will be equal to 1 if EnabledSensors property is equal to the requested setting
                    else
                        isSet = false;
                    end                    
                else                    
                    isSet = false;
                end
            else
                fprintf(strcat('Warning: setenabledsensors - Cannot set enabled sensors for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end           
        end % function setenabledsensors
        
        function isDisabled = disableallsensors(thisShimmer)
           %DISABLEALLSENSORS - Disables all sensors on the Shimmer
            %
            %   DISABLEALLSENSORS disables all sensors on the Shimmer.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.disableallsensors
            %              
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.disableallsensors;
            %
            %   See also setenabledsensors getenabledsignalnames
            if (strcmp(thisShimmer.State,'Connected'))                                 % Shimmer must be in a Connected state
                enabledSensors = 0;
                isWritten = writeenabledsensors(thisShimmer, uint32(enabledSensors));
                if (isWritten)
                    isRead = readenabledsensors(thisShimmer);              % Following a succesful write, call the readenabledsensors function which updates the enabledSensors property with the current Shimmer enabled sensors setting
                    if (isRead)
                        isDisabled = (enabledSensors == thisShimmer.EnabledSensors);  % isSet will be equal to 1 if EnabledSensors property is equal to the requested setting
                        thisShimmer.EMGflag = false;
                        thisShimmer.ECGflag = false;
                    else
                        isDisabled = false;
                    end
                else
                    isDisabled = false;
                end
            else
                fprintf(strcat('Warning: disableallsensors - Cannot disable all sensors for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isDisabled = false;
            end
        end % function disableallsensors
        
        

        function isSet = setconfigbyte0(thisShimmer, configByte0)
            %SETCONFIGBYTE0 - Set the config byte0 on the Shimmer
            %
            %   SETCONFIGBYTE0(CONFIGBYTE0) sets the config byte0 value on the
            %   Shimmer to the value of the input CONFIGBYTE0.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setconfigbyte0(configByte0)
            %
            %   INPUT: configByte0 - Unsigned 8 bit value defining the desired
            %                        config byte 0 value.
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setconfigbyte0(192);
            %
            %   See also getconfigbyte0 setfivevoltreg setpmux
            
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                
                isWritten = writeconfigbyte0(thisShimmer, configByte0);     % Write configByte0 value to the Shimmer                
                if (isWritten)
                    isRead = readconfigbyte0(thisShimmer);                  % Following a succesful write, call the readconfigbyte0 function which updates the ConfigByte0 property with the current Shimmer config byte0 setting
                    
                    if (isRead)
                        isSet = (configByte0 == thisShimmer.ConfigByte0);   % isSet will be equal to 1 the current config byte 0 setting is equal to the requested setting
                        thisShimmer.inquiry;                                % call inquiry so that class properties are updated
                    else
                        isSet = false;
                    end                    
                else
                    isSet = false;
                end
            elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                fprintf(strcat('Warning: setconfigbyte0 - This command is not valid for Shimmer3; please use setconfigbytes.\n'));
                isSet = false;
            else
                fprintf(strcat('Warning: setconfigbyte0 - Cannot set config byte0 for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
            
        end % function setconfigbyte0
        
        function isSet = setconfigbytes(thisShimmer, configByte0, configByte1, configByte2, configByte3)
            %SETCONFIGBYTES - Set the config bytes on the Shimmer
            %
            %   SETCONFIGBYTES(CONFIGBYTE0, CONFIGBYTE1, CONFIGBYTE2, CONFIGBYTE3) sets
            %   the config bytes values on the Shimmer to the value of the
            %   inputs CONFIGBYTE0, CONFIGBYTE1, CONFIGBYTE2 and CONFIGBYTE3. 
            %   The function will return a 1 if the operation was
            %   successful; otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setconfigbyte0(configByte0, configByte1, configByte2, configByte3)
            %
            %   INPUT: configByte0 - Unsigned 8 bit value defining the desired
            %                        config byte 0 value.
            %   INPUT: configByte1 - Unsigned 8 bit value defining the desired
            %                        config byte 1 value.
            %   INPUT: configByte2 - Unsigned 8 bit value defining the desired
            %                        config byte 2 value.
            %   INPUT: configByte3 - Unsigned 8 bit value defining the desired
            %                        config byte 3 value.
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setconfigbytes(192,128,255,1);
            %
            %   See also getconfigbytes setconfigbyte0 getconfigbyte0
            %   setfivevoltreg setpmux 
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                
                if(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    isWritten = writeconfigbyte0(thisShimmer, configByte0);     % Write configByte0 value to the Shimmer
                    if (isWritten)
                        isRead = readconfigbyte0(thisShimmer);                  % Following a succesful write, call the readconfigbyte0 function which updates the ConfigByte0 property with the current Shimmer config byte0 setting
                        if (isRead)
                            isSet = (configByte0 == thisShimmer.ConfigByte0);   % isSet will be equal to 1 the current config byte 0 setting is equal to the requested setting
                            thisShimmer.inquiry;                                % call inquiry so that class properties are updated
                        else
                            isSet = false;
                        end
                    else 
                        isSet = false;
                    end
                elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    isWritten = writeconfigbytes(thisShimmer, configByte0, configByte1, configByte2, configByte3);     % Write ConfigByte0, ConfigByte1, ConfigByte2, ConfigByte3 values to the Shimmer
                    if (isWritten)
                        isRead = readconfigbytes(thisShimmer);                  % Following a succesful write, call the readconfigbytes function which updates the ConfigByte0, ConfigByte1, ConfigByte2, ConfigByte3 properties with the current Shimmer config bytes setting
                        if (isRead)
                            isSet = (configByte0 == thisShimmer.ConfigByte0 && ...
                                configByte1 == thisShimmer.ConfigByte1 && ...
                                configByte2 == thisShimmer.ConfigByte2 && ...
                                configByte3 == thisShimmer.ConfigByte3);        % isSet will be equal to 1 if the current config bytes setting is equal to the requested setting
                                thisShimmer.inquiry;                            % call inquiry so that class properties are updated
                        else
                            isSet = false;
                            disp('Warning: setconfigbytes - Config bytes written but not successfully verified.');
                        end
                    else 
                        isSet = false;
                        disp('Warning: setconfigbytes - Config bytes write unsuccessful.');
                    end 
                else
                    isSet = false;
                end
            else
                fprintf(strcat('Warning: setconfigbytes - Cannot set config bytes for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setconfigbytes
        
        function isSet = setfivevoltreg(thisShimmer, setBit)
            %SETFIVEVOLTREG - Enable/disable the 5 volt Regulator on the
            %                    Shimmer ExpBoard board
            %
            %   SETFIVEVOLTREG(SETBIT) sets the 5 volt regulator bit value
            %   on the Shimmer to the value of the input SETBIT. The 5 volt
            %   regulator bit is the MSB of config byte0.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setfivevoltreg(setBit)
            %
            %   INPUT: setBit - Bit value defining the desired setting of the
            %                   5 volt regulator (1=ENABLED, 0=DISABLED).
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setfivevoltreg(1);
            %
            %   See also getfivevoltreg setconfigbyte0
            
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 )                     % Shimmer must be in a Connected state
                
                configByte0 = bitset(uint8(thisShimmer.ConfigByte0), 8, setBit);
                
                isWritten = writeconfigbyte0(thisShimmer,configByte0);      % Write configByte0 value to the Shimmer
                
                if (isWritten)
                    
                    isRead = readconfigbyte0(thisShimmer);                  % Following a succesful write, call the readconfigbyte0 function which updates the ConfigByte0 property with the current Shimmer config byte0 setting
                    
                    if (isRead)
                        fiveVoltReg = getfivevoltreg(thisShimmer);
                        isSet = (fiveVoltReg == setBit);                    % isSet will be equal to 1 the current 5 volt regulator setting is equal to the requested setting
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end

            elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                disp('Warning: setfivevoltreg - 5V regulator is not present on Shimmer3.');
            else
                
                fprintf(strcat('Warning: setfivevoltreg - Cannot set 5 volt regulator for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
                
            end
        end % function setfivevoltreg
        
        function isSet = setpmux(thisShimmer, setBit)
            %SETPMUX - Set the PMux bit on the Shimmer
            %
            %   SETPMUX(SETBIT) sets the PMux bit value on the Shimmer to the
            %   value of the input SETBIT. The PMux bit is the 2nd MSB of
            %   config byte0.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setpmux(setBit)
            %
            %   INPUT: setBit - Bit value defining the desired setting of the
            %                   PMux (1=ON, 0=OFF).
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setpmux(1);
            %
            %   See also getpmux setconfigbyte0
            
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 )                     % Shimmer must be in a Connected state
                
                configByte0 = bitset(uint8(thisShimmer.ConfigByte0), 7, setBit);
                
                isWritten = writeconfigbyte0(thisShimmer,configByte0);      % Write configByte0 value to the Shimmer
                
                if (isWritten)
                    isRead = readconfigbyte0(thisShimmer);                           % Following a succesful write, call the readconfigbyte0 function which updates the ConfigByte0 property with the current Shimmer config byte0 setting
                    
                    if (isRead)
                        pMux = getpmux(thisShimmer);
                        isSet = (pMux == setBit);
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end
                
            elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                disp('Warning: setpmux - PMUX is not present on Shimmer3.');
            else
                
                fprintf(strcat('Warning: setpmux - Cannot set PMux for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
                
            end
        end % function setpmux
        
        function isSet = setledblink(thisShimmer, ledBlink)
            %SETLEDBLINK - Set led blink on the Shimmer
            %
            %   SETLEDBLINK(LEDBLINK) sets which of the LEDs on the Shimmer
            %   is blinking, according to the value of the input LEDBLINK.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setledblink(ledBlink)
            %
            %   INPUT: ledBlink - Numeric value defining the desired LED.
            %                     Valid range settings are 0 (Green), 1
            %                     (Yellow) and 2 (Red).
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setledblink(1);
            %
            %   See also setledblinkwhilestreaming writeledblinkwhilestreaming readledblink writeledblink
            
           
            if (thisShimmer.FirmwareIdentifier == 3)
                disp('Warning: setledblink - Not supported for LogAndStream.');
                
            elseif (strcmp(thisShimmer.State,'Connected') )                     % Shimmer must be in a Connected state, and firmware must not be LogAndStream.
                
                isWritten = writeledblink(thisShimmer,ledBlink);           % Write ledBlink to the Shimmer
                
                if (isWritten)
                    isRead = readledblink(thisShimmer);                     % Following a succesful write, call the readledblink function which updates the BlinkLED property with the current setting
                    
                    if (isRead)
                        isSet = (ledBlink == thisShimmer.BlinkLED);          % isSet will be equal to 1 if the current setting is equal to the requested setting
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end
            else
                fprintf(strcat('Warning: setledblink - Cannot set led blink for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setledblink
        
        function isSet = setaccelhrmode(thisShimmer, enable)
            %SETACCELHRMODE - Set the accelerometer high resolution mode on the Shimmer
            %
            %   SETACCELHRMODE(ENABLE) sets the accelerometer high resolution mode on the Shimmer
            %
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setaccelhrmode(enable)
            %
            %   INPUT: enable - Numeric value defining 1 = enable, 0 = disabled
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setaccelhrmode(enable);
            %
            %   See also getaccelhrmode
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                if (~thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    fprintf('Warning: setaccelhrmode - Command only supported on Shimmer3.\n');
                    iSet = false;
                else
                    isWritten = writeaccelhrmode(thisShimmer, enable);     % Write setting to the Shimmer
                    
                    if (isWritten)
                        isRead = readaccelhrmode(thisShimmer);             % Following a succesful write, call the readaccelhrmode function which updates the AccelWideRangeHRMode  property with the current Shimmer setting
                        
                        if (isRead)
                            thisShimmer.readconfigbytes;                   % update config bytes class properties
                            isSet = (enable == thisShimmer.AccelWideRangeHRMode);  % isSet will be equal to 1 the current setting is equal to the requested setting
                        else
                            isSet = false;
                        end
                    else
                        isSet = false;
                    end
                end
            else
                fprintf(strcat('Warning: setaccelhrmode - Cannot set accel high resolution mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setaccelhrmode
        
        function isSet = setaccellpmode(thisShimmer, enable)
            %SETACCELLPMODE - Set the accelerometer low power mode on the Shimmer
            %
            %   SETACCELLPMODE(ENABLE) sets the accelerometer low power mode on the Shimmer
            %
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setaccellpmode(enable)
            %
            %   INPUT: enable - Numeric value defining 1 = enable, 0 = disabled
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setaccellpmode(enable);
            %
            %   See also getaccellpmode
            
            if (strcmp(thisShimmer.State,'Connected'))                                 % Shimmer must be in a Connected state
                if (~thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    fprintf('Warning: setaccellpmode - Command only supported on Shimmer3.\n');
                    iSet = false;
                else
                    isWritten = writeaccellpmode(thisShimmer, enable);                 % Write setting to the Shimmer
                    
                    if (isWritten)
                        isRead = readaccellpmode(thisShimmer);                         % Following a succesful write, call the readaccellpmode function which updates the AccelWideRangeLPMode  property with the current Shimmer setting
                        
                        if (isRead)
                            thisShimmer.readconfigbytes;                               % update config bytes class properties
                            isSet = (enable == thisShimmer.AccelWideRangeLPMode);      % isSet will be equal to 1 the current setting is equal to the requested setting
                            if (~enable)
                                thisShimmer.setsamplingrate(thisShimmer.SamplingRate); % make sure data rates are set back to value that is closest to and not lower than Shimmer sampling rate
                            end
                        else
                            isSet = false;
                        end
                    else
                        isSet = false;
                    end
                end
            else
                fprintf(strcat('Warning: setaccellpmode - Cannot set accel low power mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setaccellpmode
           
        function isSet = setaccelrange(thisShimmer, accelRange)
            %SETACCELRANGE - Set the accelerometer range of the Shimmer
            %
            %SETACCELRANGE(ACCELRANGE) sets the accelerometer range of the
            %   Shimmer to the value of the input ACCELRANGE.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setaccelrange(accelRange)
            %
            %   INPUT: accelRange - Numeric value defining the desired
            %                       accelerometer range.
            %                       Valid range setting values for the Shimmer
            %                       2 are 0 (+/- 1.5g), 1 (+/- 2g), 2 (+/- 4g)
            %                       and 3 (+/- 6g).
            %                       Valid range setting values for the Shimmer
            %                       2r are 0 (+/- 1.5g) and 3 (+/- 6g).
            %                       Valid range setting values for the
            %                       Shimmer3 with LSM303DLHC are 0 (+/- 2g),
            %                       1 (+/- 4g), 2 (+/- 8g) and 3 (+/- 16g).
            %                       Valid range setting values for the
            %                       Shimmer3 with LSM303AHTR are 0 (+/- 2g),
            %                       1 (+/- 16g), 2 (+/- 4g) and 3 (+/- 8g).
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setaccelrange(0);
            %
            %   See also getaccelrange
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                
                isWritten = writeaccelrange(thisShimmer,accelRange);       % Write accelerometer range to the Shimmer
                
                if (isWritten)
                    isRead = readaccelrange(thisShimmer);                  % Following a succesful write, call the readaccelrange function which updates the accelRange property with the current Shimmer accel range setting
                    
                    if (isRead)
                        isSet = (accelRange == thisShimmer.AccelRange);    % isSet will be equal to 1 the current Accel range setting is equal to the requested setting
                        disp('Please ensure you are using the correct calibration parameters. Note that the Shimmer only stores one set (one range per sensor) of calibration parameters.');
                        if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            if (thisShimmer.DefaultAccelCalibrationParameters == true)
                                thisShimmer.AccelCalParametersOV = thisShimmer.AccelCalParametersOVShimmer2;
                                thisShimmer.AccelCalParametersAM = thisShimmer.AccelCalParametersAMShimmer2;
                                %   check accel range, valid range setting values for the Shimmer
                                %   2 are 0 (+/- 1.5g), 1 (+/- 2g), 2 (+/- 4g)and 3 (+/- 6g)
                                if thisShimmer.getaccelrange==0
                                    thisShimmer.AccelCalParametersSM = thisShimmer.AccelCalParametersSM1p5gShimmer2;
                                end
                                
                                if thisShimmer.getaccelrange==1
                                    thisShimmer.AccelCalParametersSM = thisShimmer.AccelCalParametersSM2gShimmer2;
                                end
                                
                                if thisShimmer.getaccelrange==2
                                    thisShimmer.AccelCalParametersSM = thisShimmer.AccelCalParametersSM4gShimmer2;
                                end
                                
                                if thisShimmer.getaccelrange==3
                                    thisShimmer.AccelCalParametersSM = thisShimmer.AccelCalParametersSM6gShimmer2;
                                end
                            end
                        else
                            thisShimmer.readconfigbytes;                   % update config bytes class properties
                            thisShimmer.DAccelCalParametersOV = thisShimmer.AccelWideRangeCalParametersOVShimmer3;
                            if thisShimmer.HardwareCompatibilityCode < 2
                                thisShimmer.DAccelCalParametersAM = thisShimmer.AccelWideRangeCalParametersAMShimmer3;
                                if (thisShimmer.getaccelrange==0 && thisShimmer.DefaultAccelCalibrationParameters == true)
                                    thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM2gShimmer3;
                                end
                                if (thisShimmer.getaccelrange==1 && thisShimmer.DefaultDAccelCalibrationParameters == true)
                                    thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM4gShimmer3;
                                end
                                if (thisShimmer.getaccelrange==2 && thisShimmer.DefaultDAccelCalibrationParameters == true)
                                    thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM8gShimmer3;
                                end
                                if (thisShimmer.getaccelrange==3 && thisShimmer.DefaultDAccelCalibrationParameters == true)
                                    thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM16gShimmer3;
                                end
                            elseif thisShimmer.HardwareCompatibilityCode >= 2
                                thisShimmer.DAccelCalParametersAM = thisShimmer.AccelWideRangeCalParametersAMShimmer3_2;
                                if (thisShimmer.getaccelrange==0 && thisShimmer.DefaultAccelCalibrationParameters == true)
                                    thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM2gShimmer3_2;
                                end
                                if (thisShimmer.getaccelrange==1 && thisShimmer.DefaultDAccelCalibrationParameters == true)
                                    thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM16gShimmer3_2;
                                end
                                if (thisShimmer.getaccelrange==2 && thisShimmer.DefaultDAccelCalibrationParameters == true)
                                    thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM4gShimmer3_2;
                                end
                                if (thisShimmer.getaccelrange==3 && thisShimmer.DefaultDAccelCalibrationParameters == true)
                                    thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM8gShimmer3_2;
                                end
                            end
                        end
                    else
                        isSet = false;
                    end
                    
                else
                    
                    isSet = false;
                    
                end
                
            else
                fprintf(strcat('Warning: setaccelrange - Cannot set accel range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setaccelrange
      
        function isSet = setgsrrange(thisShimmer, gsrRange)
            %SETGSRRANGE - Set the Gsr Range on the Shimmer
            %
            %   SETGSRRANGE(GSRRANGE) sets the gsr range  on the Shimmer to the
            %   value of the input GSRRANGE.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setgsrrange(gsrRange)
            %
            %   INPUT: gsrRange - Numeric value defining the desired gsr range.
            %                     Valid range settings are 0 (10kohm to 56kohm),
            %                     1 (56kohm to 220kohm), 2 (220kohm to 680kohm),
            %                     3 (680kohm to 4.7Mohm) and 4 (Auto Range).
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setgsrrange(1);
            %
            % 
            
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                
                isWritten = writegsrrange(thisShimmer,gsrRange);           % Write gsr range to the Shimmer
                
                if (isWritten)
                    isRead = readgsrrange(thisShimmer);                    % Following a succesful write, call the readgsrrange function which updates the gsrRange property with the current Shimmer gsr range setting
                    
                    if (isRead)
                        thisShimmer.readconfigbytes;                       % update config bytes class properties
                        isSet = (gsrRange == thisShimmer.GsrRange);        % isSet will be equal to 1 the current gsr range setting is equal to the requested setting
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end
                
            else
                fprintf(strcat('Warning: setgsrrange - Cannot set gsr range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setgsrrange
        
        function isSet = setmagrange(thisShimmer, magRange)
            %SETMAGRANGE - Set the Mag Range on the Shimmer
            %
            %   SETMAGRANGE(MAGRANGE) sets the mag range  on the Shimmer to the
            %   value of the input MAGRANGE. Requires the use of BTStream,
            %   will not work with BoilerPlate firmware.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setmagrange(magRange)
            %
            %   INPUT: magRange - Numeric value defining the desired mag range.
            %                     Valid range settings are 0 (0.7 gauss), 1
            %                     (1.0 gauss), 2 (1.5 gauss), 3 (2.0 gauss), 4 (3.2
            %                     gauss), 5 (3.8 gauss), 6 (4.5 gauss). The mag
            %                     ranges for Shimmer3 with LSM303DLHC are
            %                     1 (1.3 gauss), 2 (1.9 gauss), 3 (2.5 gauss), 4 (4.0 gauss),
            %                     5 (4.7 gauss), 6 (5.6 gauss), 7 (8.1 gauss). For
            %                     Shimmer 3 with LSM303AHTR the mag range
            %                     is fixed at 49.152 gauss.
            %                 
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setmagrange(1);
            %
            %   See also getmagrange
            
            if (thisShimmer.FirmwareCompatibilityCode == 0)
                fprintf(strcat('Warning: setmagrange - Command not supported for this firmware version, please update firmware.\n'));
            elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.HardwareCompatibilityCode == 2)
                fprintf(strcat('Warning: setmagrange - Shimmer3 with LSM303AHTR has a fixed mag range of 49.152 gauss.\n'));
            else
                
                if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                    
                    isWritten = writemagrange(thisShimmer,magRange);           % Write mag range to the Shimmer
                    
                    if (isWritten)
                        isRead = readmagrange(thisShimmer);                    % Following a succesful write, call the readmagrange function which updates the magRange property with the current Shimmer mag range setting
                        
                        if (isRead)
                            thisShimmer.readconfigbytes;                       % update config bytes class properties
                            isSet = (magRange == thisShimmer.MagRange);        % isSet will be equal to 1 the current mag range setting is equal to the requested setting
                        else
                            isSet = false;
                        end
                    else
                        isSet = false;
                    end
                    
                else
                    fprintf(strcat('Warning: setmagrange - Cannot set mag range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                    isSet = false;
                end
            end
        end % function setmagrange
         
        function isSet = setinternalexppower(thisShimmer, enable)
            %SETINTERNALEXPPOWER - Set the internal exp power setting on the Shimmer3
            %
            %   SETINTERNALEXPPOWER(ENABLE) sets the internal exp power setting on the Shimmer3 to the
            %   value of the input Enable.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setinternalexppower(enable)
            %
            %   INPUT: enable - Numeric value defining 1 = enable, 0 = disabled
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setinternalexppower(1);
            %
            %   See also getinternalexppower
                       
            if (strcmp(thisShimmer.State,'Connected'))                          % Shimmer must be in a Connected state
                if (thisShimmer.FirmwareCompatibilityCode >= 2)
                    isWritten = writeinternalexppower(thisShimmer,enable);      % Write internal exp power to the Shimmer
                    
                    if (isWritten)
                        isRead = readinternalexppower(thisShimmer);             % Following a succesful write, call the readinternalexppower function which updates the InternalExpPower property with the current Shimmer setting
                        if (isRead)
                            thisShimmer.readconfigbytes;                        % update config bytes class properties
                            isSet = (enable == thisShimmer.InternalExpPower);   % isSet will be equal to 1 the current internal exp power setting is equal to the requested setting
                        else
                            isSet = false;
                        end
                    else
                        isSet = false;
                    end
                else
                    fprintf(strcat('Warning: setinternalexppower - Command only supported on Shimmer3 with BTStream version 0.2 or LogAndStream version 0.1 and onwards.\n'));
                end
            else
                fprintf(strcat('Warning: setinternalexppower - Cannot set external  for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setinternalexppower
                
        function isSet = setgyrorange(thisShimmer, gyroRange)
            %SETGYRORANGE - Set the Gyro Range on the Shimmer3
            %
            %   SETGYRORANGE(GYRORANGE) sets the gyro range  on the Shimmer3 to the
            %   value of the input GYRORANGE.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setgyrorange(gyroRange)
            %
            %   INPUT: magRange - Numeric value defining the desired mag range.
            %                     Valid range settings are 0 (250 dps), 1
            %                     (500 dps), 2 (1000 dps), 3 (2000 dps)
            
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setgyrorange(1);
            %
            %   See also getgyrorange
            
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    isWritten = writegyrorange(thisShimmer,gyroRange);     % Write gyro range to the Shimmer
                    
                    if (isWritten)
                        isRead = readgyrorange(thisShimmer);               % Following a succesful write, call the readmagrange function which updates the magRange property with the current Shimmer mag range setting
                        
                        if (isRead)
                            thisShimmer.readconfigbytes;                   % update config bytes class properties
                            isSet = (gyroRange == thisShimmer.GyroRange);  % isSet will be equal to 1 the current mag range setting is equal to the requested setting
                            thisShimmer.GyroCalParametersOV=thisShimmer.GyroCalParametersOVShimmer3;
                            thisShimmer.GyroCalParametersAM=thisShimmer.GyroCalParametersAMShimmer3;
                            if (thisShimmer.getgyrorange == 3)
                                thisShimmer.GyroCalParametersSM=thisShimmer.GyroCalParametersSM2000dpsShimmer3;
                            elseif (thisShimmer.getgyrorange == 2)
                                thisShimmer.GyroCalParametersSM=thisShimmer.GyroCalParametersSM1000dpsShimmer3;
                            elseif (thisShimmer.getgyrorange == 1)
                                thisShimmer.GyroCalParametersSM=thisShimmer.GyroCalParametersSM500dpsShimmer3;
                            elseif (thisShimmer.getgyrorange == 0)
                                thisShimmer.GyroCalParametersSM=thisShimmer.GyroCalParametersSM250dpsShimmer3;
                            end
                        else
                            isSet = false;
                        end
                    else
                        isSet = false;
                    end
                else
                    fprintf(strcat('Warning: setgyrorange - Command only supported on Shimmer3.'));
                end
            else
                fprintf(strcat('Warning: setgyrorange - Cannot set gyro range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setgyrorange
        
        function isSet = setpressureresolution(thisShimmer, pressureRes)
            %SETPRESSURERESOLUTION - Set the pressure resolution on the Shimmer3
            %
            %   SETPRESSURERESOLUTION(PRESSURERES) sets the resolution of the pressure sensor on the Shimmer3 to the
            %   value of the input PRESSURERES.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0. Please note the average
            %   conversion time when setting the resolution. For further
            %   detail refer to the BMP180/BMP280 datasheet.
            %
            %   SYNOPSIS: isSet = thisShimmer.setpressureresolution(pressureRes)
            %
            %   INPUT: pressureRes - Numeric value defining the desired resolution of the pressure sensor.
            %                     Valid range settings are 0 ("Low"), 1
            %                     ("Standard"), 2 ("High"), 3 ("Very High" / "Ultra High")
            
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setpressureresolution(1);
            %
            %   See also getpressureresolution
            
            
            if (strcmp(thisShimmer.State,'Connected'))                                    % Shimmer must be in a Connected state
                if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 2 )
                    disp('Warning: setpressureresolution - Command not supported for this firmware version, please update firmware.')
                elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode >= 2 )
                    isWritten = writepressureresolution(thisShimmer,pressureRes);         % Write mag range to the Shimmer
                    
                    if (isWritten)
                        isRead = readpressureresolution(thisShimmer);                     % Following a succesful write, call the readmagrange function which updates the magRange property with the current Shimmer mag range setting
                        
                        if (isRead)
                            thisShimmer.readconfigbytes;                                  % update config bytes class properties
                            isSet = (pressureRes == thisShimmer.PressureResolution);      % isSet will be equal to 1 the current mag range setting is equal to the requested setting
                            
                        else
                            isSet = false;
                        end
                    else
                        isSet = false;
                    end
                else
                    fprintf(strcat('Warning: setpressureresolution - Command only supported on Shimmer3.\n'));
                end
            else
                fprintf(strcat('Warning: setpressureresolution - Cannot set resolution for pressure sensor for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setpressureresolution
        
        function isSet = setmagrate(thisShimmer, magRate)
            %SETMAGRATE - Set the Mag Data Rate on the Shimmer
            %
            %   SETMAGRATE(MAGRATE) sets the mag data rate on the Shimmer 
            %   to the value of the input MAGRATE.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setmagrate(magRate)
            %
            %   INPUT: magRate - Numeric value defining the desired mag
            %                    data rate. Valid rate settings for Shimmer 2 are 0 (0.5
            %                    Hz), 1 (1.0 Hz), 2 (2.0 Hz), 3 (5.0 Hz), 4
            %                    (10.0 Hz), 5 (20.0 Hz), 6 (50.0 Hz). For
            %                    Shimmer3 with LSM303DLHC valid settings are
            %                    0 (0.75Hz), 1 (1.5Hz), 2 (3Hz), 3 (7.5Hz),
            %                    4 (15Hz), 5 (30Hz), 6 (75Hz), 7 (220Hz). For 
            %                    Shimmer3 with LSM303AHTR valid settings are
            %                    0 (10.0Hz), 1 (20.0Hz)), 2 (50.0Hz), 3 (100.0Hz).
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setmagrate(1);
            %
            %   
            
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                
                isWritten = writemagrate(thisShimmer,magRate);             % Write mag range to the Shimmer
                
                if (isWritten)
                    isRead = readmagrate(thisShimmer);                     % Following a succesful write, call the readmagrange function which updates the magRange property with the current Shimmer mag range setting
                    
                    if (isRead)
                        thisShimmer.readconfigbytes;                       % update config bytes class properties
                        isSet = (magRate == thisShimmer.MagRate);          % isSet will be equal to 1 the current mag range setting is equal to the requested setting
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end
                
            else
                fprintf(strcat('Warning: setmagrate - Cannot set mag rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setmagrate
              
        function isSet = setaccelrate(thisShimmer, accelRate)
            %SETACCELRATE - Set the Wide Range Accel (Digital Accel) Data Rate on the Shimmer3
            %
            %   SETACCELRATE(ACCELRATE) sets the mag data rate on the Shimmer 
            %   to the value of the input ACCELRATE.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setaccelrate(accelRate)
            %
            %   INPUT: accelRate - Numeric value defining the desired accel
            %                    data rate. Valid rate settings for Shimmer3 
            %                    with LSM303DLHC are 1 (1.0 Hz), 2 (10.0 Hz), 3 (25.0 Hz), 4
            %                    (50.0 Hz), 5 (100.0 Hz), 6 (200.0 Hz), 7 (400.0 Hz) and 9 (1344.0Hz).
            %                    Valid rate settings for Shimmer3 with LSM303AHTR are
            %                    1 (12.5 Hz), 2 (25.0 Hz), 3 (50.0 Hz), 4 (100.0 Hz), 
            %                    5 (200.0 Hz), 6 (400.0 Hz), 7 (800.0 Hz), 8 (1600.0 Hz), 
            %                    9 (3200.0Hz) and 10 (6400.0Hz).  
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setaccelrate(1);
            %
        
            
            
            if (strcmp(thisShimmer.State,'Connected'))                                  % Shimmer must be in a Connected state
                if (thisShimmer.ShimmerVersion == 3)
                    
                    isWritten = writeaccelrate(thisShimmer,accelRate);                  % Write mag range to the Shimmer
                    
                    if (isWritten)
                        isRead = readaccelrate(thisShimmer);                            % Following a succesful write, call the readmagrange function which updates the magRange property with the current Shimmer mag range setting
                        
                        if (isRead)
                            thisShimmer.readconfigbytes;                                % update config bytes class properties
                            isSet = (accelRate == thisShimmer.AccelWideRangeDataRate);  % isSet will be equal to 1 the current mag range setting is equal to the requested setting
                        else
                            isSet = false;
                        end
                    else
                        isSet = false;
                    end
                else
                    fprintf('Warning: setaccelrate - Only supported for Shimmer3');
                end
            else
                fprintf(strcat('Warning: setaccelrate - Cannot set accel rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setaccelrate
        
        function isSet = setgyrorate(thisShimmer, gyroRate)
            %SETGYRORATE - Set the Gyroscope Rate (MPU 9150) on the Shimmer3
            %
            %   SETGYRORATE(GYRORATE) sets the mag data rate on the Shimmer 
            %   to the value of the input GYRORATE.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setmagrate(magRate)
            %
            %   INPUT: gyroRate - Numeric value defining the desired mag
            %                    data rate. Valid rate settings for Shimmer
            %                    3 are 255 (31.25 Hz), 155(51.28 Hz), 45
            %                    (173.91 Hz), 30 (258.06 Hz), 14 (533.33 Hz), 6 (1142.86 Hz). 
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setgyrorate(255);
            %
            
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                if (thisShimmer.ShimmerVersion == 3)
                    
                    isWritten = writegyrorate(thisShimmer,gyroRate);       % Write mag range to the Shimmer
                    
                    if (isWritten)
                        isRead = readgyrorate(thisShimmer);                % Following a succesful write, call the readmagrange function which updates the magRange property with the current Shimmer mag range setting
                        
                        if (isRead)
                            thisShimmer.readconfigbytes;                   % update config bytes class properties
                            isSet = (gyroRate == thisShimmer.GyroRate);    % isSet will be equal to 1 the current mag range setting is equal to the requested setting
                        else
                            isSet = false;
                        end
                    else
                        isSet = false;
                    end
                else
                    fprintf(strcat('Warning: setgyrorate - Command only supported on Shimmer3.'));
                end
            else
                fprintf(strcat('Warning: setgyrorate - Cannot set gyro rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setgyrorate
               
        function isSet = setinternalboard(thisShimmer, internalBoard)
            %SETINTERNALBOARD - Sets the internal daughter board on the Shimmer
            %
            %   SETINTERNALBOARD(INTERNALBOARD) sets the internal daughter
            %   board on the Shimmer to the value defined in the input
            %   INTERNALBOARD. The function will return a 1 if the
            %   operation was successful otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet =
            %   thisShimmer.setinternalboard(internalBoard)
            %
            %   INPUT: internalBoard - String value defining the desired
            %   daughter board.
            %                          Valid values are 'None', 'Gyro',
            %                          'Mag', '9DOF', 'ECG', 'EMG', 'EXG',
            %                          'GSR', 'Strain Gauge' and 'Bridge
            %                          Amplifier'.
            %
            %   OUTPUT: isSet - Boolean value which indicates if the
            %   operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setinternalboard('9DOF');
            %
            %   See also getinternalboard setexternalboard
                        
            if (any(strcmp(thisShimmer.INTERNAL_BOARDS_AVAILABLE, internalBoard)))   % TRUE if the internalBoard variable is contained in the list of available boards
                thisShimmer.InternalBoard = cellstr(internalBoard);
                isSet = true;
            else
                fprintf('Warning: setinternalboard - Invalid selection for internal board.\n');
                thisShimmer.INTERNAL_BOARDS_AVAILABLE                                % Print list of available boards to the command window
                isSet = false;
            end
            
        end % function internalBoard
        
        function externalBoard = setexternalboard(thisShimmer, externalBoard)
            %SETEXTERNALBOARD - Sets the external daughter board on the Shimmer
            %
            %   SETEXTERNALBOARD(EXTERNALBOARD) sets the external daughter
            %   board on the Shimmer to the value defined in the input
            %   EXTERNALBOARD.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setexternalboard(externalBoard)
            %
            %   INPUT: externalBoard - String value defining the desired daughter board.
            %                          Valid values are 'None', 'ExpBoard', 'External' and 'Heart Rate'.
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setexternalboard('ExpBoard');
            %
            %   See also getexternalboard setinternalboard
            
            
            if (any(strcmp(thisShimmer.EXTERNAL_BOARDS_AVAILABLE, externalBoard)))   % Is the externalBoard variable contained in the list of available boards
                if ((strcmp(externalBoard,'ExpBoard') || strcmp(externalBoard,'External')) && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3) % if it is the expansion board check that the pmux has been set to 0
                    if thisShimmer.getpmux==0
                        thisShimmer.ExternalBoard = externalBoard;
                        isSet = true;
                    else  % if not set pmux to 0
                        if thisShimmer.setpmux(0)
                            thisShimmer.ExternalBoard = externalBoard;
                            isSet = true;
                        else
                            externalBoard = thisShimmer.ExternalBoard;
                            fprintf('Warning: setexternalboard - PMux failed to configure:\n');
                            thisShimmer.EXTERNAL_BOARDS_AVAILABLE                                % Print list of available boards to the command window
                            isSet = false;
                        end
                    end
                else
                    thisShimmer.ExternalBoard = externalBoard;
                    isSet = true;
                end
                
            else
                externalBoard = thisShimmer.ExternalBoard;
                fprintf('Warning: setexternalboard - Invalid selection for external board. The valid selections are:\n');
                thisShimmer.EXTERNAL_BOARDS_AVAILABLE                                % Print list of available boards to the command window
                isSet = false;
            end
            
        end % function externalBoard
        
        function setbattlimitwarning(thisShimmer,limit)
            %SETBATTLIMITWARNING - Sets the battery voltage limit for the Shimmer
            %device before the yellow LED on the Shimmer is triggered. Users
            %should note that for this to work, the battery voltage sensor has
            %to be enabled via shimmer.setenabledsensors('BattVolt',1) and the
            %data has to be obtained by the driver via shimmer.getdata('Battery
            %Voltage','a').
            %
            %   SETBATTLIMITWARNING  Sets the battery voltage limit for the Shimmer
            %   device before the yellow LED on the Shimmer is triggered
            %
            %   SYNOPSIS: thisShimmer.setbattlimitwarning(limit) 
            %
            %   INPUT: limit - a voltage value
            %
            %   EXAMPLE: shimmer1.setbattlimitwarning(3.4)
            %
            %   See also setledblinkwhilestreaming
            
            thisShimmer.BattVoltLimit=limit;
        end
        
        function setorientation3D(thisShimmer, setBit)
            %SETORIENTATION3D- Enables/disables orientation3D
            %
            %   SETORIENTATION3D(SETBIT) enables/disables orientation3D 
            %   based on the value of SETBIT.
            %
            %   SYNOPSIS: thisShimmer.setorientation3D(setBit)
            %
            %   INPUT: setBit - Boolean value defining whether
            %                   orientation3D is to be enabled
            %                   (1) or disabled (0).
            %
            %   EXAMPLE:  shimmer1.setorientation3D(1);
            
            thisShimmer.Orientation3D = setBit;            
        end % setorientation3D
        
        function enabletimestampunix(thisShimmer, enableBit)
            %ENABLETIMESTAMPUNIX - Enables/disables Timestamp Unix
            %
            %   ENABLETIMESTAMPUNIX(ENABLEBIT) enables/disables Timestamp in
            %   Unix time format (in milliseconds).
            %
            %   SYNOPSIS:  isSet = thisShimmer.enabletimestampunix(enableBit)
            %
            %   INPUT: enableBit - Boolean value defining whether Timestamp
            %                      Unix to be enabled (1) or disabled (0).
            %
            %   EXAMPLE: shimmer1.enabletimestampunix(1);
            
            thisShimmer.EnableTimestampUnix = enableBit;
        end % enabletimestampunix
        
        function isSet = setgyroinusecalibration(thisShimmer, setBit)
            %SETGYROINUSECALIBRATION - Enables/disables gyro in-use
            %calibration for the Shimmer
            %
            %   SETGYROINUSECALIBRATION(SETBIT) enables/disables gyro
            %   in-use calibration for the Shimmer based on the value of
            %   SETBIT. 
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setgyroinusecalibration(setBit)
            %
            %   INPUT: setBit - Boolean value defining whether the gyro
            %                   in-use calibration method is to be enabled 
            %                   (1) or disabled (0).
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1 = TRUE, 0 = FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setgyroinusecalibration(1);
                        
            if (strcmp(thisShimmer.State,'Connected'))                             % Shimmer must be in a Connected state
                
                isRead = readenabledsensors(thisShimmer);                          % call the readenabledsensors function to update the enabledSensors property with the current Shimmer enabled sensors setting
                
                if(isRead)
                    if(setBit)
                        if(bitand(thisShimmer.EnabledSensors,hex2dec('0040')) ~= 0)    % Check that gyroscope is enabled.
                            thisShimmer.GyroInUseCalibration = setBit;
                            isSet = true;  
                            
                            isRead = readsamplingrate(thisShimmer);
                            if(isRead)
                                thisShimmer.GyroBufferSize = ceil(2*thisShimmer.SamplingRate); % buffer 2 seconds of data.
                            else
                                thisShimmer.GyroBufferSize = 100;                              % default 100 samples if sampling rate is unknown.
                            end
                            thisShimmer.GyroBuffer = [];
                        else
                            fprintf(strcat('Warning: setgyroinusecalibration - Cannot enable gyro in-use calibration because gyroscope is not enabled.\n'));
                            thisShimmer.GyroInUseCalibration = 0;
                            isSet = false;
                        end
                    else
                        thisShimmer.GyroInUseCalibration = 0;
                        isSet = true;  
                    end
                else
                    isSet = false;                    
                end                
            else
                fprintf(strcat('Warning: setgyroinusecalibration - Cannot set gyro in-use calibration for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
                        
        end % function setgyroinusecalibration
        
        function isSet = setbuffersize(thisShimmer, bufferSize)
            %SETBUFFERSIZE - Set the value of the buffer size on the
            %                    Shimmer
            %
            %   SETBUFFERSIZE(BUFFERSIZE) sets the size of the data buffer 
            %   on the Shimmer to the value of the input BUFFERSIZE.
            %   The function will return a 1 if the operation was
            %   successful; otherwise it will return a 0.
            %   For compatibility with this instrument driver, the only
            %   valid value is bufferSize = 1.
            %
            %   SYNOPSIS: isSet = thisShimmer.setbuffersize(bufferSize)
            %
            %   INPUT: bufferSize - Integer value for the buffer size (must
            %   be = 1 for compatibility with this instrument driver).
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setbuffersize(1);
            
            if(bufferSize ~= 1)
                fprintf(strcat('Warning: setbuffersize - Buffer size ~= 1 is not compatible with the instrument driver.\n'));
                isSet = false;
            else
                if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                    
                    isWritten = writebuffersize(thisShimmer,bufferSize);       % Write buffer size value to the Shimmer                   
                   
                    if (isWritten)
                        
                        isRead = readbuffersize(thisShimmer);                  % Following a succesful write, call the readbuffersize function which updates the BufferSize property with the current Shimmer buffer size setting
                        
                        if (isRead)
                            ShimmerBufferSize = thisShimmer.BufferSize;
                            isSet = (ShimmerBufferSize == bufferSize);         % isSet will be equal to 1 the current buffer size setting is equal to the requested setting
                        else
                            isSet = false;
                        end
                        
                    else
                        isSet = false;
                    end
                elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    isSet = false;
                    disp('Warning: setbuffersize - Buffer size is currently not configurable for Shimmer3.');
                else
                    fprintf(strcat('Warning: setbuffersize - Cannot set buffer size for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                    isSet = false;     
                end
            end
        end % function setbuffersize
         
        function isSet = setemgcalibrationparameters(thisShimmer, offset, gain)
            %SETEMGCALIBRATIONPARAMETERS - Set the emg offset and gain of the Shimmer
            %
            %   SETEMGCALIBRATIONPARAMETERS(OFFSET,GAIN) stores the offset and
            %   gain for EMG calibration onto the Shimmer device.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setemgcalibrationparameters(accelRange)
            %
            %   INPUT: offset - the offset of the data emg calibration
            %                   parameter
            %          gain - the gain of the data emg calibration
            %                   parameter
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setemgcalibrationparameters(2060,750);
            %
            %   See also getcalibrationparameters
                       
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                
                isWritten = writeemgcalibrationparameters(thisShimmer,offset,gain);       % Write accelerometer range to the Shimmer
                
                if (isWritten)
                    
                    thisShimmer.EMGOffset = offset;
                    thisShimmer.EMGGain = gain;
                    
                else
                    
                    isSet = false;
                    
                end
                
            else
                fprintf(strcat('Warning: setemgcalibrationparameters - Cannot set EMG calibration parameters for COM ',thisShimmer.ComPort,' as Shimmer is not connected\n'));
                fprintf('or Shimmer is a Shimmer3. Set EMG calibration parameters is not supported for Shimmer3.\n');
                isSet = false;
            end
        end % function setemgcalibrationparameters
        
        function isSet = setecgcalibrationparameters(thisShimmer,offsetrall,gainrall,offsetlall,gainlall)
            %SETECGCALIBRATIONPARAMETERS - Set the gain and offset for ECG RA-LL and ECG LA-LL of the Shimmer
            %
            %   SETECGCALIBRATIONPARAMETERS(OFFSET,GAIN) stores the offset and
            %   gain for ECG calibration onto the Shimmer device.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setecgcalibrationparameters(accelRange)
            %
            %   INPUT: offsetrall - the offset of the data ecg ra-ll calibration
            %                   parameter
            %          gainrall - the gain of the data ecg ra-ll calibration
            %                   parameter
            %          offsetlall - the offset of the data ecg la-ll calibration
            %                   parameter
            %          gainlall - the gain of the data ecg la-ll calibration
            %                   parameter
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setecgcalibrationparameters(2060,750);
            %
            %   See also getcalibrationparameters
            
            
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                
                isWritten = writeecgcalibrationparameters(thisShimmer,offsetrall,gainrall,offsetlall,gainlall);       % Write accelerometer range to the Shimmer
                
                if (isWritten)
                    
                    thisShimmer.ECGRALLOffset = offsetrall;
                    thisShimmer.ECGRALLGain = gainrall;
                    thisShimmer.ECGLALLOffset = offsetlall;
                    thisShimmer.ECGLALLGain = gainlall;
                    
                else
                    
                    isSet = false;
                    
                end
                
            else
                fprintf(strcat('Warning: setecgcalibrationparameters - Cannot set ECG calibration parameters for COM ',thisShimmer.ComPort,' as Shimmer is not connected\n'));
                fprintf('or Shimmer is a Shimmer3. Set ECG calibration parameters is not supported for Shimmer3.\n');
                isSet = false;
            end
        end % function setecgcalibrationparameters
        
        function isSet = setexgrate(thisShimmer, exgRate, chipIdentifier) % function setexgrate  
            %SETEXGRATE - Set the exg rate on the Shimmer
            %
            %   SETEXGRATE(EXGRATE, CHIPIDENTIFIER) sets the ExG Data Rate
            %   for chip CHIPIDENTIER on the Shimmer to the value of the
            %   input EXGRATE. The function will return a 1 if the
            %   operation was successful otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setexgrate(exgRate, chipIdentifier)
            %
            %   INPUT: exgRate -  Numeric value defining the desired exg
            %                     data rate. Valid rate settings are 0
            %                     (125 Hz), 1 (250 Hz), 2 (500 Hz
            %                     (default)), 3 (1000 Hz), 4 (2000 Hz), 5
            %                     (4000 Hz), 6 (8000 Hz)
            % 
            %   INPUT: chipIdentifier - numeric value to select SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: isSet -   Boolean value which indicates if the
            %                     operation was successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setexgrate(4,2);
            %
            %   See also getexgrate
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                fprintf('Warning: setexgrate - Command is not supported for this firmware version, please update firmware.\n');
                isSet = false;
            elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                
                isWritten = writeexgrate(thisShimmer, exgRate, chipIdentifier); % Write exgRate to the Shimmer
                
                if (isWritten)
                    isRead = readexgrate(thisShimmer, chipIdentifier);          % Following a succesful write, call the readexgrate function which updates the exgRate property with the current Shimmer exg rate setting
                    
                    if (isRead)
                        if (chipIdentifier == 1)
                            isSet = (exgRate == thisShimmer.EXG1Rate);          % isSet will be equal to 1 the current exg rate setting is equal to the requested setting
                        else
                            isSet = (exgRate == thisShimmer.EXG2Rate);
                        end
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: setexgrate - ExG rate is not supported for Shimmer2/2r.');
                isSet = false;
            else
                fprintf(strcat('Warning: setexgrate - Cannot set exg rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setexgrate
        
        function isSet = setexggain(thisShimmer, exgGain, chipIdentifier, channelIdentifier) % function setexggain 
            %SETEXGGAIN - Set the exg gain on the Shimmer
            %
            %   SETEXGGAIN(EXGGAIN, CHIPIDENTIFIER, CHANNELIDENTIFIER) sets the exg gain for
            %   chip CHIPIDENTIER and channel CHANNELIDENTIFIER on the Shimmer to the value of the input
            %   EXGGAIN. The function will return a 1 if the operation was
            %   successful otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setexggain(exgGain, chipIdentifier, channelIdentifier)
            %
            %   INPUT: exgGain -  Numeric value defining the desired exg
            %                     gain. Valid range settings are 0 (6x), 1 (1x),
            %                     2 (2x), 3 (3x), 4 (4x), 5 (8x), 6 (12x)
            % 
            %   INPUT: chipIdentifier - numeric value to select ExG chip 1
            %                           or 2
            %
            %   INPUT: channelIdentifier - numeric value to select SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: isSet -   Boolean value which indicates if the
            %                     operation was successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setexggain(5,2,1);
            %
            %   See also getexggain
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                fprintf('Warning: setexggain - Command is not supported for this firmware version, please update firmware.\n');
                isSet = false;
            elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                
                isWritten = writeexggain(thisShimmer, exgGain, chipIdentifier, channelIdentifier); % Write exgGain to the Shimmer
                
                if (isWritten)
                    isRead = readexggain(thisShimmer, chipIdentifier, channelIdentifier);          % Following a succesful write, call the readexggain function which updates the exgGain property with the current Shimmer exg gain setting
                    if (isRead)
                        exgGainCheck = convertEXGGain(thisShimmer, exgGain);
                        
                        if (chipIdentifier == 1)
                            if (channelIdentifier == 1)
                                isSet = (exgGainCheck == thisShimmer.EXG1CH1Gain);          % isSet will be equal to 1 if the current exg gain setting is equal to the requested setting
                            else
                                isSet = (exgGainCheck == thisShimmer.EXG1CH2Gain);          % isSet will be equal to 1 if the current exg gain setting is equal to the requested setting
                            end
                        else
                            if(channelIdentifier == 1)
                                isSet = (exgGainCheck == thisShimmer.EXG2CH1Gain);          % isSet will be equal to 1 if the current exg gain setting is equal to the requested setting
                            else
                                isSet = (exgGainCheck == thisShimmer.EXG2CH2Gain);          % isSet will be equal to 1 if the current exg gain setting is equal to the requested setting
                            end
                        end
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end
                
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: setexggain - ExG gain is not supported for Shimmer2/2r.');
                isSet = false;
            else
                fprintf(strcat('Warning: setexggain - Cannot set exg gain for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setexggain

        function isSet = setexgreferenceelectrodeconfiguration(thisShimmer, referenceElectrodeConfiguration)
            %SETEXGREFERENCEELECTRODECONFIGURATION - Set the ExG reference electrode configuration. on the Shimmer
            %
            %   SETEXGREFERENCEELECTRODECONFIGURATION(REFERENCEELECTRODECONFIGURATION) sets the ExG reference
            %   electrode configuration to the value of the input DETECTIONMODE. The function will return a 1
            %   if the operation was successful otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setexgreferenceelectrodeconfiguration(referenceElectrodeConfiguration)
            %
            %   INPUT: referenceElectrodeConfiguration -  Numeric value defining the reference electrode configuration.
            %                                             Valid values are:  [0,15]
            %
            %   OUTPUT: isSet -                           Boolean value which indicates if the
            %                                             operation was successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setexgreferenceelectrodeconfiguration(13);
            %
            %   See also getexgreferenceelectrodeconfiguration
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: setexgreferenceelectrodeconfiguration - Cannot set ExG reference electrode configuration for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: setexgreferenceelectrodeconfiguration - Only supported on Shimmer3');
                isSet = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: setexgreferenceelectrodeconfiguration - Command not supported for this firmware version, please update firmware.');
                isSet = false;
            else
                
                if (thisShimmer.EMGflag && ~(referenceElectrodeConfiguration == 0 || referenceElectrodeConfiguration == 3)) 
                    disp('Please note that for EMG valid values are ''0'' and ''3'' for ''Fixed Potential (default)'' and ''Inverse of Ch1'', respectively.');
                elseif(thisShimmer.ECGflag && ~(referenceElectrodeConfiguration == 0 || referenceElectrodeConfiguration == 13))
                    disp('Please note that for ECG valid values are ''0'' and ''13'' for ''Fixed Potential'' and ''Inverse Wilson CT (default)'', respectively.');
                elseif~(referenceElectrodeConfiguration == 0 || referenceElectrodeConfiguration == 3 || referenceElectrodeConfiguration == 13)
                    disp('Please note that for ECG valid values are ''0'' and ''13'' for ''Fixed Potential'' and ''Inverse Wilson CT (default)'', respectively.');
                    disp('Please note that for EMG valid values are ''0'' and ''3'' for ''Fixed Potential (default)'' and ''Inverse of Ch1'', respectively.');
                    disp('Check for ADS 1292r datasheet for details on other settings.');
                end
                
                isWritten = writeexgreferenceelectrodeconfiguration(thisShimmer, referenceElectrodeConfiguration); % Write referenceElectrodeConfiguration to the Shimmer
                
                if (isWritten)
                    isRead = readexgreferenceelectrodeconfiguration(thisShimmer);                % Following a succesful write, call the readexgreferenceelectrodeconfiguration function which updates the properties with the current settings
                    
                    if (isRead)
                        isSet = (referenceElectrodeConfiguration == thisShimmer.EXGReferenceElectrodeConfiguration);
                    else
                        isSet = false;
                    end
                    
                else
                    isSet = false;
                end
                
            end
        end % function setexgreferenceelectrodeconfiguration

        function isSet = setexgleadoffdetectionmode(thisShimmer, detectionMode)
            %SETEXGLEADOFFDETECTIONMODE - Set the ExG lead-off detection mode on the Shimmer
            %
            %   SETEXGLEADOFFDETECTIONMODE(DETECTIONMODE) sets the ExG lead-off detection mode
            %   to the value of the input DETECTIONMODE. The function will return a 1 if the operation was
            %   successful otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setexgleadoffdetectionmode(detectionMode)
            %
            %   INPUT: detectionMode -  Numeric value defining the lead-off
            %                           detection mode.
            %                           Valid values are:  0 ('Off') or 1 ('DC Current')
            %
            %   OUTPUT: isSet -         Boolean value which indicates if the
            %                           operation was successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setexgleadoffdetectionmode(1);
            %
            %   See also getexgleadoffdetectionmode
            %   setexgleadoffcomparatorthreshold getexgleadoffcomparatorthreshold
            %   setexgleadoffdetectioncurrent getexgleadoffdetectioncurrent
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: setexgleadoffdetectionmode - Cannot set ExG lead-off detection mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: setexgleadoffdetectionmode - Only supported on Shimmer3');
                isSet = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: setexgleadoffdetectionmode - Command not supported for this firmware version, please update firmware.');
                isSet = false;
            else
                
                isWritten = writeexgleadoffdetectionmode(thisShimmer, detectionMode); % Write detectionMode to the Shimmer
                
                if (isWritten)
                    isRead = readexgleadoffdetectionmode(thisShimmer);                % Following a succesful write, call the readexgleadoffdetectionmode function which updates the properties with the current settings
                    
                    if (isRead)
                        isSet = (detectionMode == thisShimmer.EXGLeadOffDetectionMode);
                    else
                        isSet = false;
                    end
                    
                else
                    isSet = false;
                end
                
            end
        end % function setexgleadoffdetectionmode

        function isSet = setexgleadoffdetectioncurrent(thisShimmer, detectionCurrent, chipIdentifier)
            %SETEXGLEADOFFDETECTIONCURRENT - Set the ExG lead-off detection current on the Shimmer
            %
            %   SETEXGLEADOFFDETECTIONCURRENT(DETECTIONCURRENT, CHIPIDENTIFIER) sets the ExG lead-off detection current
            %   to the value of the input DETECTIONCURRENT. The function will return a 1 if the operation was
            %   successful otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setexgleadoffdetectioncurrent(detectionCurrent)
            %
            %   INPUT: detectionCurrent -  Numeric value defining the lead-off
            %                              detection current.
            %                              Valid values are:  0 ('6nA'), 4 ('22nA')
            %                              8 ('6uA') and 12 ('22uA')
            %
            %   INPUT: chipIdentifier -   chipIdentifier is either 1 or 2 to
            %                             select SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: isSet -           Boolean value which indicates if the
            %                             operation was successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setexgleadoffdetectioncurrent(4);
            %
            %   See also getexgleadoffdetectionmode setexgleadoffdetectionmode
            %   setexgleadoffcomparatorthreshold getexgleadoffcomparatorthreshold
            %   getexgleadoffdetectioncurrent
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: setexgleadoffdetectioncurrent - Cannot set ExG lead-off detection current for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: setexgleadoffdetectioncurrent - Only supported on Shimmer3.');
                isSet = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: setexgleadoffdetectioncurrent - Command not supported for this firmware version, please update firmware.');
                isSet = false;
            elseif ~(chipIdentifier == 1 || chipIdentifier == 2)
                disp('Warning: setexgleadoffdetectioncurrent - Invalid value for chipIdentifier.');
                isSet = false;
            else
                
                isWritten = writeexgleadoffdetectioncurrent(thisShimmer, detectionCurrent, chipIdentifier); % Write detectionCurrent to the Shimmer
                
                if (isWritten)
                    isRead = readexgleadoffdetectioncurrent(thisShimmer, chipIdentifier);                   % Following a succesful write, call the readexgleadoffdetectioncurrent function which updates the properties with the current settings
                    
                    if (isRead && chipIdentifier == 1)
                        isSet = (detectionCurrent == thisShimmer.EXG1ILEAD_OFF);
                    elseif (isRead && chipIdentifier == 2)
                        isSet = (detectionCurrent == thisShimmer.EXG2ILEAD_OFF);
                    else
                        isSet = false;
                    end
                    
                else
                    isSet = false;
                end
                
            end
        end % function setexgleadoffdetectioncurrent

        function isSet = setexgleadoffcomparatorthreshold(thisShimmer, comparatorThreshold, chipIdentifier)
            %SETEXGLEADOFFCOMPARATORTHRESHOLD - Set the ExG lead-off comparator threshold on the Shimmer
            %
            %   SETEXGLEADOFFCOMPARATORTHRESHOLD(COMPARATORTHRESHOLD, CHIPIDENTIFIER) sets the ExG lead-off comparator threshold
            %   to the value of the input COMPARATORTHRESHOLD. The function will return a 1 if the operation was
            %   successful otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setexgleadoffcomparatorthreshold(comparatorThreshold)
            %
            %   INPUT: comparatorThreshold -  Numeric value defining the lead-off
            %                                 detection current.
            %                                 Valid values are: 
            %                                  0 ('Pos:95% - Neg:5%'),
            %                                  32 ('Pos:92.5% - Neg:7.5%'),
            %                                  64 ('Pos:90% - Neg:10%'),
            %                                  96 ('Pos:87.5% - Neg:12.5%'),
            %                                  128 ('Pos:85% - Neg:15%'),
            %                                  160 ('Pos:80% - Neg:20%'),
            %                                  192 ('Pos:75% - Neg:25%'),
            %                                  224 ('Pos:70% - Neg:30%'),
            %                                  'Unknown'.
            %
            %   INPUT: chipIdentifier -       chipIdentifier is either 1 or 2 to
            %                                 select SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: isSet -               Boolean value which indicates if the
            %                                 operation was successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setexgleadoffcomparatorthreshold(160);
            %
            %   See also getexgleadoffdetectionmode setexgleadoffdetectionmode
            %   getexgleadoffcomparatorthreshold getexgleadoffdetectioncurrent
            %   setexgleadoffdetectioncurrent
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: setexgleadoffcomparatorthreshold - Cannot set ExG lead-off comparator threshold for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: setexgleadoffcomparatorthreshold - Only supported on Shimmer3.');
                isSet = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: setexgleadoffcomparatorthreshold - Command not supported for this firmware version, please update firmware.');
                isSet = false;
            elseif ~(chipIdentifier == 1 || chipIdentifier == 2)
                disp('Warning: setexgleadoffcomparatorthreshold - Invalid value for chipIdentifier.');
                isSet = false;
            else
                
                isWritten = writeexgleadoffcomparatorthreshold(thisShimmer, comparatorThreshold, chipIdentifier); % Write comparatorThreshold to the Shimmer
                
                if (isWritten)
                    isRead = readexgleadoffcomparatorthreshold(thisShimmer, chipIdentifier);                   % Following a succesful write, call the readexgleadoffcomparatorthreshold function which updates the properties with the current settings
                    
                    if (isRead && chipIdentifier == 1)
                        isSet = (comparatorThreshold == thisShimmer.EXG1COMP_TH);
                    elseif (isRead && chipIdentifier == 2)
                        isSet = (comparatorThreshold == thisShimmer.EXG2COMP_TH);
                    else
                        isSet = false;
                    end
                    
                else
                    isSet = false;
                end
                
            end
        end % function setexgleadoffcomparatorthreshold
        
        function isSet = setexgtestsignalparameters(thisShimmer, chipIdentifier) 
            %SETEXGTESTSIGNALPARAMETERS - Set the default exg test signal parameters
            %
            %   SETEXGTESTSIGNALPARAMETERS(CHIPIDENTIFIER) sets the default parameters to enable
            %   the test signal on the EXG chip identified by the input,
            %   chipIdentifier. Note that the ExG channels must also be
            %   enabled via the setenabledsensors method.
            %   The function will return a 1 for ISSET if the operation was
            %   successful; otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setexgtestsignalparameters(chipIdentifier)
            %
            %   INPUT: chipIdentifier - numeric value to select SENSOR_EXG1
            %   (1) or SENSOR_EXG2 (2)
            %
            %   OUTPUT: isSet - Boolean value which indicates if the
            %   operation was successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setexgtestsignalparameters(1);
            %
            %   See also setexgconfiguration setdefaultecgparameters
            %   setdefaultemgparameters 
            %            
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                fprintf('Warning: setexgtestsignalparameters - Command is not supported for this firmware version, please update firmware.\n');
                isSet = false;
            elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                isSet = thisShimmer.setexgconfiguration([2 163 16 5 5 0 0 0 2 1] ,chipIdentifier); % select SENSOR_EXG1 or SENSOR_EXG2
            else
                fprintf('Warning: setexgtestsignalparameters - Command is not supported for Shimmer2/2r.\n');
                isSet = false;
            end
        end

        function [isSetEcg1, isSetEcg2] = setdefaultecgparameters(thisShimmer) % set default ecg parameters
            %SETDEFAULTECGPARAMETERS - Set the default ecg parameters
            %
            %   SETDEFAULTECGPARAMETERS() sets the default ecg parameters for SENSOR_EXG1 and SENSOR_EXG2
            %   The function will return a 1 for both ISSETECG1 and ISSETECG2 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: [isSetEcg1, isSetEcg2] = thisShimmer.setdefaultecgparameters()
            %
            %   OUTPUT: isSetEcg1 - Boolean value which indicates if the operation was
            %                       successful or not (1=TRUE, 0=FALSE).
            %
            %   OUTPUT: isSetEcg2 - Boolean value which indicates if the operation was
            %                       successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: [isSetEcg1, isSetEcg2] = shimmer1.setdefaultecgparameters();
            %
            %   See also setexgconfiguration setdefaultemgparameters
            %   setexgtestsignalparameters
            %
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                fprintf('Warning: setdefaultecgparameters - Command is not supported for this firmware version, please update firmware.\n');
                isSetEcg1=false;
                isSetEcg2=false;
            elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                if (thisShimmer.SamplingRate <= 125)         % set default parameter for ecg datarate as close as possible to Shimmer sampling rate; but never lower
                    datarate = 0;                            % set data rate to 125Hz
                elseif (thisShimmer.SamplingRate <= 250)
                    datarate = 1;                            % set data rate to 250Hz
                elseif (thisShimmer.SamplingRate <= 500)
                    datarate = 2;                            % set data rate to 500Hz
                elseif (thisShimmer.SamplingRate <= 1000)
                    datarate = 3;                            % set data rate to 1000Hz
                elseif (thisShimmer.SamplingRate <= 2000)
                    datarate = 4;                            % set data rate to 2000Hz
                elseif (thisShimmer.SamplingRate <= 4000)
                    datarate = 5;                            % set data rate to 4000Hz
                else
                    datarate = 6;                            % set data rate to 8000Hz
                end
                isSetEcg1 = thisShimmer.setexgconfiguration([datarate 160 16 64 64 45 0 0 2 3] ,1); % set default parameters for SENSOR_EXG1
                isSetEcg2 = thisShimmer.setexgconfiguration([datarate 160 16 64 71 0 0 0 2 1] ,2);  % set default parameters for SENSOR_EXG2
                thisShimmer.setexgreferenceelectrodeconfiguration(13);                              % set reference electrode configuration to Inverse Wilson CT
            else
                fprintf('Warning: setdefaultecgparameters - Command is not supported for Shimmer2/2r.\n');
                isSetEcg1=false;
                isSetEcg2=false;
            end
        end
        
        function [isSetEmg1, isSetEmg2] = setdefaultemgparameters(thisShimmer) % set default emg parameters
            %SETDEFAULTEMGPARAMETERS - Set the default emg parameters
            %
            %  SETDEFAULTEMGPARAMETERS() sets the default emg parameters for SENSOR_EXG1 and SENSOR_EXG2
            %   The function will return a 1 for both ISSETEMG1 and ISSETEMG2 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: [isSetEmg1, isSetEmg2] = thisShimmer.setdefaultemgparameters()
            %
            %   OUTPUT: isSetEmg1 - Boolean value which indicates if the operation was
            %                       successful or not (1=TRUE, 0=FALSE).
            %
            %   OUTPUT: isSetEmg2 - Boolean value which indicates if the operation was
            %                       successful or not (1=TRUE, 0=FALSE).
            %   
            %   EXAMPLE: [isSetEmg1, isSetEmg2] = shimmer1.setdefaultemgparameters();
            %
            %   See also setexgconfiguration setdefaultecgparameters
            %   setexgtestsignalparameters
            %
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                fprintf('Warning: setdefaultecgparameters - Command is not supported for this firmware version, please update firmware.\n');
                isSetEmg1=false;
                isSetEmg2=false;
            elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                if (thisShimmer.SamplingRate <= 125)          % set default parameter for ecg datarate as close as possible to Shimmer sampling rate; but never lower
                    datarate = 0;                            % set data rate to 125Hz
                elseif (thisShimmer.SamplingRate <= 250)
                    datarate = 1;                            % set data rate to 250Hz
                elseif (thisShimmer.SamplingRate <= 500)
                    datarate = 2;                            % set data rate to 500Hz
                elseif (thisShimmer.SamplingRate <= 1000)
                    datarate = 3;                            % set data rate to 1000Hz
                elseif (thisShimmer.SamplingRate <= 2000)
                    datarate = 4;                            % set data rate to 2000Hz
                elseif (thisShimmer.SamplingRate <= 4000)
                    datarate = 5;                            % set data rate to 4000Hz
                else
                    datarate = 6;                            % set data rate to 8000Hz
                end
                isSetEmg1 = thisShimmer.setexgconfiguration([datarate 160 16 105 96 32 0 0 2 3] ,1); % set default parameters for SENSOR_EXG1
                isSetEmg2 = thisShimmer.setexgconfiguration([datarate 160 16 129 129 0 0 0 2 1] ,2); % set default parameters for SENSOR_EXG2
                thisShimmer.setexgreferenceelectrodeconfiguration(0);                                % set reference electrode configuration to Fixed Potential
            else
                fprintf('Warning: setdefaultemgparameters - Command is not supported for Shimmer2/2r.\n');
                isSetEmg1=false;
                isSetEmg2=false;
            end
        end
    
        function isSet = setexgconfiguration(thisShimmer, exgconfiguration, chipIdentifier)
            %SETEXGCONFIGURATIONS - Set the EXG Configuration on the Shimmer
            %
            %   SETEXGCONFIGURATION(EXGCONFIGURATION,CHIPIDENTIFIER) sets the ExG configuration on the Shimmer to the
            %   value of the input ECGCONFIGURATION, on the chosen ExG Chip 
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setexgconfiguration(exgConfiguration,chipIdentifier)
            %
            %   INPUT: exgConfiguration - a 10 byte value according to the
            %   following sequence: 
            %       Config1 
            %       Config2 
            %       LOFF 
            %       CH1SET 
            %       CHS2SET 
            %       RLD_SENS 
            %       LOFF_SENS 
            %       LOFF_STAT 
            %       RESP1 
            %       RESP2  
            %
            %   INPUT: chipIdentifier - numeric value to select SENSOR_EXG1
            %   (1) or SENSOR_EXG2 (2)
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setexgconfiguration([2 160 16 0 0 44 15 0 2 2] ,1);
            %
            %   See also setdefaultecgparameters setdefaultemgparameters
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                fprintf('Warning: setexgconfiguration - Command is not supported for this firmware version, please update firmware.\n');
                isSet = false;
            elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)     % Shimmer must be in a Connected state
                isWritten = writeexgconfiguration(thisShimmer,exgconfiguration,chipIdentifier);           % Write exg configuration to the Shimmer
                if (isWritten)
                    isRead = readexgconfiguration(thisShimmer,chipIdentifier);                     % Following a succesful write, call the readexgconfiguration function which updates the exg configuration. 
                    
                    if (isRead)
                        isSet = true;
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: setexgconfiguration - ExG configuration is not available for Shimmer2/2r.');
                isSet = false;
            else
                fprintf(strcat('Warning: setexgconfiguration - Cannot set exgconfiguration for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isSet = false;
            end
        end % function setexgconfiguration
        
        function isSet = setrealtimeclock(thisShimmer)
            %SETREALTIMECLOCK - Set the Real Time Clock
            %
            %   SETREALTIMECLOCK sets the Real Time Clock on Shimmer3.
            %   The function will return a 1 if the operation was successful
            %   otherwise it will return a 0.
            %
            %   SYNOPSIS: isSet = thisShimmer.setrealtimeclock
            %
            %   OUTPUT: isSet - Boolean value which indicates if the operation was
            %                   successful or not (1=TRUE, 0=FALSE).
            %
            %   EXAMPLE: isSet = shimmer1.setrealtimeclock
            %
            %   See also getrealtimeclock
            %
            if (thisShimmer.IsSDLogging == 1)
                fprintf('Warning: setrealtimeclock - Shimmer is logging to SD Card.\n');
                fprintf('Stop logging before setting Real Time Clock.\n');
                isSet = false;
            elseif  (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                isSet = false;
                fprintf('Warning: setrealtimeclock - Command not available for this hardware version.\n');
            elseif (thisShimmer.FirmwareCompatibilityCode < 6)
                isSet = false;
                fprintf('Warning: setrealtimeclock - Command not available for this firmware version.\n');
            elseif (~strcmp(thisShimmer.State,'Connected'))
                isSet = false;
                fprintf(strcat('Warning: setrealtimeclock - Cannot set Real Time Clock for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            else
                systemTimeMilliseconds = thisShimmer.convertMatlabTimeToUnixTimeMilliseconds(clock); % System Time in milliseconds
                systemTimeTicks = uint64(systemTimeMilliseconds*32.768);                             % System Time in tick of 32768Hz clock
                
                isWritten = writerealtimeclock(thisShimmer,systemTimeTicks);                         % Write Real Time Clock (in ticks) to Shimmer.

                if (isWritten)
                    isRead = readrealtimeclock(thisShimmer);                                         % Following a successful write, call read the realtimeclock from the Shimmer.

                    if (isRead)
                        % Ignore least significant two bytes to compensate for time it takes to write to and read from Shimmer. 
                        mask = (2^64-1-2^16-1);                            
                        isSet = (bitand(systemTimeTicks,mask) == bitand(thisShimmer.RealTimeClockTicks,mask));
                        if (isSet)
                            thisShimmer.getrealtimeclock;
                        end
                    else
                        isSet = false;
                    end
                else
                    isSet = false;
                end
            end
        end % function setrealtimeclock
        
    end % methods
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Get Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
                
        function comPort = getcomport(thisShimmer)
            %GETCOMPORT - Get the Com Port of the Shimmer
            %
            %   COMPORT = GETCOMPORT returns the Com Port setting
            %   of the Shimmer.
            %
            %   SYNOPSIS: comPort = thisShimmer.getcomport()
            %
            %   OUTPUT: comPort - String value defining the Com Port setting of
            %                     the shimmer
            %
            %   EXAMPLE: comPort = shimmer1.getcomport;
            %
            
            comPort = thisShimmer.ComPort;
        end
                
        function state = getstate(thisShimmer)
            %GETSTATE - Get the state of the Shimmer
            %
            %   STATE = GETSTATE returns the state of the Shimmer.
            %
            %   SYNOPSIS: state = thisShimmer.getstate()
            %
            %   OUTPUT: state - String value defining the state of the Shimmer.
            %                   Valid return values are 'Disconnected',
            %                   'Connected' and 'Streaming'.
            %
            %   EXAMPLE: state = shimmer1.getstate;
            %
            
            state = thisShimmer.State;
        end        
        
        function shimmerVersion = getshimmerversion(thisShimmer)
            %GETSHIMMERVERSION - Get the version number of the Shimmer
            %
            %   SHIMMERVERSION = GETSHIMMERVERSION returns the version number
            %   of the Shimmer.
            %
            %   SYNOPSIS: shimmerVersion = thisShimmer.getshimmerversion()
            %
            %   OUTPUT: shimmerVersion - numeric value defining the version
            %                            number of the Shimmer.
            %
            %   EXAMPLE: shimmerVersion = shimmer1.getshimmerversion;
            %
            shimmerVersion = thisShimmer.ShimmerVersion;
        end        
        
        function samplingRate = getsamplingrate(thisShimmer)
            %GETSAMPLINGRATE - Get the sampling rate of the Shimmer
            %
            %   SAMPLINGRATE = GETSAMPLINGRATE returns the sampling rate setting
            %   of the Shimmer.
            %
            %   SYNOPSIS: samplingRate = thisShimmer.getsamplingrate()
            %
            %   OUTPUT: samplingRate - Numeric value defining the sampling
            %                          rate setting of the shimmer in Hertz.
            %
            %   EXAMPLE: samplingRate = shimmer1.getsamplingrate;
            %
            %   See also setsamplingrate
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                samplingRate = thisShimmer.SamplingRate;
            else
                samplingRate = 'Nan';
                fprintf(strcat('Warning: getsamplingrate - Cannot determine sampling rate as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
        end  % function getsamplingrate
        
        function baudRate = getbaudrate(thisShimmer)
            %GETBAUDRATE - Get the baud rate of the Shimmer
            %
            %   BAUDRATE = GETBAUDRATE returns the baud rate setting
            %   of the Shimmer.
            %
            %   SYNOPSIS: baudRate = thisShimmer.getbaudrate()
            %
            %   OUTPUT: baudRate - Numeric value defining the baud
            %                          rate setting of the shimmer in kB/s.
            %
            %   EXAMPLE: baudRate = shimmer1.getbaudrate;
            %
            %   See also setbaudrate
            if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                baudRate = 'Nan';
                disp('Warning: getbaudrate - This function is only supported for Shimmer3.');
            elseif (thisShimmer.FirmwareCompatibilityCode < 5) 
                baudRate = 'Nan';
                disp('Warning: getbaudrate - This function is not supported for this firmware version, please update firmware.');
            elseif ~(strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                baudRate = 'Nan';
                fprintf(strcat('Warning: getbaudrate - Cannot determine baud rate as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            else
                baudRate = thisShimmer.BaudRate;
            end
        end  % function getbaudrate
        
        function isRead = getsdcarddirectoryname(thisShimmer)
            % GETSDCARDDIRECTORYNAME - Reads the SD card directory name of
            % the Shimmer.
            %
            % This command should only be sent to the Shimmer if it is in
            % 'Connected' state. After the acknowledgement, Shimmer
            % remains in 'Connected' state. The directory name is only
            % meaningful when the device is actively logging.
            %
            if ~strcmp(thisShimmer.State,'Connected')
                fprintf(strcat('Warning: getsdcarddirectoryname - Cannot read SD card directory name for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isRead = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: getsdcarddirectoryname - This function is only supported for Shimmer3.');
                isRead = false;
            elseif(thisShimmer.FirmwareIdentifier ~= 3)
                disp('Warning: getsdcarddirectoryname - This function is only supported for LogAndStream firmware.');
                isRead = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 4)
                disp('Warning: getsdcarddirectoryname - This function is not supported in this firmware version, please update firmware.');
                isRead = false;
            else
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_DIR_COMMAND);               % Send the GET_DIR_COMMAND command the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);  % Wait for Acknowledgment from Shimmer
                if ~(isAcknowledged == true)
                    thisShimmer.SdCardDirectoryName = 'Nan';                            % Set the property to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: getsdcarddirectoryname - Dir response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                else
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 3);     % Read the 3 bytes response from the realterm buffer
                    
                    if ( ~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.INSTREAM_CMD_RESPONSE) && (shimmerResponse(2) == thisShimmer.DIR_RESPONSE) )
                        directoryNameLength = double(shimmerResponse(3));                           % Length of SD Directory Name
                        [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, directoryNameLength);     % Read the directoryNameLength bytes response from the realterm buffer
                        thisShimmer.SdCardDirectoryName = native2unicode(shimmerResponse', 'US-ASCII');   % Convert bytes to ASCII string
                        fprintf(['SD Card Directory Name is: ', thisShimmer.SdCardDirectoryName '\n']);
                        isRead = true;
                    else
                        thisShimmer.SdCardDirectoryName = 'Nan';                        % Set the  to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: getsdcarddirectoryname - Instream command response and Dir response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                end
            end
        end  % function getsdcarddirectoryname
     
        


        function batteryVoltage = getbatteryvoltage(thisShimmer)
            % GETBATTERYVOLTAGE - Reads the momentary battery voltage of
            % the Shimmer.
            %
            % This command should only be sent to the Shimmer if it is in
            % 'Connected' state. After the acknowledgement, Shimmer
            %  remains in its previous state.
            %
            if ~(strcmp(thisShimmer.State,'Connected'))
                fprintf(strcat('Warning: getbatteryvoltage - Cannot read battery voltage for COM ',thisShimmer.ComPort,' as Shimmer is not connected,\n'));
                batteryVoltage = 'Nan';
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: getbatteryvoltage - This function is only supported for Shimmer3.');
                batteryVoltage = 'Nan';
            elseif (thisShimmer.FirmwareCompatibilityCode < 6)
                disp('Warning: getbatteryvoltage - This function is not supported in this firmware version, please update firmware.');
                batteryVoltage = 'Nan';
            else
                if strcmp(thisShimmer.State,'Connected')
                    clearreaddatabuffer(thisShimmer);                                                            % As a precaution always clear the read data buffer before a write
                end
                writetocomport(thisShimmer, thisShimmer.GET_VBATT_COMMAND);                                      % Send the GET_VBATT_COMMAND command the Shimmer
                
                if strcmp(thisShimmer.State,'Connected')
                    clearreaddatabuffer(thisShimmer);                                                            % As a precaution always clear the read data buffer before a write
                    
                    isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);                       % Wait for Acknowledgment from Shimmer
                    if ~(isAcknowledged == true)
                        thisShimmer.LatestBatteryVoltageReading = 'Nan';                                         % Set the property to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: getbatteryvoltage - Dir response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        batteryVoltage = 'Nan';
                    else
                        [shimmerResponse, ~] = readdatabuffer(thisShimmer, 5);                                   % Read the 5 bytes response from the realterm buffer
                        
                        if ( ~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.INSTREAM_CMD_RESPONSE) && (shimmerResponse(2) == thisShimmer.VBATT_RESPONSE) )
                            battAdcValue = uint32(uint16(shimmerResponse(4))*256 + uint16(shimmerResponse(3)));  % battery ADC value
                            batteryVoltage = calibrateu12ADCValue(thisShimmer,battAdcValue,0,3.0,1.0)*1.988;     % calibrate 12-bit ADC value with offset = 0; vRef=3.0; gain=1.0
                            fprintf(['Battery Voltage: ' num2str(batteryVoltage) '[mV]' '\n']);
                            thisShimmer.LatestBatteryVoltageReading = batteryVoltage;
                        else
                            thisShimmer.LatestBatteryVoltageReading = 'Nan';                                     % Set the  to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: getbatteryvoltage - Instream command response and VBatt response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            batteryVoltage = 'Nan';
                        end
                    end
                end
            end
        end  % function getbatteryvoltage

        function isRead = getstatus(thisShimmer)
            % GETSTATUS - Gets the status of the Shimmer
            %
            % This command should only be sent to the Shimmer if it is in
            % 'Connected' state. After the acknowledgement, Shimmer
            % remains in 'Connected' state. The status indicates whether
            % the Shimmer is docked (1)/not docked (0) and whether the Shimmer is
            % actively sensing (1) / not actively sensing (0).
            %
            if ~strcmp(thisShimmer.State,'Connected')
                fprintf(strcat('Warning: getstatus - Cannot get Status for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isRead = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: getstatus - This function is only supported for Shimmer3.');
                isRead = false;
            elseif(thisShimmer.FirmwareIdentifier ~= 3)
                disp('Warning: getstatus - This function is only supported for LogAndStream firmware.');
                isRead = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 4)
                disp('Warning: getstatus - This function is not supported in this firmware version, please update firmware.');
                isRead = false;
            else
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_STATUS_COMMAND);            % Send the GET_STATUS_COMMAND command the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);  % Wait for Acknowledgment from Shimmer
                if ~(isAcknowledged == true)
                    thisShimmer.IsSensing = 'Nan';                                      % Set the property to 'Nan' to indicate unknown
                    thisShimmer.IsDocked = 'Nan';                                       % Set the property to 'Nan' to indicate unknown
                    thisShimmer.IsSDLogging = 'Nan';                                    % Set the property to 'Nan' to indicate unknown
                    thisShimmer.IsStreaming = 'Nan';                                    % Set the property to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: getstatus - Get Status command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    fprintf(strcat('Warning: getstatus - Connection has been lost for Shimmer COM',thisShimmer.ComPort,'.\n')); % Assume connection as has been lost as no status response has been returned
                    thisShimmer.disconnect;                                             % Disconnect to close serial port cleanly
                    isRead = false;
                else
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 3);     % Read the 3 bytes response from the realterm buffer
                    
                    if ( ~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.INSTREAM_CMD_RESPONSE) && (shimmerResponse(2) == thisShimmer.STATUS_RESPONSE) )
                        statusResponse = shimmerResponse(3);                            % Status byte
                        thisShimmer.IsSensing = bitand(statusResponse,2)/2;             % Update property
                        thisShimmer.IsDocked = bitand(statusResponse,1);                % Update property
                        thisShimmer.IsSDLogging = bitand(statusResponse,8)/8;           % Update property
                        thisShimmer.IsStreaming = bitand(statusResponse,16)/16;         % Update property
                        disp(strcat('Sensing: ',num2str(thisShimmer.IsSensing)));
                        disp(strcat('Docked: ',num2str(thisShimmer.IsDocked)));
                        disp(strcat('SDLogging: ',num2str(thisShimmer.IsSDLogging)));
                        disp(strcat('Streaming: ',num2str(thisShimmer.IsStreaming)));
                        isRead = true;
                    else
                        thisShimmer.IsSensing = 'Nan';                                  % Set the property to 'Nan' to indicate unknown
                        thisShimmer.IsDocked = 'Nan';                                   % Set the property to 'Nan' to indicate unknown
                        thisShimmer.IsSDLogging = 'Nan';                                % Set the property to 'Nan' to indicate unknown
                        thisShimmer.IsStreaming = 'Nan';                                % Set the property to 'Nan' to indicate unknown
                        isRead = false;
                        fprintf(strcat('Warning: getstatus - Instream command response and Get Status  response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                end
            end
        end  % function getstatus

        
        function configByte0 = getconfigbyte0(thisShimmer)
            %GETCONFIGBYTE0 - Get the config byte0 setting of the Shimmer
            %
            %   CONFIGBYTE0 = GETCONFIGBYTE0 returns the config byte0
            %   setting of the Shimmer.
            %
            %   SYNOPSIS: configByte0 = thisShimmer.getconfigbyte0()
            %
            %   OUTPUT: configByte0 - Unsigned 8bit value equal to the config
            %                         byte0 setting on the Shimmer.
            %
            %   EXAMPLE: configByte0 = shimmer1.getconfigbyte0;
            %
            %   See also setconfigbyte0 getfivevoltreg getpmux
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                configByte0 = thisShimmer.ConfigByte0;
            else
                configByte0 = 'Nan';
                fprintf(strcat('Warning: getconfigbyte0 - Cannot determine config byte0 as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
        end
        
        function [configByte0, configByte1, configByte2, configByte3] = getconfigbytes(thisShimmer)
            %GETCONFIGBYTES - Get the config bytes setting of the Shimmer
            %
            %   [CONFIGBYTE0, CONFIGBYTE1, CONFIGBYTE2, CONFIGBYTE3] = GETCONFIGBYTES
            %   returns the config bytes of the Shimmer.
            %
            %   SYNOPSIS: [configByte0, configByte1, configByte2, configByte3] = thisShimmer.getconfigbytes()
            %
            %   OUTPUT: configByte0 - Unsigned 8bit value equal to the config
            %                         byte0 setting on the Shimmer.
            %   OUTPUT: configByte1 - Unsigned 8bit value equal to the config
            %                         byte1 setting on the Shimmer.
            %   OUTPUT: configByte2 - Unsigned 8bit value equal to the config
            %                         byte2 setting on the Shimmer.
            %   OUTPUT: configByte3 - Unsigned 8bit value equal to the config
            %                         byte3 setting on the Shimmer.
            %
            %   EXAMPLE: [configByte0, configByte1, configByte2, configByte3] = shimmer1.getconfigbytes;
            %
            %   See also setconfigbyte0 getfivevoltreg getpmux
            %   getconfigbyte0 setconfigbytes
            
            if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                configByte0 = thisShimmer.ConfigByte0;
                configByte1 = thisShimmer.ConfigByte1;
                configByte2 = thisShimmer.ConfigByte2;
                configByte3 = thisShimmer.ConfigByte3;
            else
                configByte0 = 'Nan';
                configByte1 = 'Nan';
                configByte2 = 'Nan';
                configByte3 = 'Nan';
                fprintf(strcat('Warning: getconfigbytes - Cannot determine config bytes as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
        end
        
        function fiveVoltReg = getfivevoltreg(thisShimmer)
            %GETFIVEVOLTREG - Get the 5 volt Regulator setting of the Shimmer
            %
            %   FIVEVOLTREG = GETFIVEVOLTREG returns the bit value
            %   setting for the 5V Regulator of the ExpBoard board.
            %
            %   SYNOPSIS: fiveVoltReg = thisShimmer.getfivevoltreg()
            %
            %   OUTPUT: fiveVoltReg - Boolean value defining the setting of the
            %                         5 volt regulator (1=ENABLED, 0=DISABLED).
            %
            %   EXAMPLE: fiveVoltReg = shimmer1.getfivevoltreg;
            %
            %   See also setfivevoltreg getconfigbyte0
            
            if ((strcmp(thisShimmer.State,'Connected') || strcmp(thisShimmer.State,'Streaming')) && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                
                if (~strcmp(thisShimmer.ConfigByte0,'NaN'))
                    fiveVoltReg = bitget(thisShimmer.ConfigByte0, 8);      % Isolate the 5V regulator setting, the MSB in the ConfigByte0 value
                    
                else
                    fprintf(strcat('Warning: getfivevoltreg - Cannot determine 5 volt Regulator setting as\n Config Byte0 for COM ',thisShimmer.ComPort,' is unknown.\n'));
                    fiveVoltReg = 'Nan';
                end
            elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)  
                fiveVoltReg = 'Nan';
                disp('Warning: getfivevoltreg - 5V regulator is not present on Shimmer3.');
            else
                fiveVoltReg = 'Nan';
                fprintf(strcat('Warning: getfivevoltreg - Cannot determine 5 volt Regulator setting as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
        end
                       
        function pMux = getpmux(thisShimmer)
            %GETPMUX - Get the PMux setting of the Shimmer
            %
            %   PMUX = GETPMUX returns the bit value setting for the PMux on
            %   the Shimmer.
            %
            %   SYNOPSIS: pMux = thisShimmer.getPMux()
            %
            %   OUTPUT: pMux - Boolean value defining the setting of the PMux
            %                  (1=ENABLED, 0=DISABLED).
            %
            %   EXAMPLE: pMux = shimmer1.getPMux;
            %
            %   See also setPMux getconfigbyte0
            
            if ((strcmp(thisShimmer.State,'Connected') || strcmp(thisShimmer.State,'Streaming'))&& thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 )                     % Shimmer must be in a Connected state or in Streaming State as this does not require any wireless tx/rx
                
                if (~strcmp(thisShimmer.ConfigByte0,'NaN'))
                    pMux = bitget(thisShimmer.ConfigByte0, 7);             % Isolate the PMux setting, the 2nd MSB in the ConfigByte0 value
                    
                else
                    fprintf(strcat('Warning: getpmux - Cannot determiner PMux setting as\n Config Byte0 for COM ',thisShimmer.ComPort,' is unknown.\n'));
                    pMux = 'Nan';
                end
            elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3) 
                pMux = 'NaN';
                disp('Warning: getpmux - PMUX is not present on Shimmer3.');
            else
                pMux = 'Nan';
                fprintf(strcat('Warning: getpmux - Cannot determine PMux setting as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
        end
                      
        function accelRange = getaccelrange(thisShimmer)
            %GETACCELRANGE - Get the accelerometer range of the Shimmer
            %
            %   ACCELRANGE = GETACCELRANGE returns the accelerometer
            %   range setting of the Shimmer.
            %
            %   SYNOPSIS: accelRange = thisShimmer.getaccelrange()
            %
            %   OUTPUT: accelRange - Numeric value defining the accelerometer
            %                        range.
            %                        Valid range setting values for the Shimmer
            %                        2 are 0 (+/- 1.5g), 1 (+/- 2g), 2 (+/- 4g)
            %                        and 3 (+/- 6g).
            %                        Valid range setting values for the Shimmer
            %                        2r are 0 (+/- 1.5g) and 3 (+/- 6g).
            %
            %   EXAMPLE: accelRange = shimmer1.getaccelrange;
            %
            %   See also setaccelrange
            
            if (strcmp(thisShimmer.State,'Connected')||strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Connected state
                accelRange = thisShimmer.AccelRange;
            else
                accelRange = 'Nan';
                fprintf(strcat('Warning: getaccelrange - Cannot determine accelerometer range as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
        end
        
        function pressureResolution = getpressureresolution(thisShimmer)
            %GETPRESSURERESOLUTION - Get the pressure resolution of the Shimmer
            %
            %   GETPRESSURERESOLUTION = GETPRESSURERESOLUTION returns the pressure resolution setting of the Shimmer.
            %
            %   SYNOPSIS: pressureResolution = thisShimmer.getpressureresolution()
            %
            %   OUTPUT: pressureResolution - Numeric value defining the pressure Resolution.
            %                      Valid range settings are 0 ("Low"), 1
            %                     ("Standard"), 2 ("High"), 3 ("Very High" / "Ultra High")
            %
            %   EXAMPLE: pressureResolution = shimmer1.getpressureresolution();
            %
            %   See also setpressureresolution
            
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode >= 2)                     % Shimmer must be in a Connected state
                pressureResolution = thisShimmer.PressureResolution;
            else
                pressureResolution = 'Nan';
                if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    disp('Warning: getpressureresolution - Pressure sensor is not present on Shimmer2/2r.');
                elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    disp('Warning: getpressureresolution - Pressure sensor is not supported for this firmware version, please update firmware.');
                else
                    fprintf(strcat('Warning: getpressureresolution - Cannot determine pressure resolution as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
                end
            end
        end
               
        function magRange = getmagrange(thisShimmer)
            %GETMAGRANGE - Get the mag range of the Shimmer
            %
            %   MAGRANGE = GETMAGRANGE returns the mag range setting of the Shimmer.
            %
            %   SYNOPSIS: magRange = thisShimmer.getmagrange()
            %
            %   OUTPUT: magRange - Numeric value defining the mag range.
            %                      Valid range settings are 0 (0.7 gauss), 1
            %                      (1.0 gauss), 2 (1.5 gauss), 3 (2.0 gauss), 4 (3.2
            %                      gauss), 5 (3.8 gauss), 6 (4.5 gauss). The mag
            %                      ranges for Shimmer3 with LSM303DLHC are
            %                      1 (1.3 gauss), 2 (1.9 gauss), 3 (2.5 gauss), 4 (4.0 gauss),
            %                      5 (4.7 gauss), 6 (5.6 gauss), 7 (8.1 gauss). For
            %                      Shimmer 3 with LSM303AHTR the mag range
            %                      is fixed at 49.152 gauss.
            %
            %   EXAMPLE: magRange = shimmer1.getmagrange();
            %
            %   See also setmagrange
            if (thisShimmer.FirmwareCompatibilityCode == 0)
                fprintf(strcat('Warning: setmagrange - Command not supported for this firmware version, please update firmware.\n'));
            else
                if (strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                    magRange = thisShimmer.MagRange;
                else
                    magRange = 'Nan';
                    fprintf(strcat('Warning: getmagrange - Cannot determine mag range as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
                end
            end
        end
               
        function gyroRange = getgyrorange(thisShimmer)
            %GETGYRORANGE - Get the gyro range of the Shimmer
            %
            %   GYRORANGE = GETGYRORANGE returns the gyro range setting of the Shimmer.
            %
            %   SYNOPSIS: gyroRange = thisShimmer.getgyrorange()
            %
            %   OUTPUT: gyroRange - Numeric value defining the gyro range.
            %                     Valid output settings are 0 (250dps),
            %                     1 (500dps), 2 (1000dps), 3 (2000dps)
            %
            %   EXAMPLE: gyroRange = shimmer1.getgyrorange();
            %
            %   See also setgyrorange
            
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                gyroRange = thisShimmer.GyroRange;
            else
                gyroRange = 'Nan';
                fprintf(strcat('Warning: getgyrorange - Cannot determine gyro range as COM ',thisShimmer.ComPort,' Shimmer is not connected\n'));
                fprintf('or Shimmer is not a Shimmer3.\n'); 
            end
        end
               
        function internalExpPower = getinternalexppower(thisShimmer)
            %GETINTERNALEXPPOWER - Get the internal exp power setting of the Shimmer3
            %
            %   INTERNALEXPPOWER = GETINTERNALEXPPOWER returns the internal exp power setting of the Shimmer.
            %
            %   SYNOPSIS: internalExpPower = thisShimmer.getinternalexppower()
            %
            %   OUTPUT: internalExpPower - Numeric value defining the
            %   internal exp power setting, 1=enabled , 0=disabled
            %
            %   EXAMPLE: internalExpPower = shimmer1.getinternalExpPower();
            %
            %   See also setinternalexppower
            
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode >= 2) % Shimmer must be in a Connected state
                internalExpPower = thisShimmer.InternalExpPower;
            else
                internalExpPower = 'Nan';
                if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    disp('Warning: getinternalexppower - Internal Exp Power is not supported on Shimmer2/2r.');
                elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    disp('Warning: getinternalexppower - Internal Exp Power is not supported for this firmware version, please update firmware version.');
                else
                    fprintf(strcat('Warning: getinternalexppower - Cannot determine internal exp power setting as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
                end
            end
        end
        
        function accelHRMode = getaccelhrmode(thisShimmer)
            %GETACCELHRMODE - Get the accel high resolution mode setting of
            %the Shimmer3
            %
            %   ACCELHRMODE = GETACCELHRMODE returns accel high resolution
            %   setting setting of the Shimmer.
            %
            %   SYNOPSIS: accelHRMode = thisShimmer.getaccelhrmode
            %
            %   OUTPUT: accelHRMode - Numeric value defining the accel high
            %   resolution setting, 1=enabled , 0=disabled
            %
            %   EXAMPLE: accelHRMode = shimmer1.getaccelhrmode();
            %
            %   See also setaccelhrmode
            
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                accelHRMode = thisShimmer.AccelWideRangeHRMode;
            else
                accelHRMode = 'Nan';
                if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    disp('Warning: getaccelhrmode - Accel HR mode is not supported on Shimmer2/2r.');
                else
                    fprintf(strcat('Warning: getaccelhrmode - Cannot determine Accel HR mode setting as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
                end
            end
        end
        
                function accelLPMode = getaccellpmode(thisShimmer)
            %GETACCELLPMODE - Get the accel low power mode setting of
            %the Shimmer3
            %
            %   ACCELLPMODE = GETACCELLPMODE returns accel low power
            %   setting setting of the Shimmer.
            %
            %   SYNOPSIS: accelLPMode = thisShimmer.getaccellpmode
            %
            %   OUTPUT: accelLPMode - Numeric value defining the accel high
            %   resolution setting, 1=enabled , 0=disabled
            %
            %   EXAMPLE: accelLPMode = shimmer1.getaccellpmode();
            %
            %   See also setaccellpmode
            
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                accelLPMode = thisShimmer.AccelWideRangeLPMode;
            else
                accelLPMode = 'Nan';
                if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    disp('Warning: getaccellpmode - Accel LP mode is not supported on Shimmer2/2r.');
                else
                    fprintf(strcat('Warning: getaccellpmode - Cannot determine Accel LP mode setting as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
                end
            end
        end
        
        function internalBoard = getinternalboard(thisShimmer)
            %GETINTERNALBOARD - Get the internal board setting of the Shimmer
            %
            %   INTERNALBOARD = GETINTERNALBOARD returns the internal board
            %   setting of the Shimmer.
            %
            %   SYNOPSIS: internalBoard = thisShimmer.getinternalboard()
            %
            %   OUTPUT: internalBoard - String value defining the internal
            %                           daughter board of the Shimmer.
            %                            Valid values are 'None', 'Gyro', 'Mag',
            %                           '9DOF', 'ECG', 'EMG', 'EXG', 'GSR' and
            %                           'Strain Gauge'.
            %
            %   EXAMPLE: internalBoard = shimmer1.getinternalboard;
            %
            %   See also setinternalboard getexternalboard
            
            internalBoard = thisShimmer.InternalBoard;
        end
                
        function externalBoard = getexternalboard(thisShimmer)
            %GETEXTERNALBOARD - Get the external board setting of the Shimmer
            %
            %   EXTERNALBOARD = GETEXTERNALBOARD returns the external board
            %   setting of the Shimmer.
            %
            %   SYNOPSIS: externalBoard = thisShimmer.getexternalboard()
            %
            %   OUTPUT: externalBoard - String value defining the external
            %                           daughter board of the Shimmer.
            %                           Valid values are 'None', 'ExpBoard' and
            %                           'Heart Rate'.
            %
            %   EXAMPLE: externalBoard = shimmer1.getexternalboard;
            %
            %   See also setexternalboard getinternalboard
            
            externalBoard = thisShimmer.ExternalBoard;
        end
        
        function orientation3D = getorientation3D(thisShimmer)
            %GETORIENTATION3D - Get the orientation3D setting of the Shimmer
            %
            %   ORIENTATION3D = GETORIENTATION3D returns the orientation3D
            %   setting of the Shimmer.
            %
            %   SYNOPSIS: orientation3D = thisShimmer.getorientation3D()
            %
            %   OUTPUT: orientation3D - Numeric value defining the orientation3D
            %   setting, 1=enabled , 0=disabled
            %
            %   EXAMPLE: orientation3D = shimmer1.getorientation3D;
            %
            %   See also setorientation3D
            
            orientation3D = thisShimmer.Orientation3D;
        end
        
        function isRead = getcalibrationparameters(thisShimmer,sensorName)
            %GETCALIBRATIONPARAMETERS - retrieves the calibration parameters
            %stored on the Shimmer device for either the accelerometer,
            %gyroscope or magnetometer.
            %
            %
            %   ISREAD = GETCALIBRATIONPARAMETERS(SENSORNAME) retrieves
            %   calibration parameters for the snensor defined by SENSORNAME
            %
            %   SYNOPSIS: isRead = thisShimmer.getcalibrationparameters(sensorName)
            %
            %   INPUT: sensorName - String value that defines the sensor
            %                       sensorName = 'Accelerometer' retrieves calibration parameters for the accelerometer
            %                       sensorName = 'Gyroscope' retrieves calibration parameters for the gyroscope
            %                       sensorName = 'Magnetometer' retrieves calibration parameters for the magnetometer
            %                       sensorName = 'ECG' retrieves calibration parameters for ECG
            %                       sensorName = 'EMG' retrieves calibration parameters for EMG
            %
            %   OUTPUT: isRead - Returns a boolean value indicating the status of the read
            %                    isRead = true means that there are calibration parameters on the Shimmer
            %                    isRead = false means that there are no calibration parameters on the Shimmer,thus default parameters are used
            %
            %   EXAMPLE: isRead = shimmer1.getcalibrationparameters('Accelerometer');
            %
            
            skip=0;
            if strcmp(thisShimmer.getstate,'Streaming')
                disp('Please stop streaming');
            else
                clearreaddatabuffer(thisShimmer);
                if strcmp('Accelerometer',sensorName)
                    writetocomport(thisShimmer,thisShimmer.GET_ACCEL_CALIBRATION_PARAMETERS_COMMAND);    % Send command to get Accelerometer Calibration Data
                elseif strcmp('Gyroscope',sensorName)
                    writetocomport(thisShimmer,thisShimmer.GET_GYRO_CALIBRATION_PARAMETERS_COMMAND);    % Send command to get Gyroscope Calibration Data
                elseif strcmp('Magnetometer',sensorName)
                    writetocomport(thisShimmer,thisShimmer.GET_MAGNE_CALIBRATION_PARAMETERS_COMMAND);    % Send command to get Magnetometer Calibration Data
                elseif strcmp('All',sensorName)
                    if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                        writetocomport(thisShimmer,thisShimmer.GET_ALL_CALIBRATION_COMMAND);    % Send command to get ALL Calibration Data
                    else
                        writetocomport(thisShimmer,thisShimmer.GET_ALL_CALIBRATION_COMMAND);    % Send command to get ALL Calibration Data
                    end
                elseif strcmp('ECG',sensorName)
                    if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        writetocomport(thisShimmer,thisShimmer.GET_ECG_CALIBRATION_COMMAND);    % Send command to get ECG Calibration Data
                    else
                        disp('Warning: getcalibrationparameters - ECG calibration parameters not yet supported for Shimmer3.');
                        skip=1;
                        isRead=false;
                    end
                elseif strcmp('EMG',sensorName)
                    if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        writetocomport(thisShimmer,thisShimmer.GET_EMG_CALIBRATION_COMMAND);    % Send command to get EMG Calibration Data
                    else
                        disp('Warning: getcalibrationparameters - EMG calibration parameters not yet supported for Shimmer3.');
                        skip=1;
                        isRead=false;
                    end
                else
                    disp('Warning: getcalibrationparameters - Wrong sensor name given');
                    skip=1;
                end
                
                if(skip==0)
                    
                    isStarted = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);    % Wait for Acknowledgment from Shimmer
                    
                    if(isStarted)
                        
                        serialData = [];
                        
                        if (strcmp('Accelerometer',sensorName)||strcmp('All',sensorName))
                            nIterations = 0;
                            while(length(serialData) < 22 && nIterations < 4 )
                                [tempSerialData, isFileOpen] = readdatabuffer(thisShimmer, inf);  % Read all available serial data from the com port
                                serialData = [serialData; tempSerialData];
                                pause(.2);
                                nIterations = nIterations + 1;
                            end
                            [OffsetVector,AlignmentMatrix,SensitivityMatrix]=retrievekinematicparameters(thisShimmer,serialData(2:22));
                            serialData(2:22)=[];
                            %Verify calibration parameters,
                            if SensitivityMatrix(1)== -1                     % Check for calibration parameters on Shimmer
                                str = sprintf('Warning: getcalibrationparameters - Calibration parameters for accelerometer not found for Shimmer on Com Port %s, default values will be used', thisShimmer.ComPort);
                                thisShimmer.DefaultAccelCalibrationParameters=true;
                                disp(str);
                                if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                                    thisShimmer.AccelCalParametersOV = thisShimmer.AccelCalParametersOVShimmer2;
                                    thisShimmer.AccelCalParametersAM = thisShimmer.AccelCalParametersAMShimmer2;
                                    %   check accel range, valid range setting values for the Shimmer
                                    %   2 are 0 (+/- 1.5g), 1 (+/- 2g), 2 (+/- 4g)and 3 (+/- 6g)
                                    if thisShimmer.getaccelrange==0
                                        thisShimmer.AccelCalParametersSM = thisShimmer.AccelCalParametersSM1p5gShimmer2;
                                    end

                                    if thisShimmer.getaccelrange==1
                                        thisShimmer.AccelCalParametersSM = thisShimmer.AccelCalParametersSM2gShimmer2;
                                    end

                                    if thisShimmer.getaccelrange==2
                                        thisShimmer.AccelCalParametersSM = thisShimmer.AccelCalParametersSM4gShimmer2;
                                    end

                                    if thisShimmer.getaccelrange==3
                                        thisShimmer.AccelCalParametersSM = thisShimmer.AccelCalParametersSM6gShimmer2;
                                    end
                                else
                                    if thisShimmer.HardwareCompatibilityCode < 2
                                        thisShimmer.AccelCalParametersSM = thisShimmer.AccelLowNoiseCalParametersSM2gShimmer3;
                                        thisShimmer.AccelCalParametersOV = thisShimmer.AccelLowNoiseCalParametersOVShimmer3;
                                    else
                                        thisShimmer.AccelCalParametersSM = thisShimmer.AccelLowNoiseCalParametersSM2gShimmer3_2;
                                        thisShimmer.AccelCalParametersOV = thisShimmer.AccelLowNoiseCalParametersOVShimmer3_2;
                                    end
                                    thisShimmer.AccelCalParametersAM = thisShimmer.AccelLowNoiseCalParametersAMShimmer3;
                                end
                                isRead=false;
                            else
                                thisShimmer.UsingDefaultAccelCalParams=0;
                                thisShimmer.AccelCalParametersOV=OffsetVector;
                                thisShimmer.AccelCalParametersSM=SensitivityMatrix;
                                thisShimmer.AccelCalParametersAM=AlignmentMatrix;
                                thisShimmer.DefaultAccelCalibrationParameters=false;
                                isRead=true;
                            end
                        end
                        if (strcmp('Gyroscope',sensorName)||strcmp('All',sensorName))
                            nIterations = 0;
                            while(length(serialData) < 22 && nIterations < 4 )
                                [tempSerialData, isFileOpen] = readdatabuffer(thisShimmer, inf);  % Read all available serial data from the com port
                                serialData = [serialData; tempSerialData];
                                pause(.2);
                                nIterations = nIterations + 1;
                            end
                            [OffsetVector,AlignmentMatrix,SensitivityMatrix]=retrievekinematicparameters(thisShimmer,serialData(2:22));
                            serialData(2:22)=[];
                            if SensitivityMatrix(1)== -1                       % Check for calibration parameters on Shimmer
                                str = sprintf('Warning: getcalibrationparameters - Calibration parameters for gyroscope not found for Shimmer on Com Port %s, default values will be used', thisShimmer.ComPort);
                                disp(str);
                                thisShimmer.DefaultGyroCalibrationParameters=true;
                                if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                                    thisShimmer.GyroCalParametersOV=thisShimmer.GyroCalParametersOVShimmer2;
                                    thisShimmer.GyroCalParametersSM=thisShimmer.GyroCalParametersSMShimmer2;
                                    thisShimmer.GyroCalParametersAM=thisShimmer.GyroCalParametersAMShimmer2;
                                else
                                    thisShimmer.GyroCalParametersOV=thisShimmer.GyroCalParametersOVShimmer3;
                                    thisShimmer.GyroCalParametersAM=thisShimmer.GyroCalParametersAMShimmer3;
                                    if (thisShimmer.getgyrorange == 3)
                                        thisShimmer.GyroCalParametersSM=thisShimmer.GyroCalParametersSM2000dpsShimmer3;
                                    elseif (thisShimmer.getgyrorange == 2)
                                        thisShimmer.GyroCalParametersSM=thisShimmer.GyroCalParametersSM1000dpsShimmer3;
                                    elseif (thisShimmer.getgyrorange == 1)
                                        thisShimmer.GyroCalParametersSM=thisShimmer.GyroCalParametersSM500dpsShimmer3;
                                    elseif (thisShimmer.getgyrorange == 0)
                                        thisShimmer.GyroCalParametersSM=thisShimmer.GyroCalParametersSM250dpsShimmer3;
                                    end
                                end
                                isRead=false;
                            else
                                thisShimmer.GyroCalParametersOV=OffsetVector;
                                thisShimmer.GyroCalParametersSM=SensitivityMatrix./100;
                                thisShimmer.GyroCalParametersAM=AlignmentMatrix;
                                thisShimmer.DefaultGyroCalibrationParameters=false;
                                isRead=true;
                            end
                        end
                        if (strcmp('Magnetometer',sensorName)||strcmp('All',sensorName))
                            nIterations = 0;
                            while(length(serialData) < 22 && nIterations < 4)
                                [tempSerialData, isFileOpen] = readdatabuffer(thisShimmer, inf);  % Read all available serial data from the com port
                                serialData = [serialData; tempSerialData];
                                pause(.2);
                                nIterations = nIterations + 1;
                            end
                            [OffsetVector,AlignmentMatrix,SensitivityMatrix]=retrievekinematicparameters(thisShimmer,serialData(2:22));
                            serialData(2:22)=[];
                            if SensitivityMatrix(1)== -1                        % Check for calibration parameters on Shimmer
                                str = sprintf('Warning: getcalibrationparameters - Calibration parameters for magnetometer not found for Shimmer on Com Port %s, default values will be used', thisShimmer.ComPort);
                                disp(str);
                                thisShimmer.DefaultMagneCalibrationParameters=true;
                                if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                                    thisShimmer.MagneCalParametersOV=thisShimmer.MagneCalParametersOVShimmer2;
                                    thisShimmer.MagneCalParametersSM=thisShimmer.MagneCalParametersSMShimmer2;
                                    thisShimmer.MagneCalParametersAM=thisShimmer.MagneCalParametersAMShimmer2;
                                else
                                    if thisShimmer.HardwareCompatibilityCode < 2
                                        thisShimmer.MagneCalParametersOV=thisShimmer.MagneCalParametersOVShimmer3;
                                        thisShimmer.MagneCalParametersAM=thisShimmer.MagneCalParametersAMShimmer3;
                                        if (thisShimmer.getmagrange == 1)
                                            thisShimmer.MagneCalParametersSM=thisShimmer.MagneCalParametersSM1_3gaussShimmer3;
                                        elseif (thisShimmer.getmagrange == 2)
                                            thisShimmer.MagneCalParametersSM=thisShimmer.MagneCalParametersSM1_9gaussShimmer3;
                                        elseif (thisShimmer.getmagrange == 3)
                                            thisShimmer.MagneCalParametersSM=thisShimmer.MagneCalParametersSM2_5gaussShimmer3;
                                        elseif (thisShimmer.getmagrange == 4)
                                            thisShimmer.MagneCalParametersSM=thisShimmer.MagneCalParametersSM4_0gaussShimmer3;
                                        elseif (thisShimmer.getmagrange == 5)
                                            thisShimmer.MagneCalParametersSM=thisShimmer.MagneCalParametersSM4_7gaussShimmer3;
                                        elseif (thisShimmer.getmagrange == 6)
                                            thisShimmer.MagneCalParametersSM=thisShimmer.MagneCalParametersSM5_6gaussShimmer3;
                                        elseif (thisShimmer.getmagrange == 7)
                                            thisShimmer.MagneCalParametersSM=thisShimmer.MagneCalParametersSM8_1gaussShimmer3;
                                        end
                                    else
                                        thisShimmer.MagneCalParametersSM = thisShimmer.MagneCalParametersSM49_2gaussShimmer3;
                                    end
                                end
                                isRead=false;
                            else
                                thisShimmer.MagneCalParametersOV=OffsetVector;
                                thisShimmer.MagneCalParametersSM=SensitivityMatrix;
                                thisShimmer.MagneCalParametersAM=AlignmentMatrix;
                                thisShimmer.DefaultMagneCalibrationParameters=false;
                                isRead=true;
                            end
                        end
                        
                        %read the second set of calibration parameters if
                        %this is a Shimmer3
                        
                        if (strcmp('All',sensorName)&& thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            nIterations = 0;
                            while(length(serialData) < 22 && nIterations < 4)
                                [tempSerialData, isFileOpen] = readdatabuffer(thisShimmer, inf);  % Read all available serial data from the com port
                                serialData = [serialData; tempSerialData];
                                pause(.2);
                                nIterations = nIterations + 1;
                            end
                            [OffsetVector,AlignmentMatrix,SensitivityMatrix]=retrievekinematicparameters(thisShimmer,serialData(2:22));
                            serialData(2:22)=[];
                            %Verify calibration parameters,
                            if SensitivityMatrix(1)== -1                     % Check for calibration parameters on Shimmer
                                str = sprintf('Warning: getcalibrationparameters - Calibration parameters for (wide range) accelerometer not found for Shimmer on Com Port %s, default values will be used', thisShimmer.ComPort);
                                thisShimmer.DefaultDAccelCalibrationParameters = true;
                                disp(str);
                                if thisShimmer.HardwareCompatibilityCode < 2
                                    thisShimmer.DAccelCalParametersAM = thisShimmer.AccelWideRangeCalParametersAMShimmer3;
                                    thisShimmer.DAccelCalParametersOV = thisShimmer.AccelWideRangeCalParametersOVShimmer3;
                                    if thisShimmer.getaccelrange==0
                                        thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM2gShimmer3;
                                    end
                                    if thisShimmer.getaccelrange==1
                                        thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM4gShimmer3;
                                    end
                                    if thisShimmer.getaccelrange==2
                                        thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM8gShimmer3;
                                    end
                                    if thisShimmer.getaccelrange==3
                                        thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM16gShimmer3;
                                    end
                                elseif thisShimmer.HardwareCompatibilityCode >= 2
                                    thisShimmer.DAccelCalParametersAM = thisShimmer.AccelWideRangeCalParametersAMShimmer3_2;
                                    thisShimmer.DAccelCalParametersOV = thisShimmer.AccelWideRangeCalParametersOVShimmer3_2;
                                    if thisShimmer.getaccelrange==0
                                        thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM2gShimmer3_2;
                                    end
                                    if thisShimmer.getaccelrange==1
                                        thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM16gShimmer3_2;
                                    end
                                    if thisShimmer.getaccelrange==2
                                        thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM4gShimmer3_2;
                                    end
                                    if thisShimmer.getaccelrange==3
                                        thisShimmer.DAccelCalParametersSM = thisShimmer.AccelWideRangeCalParametersSM8gShimmer3_2;
                                    end
                                end
                                isRead=false;
                            else
                                thisShimmer.DAccelCalParametersOV = OffsetVector;
                                thisShimmer.DAccelCalParametersSM = SensitivityMatrix;
                                thisShimmer.DAccelCalParametersAM = AlignmentMatrix;
                                thisShimmer.DefaultDAccelCalibrationParameters = false;
                                isRead=true;
                            end
                        end
                        
                        if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                            if (strcmp('EMG',sensorName)||strcmp('All',sensorName))
                                nIterations = 0;
                                while(length(serialData) < 5 && nIterations < 4)
                                    [tempSerialData, isFileOpen] = readdatabuffer(thisShimmer, inf);  % Read all available serial data from the com port
                                    serialData = [serialData; tempSerialData];
                                    pause(.2);
                                    nIterations = nIterations + 1;
                                end
                                if serialData(3)~= 255                        % Check for calibration parameters on Shimmer
                                    thisShimmer.DefaultEMGCalibrationParameters=false;
                                    thisShimmer.EMGOffset=double(bitshift(double(serialData(2)),8)+double(serialData(3)));
                                    thisShimmer.EMGGain=bitshift(double(serialData(4)),8)+double(serialData(5));
                                else
                                    str = sprintf('Warning: getcalibrationparameters - Calibration parameters for EMG not found for Shimmer on Com Port %s, default values will be used', thisShimmer.ComPort);
                                    disp(str);
                                    thisShimmer.DefaultEMGCalibrationParameters=true;
                                    thisShimmer.EMGOffset = 2060;
                                    thisShimmer.EMGGain = 750;
                                end
                                serialData(2:5)=[];
                            end
                            if (strcmp('ECG',sensorName)||strcmp('All',sensorName))
                                nIterations = 0;
                                while(length(serialData) < 9 && nIterations < 4)
                                    [tempSerialData, isFileOpen] = readdatabuffer(thisShimmer, inf);  % Read all available serial data from the com port
                                    serialData = [serialData; tempSerialData];
                                    pause(.2);
                                    nIterations = nIterations + 1;
                                end
                                if serialData(3)~= 255                        % Check for calibration parameters on Shimmer
                                    thisShimmer.DefaultECGCalibrationParameters=false;
                                    thisShimmer.ECGLALLOffset=bitshift(double(serialData(2)),8)+double(serialData(3));
                                    thisShimmer.ECGLALLGain=bitshift(double(serialData(4)),8)+double(serialData(5));
                                    thisShimmer.ECGRALLOffset=bitshift(double(serialData(6)),8)+double(serialData(7));
                                    thisShimmer.ECGRALLGain=bitshift(double(serialData(8)),8)+double(serialData(9));
                                else
                                    str = sprintf('Warning: getcalibrationparameters - Calibration parameters for ECG not found for Shimmer on Com Port %s, default values will be used', thisShimmer.ComPort);
                                    disp(str);
                                    thisShimmer.DefaultECGCalibrationParameters=true;
                                    thisShimmer.ECGLALLOffset = 2060;
                                    thisShimmer.ECGLALLGain = 175;
                                    thisShimmer.ECGRALLOffset = 2060;
                                    thisShimmer.ECGRALLGain = 175;
                                end
                            end
                        end                        
                    end
                end
            end
        end % get calibration parameters       
        
        function signalNameArray = getenabledsignalnames(thisShimmer)
            %GETENABLEDSIGNALNAMES - Get the names of the enabled sensor signals
            %
            %   SIGNALNAMEARRAY = GETENABLEDSIGNALNAMES returns the names of
            %   all signals enabled on the Shimmer.
            %
            %   SYNOPSIS: signalNameArray = thisShimmer.getenabledsignalnames()
            %
            %   OUTPUT: signalNameArray - cell array containing the names of all
            %                             signals enabled on the Shimmer. The
            %                             indices of the signal names correspond
            %                             to the indices of the columns of data
            %                             returned by the method GETDATA.
            %
            %   EXAMPLE: signalNameArray = shimmer1.getenabledsignalnames;
            %
            %   See also setenabledsensors getsignalname disableallsensors
            %   getsignalindex SetEnabledSensorsMacrosClass
                                   
            signalNameArray = thisShimmer.SignalNameArray;
            
            % if pmux =1 and ExpBoard is listed change it to
            % VSenseReg/VSenseBatt
            if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                if (thisShimmer.getpmux==1)
                    for i=1:length(signalNameArray)
                        if strcmp(signalNameArray{i},'ExpBoard A7')
                            signalNameArray{i}='VSenseBatt';
                        elseif strcmp(signalNameArray{i},'ExpBoard A0')
                            signalNameArray{i}='VSenseReg';
                        end
                        
                    end
                end
            end
        end
               
        function signalName = getsignalname(thisShimmer,iSignal)
            %GETSIGNALNAME - Get the name of a sensor signal
            %
            %   SIGNALNAME = GETSIGNALNAME(ISIGNAL) returns the name of the
            %   sensor signal corresponding to the index ISIGNAL.
            %
            %   SYNOPSIS: signalName = thisShimmer.getsignalname(iSignal)
            %
            %   INPUT: iSignal - Signed non-zero integer value that defines the
            %                    index of the data signal of interest.
            %
            %   OUTPUT: signalName - String value containing the name of the signal
            %                        at index iSignal. The indices of the signal
            %                        names correspond to the indices of the columns
            %                        of data returned by the method GETDATA.
            %                        Valid values are 'Accelerometer X',
            %                        'Accelerometer Y', 'Accelerometer Z',
            %                        'Gyroscope X', 'Gyroscope Y', 'Gyroscope Z',
            %                        'Magnetometer X', 'Magnetometer Y',
            %                        'Magnetometer Z', 'ECG RA-LL', 'ECG LA-LL',
            %                        'GSR Raw', 'GSR Res', 'EMG', 'ExpBoard A0',
            %                        'ExpBoard A7', 'Strain Gauge High',
            %                        'Strain Gauge Low' and 'Heart Rate'.
            %
            %   EXAMPLE: signalName = shimmer1.getsignalname(2);
            %
            %   See also setenabledsensors getenabledsignalnames getsignalindex
            
            signalName = char(thisShimmer.SignalNameArray(iSignal));       % Determine the name of the signal in the column index defined in iSignal
        end
                
        function iSignal = getsignalindex(thisShimmer,signalName)
            %GETSIGNALINDEX - Get the index of a sensor signal
            %
            %   SIGNALINDEX = GETSIGNALINDEX(SIGNALNAME) returns the index of
            %   the sensor signal corresponding to the name SIGNALNAME.
            %
            %   SYNOPSIS: iSignal = thisShimmer.getsignalindex(signalName)
            %
            %   INPUT: signalName - String value that defines the name of the
            %                       data signal of interest.
            %                       Valid values are 'Accelerometer X',
            %                       'Accelerometer Y', 'Accelerometer Z',
            %                       'Gyroscope X', 'Gyroscope Y', 'Gyroscope Z',
            %                       'Magnetometer X', 'Magnetometer Y',
            %                       'Magnetometer Z', 'ECG RA-LL', 'ECG LA-LL',
            %                       'GSR Raw', 'GSR Res', 'EMG', 'ExpBoard A0',
            %                       'ExpBoard A7', 'Strain Gauge High',
            %                       'Strain Gauge Low' and 'Heart Rate'.
            %
            %   OUTPUT: signalName - Signed non-zero integer value that defines
            %                        the index of the data signal of interest.
            %
            %   EXAMPLE: iSignal = shimmer1.getsignalindex('Accelerometer X');
            %
            %   See also setenabledsensors getenabledsignalnames getsignalname
            
            iSignal = find(strcmp(thisShimmer.SignalNameArray,signalName));% Determine the column index of the signal defined in signalName
        end       
        
               
        function uncalibratedData = getuncalibrateddata(thisShimmer)
            %GETUNCALIBRATEDDATA - Get uncalibrated data from the data buffer
            %(!!Legacy method, use getdata as alternative!!)
            %
            %   UNCALIBRATEDDATA = GETUNCALIBRATEDDATA returns a 2D array
            %   of uncalibrated data from the data buffer.
            %
            %   SYNOPSIS: uncalibratedData = thisShimmer.getuncalibrateddata()
            %
            %   OUTPUT: uncalibratedData - [m x n] array of uncalibrated data
            %                              read from the data buffer,
            %                              where m = number of data samples and
            %                              n = number of data signals.
            %
            %   EXAMPLE: uncalibratedData = shimmer1.getuncalibrateddata;
            %
            %   See also setenabledsensors getenabledsignalnames getsignalname getsignalindex
            if ~thisShimmer.WarningGetUncalibratedData
                fprintf('Warning: getuncalibrateddata - !!Legacy method, please use getdata as alternative!!\n')
                thisShimmer.WarningGetUncalibratedData = 1;
            end
            if (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                [parsedData,~] = capturedata(thisShimmer);
                uncalibratedData = parsedData;
            else
                uncalibratedData = [];
                fprintf(strcat('Warning: getuncalibrateddata - Cannot get data for COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
        end
        
        function [sensorData,signalName,signalFormat,signalUnit] = deprecatedgetdata(thisShimmer,varargin)
            %DEPRECATEDGETDATA - Get sensor data from the data buffer and
            %calibrates them depending on user instructions
            %
            %   SENSORDATA = DEPRECATEDGETDATA returns a 2D array of sensor
            %   data from the data buffer and the corresponding signal
            %   names
            %
            %   SYNOPSIS: [sensorData,signalName,signalFormat,signalUnit] = thisShimmer.deprecatedgetdata()
            %
            %   INPUT: varargin - Multi-variable value defining the SENSORNAME
            %                     followed by the DATAMODE where DATAMODE =
            %                     'a' retrieves data in both calibrated and
            %                     uncalibrated format DATAMODE = 'u'
            %                     retrieves data in uncalibrated format
            %                     DATAMODE = 'c' retrieves data in
            %                     calibrated format e.g.get
            %                     (SENSORNAME1,DATAMODE1,SENSORNAME2,DATAMODE2)
            %                     Valid values for SENSORNAME are
            %                     'Accelerometer', 'Gyroscope',
            %                     'Magnetometer', 'ECG', 'EMG', 'EXG',
            %                     'GSR' 'ExpBoard', 'Strain Gauge', 'Bridge
            %                     Amplifier', 'Heart Rate','ADC','Pressure'
            %                     and 'Battery Voltage'. Please note that
            %                     'Strain Gauge' is only support for
            %                     Shimmer2/2r, while 'Bridge Amplifier' and
            %                     Pressure are only supported for Shimmer3
            %
            %   OUTPUT: sensorData - [m x n] array of sensor data
            %                              read from the data buffer, where
            %                              m = number of data samples and n
            %                              = number of data signals. Output
            %                              will depend on the data mode.
            %
            %           signalName - a 1 dimensional cell array of the
            %                        signal names. The index of each signal
            %                        name corresponds to the index of the
            %                        sensor data. 
            %
            %           signalFormat - a 1 dimensional cell array of the
            %                        signal names. The index of each signal
            %                        format corresponds to the index of the
            %                        sensor data. 
            %
            %           signalUnit - a 1 dimensional cell array of the
            %                        signal units. The index of each signal
            %                        unit corresponds to the index of the
            %                        sensor data. Signal units end with a
            %                        '*' when default calibration
            %                        parameters are used.
            %
            %   EXAMPLE:  [newData,signalNameArray,signalFormatArray,signalUnitArray] = shimmer1.deprecatedgetdata('Time Stamp','c','Accelerometer','c','Gyroscope','c','Magnetometer','u');
            %
            %   See also setenabledsensors getenabledsignalnames
            %   getsignalname getsignalindex getpercentageofpacketsreceived
            %   getdata
            
            iArg=1;
            if ~thisShimmer.WarningGetDeprecatedGetData
                fprintf('Warning: deprecatedgetdata - !!Legacy method, please use getdata as alternative!!\n');
                thisShimmer.WarningGetDeprecatedGetData = 1;
            end
            [parsedData,~] = capturedata(thisShimmer);
            parsedData = double(parsedData);
            if (~isempty(parsedData))
                sensorData=double([]);
                signalName=[];
                signalFormat=[];
                signalUnit=[];
                while iArg<length(varargin)
                    property= char(varargin(iArg));
                    dataMode= char(varargin(iArg+1));
                    
                    switch (property)
                        case('Accelerometer')
                            if  (bitand(thisShimmer.EnabledSensors, hex2dec('80'))>0 || (bitand(thisShimmer.EnabledSensors, hex2dec('1000'))>0 && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3))
                                if(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                                    [accelData,tempSignalName,tempSignalFormat,tempSignalUnit]=getacceldata(thisShimmer,dataMode,parsedData);
                                    sensorData=[sensorData accelData];
                                    signalName=[signalName tempSignalName];
                                    signalFormat=[signalFormat tempSignalFormat];
                                    signalUnit=[signalUnit tempSignalUnit];
                                elseif (bitand(thisShimmer.EnabledSensors, hex2dec('80'))>0 && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                                    [accelData,tempSignalName,tempSignalFormat,tempSignalUnit]=getlownoiseacceldata(thisShimmer,dataMode,parsedData);
                                    sensorData=[sensorData accelData];
                                    signalName=[signalName tempSignalName];
                                    signalFormat=[signalFormat tempSignalFormat];
                                    signalUnit=[signalUnit tempSignalUnit];
                                end
                                if (bitand(thisShimmer.EnabledSensors, hex2dec('1000'))>0 && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                                    [accelData,tempSignalName,tempSignalFormat,tempSignalUnit]=getwiderangeacceldata(thisShimmer,dataMode,parsedData);
                                    sensorData=[sensorData accelData];
                                    signalName=[signalName tempSignalName];
                                    signalFormat=[signalFormat tempSignalFormat];
                                    signalUnit=[signalUnit tempSignalUnit];
                                end
                            else
                                disp('Warning: getdata - Accelerometer is not enabled, see setenabledsensors.');
                            end
                        case('Gyroscope')
                            if  (bitand(thisShimmer.EnabledSensors, hex2dec('40'))>0)
                                [gyroData,tempSignalName,tempSignalFormat,tempSignalUnit]=getgyrodata(thisShimmer,dataMode,parsedData);
                                sensorData=[sensorData gyroData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                                if(thisShimmer.GyroInUseCalibration)
                                    if(dataMode == 'u')
                                        thisShimmer.GyroBuffer = [thisShimmer.GyroBuffer; gyroData];
                                    else
                                        [uncalibratedGyroData,~,~,~] = getgyrodata(thisShimmer,'u',parsedData);
                                        thisShimmer.GyroBuffer = [thisShimmer.GyroBuffer; uncalibratedGyroData];
                                    end
                                    bufferOverflow = size(thisShimmer.GyroBuffer,1) - thisShimmer.GyroBufferSize;
                                    if(bufferOverflow > 0)
                                        thisShimmer.GyroBuffer = thisShimmer.GyroBuffer((bufferOverflow+1):end,:);
                                    end
                                    if(nomotiondetect(thisShimmer))
                                        estimategyrooffset(thisShimmer);
                                    end
                                end
                            else
                                disp('Warning: getdata - Gyroscope is not enabled, see setenabledsensors.');
                            end
                        case('Magnetometer')
                            if  (bitand(thisShimmer.EnabledSensors, hex2dec('20'))>0)
                                [magData,tempSignalName,tempSignalFormat,tempSignalUnit]=getmagdata(thisShimmer,dataMode,parsedData);
                                sensorData=[sensorData magData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            else
                                disp('Warning: getdata - Magnetometer is not enabled, see setenabledsensors.');
                            end
                        case('EMG')
                            if (bitand(thisShimmer.EnabledSensors, hex2dec('8'))>0 && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                                [emgData,tempSignalName,tempSignalFormat,tempSignalUnit]= getemgdata(thisShimmer,dataMode,parsedData);
                                sensorData=[sensorData emgData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            elseif ((bitand(thisShimmer.EnabledSensors, hex2dec('10'))>0 || bitand(thisShimmer.EnabledSensors, hex2dec('100000'))>0) && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                                [emgData,tempSignalName,tempSignalFormat,tempSignalUnit] = getexgdata(thisShimmer,dataMode,parsedData);
                                sensorData=[sensorData emgData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            else
                                disp('Warning: getdata - EMG is not enabled, see setenabledsensors.');
                            end
                        case('ECG')
                            if  (bitand(thisShimmer.EnabledSensors, hex2dec('10')) >0 || bitand(thisShimmer.EnabledSensors, hex2dec('100000'))>0)
                                if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                                    [ecgData,tempSignalName,tempSignalFormat,tempSignalUnit] = getexgdata(thisShimmer,dataMode,parsedData);
                                else
                                    [ecgData,tempSignalName,tempSignalFormat,tempSignalUnit]=getecgdata(thisShimmer,dataMode,parsedData);
                                end
                                sensorData=[sensorData ecgData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            else
                                disp('Warning: getdata - ECG is not enabled, see setenabledsensors.');
                            end
                        case('EXG')
                            if  (bitand(thisShimmer.EnabledSensors, hex2dec('10'))>0 || bitand(thisShimmer.EnabledSensors, hex2dec('100000'))>0)
                                [exgData,tempSignalName,tempSignalFormat,tempSignalUnit]=getexgdata(thisShimmer,dataMode,parsedData);
                                sensorData=[sensorData exgData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            else
                                disp('Warning: getdata - EXG is not enabled, see setenabledsensors.');
                            end
                        case('GSR')
                            if  (bitand(thisShimmer.EnabledSensors, hex2dec('4'))>0)
                                if(ischar(thisShimmer.GsrRange))
                                    disp('Warning: getdata - GSR range undefined, see setgsrrange().');
                                else
                                    [gsrData,tempSignalName,tempSignalFormat,tempSignalUnit]=getgsrdata(thisShimmer,dataMode,parsedData);
                                    sensorData=[sensorData gsrData];
                                    signalName=[signalName tempSignalName];
                                    signalFormat=[signalFormat tempSignalFormat];
                                    signalUnit=[signalUnit tempSignalUnit];
                                end
                            else
                                disp('Warning: getdata - GSR is not enabled, see setenabledsensors.');
                            end
                        case({'Strain Gauge', 'Bridge Amplifier'})
                            if (bitand(thisShimmer.EnabledSensors, hex2dec('8000'))>0 && (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3))
                                [strainGaugeData,tempSignalName,tempSignalFormat,tempSignalUnit]=getstraingaugedata(thisShimmer,dataMode,parsedData);
                                sensorData=[sensorData strainGaugeData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            elseif (bitand(thisShimmer.EnabledSensors, hex2dec('8000'))>0 && (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3) && thisShimmer.FirmwareCompatibilityCode >= 4)
                                [bridgeAmplifierData,tempSignalName,tempSignalFormat,tempSignalUnit]=getbridgeamplifierdata(thisShimmer,dataMode,parsedData);
                                sensorData=[sensorData bridgeAmplifierData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            elseif (bitand(thisShimmer.EnabledSensors, hex2dec('8000'))<0 && (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3))
                                disp('Warning: getdata - Strain Gauge is not enabled, see setenabledsensors.');
                            elseif (bitand(thisShimmer.EnabledSensors, hex2dec('8000'))<0 && (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3) && thisShimmer.FirmwareCompatibilityCode >= 4)
                                disp('Warning: getdata - Bridge Amplifier is not enabled, see setenabledsensors.');
                            elseif (thisShimmer.FirmwareCompatibilityCode < 4 && (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3))
                                disp('Warning: getdata - Bridge Amplifier is not supported for this firmware version, please update firmware.'); 
                            end
                        case('Heart Rate')
                            if (bitand(thisShimmer.EnabledSensors, hex2dec('4000'))>0 && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                                [heartRateData,tempSignalName,tempSignalFormat,tempSignalUnit]=getheartratedata(thisShimmer,parsedData);
                                sensorData=[sensorData heartRateData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            else
                                disp('Warning: getdata - HeartRate is not enabled, see setenabledsensors');
                                disp('or Shimmer is not a Shimmer2/2r.');
                            end
                        case('ExpBoard')
                            if (bitand(thisShimmer.EnabledSensors, hex2dec('2'))>0 || bitand(thisShimmer.EnabledSensors, hex2dec('1'))>0)
                                [expBoardData,tempSignalName,tempSignalFormat,tempSignalUnit]=getexpboarddata(thisShimmer,dataMode,parsedData);
                                sensorData=[sensorData expBoardData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            else
                                disp('Warning: getdata - ExpBoard is not enabled, see setenabledsensors.');
                            end
                        case('Time Stamp')
                            [timeStampData,tempSignalName,tempSignalFormat,tempSignalUnit]=gettimestampdata(thisShimmer,dataMode,parsedData);
                            sensorData=[sensorData timeStampData];
                            signalName=[signalName tempSignalName];
                            signalFormat=[signalFormat tempSignalFormat];
                            signalUnit=[signalUnit tempSignalUnit];
                        case('Battery Voltage')
                            if ((bitand(thisShimmer.EnabledSensors, hex2dec('2000'))>0 && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3) || thisShimmer.getpmux==1)
                                if ((bitand(thisShimmer.EnabledSensors, hex2dec('2'))>0 || bitand(thisShimmer.EnabledSensors, hex2dec('1'))>0) || (bitand(thisShimmer.EnabledSensors, hex2dec('2000'))>0 && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3))
                                    [battVoltData,tempSignalName,tempSignalFormat,tempSignalUnit]=getbattvoltdata(thisShimmer,dataMode,parsedData);
                                    %take average of batt volt data and send a command to change LED if it is reaching low power
                                    sensorData=[sensorData battVoltData];
                                    signalName=[signalName tempSignalName];
                                    signalFormat=[signalFormat tempSignalFormat];
                                    signalUnit=[signalUnit tempSignalUnit];
                                else
                                    disp('Warning: getdata - Battery Voltage is not enabled, see setenabledsensors.');
                                end
                            else
                                disp('Warning: getdata - Battery Voltage is not enabled, see setenabledsensors.');
                            end
                        case('Quaternion')
                            if (bitand(thisShimmer.EnabledSensors, hex2dec('E0'))>0) % Accel, Gyro and Mag enabled
                                [accelData,~,~,~]=getacceldata(thisShimmer,'c',parsedData);
                                [gyroData,~,~,~]=getgyrodata(thisShimmer,'c',parsedData);
                                [magData,~,~,~]=getmagdata(thisShimmer,'c',parsedData);
                                [quaternionData,tempSignalName,tempSignalFormat,tempSignalUnit]=getquaterniondata(thisShimmer,'c',accelData,gyroData,magData);
                                
                                sensorData=[sensorData quaternionData];
                                signalName=[signalName tempSignalName];
                                signalFormat=[signalFormat tempSignalFormat];
                                signalUnit=[signalUnit tempSignalUnit];
                            else
                                disp('Warning: getdata - Cannot estimate quaternions because accelerometer, gyroscope and/or magnetometer are not enabled, see setenabledsensors.');
                            end
                        case('ADC')
                            if (thisShimmer.ShimmerVersion==thisShimmer.SHIMMER_3)
                                if (bitand(thisShimmer.EnabledSensors, hex2dec('800f03'))>0)
                                    [adcData,tempSignalName,tempSignalFormat,tempSignalUnit]=getadcdata(thisShimmer,dataMode,parsedData);
                                    sensorData=[sensorData adcData];
                                    signalName=[signalName tempSignalName];
                                    signalFormat=[signalFormat tempSignalFormat];
                                    signalUnit=[signalUnit tempSignalUnit];
                                else
                                    disp('Warning: getdata - No ADCs have been enabled, see setenabledsensors command.');
                                end
                            else
                                disp('Warning: getdata - This command is only supported on Shimmer3.');
                            end
                        case('Pressure')
                            if (thisShimmer.FirmwareCompatibilityCode < 2 && thisShimmer.ShimmerVersion==thisShimmer.SHIMMER_3)
                                disp('Warning: getdata - Pressure sensor not supported for this firmware version, please update firmware.');
                            elseif (thisShimmer.ShimmerVersion==thisShimmer.SHIMMER_3)
                                if (bitand(thisShimmer.EnabledSensors, hex2dec('40000'))>0)
                                    [pressureData,tempSignalName,tempSignalFormat,tempSignalUnit]=getpressuredata(thisShimmer,dataMode,parsedData);
                                    sensorData=[sensorData pressureData];
                                    signalName=[signalName tempSignalName];
                                    signalFormat=[signalFormat tempSignalFormat];
                                    signalUnit=[signalUnit tempSignalUnit];
                                else
                                    disp('Warning: getdata - Pressure sensor not enabled, see setenabledsensors command.');
                                end
                            else
                                disp('Warning: getdata - This command is only supported on Shimmer3.');
                            end
                    end
                    iArg=iArg+2;
                end
            else
                sensorData=double([]);
                signalName=[];
                signalFormat=[];
                signalUnit=[];
            end 
            
        end % function getdeprecateddata
        
        function [sensorData,signalName,signalFormat,signalUnit] = getdata(thisShimmer,varargin)
            %GETDATA - Get sensor data from the data buffer and calibrates
            %them depending on user instructions
            %
            %   SENSORDATA = GETDATA returns a 2D array of sensor data from
            %   the data buffer and the corresponding signal names
            %
            %   SYNOPSIS: [sensorData,signalName,signalFormat,signalUnit] = thisShimmer.getdata(dataMode)
            %
            %   INPUT: dataMode - Value defining the DATAMODE where
            %                     DATAMODE = 'a' retrieves data in both calibrated
            %                     and uncalibrated format DATAMODE = 'u'
            %                     retrieves data in uncalibrated format
            %                     DATAMODE = 'c' retrieves data in
            %                     calibrated format.
            %
            %                     Valid values for DATAMODE are 'a', 'c'
            %                     and 'u'.
            %
            %   OUTPUT: sensorData - [m x n] array of sensor data
            %                              read from the data buffer, where
            %                              m = number of data samples and n
            %                              = number of data signals. Output
            %                              will depend on the data mode.
            % 
            %           signalName - a 1 dimensional cell array of the
            %                        signal names. The index of each signal
            %                        name corresponds to the index of the
            %                        sensor data. 
            %
            %           signalFormat - a 1 dimensional cell array of the
            %                        signal names. The index of each signal
            %                        format corresponds to the index of the
            %                        sensor data. 
            %
            %           signalUnit - a 1 dimensional cell array of the
            %                        signal units. The index of each signal
            %                        unit corresponds to the index of the
            %                        sensor data. Signal units end with a
            %                        '*' when default calibration
            %                        parameters are used.
            %
            %
            %   EXAMPLE:  [newData,signalNameArray,signalFormatArray,signalUnitArray] = shimmer1.getdata('c');
            %
            %   See also setenabledsensors getenabledsignalnames
            %   getsignalname getsignalindex getpercentageofpacketsreceived
            
            sensorData=double([]);
            signalName=[];
            signalFormat=[];
            signalUnit=[];
            
            if (nargin > 2)                                                % getdata requires only one argument, dataMode = a, u, or c.
                disp('Warning: getdata - Requires only one argument: dataMode = ''a'', ''u'', or ''c''.');
                disp('Warning: getdata - Getdata has changed since MATLAB ID v2.3;');
                disp('deprecatedgetdata() facilitates backwards compatibility');
            elseif ~(strcmp(varargin{1},'c') || strcmp(varargin{1},'u') || strcmp(varargin{1},'a'));
                disp('Warning: getdata - Valid arguments for getdata = ''a'', ''u'', or ''c''.');
            else
                dataMode = varargin{1};
                [parsedData,systemTime] = capturedata(thisShimmer);
                parsedData = double(parsedData);
                
                if (~isempty(parsedData))
                                    
                    numSignals = length(thisShimmer.SignalNameArray);      % get number of signals from signalNameArray
                                                      
                    s = 1;
                    while s <= numSignals;                                 % check signalNameArray for enabled signals
                        if strcmp(thisShimmer.SignalNameArray(s),'Timestamp');
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=gettimestampdata(thisShimmer,dataMode,parsedData); % Time Stamp
                            s = s+1;

                            if (thisShimmer.EnableTimestampUnix && strcmp(dataMode,'c'))
                                thisShimmer.LastSampleSystemTimeStamp = thisShimmer.convertMatlabTimeToUnixTimeMilliseconds(systemTime);
                                nSamp = size(tempData,1);
                                timeStampUnixData = zeros(nSamp,1);
                                timeStampUnixData(nSamp) = thisShimmer.LastSampleSystemTimeStamp;
                                for iUnix = 1:nSamp-1
                                    timeStampUnixData(iUnix,1)=NaN;
                                end
                                timeStampUnixSignalName = 'Time Stamp Unix';
                                timeStampUnixDataSignalFormat = 'CAL';
                                timeStampUnixSignalUnit = 'milliseconds'; 
                                tempData = [tempData timeStampUnixData];
                                tempSignalName = [tempSignalName timeStampUnixSignalName];
                                tempSignalFormat = [tempSignalFormat timeStampUnixDataSignalFormat];
                                tempSignalUnit = [tempSignalUnit timeStampUnixSignalUnit];
                            end
                        elseif strcmp(thisShimmer.SignalNameArray(s),'Low Noise Accelerometer X');
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getlownoiseacceldata(thisShimmer,dataMode,parsedData); % Shimmer3 only
                            s = s+3;            % skip Y and Z
                        elseif strcmp(thisShimmer.SignalNameArray(s),'Battery Voltage'); % Shimmer3 battery voltage
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getbattvoltdata(thisShimmer,dataMode,parsedData); %takes average of batt volt data and send a command to change LED if it is reaching low power
                            s = s+1;
                        elseif strcmp(thisShimmer.SignalNameArray(s),'Wide Range Accelerometer X');
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getwiderangeacceldata(thisShimmer,dataMode,parsedData); % Shimmer3 only
                            s = s+3;            % skip Y and Z
                        elseif strcmp(thisShimmer.SignalNameArray(s),'Accelerometer X');
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getacceldata(thisShimmer,dataMode,parsedData);
                            s = s+3;            % skip Y and Z
                        elseif strcmp(thisShimmer.SignalNameArray(s),'Gyroscope X');
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getgyrodata(thisShimmer,dataMode,parsedData);
                            if(thisShimmer.GyroInUseCalibration)
                                if(dataMode == 'u')
                                    thisShimmer.GyroBuffer = [thisShimmer.GyroBuffer; tempData];
                                else
                                    [uncalibratedGyroData,~,~,~] = getgyrodata(thisShimmer,'u',parsedData);
                                    thisShimmer.GyroBuffer = [thisShimmer.GyroBuffer; uncalibratedGyroData];
                                end
                                bufferOverflow = size(thisShimmer.GyroBuffer,1) - thisShimmer.GyroBufferSize;
                                if(bufferOverflow > 0)
                                    thisShimmer.GyroBuffer = thisShimmer.GyroBuffer((bufferOverflow+1):end,:);
                                end
                                if(nomotiondetect(thisShimmer))
                                    estimategyrooffset(thisShimmer); 
                                end
                            end
                            s = s+3;            % skip Y and Z
                        elseif strcmp(thisShimmer.SignalNameArray(s),'Magnetometer X');
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getmagdata(thisShimmer,dataMode,parsedData);
                            s = s+3;            % skip Y and Z
                        elseif strcmp(thisShimmer.SignalNameArray(s),'EMG');  % Shimmer2r EMG
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getemgdata(thisShimmer,dataMode,parsedData);
                            s = s+1;
                        elseif strcmp(thisShimmer.SignalNameArray(s),'ECG RA-LL'); % Shimmer2r ECG
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getecgdata(thisShimmer,dataMode,parsedData);
                            s = s+2;            % skip LA-LL
                        elseif strcmp(thisShimmer.SignalNameArray(s),'GSR Raw');   % GSR
                            if(ischar(thisShimmer.GsrRange))
                                disp('Warning: getdata - GSR range undefined, see setgsrrange().');
                            else
                                [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getgsrdata(thisShimmer,dataMode,parsedData);
                                s = s+1;
                                if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                                    s = s+1;     % skip GSR Res for Shimmer2r
                                end
                            end
                        elseif (strcmp(thisShimmer.SignalNameArray(s),'External ADC A7') || strcmp(thisShimmer.SignalNameArray(s),'External ADC A6') ||...  % Shimmer3 ADCs
                                strcmp(thisShimmer.SignalNameArray(s),'External ADC A15') || strcmp(thisShimmer.SignalNameArray(s),'Internal ADC A1') ||...
                                strcmp(thisShimmer.SignalNameArray(s),'Internal ADC A12') || strcmp(thisShimmer.SignalNameArray(s),'Internal ADC A13') ||...
                                strcmp(thisShimmer.SignalNameArray(s),'Internal ADC A14'));
                             
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getadcdata(thisShimmer,dataMode,parsedData);
                            thisShimmer.GetADCFlag = 1; % Set getadcdata flag so that getadcdata is only called once.
                            s = s + 1;
                        elseif (strcmp(thisShimmer.SignalNameArray(s),'Pressure') || strcmp(thisShimmer.SignalNameArray(s),'Temperature')); % Shimmer3 BMP180/BMP280 pressure and temperature)
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getpressuredata(thisShimmer,dataMode,parsedData);
                            s = s+2;
                        elseif (strcmp(thisShimmer.SignalNameArray(s),'EXG1 STA') || strcmp(thisShimmer.SignalNameArray(s),'EXG2 STA')); % Shimmer3 EXG
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getexgdata(thisShimmer,dataMode,parsedData);
                            if (size(tempData,2) == 2)
                                s = s+size(tempData,2)+1;  % EXG1 or EXG2 enabled.
                            else
                                s = s+size(tempData,2)+2;  % EXG1 and EXG2 enabled.
                            end
                        elseif strcmp(thisShimmer.SignalNameArray(s),'Bridge Amplifier High'); % Shimmer3 Bridge Amplifier
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getbridgeamplifierdata(thisShimmer,dataMode,parsedData);
                            s = s+2;
                        elseif strcmp(thisShimmer.SignalNameArray(s),'Strain Gauge High'); % Shimmer2r Strain Gauge
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getstraingaugedata(thisShimmer,dataMode,parsedData);
                            s = s+2;
                        elseif strcmp(thisShimmer.SignalNameArray(s),'Heart Rate'); % Heart Rate
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getheartratedata(thisShimmer,dataMode,parsedData);
                            s = s+1;
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && strcmp(thisShimmer.SignalNameArray(s),'ExpBoard A7') || strcmp(thisShimmer.SignalNameArray(s),'ExpBoard A0')); % Shimmer2/2r External Expansion Board / battery voltage
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getexpboarddata(thisShimmer,dataMode,parsedData);     % Shimmer2/2r External Expansion Board
                            if (thisShimmer.getpmux == 1)
                                [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getbattvoltdata(thisShimmer,dataMode,parsedData); % Shimmer2/2r battery voltage
                            end
                            s = s+size(tempData,2);
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && strcmp(thisShimmer.SignalNameArray(s),'VSenseBatt') || strcmp(thisShimmer.SignalNameArray(s),'VSenseReg')); % Shimmer2/2r battery voltage; if getenabledsignalnames is called, ExpBoard A7 / ExpBoard A0 is changed into VSenseBatt / VSenseReg, see getenabledsignalnames.
                            [tempData,tempSignalName,tempSignalFormat,tempSignalUnit]=getbattvoltdata(thisShimmer,dataMode,parsedData);
                            s = s+size(tempData,2);
                        end
                        sensorData=[sensorData tempData];
                        signalName=[signalName tempSignalName];
                        signalFormat=[signalFormat tempSignalFormat];
                        signalUnit=[signalUnit tempSignalUnit];
                    end
                    if (thisShimmer.Orientation3D && bitand(thisShimmer.EnabledSensors, hex2dec('E0'))>0 && strcmp(dataMode,'c')) % Get Quaternion data if Orientation3D setting and sensors Accel, Gyro and Mag are enabled.
                        [accelData,~,~,~]=getacceldata(thisShimmer,'c',parsedData);
                        [gyroData,~,~,~]=getgyrodata(thisShimmer,'c',parsedData);
                        [magData,~,~,~]=getmagdata(thisShimmer,'c',parsedData);
                        [quaternionData,tempSignalName,tempSignalFormat,tempSignalUnit]=getquaterniondata(thisShimmer,'c',accelData,gyroData,magData);
                        
                        sensorData=[sensorData quaternionData];
                        signalName=[signalName tempSignalName];
                        signalFormat=[signalFormat tempSignalFormat];
                        signalUnit=[signalUnit tempSignalUnit];
                    end
                end
            end
            thisShimmer.GetADCFlag = 0; % Reset getadcdata flag.
        end % function getdata
        
        
        function exgRate = getexgrate(thisShimmer, chipIdentifier) % function getexgrate 
            % GETEXGRATE - Get the exg rate of the Shimmer
            %
            %   EXGRATE = GETEXGRATE(CHIPIDENTIFIER) returns the exg rate setting of the Shimmer.
            %
            %   SYNOPSIS: exgRate = thisShimmer.getexgrate()
            %
            %
            %   INPUT: chipIdentifier - numeric value to select SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: exgRate - Numeric value defining the exg rate.
            %                     Valid range settings are 0 (125 Hz), 1
            %                     (250 Hz), 2 (500 Hz (default)), 3 (1000 Hz),
            %                     4 (2000 Hz), 5 (4000 Hz), 6 (8000 Hz)
            %
            %   EXAMPLE: exgRate = shimmer1.getexgrate();
            %
            %   See also setexgrate getexggain setexggain
            if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: getexgrate - ExG rate is not supported for this firmware version, please update firmware.');
                exgRate = 'Nan';
            elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                if (chipIdentifier == 1)
                    exgRate = thisShimmer.EXG1Rate;
                elseif (chipIdentifier == 2)
                    exgRate = thisShimmer.EXG2Rate;
                else
                    exgRate = 'Nan';
                    fprintf(strcat('Warning: getexgrate - Invalid chip selection.\n'));
                end
            elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                exgRate = 'NaN';
                disp('Warning: getexgrate - ExG rate is not supported in Shimmer2/2r.');
            else
                exgRate = 'Nan';
                fprintf(strcat('Warning: getexgrate - Cannot determine exg rate as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
        end % function getexgrate
        
        
        function exgGain = getexggain(thisShimmer, chipIdentifier, channelIdentifier) % function getexggain
            % GETEXGGAIN - Get the exg gain of the Shimmer
            %
            %   EXGGAIN = GETEXGGAIN(CHIPIDENTIFIER, CHANNELIDENTIFIER) returns the exg gain setting of the Shimmer.
            %
            %   SYNOPSIS: exgGain = thisShimmer.getexggain(chipIdentifier, channelIdentifier)
            %
            %
            %   INPUT: chipIdentifier - numeric value to select SENSOR_EXG1 or SENSOR_EXG2
            %
            %   INPUT: channelIdentifier - numeric value to select SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: exgGain - Numeric value defining the exg gain.
            %                     Valid settings are 0 (6x), 1 (1x), 2 (2x),
            %                     3 (3x), 4 (4x), 5 (8x), 6 (12x)
            %
            %   EXAMPLE: exgGain = shimmer1.getexggain(1,2);
            %
            %   See also setexggain getexgrate setexgrate
            if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: getexggain - ExG gain is not supported for this firmware version, please update firmware.');
                exgGain = 'Nan';
            elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)                     % Shimmer must be in a Connected state
                if (chipIdentifier == 1)
                    if(channelIdentifier == 1)
                        exgGain = thisShimmer.EXG1CH1Gain;
                    elseif(channelIdentifier == 2)
                        exgGain = thisShimmer.EXG1CH2Gain;
                    else
                        exgGain = 'Nan';
                    end
                elseif (chipIdentifier == 2)
                    if(channelIdentifier == 1)
                        exgGain = thisShimmer.EXG2CH1Gain;
                    elseif(channelIdentifier == 2)
                        exgGain = thisShimmer.EXG2CH2Gain;
                    else
                        exgGain = 'Nan';
                    end 
                else
                    exgGain = 'Nan';
                    fprintf(strcat('Warning: getexggain - Invalid chip selection.\n'));
                end
            elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                exgGain = 'Nan';
                disp('Warning: getexggain - ExG gain is not supported in Shimmer2/2r.');
            else
                exgGain = 'Nan';
                fprintf(strcat('Warning: getexggain - Cannot determine exg gain as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
        end % function getexggain
          
        %%
        function realtimeClock = getrealtimeclock(thisShimmer) % function getrealtimeclock
            %GETREALTIMECLOCK - Get the Real Time Clock
            %
            %   REALTIMECLOCK = GETREALTIMECLOCK gets the value of the
            %   Real Time Clock on Shimmer3 as MATLAB date string.
            %
            %   SYNOPSIS: realtimeClock = thisShimmer.getrealtimeclock
            %
            %   OUTPUT: realtimeClock -  MATLAB date string
            %
            %   EXAMPLE: realtimeClock = shimmer1.getrealtimeclock
            %
            %   See also setrealtimeclock
            %
            if  (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                realtimeClock = 'Nan';
                fprintf('Warning: getrealtimelock - Command not available for this hardware version.\n');
            elseif (thisShimmer.FirmwareCompatibilityCode < 6)
                realtimeClock = 'Nan';
                fprintf('Warning: getrealtimelock - Command not available for this firmware version.\n');
            elseif (~strcmp(thisShimmer.State,'Connected'))
                realtimeClock = 'Nan';
                fprintf(strcat('Warning: getrealtimeclock - Cannot set Real Time Clock for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            else
                if (strcmp(thisShimmer.RealTimeClockMilliseconds,'Nan'))
                    realtimeClock = 'Nan';
                else
                    realtimeClock = datestr(thisShimmer.convertUnixTimeMillisecondsToMatlabTime(thisShimmer.RealTimeClockMilliseconds));
                end
            end
         end % function getrealtimeclock
         

        function leadOffDetectionMode = getexgleadoffdetectionmode(thisShimmer)
            % GETEXGLEADOFFDETECTIONMODE - Gets the ExG lead-off detection mode
            %
            %   GETEXGLEADOFFDETECTIONMODE returns the ExG lead-off detection mode of the Shimmer.
            %
            %   SYNOPSIS: leadOffDetectionMode = thisShimmer.getexgleadoffdetectionmode
            %
            %   OUTPUT: leadOffDetectionMode - leadOffDetectionMode is a string
            %                                  containing the exg lead-off detection
            %                                  mode.
            %
            %                                  leadOffDetectionMode is 'DC Current',
            %                                  'Off' or 'Unknown'.
            %
            %   EXAMPLE: leadOffDetectionMode = shimmer1.getexgleadoffdetectionmode;
            %
            %   See also setexgleadoffdetectionmode
            %   getexgleadoffdetectioncurrent setexgleadoffdetectioncurrent
            %   getexgleadoffcomparatorthreshold setexgleadoffcomparatorthreshold
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: getexgleadoffdetectionmode - Cannot get ExG lead-off detection mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                leadOffDetectionMode = 'Unknown';
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: getexgleadoffdetectionmode - Only supported on Shimmer3');
                leadOffDetectionMode = 'Unknown';
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: getexgleadoffdetectionmode - Command not supported for this firmware version, please update firmware.')
                leadOffDetectionMode = 'Unknown';
            else
                if (thisShimmer.EXGLeadOffDetectionMode == 1)
                    leadOffDetectionMode = 'DC Current';
                elseif (thisShimmer.EXGLeadOffDetectionMode == 0)                                                                       
                    leadOffDetectionMode = 'Off';
                else                                                                                                           
                    leadOffDetectionMode = 'Unknown';
                end
            end
        end % function getexgleadoffdetectionmode
        
        function leadOffDetectionCurrent = getexgleadoffdetectioncurrent(thisShimmer, chipIdentifier)
            % GETEXGLEADOFFDETECTIONCURRENT - Gets the ExG lead-off
            % detection current
            %
            %   GETEXGLEADOFFDETECTIONCURRENT(CHIPIDENTIFIER) returns the ExG lead-off detection current of the Shimmer.
            %
            %   SYNOPSIS: leadOffDetectionCurrent = thisShimmer.getexgleadoffdetectioncurrent(chipIdentifier)
            %
            %   INPUT: chipIdentifier -        chipIdentifier is 1 or 2 to select
            %                                  SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: leadOffDetectionCurrent - leadOffDetectionCurrent is a string
            %                                  containing the exg lead-off detection
            %                                  current.
            %
            %                                  leadOffDetectionCurrent is '6nA',
            %                                  '22nA', '6uA' or '22uA' or 'Unknown'.
            %
            %   EXAMPLE: leadOffDetectionCurrent = shimmer1.getexgleadoffdetectioncurrent(1);
            %   
            %   See also getexgleadoffdetectionmode setexgleadoffdetectionmode
            %   setexgleadoffcomparatorthreshold getexgleadoffcomparatorthreshold
            %   setexgleadoffdetectioncurrent 
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: getexgleadoffdetectioncurrent - Cannot get ExG lead-off detection current for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                leadOffDetectionCurrent = 'Unknown';
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: getexgleadoffdetectioncurrent - Only supported on Shimmer3');
                leadOffDetectionCurrent = 'Unknown';
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: getexgleadoffdetectioncurrent - Command not supported for this firmware version, please update firmware.')
                leadOffDetectionCurrent = 'Unknown';
            else
                if (chipIdentifier == 1)
                    if (thisShimmer.EXG1ILEAD_OFF == 0)      % leadOffDetectionCurrent = '6nA';
                        leadOffDetectionCurrent = '6nA';
                    elseif (thisShimmer.EXG1ILEAD_OFF == 4)  % leadOffDetectionCurrent = '22nA';
                        leadOffDetectionCurrent = '22nA';
                    elseif (thisShimmer.EXG1ILEAD_OFF == 8)  % leadOffDetectionCurrent = '6uA';
                        leadOffDetectionCurrent = '6uA';
                    elseif (thisShimmer.EXG1ILEAD_OFF == 12) % leadOffDetectionCurrent = '22uA';
                        leadOffDetectionCurrent = '22uA';
                    else                                     % leadOffDetectionCurrent = 'Unknown';
                        leadOffDetectionCurrent = 'Unknown';
                    end
                elseif (chipIdentifier == 2)
                    if (thisShimmer.EXG2ILEAD_OFF == 0)      % leadOffDetectionCurrent = '6nA';
                        leadOffDetectionCurrent = '6nA';
                    elseif (thisShimmer.EXG2ILEAD_OFF == 4)  % leadOffDetectionCurrent = '22nA';
                        leadOffDetectionCurrent = '22nA';
                    elseif (thisShimmer.EXG2ILEAD_OFF == 8)  % leadOffDetectionCurrent = '6uA';
                        leadOffDetectionCurrent = '6uA';
                    elseif (thisShimmer.EXG2ILEAD_OFF == 12) % leadOffDetectionCurrent = '22uA';
                        leadOffDetectionCurrent = '22uA';
                    else                                     % leadOffDetectionCurrent = 'Unknown';
                        leadOffDetectionCurrent = 'Unknown';
                    end
                else
                    disp('Warning: getexgleadoffdetectioncurrent - Invalid value for chipIdentifier');
                    leadOffDetectionCurrent = 'Unknown';
                end
            end
        end % function getexgleadoffdetectioncurrent
        
        function exgReferenceElectrodeConfiguration = getexgreferenceelectrodeconfiguration(thisShimmer)
            % GETEXGREFERENCEELECTRODECONFIGURATION - Gets the ExG reference electrode configuration
            %
            %   GETEXGREFERENCEELECTRODECONFIGURATION returns the ExG reference electrode configuration
            %
            %   SYNOPSIS: exgReferenceElectrodeConfiguration = thisShimmer.getexgreferenceelectrodeconfiguration
            %
            %
            %   OUTPUT: exgReferenceElectrodeConfiguration - exgReferenceElectrodeConfiguration
            %                                                is a numerical value representing
            %                                                the exg reference electrode configuration
            %
            %
            %   EXAMPLE: exgReferenceElectrodeConfiguration = shimmer1.getexgreferenceelectrodeconfiguration;
            %
            %   See also setexgreferenceelectrodeconfiguration
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: getexgreferenceelectrodeconfiguration - Cannot get ExG reference electrode configuration for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                exgReferenceElectrodeConfiguration = 'Nan';
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: getexgreferenceelectrodeconfiguration - Only supported on Shimmer3');
                exgReferenceElectrodeConfiguration = 'Nan';
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: getexgreferenceelectrodeconfiguration - Command not supported for this firmware version, please update firmware.')
                exgReferenceElectrodeConfiguration = 'Nan';
            else
                exgReferenceElectrodeConfiguration = thisShimmer.EXGReferenceElectrodeConfiguration;
            end
        end % function getexgreferenceelectrodeconfiguration

                                   
        function leadOffComparatorThreshold = getexgleadoffcomparatorthreshold(thisShimmer, chipIdentifier)
            % GETEXGLEADOFFCOMPARATORTHRESHOLD - Gets the ExG lead-off
            % comparator threshold
            %
            %   GETEXGLEADOFFCOMPARATORTHRESHOLD(CHIPIDENTIFIER) returns the ExG lead-off comparator threshold of the Shimmer.
            %
            %   SYNOPSIS: leadOffComparatorThreshold = thisShimmer.getexgleadoffcomparatorthreshold(chipIdentifier)
            %
            %   INPUT: chipIdentifier -        chipIdentifier is 1 or 2 to select
            %                                  SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: leadOffComparatorThreshold - leadOffComparatorThreshold is a string
            %                                  containing the exg lead-off
            %                                  comparator threshold.
            %
            %                                  leadOffComparatorThreshold is
            %                                  one of the following:
            %                                  'Pos:95% - Neg:5%',
            %                                  'Pos:92.5% - Neg:7.5%',
            %                                  'Pos:90% - Neg:10%',
            %                                  'Pos:87.5% - Neg:12.5%',
            %                                  'Pos:85% - Neg:15%',
            %                                  'Pos:80% - Neg:20%',
            %                                  'Pos:75% - Neg:25%',
            %                                  'Pos:70% - Neg:30%',
            %                                  'Unknown'.
            %
            %   EXAMPLE: leadOffComparatorThreshold = shimmer1.getexgleadoffcomparatorthreshold(1);
            %
            %   See also getexgleadoffdetectionmode setexgleadoffdetectionmode
            %   setexgleadoffcomparatorthreshold getexgleadoffdetectioncurrent
            %   setexgleadoffdetectioncurrent 
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: getexgleadoffcomparatorthreshold - Cannot get ExG lead-off comparator threshold for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                leadOffComparatorThreshold = 'Unknown';
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: getexgleadoffcomparatorthreshold - Only supported on Shimmer3');
                leadOffComparatorThreshold = 'Unknown';
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: getexgleadoffcomparatorthreshold - Command not supported for this firmware version, please update firmware.')
                leadOffComparatorThreshold = 'Unknown';
            else
                if (chipIdentifier == 1)
                    if (thisShimmer.EXG1COMP_TH  == 0)      
                        leadOffComparatorThreshold = 'Pos:95% - Neg:5%';
                    elseif (thisShimmer.EXG1COMP_TH == 32)  
                        leadOffComparatorThreshold = 'Pos:92.5% - Neg:7.5%';
                    elseif (thisShimmer.EXG1COMP_TH == 64)  
                        leadOffComparatorThreshold = 'Pos:90% - Neg:10%';
                    elseif (thisShimmer.EXG1COMP_TH == 96) 
                        leadOffComparatorThreshold = 'Pos:87.5% - Neg:12.5%';
                    elseif (thisShimmer.EXG1COMP_TH == 128) 
                        leadOffComparatorThreshold = 'Pos:85% - Neg:15%';
                    elseif (thisShimmer.EXG1COMP_TH == 160)  
                        leadOffComparatorThreshold = 'Pos:80% - Neg:20%';
                    elseif (thisShimmer.EXG1COMP_TH == 192)
                        leadOffComparatorThreshold = 'Pos:75% - Neg:25%';
                    elseif (thisShimmer.EXG1COMP_TH == 224)
                        leadOffComparatorThreshold = 'Pos:70% - Neg:30%';
                    else                                                   
                        leadOffComparatorThreshold = 'Unknown';
                    end
                elseif (chipIdentifier == 2)
                   if (thisShimmer.EXG2COMP_TH  == 0)      
                        leadOffComparatorThreshold = 'Pos:95% - Neg:5%';
                    elseif (thisShimmer.EXG2COMP_TH == 32)  
                        leadOffComparatorThreshold = 'Pos:92.5% - Neg:7.5%';
                    elseif (thisShimmer.EXG2COMP_TH == 64)  
                        leadOffComparatorThreshold = 'Pos:90% - Neg:10%';
                    elseif (thisShimmer.EXG2COMP_TH == 96) 
                        leadOffComparatorThreshold = 'Pos:87.5% - Neg:12.5%';
                    elseif (thisShimmer.EXG2COMP_TH == 128) 
                        leadOffComparatorThreshold = 'Pos:85% - Neg:15%';
                    elseif (thisShimmer.EXG2COMP_TH == 160)  
                        leadOffComparatorThreshold = 'Pos:80% - Neg:20%';
                    elseif (thisShimmer.EXG2COMP_TH == 192)
                        leadOffComparatorThreshold = 'Pos:75% - Neg:25%';
                    elseif (thisShimmer.EXG2COMP_TH == 224)
                        leadOffComparatorThreshold = 'Pos:70% - Neg:30%';
                    else                                                   
                        leadOffComparatorThreshold = 'Unknown';
                    end
                else
                    disp('Warning: getexgleadoffcomparatorthreshold - Invalid value for chipIdentifier');
                    leadOffComparatorThreshold = 'Unknown';
                end
            end
        end % function getleadoffcomparatorthreshold
                      
        
        function [ExpBoardID, IDByteArray] = getexpboardid(thisShimmer) % function getexpboardid
            % getexpboardid - Get Expansion Board ID bytes
            %
            %   [EXPBOARDID IDBYTEARRAY] = GETEXPBOARDID() returns Expansion Board ID bytes.
            %
            %   SYNOPSIS: IDByteArray = thisShimmer.getexpboardid()
            %
            %   OUTPUT: ExpBoardID - String value specifying the Expansion Board ID
            %
            %           IDByteArray - Array containing Expansion Board ID bytes:
            %                           ID Byte 0 - Board ID
            %                           ID Byte 1 - Board Rev
            %                           ID Byte 2 - Special revision
            %
            %   EXAMPLE: [ExpBoardID, IDByteArray] = shimmer1.getexpboardid;
            %
            %   See also setinternalboard getinternalboard
            ExpBoardID = 'Nan';
            BoardName = 'Nan';
            BoardRev = 'Nan';
            BoardSpecialRev = 'Nan';
            
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: getexpboardid - Cannot get Expansion Board ID bytes for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: getexpboardid - Command only supported for Shimmer3.')
            elseif (thisShimmer.FirmwareCompatibilityCode < 5 ) 
                disp('Warning: getexpboardid - Command not supported for this firmware version, please update firmware.')
                IDByteArray = 'Nan';
            else
                IDByteArray = readexpansionboardidbytes(thisShimmer, 3, 0);    % call readexpansionboardidbytes with numBytes = 3 and offset = 0
                
                IDByte0 = IDByteArray(1);
                IDByte1 = IDByteArray(2);
                IDByte2 = IDByteArray(3);
                
                if IDByte0 == 8 || IDByte0 == 49                               % Assign BoardName based on IDByte0 value
                    BoardName = 'Bridge Amplifier+';
                elseif IDByte0 == 14 || IDByte0 == 48  
                    BoardName = 'GSR+';
                elseif IDByte0 == 31
                    BoardName = 'IMU';
                elseif IDByte0 == 36
                    BoardName = 'PROTO3 Mini';
                elseif IDByte0 == 37 || IDByte0 == 47
                    BoardName = 'ExG';
                elseif IDByte0 == 38
                    BoardName = 'PROTO3 Deluxe';
                else
                    BoardName = 'Unknown';
                end
                
                if strcmp(BoardName,'Unknown')
                    ExpBoardID = 'Unknown';
                else
                    ExpBoardID = strcat(BoardName,' (SR',num2str(IDByte0),'.',num2str(IDByte1),'.',num2str(IDByte2),')');
                end
            end
        end % function getexpboardid
        
    end % methods
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Action Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        
        function isConnected = connect(thisShimmer)            
            %CONNECT - Connect to the Shimmer
            %
            %   CONNECT() establishes a bluetooth connection between the Shimmer
            %   and host application using the com port defined in the ComPort
            %   property of the object.
            %
            %   SYNOPSIS: isConnected = thisShimmer.connect()
            %
            %   OUTPUT: isConnected - Boolean value which indicates if the
            %                         operation was successful or not (1=TRUE,
            %                         0=FALSE).
            %
            %   EXAMPLE: isConnected = shimmer1.connect;
            %
            %   See also start stop disconnect toggleLed startdatalogandstream
            %   stopdatalogandstream stoploggingonly startloggingonly 
            
            if (strcmp(thisShimmer.State,'Connected'))
                fprintf(strcat('Warning: connect - Shimmer COM',thisShimmer.ComPort,', is already connected.\n'));
                isConnected = true;
            elseif (strcmp(thisShimmer.State,'Streaming'))
                fprintf(strcat('Warning: connect - Cannot connect Shimmer COM',thisShimmer.ComPort,', because it is already Streaming.\n'));
                isConnected = true;
            elseif (strcmp(thisShimmer.State,'Disconnected'))
                isOpen = opencomport(thisShimmer);                                      % Attempt to establish a connection by opening the comport
                if (isOpen)
                    thisShimmer.State = 'Connected';                                    % Set Shimmer state to Connected
                    readshimmerversion(thisShimmer);                                    % Requests the Shimmer version from the Shimmer and stores the response in thisShimmer.ShimmerVersion
                    readfirmwareversion(thisShimmer);
                    
                    if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode >= 5)        
                        determinehwcompcode(thisShimmer);                               % Check Expansion Board ID and set 'HardwareCompatibilityCode' accordingly
                    else
                        thisShimmer.HardwareCompatibilityCode = 1;                      % Assign HardwareCompatibilityCode = 1 for older Shimmer hardware and firmware versions
                    end
                    isConnected = inquiry(thisShimmer);
                    if (~isConnected)                                                   % the inquiry failed
                        fprintf(strcat('Warning: connect - Failed to receive response from Shimmer inquiry for COM',thisShimmer.ComPort,'. Disconnecting Shimmer.\n'));
                        disconnect(thisShimmer);
                    else
                        if (thisShimmer.FirmwareIdentifier ~= 3)                        % not for LogAndStream
                            thisShimmer.setledblink(0);                                 % set colour to green
                        end
                                                
                        if (thisShimmer.FirmwareCompatibilityCode >= 2)
                            thisShimmer.readpressuresensorcalibrationcoefficients();
                        end
                        
                        if (thisShimmer.FirmwareCompatibilityCode >= 3)
                            thisShimmer.readexgconfiguration(1);
                            thisShimmer.readexgconfiguration(2);
                        end
                        
                        if (thisShimmer.FirmwareCompatibilityCode >= 5)                 
                            thisShimmer.readbaudrate;                                   % Reads the Baud Rate of the Shimmer and stores the the response in thisShimmer.BaudRate
                        end
                        
                        if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            setbuffersize(thisShimmer,1);                               % Set the buffer size to 1 sample
                        end
                        
                        if (thisShimmer.FirmwareCompatibilityCode == 0)
                            thisShimmer.getcalibrationparameters('Accelerometer');      % Get calibration parameters from the InfoMem for the accelerometer sensor
                            thisShimmer.getcalibrationparameters('Gyroscope');          % Get calibration parameters from the InfoMem for the gyroscope sensor
                            thisShimmer.getcalibrationparameters('Magnetometer');       % Get calibration parameters from the InfoMem for the magnetometer sensor
                        else
                            thisShimmer.getcalibrationparameters('All');
                        end
                        
                        if (thisShimmer.FirmwareCompatibilityCode >= 6)
                            thisShimmer.getbatteryvoltage;
                            if(thisShimmer.FirmwareIdentifier == 3)
                                thisShimmer.readrealtimeclock;
                                [year,month,day,~,~,~ ] = datevec(thisShimmer.getrealtimeclock);
                                if (year == 1970 && month == 1 && day == 1)
                                    thisShimmer.setrealtimeclock;
                                end
                                fprintf(['Real Time Clock on Shimmer: ' thisShimmer.getrealtimeclock '.\n'])
                            end
                        end
                        
                        if (thisShimmer.FirmwareCompatibilityCode >= 7)
                            thisShimmer.disablevbattfreq;
                        end
                        
                        thisShimmer.WarningGetUncalibratedData = 0;                                 % Set warning property to zero
                        thisShimmer.WarningGetDeprecatedGetData = 0;                                % Set warning property to zero
                    end
                else
                    fprintf(strcat('Warning: connect - Serial port failed to open for COM',thisShimmer.ComPort,'. Disconnecting Shimmer.\n'));
                    disconnect(thisShimmer);
                    isConnected=false;
                end
            end
        end % function connect
                           
        function isStarted = start(thisShimmer)
            %START - Start data streaming from the Shimmer
            %
            %   START() starts data streaming from the Shimmer if the Shimmer
            %   is previously in a 'Connected' state.
            %
            %   SYNOPSIS: isStarted = thisShimmer.start()
            %
            %   OUTPUT: isStarted - Boolean value which indicates if the
            %                       operation was successful or not (1=TRUE,
            %                       0=FALSE).
            %
            %   EXAMPLE: isStarted = shimmer1.start;
            %
            %   See also connect stop disconnect toggleLed startdatalogandstream
            %   stopdatalogandstream stoploggingonly startloggingonly 
            if (strcmp(thisShimmer.State,'Streaming'))                               % Check if Shimmer is already in Streaming state 
                isStarted = false;
                fprintf(strcat('Warning: start - Shimmer at COM ',thisShimmer.ComPort,' is already Streaming.\n'));
            elseif (strcmp(thisShimmer.State,'Connected'))                           % Shimmer must be in a Connected state to start streaming
                
                resetstreaming(thisShimmer);                                         % Also clears the read data buffer 
                writetocomport(thisShimmer, thisShimmer.START_STREAMING_COMMAND);    % Send command to start the Shimmer streaming data
                isStarted = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);    % Wait for Acknowledgment from Shimmer
                
                if (isStarted == true)
                    thisShimmer.State = 'Streaming';
                else
                    fprintf(strcat('Warning: start - Start command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
                
            else
                isStarted = false;
                fprintf(strcat('Warning: start - Cannot start streaming as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
            
        end % function start       
            
        function isStopped = stop(thisShimmer)            
            %STOP - Stop data streaming from the Shimmer
            %
            %   STOP() stops data streaming from the Shimmer if the Shimmer
            %   is previously in a 'Streaming' state.
            %
            %   SYNOPSIS: isStopped = thisShimmer.stop()
            %
            %   OUTPUT: isStopped - Boolean value which indicates if the
            %                       operation was successful or not (1=TRUE,
            %                       0=FALSE).
            %
            %   EXAMPLE: isStopped = shimmer1.stop;
            %
            %   See also connect start disconnect toggleLed startdatalogandstream
            %   stopdatalogandstream stoploggingonly startloggingonly 
            if (strcmp(thisShimmer.State,'Streaming'))                               % TRUE if the Shimmer is streaming
                clearreaddatabuffer(thisShimmer);                                    % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.STOP_STREAMING_COMMAND);     % Send the Stop Command to the Shimmer
                thisShimmer.BlinkLED = -1;
                thisShimmer.WriteLEDBLINKWAITFORACK=0;
                
                thisShimmer.State = 'Connected';
                isStopped = true;
            else
                fprintf(strcat('Warning: stop - COM ',thisShimmer.ComPort,' Shimmer is not Streaming.\n'));
                isStopped = false;
            end
            
        end % function stop        
        
        function isDisconnected = disconnect(thisShimmer)            
            %DISCONNECT - Disconnect from the Shimmer
            %
            %   DISCONNECT() closes the bluetooth connection between the Shimmer
            %   and host application.
            %
            %   SYNOPSIS: isDisconnected = thisShimmer.disconnect()
            %
            %   OUTPUT: isDisconnected - Boolean value which indicates if the
            %                            operation was successful or not (1=TRUE,
            %                            0=FALSE).
            %
            %   EXAMPLE: isDisconnected = shimmer1.disconnect;
            %
            %   See also connect start stop toggleLed startdatalogandstream
            %   stopdatalogandstream stoploggingonly startloggingonly 
            
            if (~strcmp(thisShimmer.State,'Disconnected'))                 % TRUE if the Shimmer is streaming or connected
                if (strcmp(thisShimmer.State,'Streaming'))
                    if( thisShimmer.FirmwareIdentifier ~= 3)
                        thisShimmer.stop;                                  % Stop streaming before closing COM Port
                    end
                end
                isOpen=closecomport(thisShimmer);                          % Close the COM Port
                
                if (~isOpen)                                               % TRUE if COM Port is closed
                    thisShimmer.State = 'Disconnected';                    % Set Shimmer state to Disconnected
                end
                
                isDisconnected = ~isOpen;
                
            else                                                           % TRUE if Shimmer is disconnected
                isDisconnected = true;
            end
           
        end % function disconnect
               
        function isToggled = toggleLed(thisShimmer)            
            %TOGGLELED - Toggle the Shimmer LED
            %
            %   TOGGLELED() toggles the red LED on the Shimmer.
            %
            %   SYNOPSIS: isToggled = thisShimmer.toggleLed()
            %
            %   OUTPUT: isToggled - Boolean value which indicates if the
            %                       operation was successful or not (1=TRUE,
            %                       0=FALSE).
            %
            %   EXAMPLE: isToggled = shimmer1.toggleLed;
            %
            %   See also connect start stop disconnect startdatalogandstream
            %   stopdatalogandstream stoploggingonly startloggingonly 
            if (strcmp(thisShimmer.State,'Connected'))                               % Shimmer must be in a Connected state to toggle the LED
                clearreaddatabuffer(thisShimmer);                                    % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.TOGGLE_LED_COMMAND);         % Send command to toggle the LED.
                isToggled = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);    % Wait for Acknowledgment from Shimmer
                
                if (~isToggled)
                    fprintf(strcat('Warning: toggleLed - Toggle LED response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
                
            else
                isToggled = false;
                fprintf(strcat('Warning: toggleLed - Cannot toggle LED as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n.\n'));
            end
            
        end % function toggleLed    
        
        function isStarted = startloggingonly(thisShimmer)
            %STARTLOGGINGONLY - Start logging to SD card of the Shimmer
            %
            %   STARTLOGGINGONLY() starts logging data to SD card of the
            %   Shimmer if the Shimmer in a 'Connected' state.
            %
            %   SYNOPSIS: isStarted = thisShimmer.startstartloggingonly()
            %
            %   OUTPUT: isStarted - Boolean value which indicates if the
            %                       operation was successful or not (1=TRUE,
            %                       0=FALSE).
            %
            %   EXAMPLE: isStarted = shimmer1.startloggingonly;
            %
            %   See also connect start stop disconnect toggleLed startdatalogandstream
            %   stopdatalogandstream stoploggingonly  
            if(thisShimmer.FirmwareIdentifier ~= 3)
                fprintf('Warning: startloggingonly - This function is only supported for LogAndStream firmware.\n');
                isStarted = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 6)
                fprintf('Warning: startloggingonly - This function is not supported in this firmware version, please update firmware.\n');
                isStarted = false;
            elseif (strcmp(thisShimmer.State,'Streaming'))                               % Check if Shimmer is already in Streaming state
                isStarted = false;
                fprintf(strcat('Warning: startloggingonly - Shimmer at COM ',thisShimmer.ComPort,' is Streaming.\n'));
                fprintf('Stop streaming first.\n');
            elseif (thisShimmer.IsSDLogging == 1)
                isStarted = false;
                fprintf(strcat('Warning: startloggingonly - Shimmer at COM ',thisShimmer.ComPort,' is already Logging.\n'));
            elseif (strcmp(thisShimmer.State,'Connected'))                           % Shimmer must be in a Connected state to start streaming
                
                resetstreaming(thisShimmer);                                         % Also clears the read data buffer
                writetocomport(thisShimmer, thisShimmer.START_LOGGING_ONLY_COMMAND); % Send command to start the Shimmer streaming data
                isStarted = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);    % Wait for Acknowledgment from Shimmer
                
                if (isStarted == true)
                    thisShimmer.IsSDLogging = 1;
                else
                    fprintf(strcat('Warning: startloggingonly - Start Logging Only command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
                
            else
                isStarted = false;
                fprintf(strcat('Warning: startloggingonly - Cannot start Logging Only as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n'));
            end
            
        end % function startloggingonly
        
        function isStopped = stoploggingonly(thisShimmer)
            %STOPLOGGINGONLY - Stop logging data to SD card of the Shimmer
            %
            %   STOPLOGGINGONLY() only stops logging of data to Shimmer SD
            %   card for Shimmer in 'Streaming' state or 'Connected' State
            %   while 'Logging'.
            %
            %   SYNOPSIS: isStopped = thisShimmer.stoploggingonly()
            %
            %   OUTPUT: isStopped - Boolean value which indicates if the
            %                       operation was successful or not (1=TRUE,
            %                       0=FALSE).
            %
            %   EXAMPLE: isStopped = shimmer1.stoploggingonly;
            %
            %   See also connect start stop disconnect toggleLed startdatalogandstream
            %   stopdatalogandstream startloggingonly 
            
            if(thisShimmer.FirmwareIdentifier ~= 3)
                fprintf('Warning: stoploggingonly - This function is only supported for LogAndStream firmware.\n');
                isStopped = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 6)
                fprintf('Warning: stoploggingonly - This function is not supported in this firmware version, please update firmware.\n');
                isStopped = false;
            elseif (thisShimmer.IsSDLogging == 1 && strcmp(thisShimmer.State,'Streaming') || strcmp(thisShimmer.State,'Connected'))                               % TRUE if the Shimmer is streaming
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.STOP_LOGGING_ONLY_COMMAND);     % Send the STOP_LOGGING_ONLY_COMMAND to the Shimmer
                
                thisShimmer.IsSDLogging = 0;
                isStopped = true;
            else
                fprintf(strcat('Warning: stoploggingonly - COM ',thisShimmer.ComPort,' Shimmer is not Logging.\n'));
                isStopped = false;
            end
            
        end % function stoploggingonly        
            
        function isStarted = startdatalogandstream(thisShimmer)
            %STARTDATALOGANDSTREAM - Start logging and streaming
            %
            %
            %   STARTDATALOGANDSTREAM() starts data streaming from the Shimmer while logging this data to the SD card of the Shimmer
            %   The previous state of the Shimmer is 'Connected'. After the acknowledgement the Shimmer is in 'Streaming' state.
            %
            %   SYNOPSIS: isStarted = thisShimmer.startdatalogandstream()
            %
            %   OUTPUT: isStarted - Boolean value which indicates if the
            %                       operation was successful or not (1=TRUE,
            %                       0=FALSE).
            %
            %   EXAMPLE: isStarted = shimmer1.startdatalogandstream;
            %
            %   See also connect start stop disconnect toggleLed startdatalogandstream
            %   stoploggingonly startloggingonly 
            
            if (strcmp(thisShimmer.State,'Streaming'))                               % Check if Shimmer is already in Streaming state
                isStarted = false;
                fprintf(strcat('Warning: startdatalogandstream - Shimmer at COM ',thisShimmer.ComPort,' is already Streaming.\n'));
            elseif ~(strcmp(thisShimmer.State,'Connected'))                               % Shimmer must be in a Connected state to start streaming
                isStarted = false;
                fprintf(strcat('Warning: startdatalogandstream - Cannot start streaming as COM ',thisShimmer.ComPort,' Shimmer is not Connected.\n.\n'));
            elseif ~(thisShimmer.FirmwareIdentifier == 3)
                isStarted = false;
                disp('Warning: startdatalogandstream - This function is only supported for LogAndStream firmware.');
            elseif (thisShimmer.FirmwareCompatibilityCode < 4)
                isStarted = false;
                disp('Warning: startdatalogandstream - This function is not supported for this firmware version, please update firmware.');
            else
                
                thisShimmer.getstatus;
                
                resetstreaming(thisShimmer);                                         % Also clears the read data buffer
                writetocomport(thisShimmer, thisShimmer.START_SDBT_COMMAND);         % Send command to start the Shimmer streaming data while logging on SD card
                isStarted = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);    % Wait for Acknowledgment from Shimmer
                
                if (isStarted == true)
                    
                    if(thisShimmer.IsDocked)
                        fprintf(strcat('Warning: startdatalogandstream - Shimmer on COM',thisShimmer.ComPort,' is docked, therefore it is not logging, but only streaming.\n'));
                        thisShimmer.IsSDLogging = 0;
                    else
                        thisShimmer.IsSDLogging = 1;
                    end
                    
                    thisShimmer.State = 'Streaming';
                else
                    fprintf(strcat('Warning: startdatalogandstream - Start command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
            end
            
        end % function startdatalogandstream
        
        function isStopped = stopdatalogandstream(thisShimmer)
            %STOPDATALOGANDSTREAM - Stop data logging and streaming
            %
            %   STOPDATALOGANDSTREAM() stop logging and streaming for
            %   Shimmer previously in a 'Streaming' state while logging data
            %   to its SD card of the Shimmer.
            %
            %   SYNOPSIS: isStopped = thisShimmer.stopdatalogandstream()
            %
            %   OUTPUT: isStopped - Boolean value which indicates if the
            %                       operation was successful or not (1=TRUE,
            %                       0=FALSE).
            %
            %   EXAMPLE: isStopped = shimmer1.stopdatalogandstream;
            %
            %   See also connect start stop disconnect toggleLed startdatalogandstream
            %   stopdatalogandstream stoploggingonly startloggingonly 
            
            if(thisShimmer.FirmwareIdentifier ~= 3)
                fprintf('Warning: stopdatalogandstream - This function is only supported for LogAndStream firmware.\n');
                isStopped = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 6)
                fprintf('Warning: stopdatalogandstream - This function is not supported in this firmware version, please update firmware.\n');
                isStopped = false;
            elseif (strcmp(thisShimmer.State,'Streaming'))   % TRUE if the Shimmer is streaming & logging
                
                clearreaddatabuffer(thisShimmer);                                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.STOP_SDBT_COMMAND);                                % Send the Stop Command to the Shimmer
                thisShimmer.BlinkLED = -1;
                thisShimmer.WriteLEDBLINKWAITFORACK=0;
                
                if (thisShimmer.IsSDLogging == 0)
                    fprintf(strcat('Warning: stopdatalogandstream - COM ',thisShimmer.ComPort,' Shimmer was not Logging, only stopped Streaming.\n'));
                end
                
                thisShimmer.State = 'Connected';
                thisShimmer.IsSDLogging = 0;
                isStopped = true;
            else
                fprintf(strcat('Warning: stopdatalogandstream - COM ',thisShimmer.ComPort,' Shimmer is not Streaming.\n'));
                isStopped = false;
            end
            
        end % function stopdatalogandstream

        
    end % methods
    
    
    methods (Access = 'private')
                
        function resetstreaming(thisShimmer)
            
            thisShimmer.SerialDataOverflow = [];
            thisShimmer.nFramePacketErrors = 0;
            clearreaddatabuffer(thisShimmer);
            
        end % function reset
             
    end % methods (Access = 'private')
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Utility Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
               
        function percentageofPacketsReceived = getpercentageofpacketsreceived(thisShimmer,continuousTimeStamp)
            %   GETPERCENTAGEOFPACKETSRECEIVED - calculates the
            %   percentage of packets received from the Shimmer by the
            %   host.
            %
            %   PERCENTAGEOFPACKETSRECEIVED = GETPERCENTAGEOFPACKETSRECEIVED(CONTINUOUSTIMESTAMP)
            %                                   returns the percentage of packets received 
            %                                   successfully in the data series associated 
            %                                   with continuous time stamp CONTINUOUSTIMESTAMP.
            %
            %   SYNOPSIS: percentageofPacketsReceived = thisShimmer.getpercentageofpacketsreceived(continuousTimeStamp)
            %
            %   INPUT: continuousTimeStamp - A 1 Dimensional series of
            %                                calibrated continuous timestamp 
            %                                data (doubleprecision).
            %
            %   OUTPUT: percentageofPacketsReceived - The percentage of packets received.
            %
            %   EXAMPLE: percentagePacketsRecieved = shimmer1.getpercentageofpacketsreceived(continuousTimeStamp);
            %
            %   See also getdata
            
            
            numberofLostPackets=0;                      
            jitterMargin = 0.1;   % accommodate for minor possible variations in internal Shimmer clock
            for i=1:length(continuousTimeStamp)-1
                
                if (continuousTimeStamp(i+1)-continuousTimeStamp(i)) > ((1+jitterMargin)/double(thisShimmer.SamplingRate)*1000) %To detect a discontinuity in the time stamp
                    numberofLostPackets=numberofLostPackets+round((continuousTimeStamp(i+1)-continuousTimeStamp(i))/(1/double(thisShimmer.SamplingRate)*1000))-1; %if there is a dropped packet, detect how many additional packets have been dropped
                end
                if (continuousTimeStamp(i+1)-continuousTimeStamp(i) <= 0)
                    disp('Warning: getpercentageofpacketsreceived - The percentage of packets received is unknown,')
                    disp('please check TimeStamp signal.')
                end
               
            end
            percentageofPacketsReceived=((length(continuousTimeStamp)-numberofLostPackets)/length(continuousTimeStamp))*100;
        end
        
        function unixTimeMilliseconds = convertMatlabTimeToUnixTimeMilliseconds(thisShimmer,matlabTime)
            %CONVERTMATLABTIMETOUNIXTIMEMILLISECONDS - converts a MATLAB  
            % date vector or date string to Unix Time in milliseconds.
            %   
            %   UNIXTIMEMILLISECONDS           = CONVERTMATLABTIMETOUNIXTIMEMILLISECONDS(MATLABTIME)
            %                                     converts either a MATLAB date vector 
            %                                     or date string to Unix Time in milliseconds.
            %
            %   SYNOPSIS: unixTimeMilliseconds = thisShimmer.convertMatlabTimeToUnixTimeMilliseconds(matlabTime)
            %
            %   INPUT:    matlabTime           - MATLAB date vector or date string
            %
            %   OUTPUT:   unixTimeMilliseconds - Unix time in milliseconds
            %
            %   See also datenum
            
            unixTimeMilliseconds = 24*3600*1000 * (datenum(matlabTime)-datenum('01-Jan-1970')) - (1*3600*1000); 
        end
  
        function matlabTime = convertUnixTimeMillisecondsToMatlabTime(thisShimmer,unixTimeMilliseconds)
            %CONVERTUNIXTIMEMILLISECONDSTOMATLABTIME - converts a MATLAB
            % date vector or date string to Unix Time in milliseconds.
            %
            %   MATLABTIME                      = CONVERTUNIXTIMEMILLISECONDSTOMATLABTIME(UNIXTIMEMILLISECONDS)
            %                                      converts Unix Time in
            %                                      milliseconds to MATLAB date number.
            %
            %   SYNOPSIS: unixTimeMilliseconds  = thisShimmer.convertMatlabTimeToUnixTimeMilliseconds(matlabTime)
            %
            %   INPUT:    unixTimeMilliseconds  - Unix time in milliseconds
            %
            %   OUTPUT:   matlabTime            - MATLAB date number
            %
            %
            %   See also datenum
            
            matlabTime = ( (unixTimeMilliseconds + 1*3600*1000) / (24*3600*1000)  ) + datenum('01-Jan-1970');
        end
        
    end
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Configure Settings Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = 'private')
        
        
        function isWritten = disablevbattfreq(thisShimmer)
            % Disables periodic sampling of battery value, which is not supported
            % in this AP
            if ~(thisShimmer.FirmwareCompatibilityCode >= 7)
               disp('Warning: disablevbattfreq - This function is not supported in this firmware version.'); 
               isWritten = 'false';
            else
                clearreaddatabuffer(thisShimmer);                                                    % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_VBATT_FREQ_COMMAND);                     % Send the SET_VBATT_FREQ_COMMAND to the Shimmer
                writetocomport(thisShimmer, char(0));                                                % Write 4 zero bytes to disable periodic sampling of battery value   
                writetocomport(thisShimmer, char(0));
                writetocomport(thisShimmer, char(0));
                writetocomport(thisShimmer, char(0));
                
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);                    % Wait for Acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: disablevbattfreq - Set VBatt Freq response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
            end
        end % function disablevbattfreq
        
        
        function isWritten = writeresettodefaultconfiguration(thisShimmer)                       % function writeresettodefaultconfiguration
            % Writes default configuration to Shimmer
            if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                disp('Warning: writeresettodefaultconfiguration - Command only supported for Shimmer3.')
            elseif ~(strcmp(thisShimmer.State,'Connected'))                     % Shimmer must be in a Connected state
                fprintf(strcat('Warning: writeresettodefaultconfiguration - Cannot set sampling range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            else
                clearreaddatabuffer(thisShimmer);                                                    % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.RESET_TO_DEFAULT_CONFIGURATION_COMMAND);     % Send the RESET_TO_DEFAULT_CONFIGURATION_COMMAND to the Shimmer
                
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);                    % Wait for Acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: writeresettodefaultconfiguration - Reset Default Configuration response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
            end
        end % function writeresettodefaultconfiguration

        function isSet = setledblinkwhilestreaming(thisShimmer, ledBlink)
            % Writes ledBlink to Shimmer3 - in Streaming state.
            if (thisShimmer.FirmwareIdentifier == 3)
                disp('Warning: setledblink - Not supported for LogAndStream.');
            elseif (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                
                isWritten = writeledblinkwhilestreaming(thisShimmer,ledBlink);           % Write ledBlink while streaming.
                
                if (isWritten)
                    thisShimmer.BlinkLED = ledBlink;
                    isSet = true;
                else
                    isSet = false;
                end
                
            else
                fprintf(strcat('Warning: setledblinkwhilestreaming - Cannot set led blink while streaming for COM ',thisShimmer.ComPort,' as Shimmer is not streaming.\n'));
                isSet = false;
            end
        end % function setledblinkwhilestreaming
        
        function isWritten = writesamplingrate(thisShimmer,samplingRate)
            % Writes sampling rate to Shimmer - in Connected state.
            if (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                                    % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_SAMPLING_RATE_COMMAND);  % Send the Set Sampling Rate Command to the Shimmer
                if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    if (samplingRate < 4.0315) && (samplingRate > 0)                     % Refer to '.\Resources\Sampling Rate Table.txt' for more details 
                        fprintf('Warning: writesamplingrate - Sampling rates between 4.0315 Hz and 0 are invalid.\n');
                        fprintf('The sampling rate will be set to the lowest valid non-zero value (4.031 Hz).\n');
                        samplingRateCharValue = char(254);

                    elseif (samplingRate == 0)                                           % SamplingRate == 0 is a special case, refer to 'Sampling Rate Table.txt' for more details
                        samplingRateCharValue = char(255);
                    else
                        samplingRateCharValue = char(1024/samplingRate);
                    end
                    writetocomport(thisShimmer, samplingRateCharValue);                 % Write the Sampling Rate char value to the Shimmer
                else
                    samplingByteValue = uint16(32768/samplingRate);
                    writetocomport(thisShimmer, char(bitand(255,samplingByteValue)));
                    writetocomport(thisShimmer, char(bitshift(samplingByteValue,-8)));
                end

                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer

                if (isWritten == false)
                    fprintf(strcat('Warning: writesamplingrate - Set sampling rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writesamplingrate - Cannot set sampling rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writesamplingrate
        
        function isWritten = writebaudrate(thisShimmer,baudRate)
            % Writes baudrate to Shimmer3 - in Connected state 
            %
            % Shimmer needs to Disconnect and (Re)Connect within this 
            % function for the baud rate actually be set.
            if ~strcmp(thisShimmer.State,'Connected')
                fprintf(strcat('Warning: writebaudrate - Cannot set baud rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isWritten = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: writebaudrate - This function is only supported for Shimmer3.');
                isWritten = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 5) 
                disp('Warning: writebaudrate - This function is not supported in this firmware version, please update firmware.');
                isWritten = false;
            else
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_BT_COMMS_BAUD_RATE);        % Send the Set Baud Rate Command to the Shimmer
                writetocomport(thisShimmer, char(baudRate));                            % Send Baud Rate to the Shimmer
                
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);       % Wait for Acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: writebaudrate - Set baud rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
                
                disp('Warning: writebaudrate - Shimmer will disconnect and reconnect now in order to set Baud Rate...');
                isDisconnected = thisShimmer.disconnect;
                
                if (thisShimmer.FirmwareIdentifier == 3)
                    pause(.4);
                end
                
                isConnected = thisShimmer.connect;
                
                if ~(isDisconnected && isConnected)
                    fprintf('Warning: writebaudrate - Baud Rate is not successfully set as Shimmer did not disconnect and reconnect properly,\n');
                    fprintf('please try again.\n');
                    isWritten = false;
                end
            end
            
        end % function writebaudrate

        
        function isWritten = writeemgcalibrationparameters(thisShimmer,offset,gain)
            % Writes EMG calibration parameters for Shimmer2/2r - in
            % Connected state.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                    % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_EMG_CALIBRATION_COMMAND);  % Send the Set Sampling Rate Command to the Shimmer
                
                writetocomport(thisShimmer, char(bitshift(offset,-8)));
                writetocomport(thisShimmer, char(bitand(255,offset)));
                writetocomport(thisShimmer, char(bitshift(gain,-8)));
                writetocomport(thisShimmer, char(bitand(255,gain)));
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: writeemgcalibrationparameters - Set EMG calibration parameters command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                else
                    thisShimmer.DefaultEMGCalibrationParameters=false;
                end
            elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)   
                isWritten = false;
                fprintf('Warning: writeemgcalibrationparameters - EMG Calibration is not supported for Shimmer3.\n');
            else
                isWritten = false;
                fprintf(strcat('Warning: writeemgcalibrationparameters - Cannot set EMG calibration parameters for COM',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeemgcalibrationparameters
        
        function isWritten = writeecgcalibrationparameters(thisShimmer,offsetrall,gainrall,offsetlall,gainlall)
            % Writes ECG calibration parameters for Shimmer2/2r - in Connected state.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                    % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_ECG_CALIBRATION_COMMAND);  % Send the Set ECG Calibration Command to the Shimmer
                                
                writetocomport(thisShimmer, char(bitshift(offsetlall,-8)));
                writetocomport(thisShimmer, char(bitand(255,offsetlall)));
                writetocomport(thisShimmer, char(bitshift(gainlall,-8)));
                writetocomport(thisShimmer, char(bitand(255,gainlall)));
                writetocomport(thisShimmer, char(bitshift(offsetrall,-8)));
                writetocomport(thisShimmer, char(bitand(255,offsetrall)));
                writetocomport(thisShimmer, char(bitshift(gainrall,-8)));
                writetocomport(thisShimmer, char(bitand(255,gainrall)));
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: writeecgcalibrationparameters - Set ECG calibration parameters command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                else
                    thisShimmer.DefaultECGCalibrationParameters=false;
                end
            elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)   
                isWritten = false;
                fprintf(strcat('Warning: writeecgcalibrationparameters - ECG Calibration is not supported for Shimmer3.\n'));
            else
                isWritten = false;
                fprintf(strcat('Warning: writeecgcalibrationparameters - Cannot Cannot set ECG calibration parameters for COM',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeecgcalibrationparameters
        
        function isWritten = writeinternalexppower(thisShimmer,enable)
            % Writes internal expansion power for Shimmer3 - in Connected state.
            % 
            % Refer to the expansion board manual to check whether internal 
            % expansion power needs to be enabled.
            if (strcmp(thisShimmer.State,'Connected')) 
                
                if (thisShimmer.FirmwareCompatibilityCode >= 2 && (enable == 0 || enable ==1))                   
                    clearreaddatabuffer(thisShimmer);                                  % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_INTERNAL_EXP_POWER_ENABLE_COMMAND); 
                    
                    writetocomport(thisShimmer, char(enable));                    
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);  % Wait for Acknowledgment from Shimmer
                    
                    if (~isWritten)
                        fprintf(strcat('Warning: writeinternalexppower - Set exp power response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    isWritten = false;
                    disp('Warning: writeinternalexppower - Internal exp power is not available on Shimmer2/2r.');
                elseif(thisShimmer.FirmwareCompatibilityCode < 2)
                    isWritten = false;
                    disp('Warning: writeinternalexppower - Internal exp power is not supported by the firmware on the device. Please update the firmware.');
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writeinternalexppower - Attempt to set exp power failed, \n'));
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writeinternalexppower - Cannot write exp power for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeinternalexppower
               
        function isWritten = writeaccelhrmode(thisShimmer,enable) 
            % Enables LSM303DLHC/LSM303AHTR High Resolution mode for Shimmer3 - in Connected state.
            if (strcmp(thisShimmer.State,'Connected'))
                
                if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && (enable == 0 || enable == 1))
                    clearreaddatabuffer(thisShimmer);                                             % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_LSM303DLHC_ACCEL_HRMODE_COMMAND); % Send SET_LSM303DLHC_ACCEL_HRMODE_COMMAND to Shimmer
                    
                    writetocomport(thisShimmer, char(enable));
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);             % Wait for Acknowledgment from Shimmer
                    
                    if (~isWritten)
                        fprintf(strcat('Warning: writeaccelhrmode - Set Accel HR mode command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    isWritten = false;
                    disp('Warning: writeaccelhrmode - Accel HR mode is not available on Shimmer2/2r.');
                else
                    isWritten = false;
                    fprintf('Warning: writeaccelhrmode - Attempt to set Accel HR mode failed, \n');
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
            else
                isWritten = false;
                fprintf(strcat('Warning: writeaccelhrmode - Cannot write Accel HR mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeaccelhrmode
        
        function isWritten = writeaccellpmode(thisShimmer,enable) 
            % Enables LSM303DLHC/LSM303AHTR Low Power mode for Shimmer3 - in Connected state.
            if (strcmp(thisShimmer.State,'Connected'))
                
                if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && (enable == 0 || enable == 1))
                    clearreaddatabuffer(thisShimmer);                                             % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_LSM303DLHC_ACCEL_LPMODE_COMMAND); % Send SET_LSM303DLHC_ACCEL_LPMODE_COMMAND to Shimmer
                    
                    writetocomport(thisShimmer, char(enable));
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);             % Wait for Acknowledgment from Shimmer
                    
                    if (~isWritten)
                        fprintf(strcat('Warning: writeaccellpmode - Set Accel LP mode command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    isWritten = false;
                    disp('Warning: writeaccellpmode - Accel LP mode is not available on Shimmer2/2r.');
                else
                    isWritten = false;
                    fprintf('Warning: writeaccellpmode - Attempt to set Accel LP mode failed, \n');
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
            else
                isWritten = false;
                fprintf(strcat('Warning: writeaccellpmode - Cannot write Accel LP mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeaccellpmode
        
        function isWritten = writeaccelrange(thisShimmer,accelRange)
            % Writes accelerometer range to Shimmer - in Connected state
            % This function is for Shimmer2, Shimmer2r and Shimmer3. For
            % Shimmer3 this function writes the range for the
            % LSM303DLHC/LSM303AHTR accelerometer.
            if (strcmp(thisShimmer.State,'Connected'))
                
                
                if (thisShimmer.ShimmerVersion == 1) && ((accelRange == 0) || (accelRange == 1) || (accelRange == 2) || (accelRange == 3))
                    validSetting = true;
                elseif (thisShimmer.ShimmerVersion == 2) && ((accelRange == 0) || (accelRange == 3))
                    validSetting = true;
                elseif (thisShimmer.ShimmerVersion == 3) && ((accelRange == 0) || (accelRange == 1)|| (accelRange == 2)|| (accelRange == 3))
                    validSetting = true;
                else 
                    validSetting = false;
                end
                
                if (validSetting == true)
                    
                    clearreaddatabuffer(thisShimmer);                                  % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_ACCEL_RANGE_COMMAND);  % Send the Set Sampling Rate Command to the Shimmer
                    
                    writetocomport(thisShimmer, char(accelRange));                     % Write the accel range char value to the Shimmer
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);  % Wait for Acknowledgment from Shimmer
                    
                    if (~isWritten)
                        fprintf(strcat('Warning: writeaccelrange - Set accel range response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writeaccelrange - Attempt to set accel range failed due to a request to set the range to an \n'));
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    fprintf('Valid range setting values for the Shimmer 2 are 0 (+/- 1.5g), 1 (+/- 2g), 2 (+/- 4g) and 3 (+/- 6g).\n');
                    fprintf('Valid range setting values for the Shimmer 2r are 0 (+/- 1.5g) and 3 (+/- 6g).\n');
                    fprintf('Valid range setting values for the Shimmer 3 are 0 (+/- 2g), 1 (+/- 4g), 2 (+/- 8g) and 3 (+/- 16g).\n');
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writeaccelrange - Cannot set accel range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeaccelrange
        
        function isWritten = writeconfigbyte0(thisShimmer,configByte0)
            % Writes config byte 0 for Shimmer2/2r - in Connected state
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_CONFIG_BYTE0_COMMAND);      % Send the SET_CONFIG_BYTE0_COMMAND to the Shimmer
                writetocomport(thisShimmer, char(configByte0));                         % Write the configByte0 char value to the Shimmer
                
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);       % Wait for acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: writeconfigbyte0 - Set config byte0 response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
            elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                isWritten = false;
                fprintf(strcat('Warning: writeconfigbyte0 - This command is not available for Shimmer3; please use writeconfigbytes.\n'));
            else
                isWritten = false;
                fprintf(strcat('Warning: writeconfigbyte0 - Cannot set config byte0 for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeconfigbyte0
        
        function isWritten = writeconfigbytes(thisShimmer,configByte0, configByte1, configByte2, configByte3)
            % Writes config bytes for Shimmer3 - in Connected state
            if (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_CONFIG_BYTE0_COMMAND);      % Send the SET_CONFIG_BYTE0_COMMAND to the Shimmer (this is equal to SET_CONFIG_BYTES_COMMAND for Shimmer3)
                writetocomport(thisShimmer, char(configByte0));                         % Write the configByte0 char value to the Shimmer
                if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    writetocomport(thisShimmer, char(configByte1));                     % Write the configByte1 char value to the Shimmer
                    writetocomport(thisShimmer, char(configByte2));                     % Write the configByte2 char value to the Shimmer
                    writetocomport(thisShimmer, char(configByte3));                     % Write the configByte3 char value to the Shimmer
                end
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);       % Wait for acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: writeconfigbytes - Set config bytes response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writeconfigbytes - Cannot set config bytes for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeconfigbytes
        
        function isWritten = writebuffersize(thisShimmer,bufferSize)
            % Writes buffersize for Shimmer2/2r - in Connected state
            % This function is currently not supported for Shimmer3.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_BUFFER_SIZE_COMMAND);       % Send the SET_BUFFER_SIZE_COMMAND to the Shimmer
                writetocomport(thisShimmer, char(bufferSize));                          % Write the bufferSize char value to the Shimmer
                
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);       % Wait for acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: writebuffersize - Set buffer size response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
            elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                isWritten = false;
                disp('Warning: writebuffersize - Buffer size is currently not configurable for Shimmer3.');
            else
                isWritten = false;
                fprintf(strcat('Warning: writebuffersize - Cannot set buffer size for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writebuffersize
        
        function isWritten = writegsrrange(thisShimmer,gsrRange)
            % Writes GSR range for Shimmer - in Connected state
            if (strcmp(thisShimmer.State,'Connected'))
                
                if ((gsrRange == 0) || (gsrRange == 1) || (gsrRange == 2) || (gsrRange == 3) || (gsrRange == 4))
                    
                    clearreaddatabuffer(thisShimmer);                                    % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_GSR_RANGE_COMMAND);      % Send the SET_GSR_RANGE_COMMAND to the Shimmer
                    
                    writetocomport(thisShimmer, char(gsrRange));                         % Write the gsr range char value to the Shimmer
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);    % Wait for acknowledgment from Shimmer
                    
                    if (isWritten == false)
                        fprintf(strcat('Warning: writegsrrange - Set gsr range response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writegsrrange - Attempt to set gsr range failed due to a request to set the range to an \n'));
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    fprintf(strcat('Valid range settings are 0 (10kohm to 56kohm), 1 (56kohm to 220kohm), 2 (220kohm to 680kohm), 3 (680kohm to 4.7Mohm) and 4 (Auto Range).\n'));
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writegsrrange - Cannot set gsr range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writegsrrange        
        
        function isWritten = writeexgconfiguration(thisShimmer,exgCon,chipIdentifier)
            % Writes (all) ExG configuration bytes to Shimmer3 - in Connected state
            if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: writeexgconfiguration - Command not supported for this firmware version, please update firmware.')
                isWritten = false;
            elseif(~(chipIdentifier == 1 || chipIdentifier == 2))
                disp('Warning: writeexgconfiguration - Invalid chip selection.');
                isWritten = false;
            elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: writeexgconfiguration - Command not supported for Shimmer2/2r.');
                isWritten = false;
            else
                if (strcmp(thisShimmer.State,'Connected') && thisShimmer.FirmwareCompatibilityCode >= 3)
                    
                    if (length(exgCon)==10)
                        clearreaddatabuffer(thisShimmer);                                   % As a precaution always clear the read data buffer before a write
                        writetocomport(thisShimmer, thisShimmer.SET_EXG_REGS_COMMAND);      % Send the SET_EXG_REGS_COMMAND to the Shimmer
                        writetocomport(thisShimmer, char(chipIdentifier-1));                % char(0) selects SENSOR_EXG1, char(1) selects SENSOR_EXG2
                        writetocomport(thisShimmer, char(0));                               % Start at byte 0
                        writetocomport(thisShimmer, char(10));                              % and write 10 bytes
                        for i=1:10
                            writetocomport(thisShimmer, char(exgCon(i)));                   % Write the ExG configuration bytes to the Shimmer
                        end
                        isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                        
                        if (isWritten == false)
                            fprintf(strcat('Warning: writeexgconfiguration - Set ExG Regs response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        end
                    else
                        isWritten = false;
                        fprintf(strcat('Warning: writeexgconfiguration - Attempt to write ExG configuration failed due to a request to set an \n'));
                        fprintf(strcat('invalid ExG configuration for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writeexgconfiguration - Cannot set exg rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                end
            end
        end % function writeexgconfiguration
                      
        function isWritten = writeexgrate(thisShimmer, exgRate, chipIdentifier)            % function write exgrate 
            % Writes ExG data rate to Shimmer3 - in Connected state
            if (chipIdentifier == 1 || chipIdentifier == 2)
           
                if ((exgRate == 1) || (exgRate == 2) || (exgRate == 3) || (exgRate == 4) || (exgRate == 5) || (exgRate == 6)  || (exgRate == 0)) % check for valid settings
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                            fprintf('Warning: writeexgrate - Command not supported for this firmware version, please update firmware.\n')
                            isWritten = false;
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            fprintf('Warning: writeexgrate - Command not supported for Shimmer2/2r.\n');
                            isWritten = false;
                        elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.FirmwareCompatibilityCode >= 3)
                      
                        if (chipIdentifier ==1)
                            EXG1Config1ClearedDataRateBits = thisShimmer.EXG1Config1;
                            EXG1Config1ClearedDataRateBits =  bitand((EXG1Config1ClearedDataRateBits),248);   % Clear the data rate bits of the EXG1Config1 byte
                            EXGConfig1UpdatedDataRateBits = bitor((EXG1Config1ClearedDataRateBits), exgRate); % Updated EXG1Config1 byte
                        else
                            EXG2Config1ClearedDataRateBits = thisShimmer.EXG2Config1;
                            EXG2Config1ClearedDataRateBits = bitand((EXG2Config1ClearedDataRateBits),248);    % Clear the data rate bits of the EXG1Config1 byte
                            EXGConfig1UpdatedDataRateBits = bitor((EXG2Config1ClearedDataRateBits), exgRate); % Updated EXG2Config1 byte
                        end
                                                                                
                            clearreaddatabuffer(thisShimmer);                                   % As a precaution always clear the read data buffer before a write
                            writetocomport(thisShimmer, thisShimmer.SET_EXG_REGS_COMMAND);      % Send the SET_EXG_REGS_COMMAND to the Shimmer
                            writetocomport(thisShimmer, char(chipIdentifier-1));                % char(0) selects SENSOR_EXG1, char(1) selects SENSOR_EXG2
                            writetocomport(thisShimmer, char(0));                               % Start at byte 0
                            writetocomport(thisShimmer, char(1));                               % and write 1 byte
                            writetocomport(thisShimmer, char(EXGConfig1UpdatedDataRateBits));   % Write the updated ExG configuration byte to the Shimmer

                            isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer

                        if (isWritten == false)
                            fprintf(strcat('Warning: writeexgrate - Set ExG Regs response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        end
               
                    else
                        isWritten = false;
                        fprintf(strcat('Warning: writeexgrate - Cannot set exg rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected\n'));
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writeexgrate - Attempt to set exg rate failed due to a request to set the rate to an \n'));
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    fprintf(strcat('Valid rate settings are 0 (125 Hz), 1 (250 Hz), 2 (500 Hz), 3 (1000 Hz), 4 (2000 Hz), 5 (4000 Hz) and 6 (8000 Hz).\n'));
                end
             else
                isWritten = false;
                fprintf(strcat('Warning: writeexgrate - Invalid chip selection.\n'));
             end
        end % function write exgrate
        
        function isWritten = writeexggain(thisShimmer, exgGain, chipIdentifier, channelIdentifier)            % function write exggain 
            % Writes ExG gain to Shimmer3 - in Connected state
            if (channelIdentifier == 1)
                % channelIdentifier 1
                if ~(chipIdentifier == 1 || chipIdentifier == 2)
                    isWritten = false;
                    fprintf(strcat('Warning: writeexggain - Invalid chip selection.\n'));
                else
                    if ((exgGain == 1) || (exgGain == 2) || (exgGain == 3) || (exgGain == 4) || (exgGain == 5) || (exgGain == 6)  || (exgGain == 0)) % check for valid settings
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                            fprintf('Warning: writeexggain - Command not supported for this firmware version, please update firmware.\n')
                            isWritten = false;
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            fprintf('Warning: writeexggain - Command not supported for Shimmer2/2r.\n');
                            isWritten = false;
                        elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.FirmwareCompatibilityCode >= 3)
                            
                            if (chipIdentifier == 1)
                                EXG1Ch1SetClearedGainBits = thisShimmer.EXG1Ch1Set;
                                EXG1Ch1SetClearedGainBits = bitand((EXG1Ch1SetClearedGainBits),143);    % Clear the gain bits of the EXG1Ch1Set byte
                                EXGCh1SetUpdatedGainBits = bitor((EXG1Ch1SetClearedGainBits), bitshift(exgGain,4));   % Updated EXG1Ch1Set byte
                            else
                                EXG2Ch1SetClearedGainBits = thisShimmer.EXG2Ch1Set;
                                EXG2Ch1SetClearedGainBits = bitand((EXG2Ch1SetClearedGainBits),143);    % Clear the gain bits of the EXG2Ch1Set byte
                                EXGCh1SetUpdatedGainBits = bitor((EXG2Ch1SetClearedGainBits), bitshift(exgGain,4));   % Updated EXG2Ch1Set byte
                            end
                            
                            clearreaddatabuffer(thisShimmer);                                   % As a precaution always clear the read data buffer before a write
                            writetocomport(thisShimmer, thisShimmer.SET_EXG_REGS_COMMAND);      % Send the SET_EXG_REGS_COMMAND to the Shimmer
                            writetocomport(thisShimmer, char(chipIdentifier-1));                % char(0) selects SENSOR_EXG1, char(1) selects SENSOR_EXG2
                            writetocomport(thisShimmer, char(3));                               % Start at byte 3
                            writetocomport(thisShimmer, char(1));                               % and write 1 byte
                            writetocomport(thisShimmer, char(EXGCh1SetUpdatedGainBits));    % Write the updated ExG configuration byte to the Shimmer
                            
                            isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                            
                            if (isWritten == false)
                                fprintf(strcat('Warning: writeexggain - Set ExG Regs response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            end
                            
                        else
                            isWritten = false;
                            fprintf(strcat('Warning: writeexggain - Cannot set exg gain for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                        end
                    else
                        isWritten = false;
                        fprintf(strcat('Warning: writeexggain - Attempt to set exg gain failed due to a request to set the rate to an \n'));
                        fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        fprintf(strcat('Valid gain settings are 0 (6x), 1 (1x), 2 (2x), 3 (3x), 4 (4x), 5 (8x) and 6 (12x).\n'));
                    end
                end
            elseif (channelIdentifier == 2)
            % channelIdentifier 2
                if (chipIdentifier == 1 || chipIdentifier == 2)

                    if ((exgGain == 1) || (exgGain == 2) || (exgGain == 3) || (exgGain == 4) || (exgGain == 5) || (exgGain == 6)  || (exgGain == 0)) % check for valid settings
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                            fprintf('Warning: writeexggain - Command not supported for this firmware version, please update firmware.\n')
                            isWritten = false;
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            fprintf('Warning: writeexggain - Command not supported for Shimmer2/2r.\n');
                            isWritten = false;
                        elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.FirmwareCompatibilityCode >= 3)

                            if (chipIdentifier == 1)
                                EXG1Ch2SetClearedGainBits = thisShimmer.EXG1Ch2Set;
                                EXG1Ch2SetClearedGainBits = bitand((EXG1Ch2SetClearedGainBits),143);    % Clear the gain bits of the EXG1Ch2Set byte
                                EXGCh2SetUpdatedGainBits = bitor((EXG1Ch2SetClearedGainBits), bitshift(exgGain,4));   % Updated EXG1Ch2Set byte
                            else
                                EXG2Ch2SetClearedGainBits = thisShimmer.EXG2Ch2Set;
                                EXG2Ch2SetClearedGainBits = bitand((EXG2Ch2SetClearedGainBits),143);    % Clear the gain bits of the EXG2Ch2Set byte
                                EXGCh2SetUpdatedGainBits = bitor((EXG2Ch2SetClearedGainBits), bitshift(exgGain,4));   % Updated EXG2Ch2Set byte
                            end

                                clearreaddatabuffer(thisShimmer);                                   % As a precaution always clear the read data buffer before a write
                                writetocomport(thisShimmer, thisShimmer.SET_EXG_REGS_COMMAND);      % Send the SET_EXG_REGS_COMMAND to the Shimmer
                                writetocomport(thisShimmer, char(chipIdentifier-1));                % char(0) selects SENSOR_EXG1, char(1) selects SENSOR_EXG2
                                writetocomport(thisShimmer, char(4));                               % Start at byte 4
                                writetocomport(thisShimmer, char(1));                               % and write 1 byte
                                writetocomport(thisShimmer, char(EXGCh2SetUpdatedGainBits));   % Write the updated ExG configuration byte to the Shimmer

                                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer

                            if (isWritten == false)
                                fprintf(strcat('Warning: writeexggain - Set ExG Regs response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            end

                        else
                            isWritten = false;
                            fprintf(strcat('Warning: writeexggain - Cannot set exg gain for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                        end
                    else
                        isWritten = false;
                        fprintf(strcat('Warning: writeexggain - Attempt to set exg gain failed due to a request to set the gain to an \n'));
                        fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        fprintf(strcat('Valid gain settings are 0 (6x), 1 (1x), 2 (2x), 3 (3x), 4 (4x), 5 (8x) and 6 (12x).\n'));
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writeexggain - Invalid chip selection.\n'));
                end
            else
                 isWritten = false;
                 fprintf(strcat('Warning: writeexggain - Invalid channel selection.\n'));
            end
        end % function write exggain
                
        function isWritten = writeexgreferenceelectrodeconfiguration(thisShimmer,exgReferenceElectrodeConfiguration)
            % WRITEEXGREFERENCEELECTRODECONFIGURATION - Writes exg reference electrode configuration
            %
            %   INPUT: exgReferenceElectrodeConfiguration - exg reference electrode configuration
            %
            %   OUTPUT: isWritten - isWritten is true if exg reference
            %                       electrode configuration is successfully
            %                       written.
            %
            %                       isWritten is false if writing has failed.
            if ~(strcmp(thisShimmer.State,'Connected'))
                isWritten = false;
                fprintf(strcat('Warning: writeexgreferenceelectrodeconfiguration - Cannot set exg rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: writeexgreferenceelectrodeconfiguration - Function only supported for Shimmer3.');
                isWritten = false;
            elseif(thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: writeexgreferenceelectrodeconfiguration - Command not supported for this firmware version, please update firmware.')
                isWritten = false;
            else
                EXG1RLD_Sens = thisShimmer.EXG1RLD_Sens;
                EXG1RLD_SensUpdatedElecRefConfBits = bitand(EXG1RLD_Sens,255-15);
                EXG1RLD_SensUpdatedElecRefConfBits = bitor(EXG1RLD_SensUpdatedElecRefConfBits,exgReferenceElectrodeConfiguration);  % Set only the bits that need to be set
                if (length(exgReferenceElectrodeConfiguration)==1 && exgReferenceElectrodeConfiguration < 16) % Check if configuration valid.
                    clearreaddatabuffer(thisShimmer);                                                         % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_EXG_REGS_COMMAND);                            % Send the SET_EXG_REGS_COMMAND to the Shimmer
                    writetocomport(thisShimmer, char(0));                                                     % char(0) selects SENSOR_EXG1
                    writetocomport(thisShimmer, char(5));                                                     % Start at byte 5
                    writetocomport(thisShimmer, char(1));                                                     % and write 1 byte
                    writetocomport(thisShimmer, char(EXG1RLD_SensUpdatedElecRefConfBits));                    % Write the updated ExG configuration byte to the Shimmer
                    
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);                         % Wait for Acknowledgment from Shimmer
                    
                    if (isWritten == false)
                        fprintf(strcat('Warning: writeexgreferenceelectrodeconfiguration - Set ExG Regs response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                else
                    isWritten = false;
                    fprintf('Warning: writeexgreferenceelectrodeconfiguration - Attempt to write ExG reference electrode configuration failed due to a request to set an \n');
                    fprintf(strcat('invalid ExG reference electrode configuration for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
            end
        end % function writeexgreferenceelectrodeconfiguration

        function isWritten = writeexgleadoffdetectioncurrent(thisShimmer,detectionCurrent,chipIdentifier)
            % WRITEEXGLEADOFFDETECTIONCURRENT - Writes exg lead-off
            % detection current
            %
            %   INPUT: detectionCurrent - detectionCurrent is a numerical
            %                             value. Valid are: 0, 4, 8 and 12.
            %
            %   INPUT: chipIdentifier -   chipIdentifier is either 1 or 2 to
            %                             select SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: isWritten -       isWritten is true if exg lead-off
            %                             detection current is successfully
            %                             written.
            %
            %                             isWritten is false if writing has failed.
            %
            if ~(strcmp(thisShimmer.State,'Connected'))
                isWritten = false;
                fprintf(strcat('Warning: writeexgleadoffdetectioncurrent - Cannot set ExG lead-off detection current for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: writeexgleadoffdetectioncurrent - Function only supported for Shimmer3.');
                isWritten = false;
            elseif(thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: writeexgleadoffdetectioncurrent - Command not supported for this firmware version, please update firmware.')
                isWritten = false;
            elseif ~(chipIdentifier == 1 || chipIdentifier == 2)
                disp('Warning: writeexgleadoffdetectioncurrent - Invalid value for chipIdentifier.')
                isWritten = false;
            elseif ~(detectionCurrent == 0 || detectionCurrent == 4 || detectionCurrent == 8 || detectionCurrent == 12)
                disp('Warning: writeexgleadoffdetectioncurrent - Invalid value for detectionCurrent.')
                isWritten = false;
            else
                if (chipIdentifier == 1)
                    EXG1Loff = thisShimmer.EXG1Loff;
                    EXGLoff_SensUpdatedElecRefConfBits = bitand(EXG1Loff,255-15); % clear bits first
                    EXGLoff_SensUpdatedElecRefConfBits = bitor(EXGLoff_SensUpdatedElecRefConfBits,detectionCurrent); % Set the detectionCurrent bits of EXG1Loff
                elseif (chipIdentifier == 2)
                    EXG2Loff = thisShimmer.EXG2Loff;
                    EXGLoff_SensUpdatedElecRefConfBits = bitand(EXG2Loff,255-15); % clear bits first
                    EXGLoff_SensUpdatedElecRefConfBits = bitor(EXGLoff_SensUpdatedElecRefConfBits,detectionCurrent); % Set the detectionCurrent bits of EXG2Loff
                end
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_EXG_REGS_COMMAND);          % Send the SET_EXG_REGS_COMMAND to the Shimmer
                writetocomport(thisShimmer, char(chipIdentifier-1));                      % char chipIdentifier selects SENSOR_EXG1/SENSOR_EXG2
                writetocomport(thisShimmer, char(2));                                   % Start at byte 2
                writetocomport(thisShimmer, char(1));                                   % and write 1 byte
                writetocomport(thisShimmer, char(EXGLoff_SensUpdatedElecRefConfBits));  % Write the updated ExG configuration byte to the Shimmer
                
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);       % Wait for Acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: writeexgleadoffdetectioncurrent - Set ExG Regs response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
            end
        end % function writeexgleadoffdetectioncurrent

        function isWritten = writeexgleadoffcomparatorthreshold(thisShimmer,comparatorThreshold,chipIdentifier)
            % WRITEEXGLEADOFFCOMPARATORTHRESHOLD - Writes exg lead-off
            % comparator threshold
            %
            %   INPUT: comparatorThreshold - detectionCurrent is a numerical
            %                                value. Valid are: 0, 32, 64, 96,
            %                                128 160, 192, 224
            %
            %   INPUT: chipIdentifier -      chipIdentifier is either 1 or 2 to
            %                                select SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: isWritten -          isWritten is true if exg lead-off
            %                                comparator threshold is successfully
            %                                written.
            %
            %                                isWritten is false if writing has failed.
            %
            if ~(strcmp(thisShimmer.State,'Connected'))
                isWritten = false;
                fprintf(strcat('Warning: writeexgleadoffcomparatorthreshold - Cannot set ExG lead-off comparator threshold for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: writeexgleadoffcomparatorthreshold - Function only supported for Shimmer3.');
                isWritten = false;
            elseif(thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: writeexgleadoffcomparatorthreshold - Command not supported for this firmware version, please update firmware.')
                isWritten = false;
            elseif ~(chipIdentifier == 1 || chipIdentifier == 2)
                disp('Warning: writeexgleadoffcomparatorthreshold - Invalid value for chipIdentifier.')
                isWritten = false;
            elseif ~(comparatorThreshold == 0 || comparatorThreshold == 32 || comparatorThreshold == 64 ...
                || comparatorThreshold == 96 || comparatorThreshold == 128 || comparatorThreshold == 160 ...
                || comparatorThreshold == 192 || comparatorThreshold == 224) 
                disp('Warning: writeexgleadoffcomparatorthreshold - Invalid value for detectionCurrent.')
                isWritten = false;
            else
                if (chipIdentifier == 1)
                    EXG1Loff = thisShimmer.EXG1Loff;
                    EXGLoff_SensUpdatedCompThresBits = bitand(EXG1Loff,255-224); % clear bits to set
                    EXGLoff_SensUpdatedCompThresBits = bitor(EXGLoff_SensUpdatedCompThresBits,comparatorThreshold); % Set the comparatorThreshold bits of EXG1Loff
                elseif (chipIdentifier == 2)
                    EXG2Loff = thisShimmer.EXG2Loff;
                    EXGLoff_SensUpdatedCompThresBits = bitand(EXG2Loff,255-224); % clear bits to set
                    EXGLoff_SensUpdatedCompThresBits = bitor(EXGLoff_SensUpdatedCompThresBits,comparatorThreshold); % Set the comparatorThreshold bits of EXG1Loff
                end
                    clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_EXG_REGS_COMMAND);          % Send the SET_EXG_REGS_COMMAND to the Shimmer
                    writetocomport(thisShimmer, char(chipIdentifier-1));                      % char chipIdentifier selects SENSOR_EXG1/SENSOR_EXG2
                    writetocomport(thisShimmer, char(2));                                   % Start at byte 2
                    writetocomport(thisShimmer, char(1));                                   % and write 1 byte
                    writetocomport(thisShimmer, char(EXGLoff_SensUpdatedCompThresBits));    % Write the updated ExG configuration byte to the Shimmer
                    
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);       % Wait for Acknowledgment from Shimmer
                    
                    if (isWritten == false)
                        fprintf(strcat('Warning: writeexgleadoffcomparatorthreshold - Set ExG Regs response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
            end
        end % function writeexgleadoffcomparatorthreshold

        function isWritten = writeexgleadoffdetectionmode(thisShimmer,detectionMode)
            % WRITEEXGLEADOFFDETECTIONMODE - Writes exg lead-off
            % detection mode
            %
            %   INPUT: detectionMode - detectionMode is a numerical
            %                          value. Valid are: 0 (Off) and 1 (DC Current)
            %
            %   OUTPUT: isWritten -    isWritten is true if exg lead-off
            %                          detection mode is successfully
            %                          written.
            %
            %                          isWritten is false if writing has failed.
            %
            if ~(strcmp(thisShimmer.State,'Connected'))
                isWritten = false;
                fprintf(strcat('Warning: writeexgleadoffdetectionmode - Cannot set ExG lead-off detection mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: writeexgleadoffdetectionmode - Function only supported for Shimmer3.');
                isWritten = false;
            elseif(thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: writeexgleadoffdetectionmode - Command not supported for this firmware version, please update firmware.')
                isWritten = false;
            elseif ~(detectionMode == 0 || detectionMode == 1)
                disp('Warning: writeexgleadoffdetectionmode - Invalid value for detectionMode.')
                isWritten = false;
            else
                chipselect = 0;
                while chipselect < 2;
                    
                    % see setexgconfiguration for byte numbers (bytes 0-9, bits 1-8)
                    
                    % SENSOR_EXG1
                    if (chipselect == 0)
                        
                        % select bytes and clear bits to set
                        EXG1Config2 = thisShimmer.EXG1Config2;                      % byte 1
                        EXGConfig2_Updated = bitand(EXG1Config2,255-64);            % clear bit 7
                        EXG1Loff = thisShimmer.EXG1Loff;                            % byte 2
                        EXGLoff_Updated = bitand(EXG1Loff,255-1);                   % clear bit 1
                        EXG1Ch1Set = thisShimmer.EXG1Ch1Set;                        % byte 3
                        EXGCh1Set_Updated = bitand(EXG1Ch1Set,255-128);             % clear bit 8
                        EXG1Ch2Set = thisShimmer.EXG1Ch2Set;                        % byte 4
                        EXGCh2Set_Updated = bitand(EXG1Ch2Set,255-128);             % clear bit 8
                        EXG1RLD_Sens = thisShimmer.EXG1RLD_Sens;                    % byte 5
                        EXGRLD_Sens_Updated = bitand(EXG1RLD_Sens,255-16) ;         % clear bit 5
                        EXG1LOFF_Sens = thisShimmer.EXG1LOFF_Sens;                  % byte 6
                        EXGLOFF_Sens_Updated = bitand(EXG1LOFF_Sens,255-7);         % clear bits 1-3
                        
                        % update the bytes
                        EXGConfig2_Updated = bitor(EXGConfig2_Updated, bitshift(detectionMode,6));    % Lead-off comparator power-down - bit 7 of byte 1  (0 = Lead-off comparators disabled (default), 1 = Lead-off comparators enabled)
                        EXGLoff_Updated = bitor(EXGLoff_Updated, 0);                                  % Lead-off frequency - bit 1 of byte 2 (0 = DC Current)
                        EXGCh1Set_Updated = bitor(EXGCh1Set_Updated, bitshift(1-detectionMode,7));    % Channel 1 power-down - bit 8 of byte 3 - (0 = Normal operation, 1 = Channel power-down)
                        EXGCh2Set_Updated = bitor(EXGCh2Set_Updated, bitshift(1-detectionMode,7));    % Channel 2 power-down - bit 8 of byte 4 - (0 = Normal operation, 1 = Channel power-down)
                        EXGRLD_Sens_Updated = bitor(EXGRLD_Sens_Updated, bitshift(detectionMode,4));  % RLD lead-off sense function - bit 5 of byte 5
                        EXGLOFF_Sens_Updated = bitor(EXGLOFF_Sens_Updated, detectionMode*7);          % Channel 2 lead-off detection positive inputs - bit 3 of byte 6, Channel 1 lead-off detection negative inputs - bit 2 of byte 6, Channel 1 lead-off detection positive inputs - bit 1 of byte 6
                        
 
                    % SENSOR_EXG2
                    else 
                        
                        % select bytes and clear bits to set
                        EXG2Config2 = thisShimmer.EXG2Config2;                      % byte 1
                        EXGConfig2_Updated = bitand(EXG2Config2,255-64);            % clear bit 7
                        EXG2Loff = thisShimmer.EXG2Loff;                            % byte 2
                        EXGLoff_Updated = bitand(EXG2Loff,255-1);                   % clear bit 1
                        EXG2Ch1Set = thisShimmer.EXG2Ch1Set;                        % byte 3
                        EXGCh1Set_Updated = bitand(EXG2Ch1Set,255-128);             % clear bit 8
                        EXG2Ch2Set = thisShimmer.EXG2Ch2Set;                        % byte 4
                        EXGCh2Set_Updated = bitand(EXG2Ch2Set,255-128);             % clear bit 8
                        EXG2RLD_Sens = thisShimmer.EXG2RLD_Sens;                    % byte 5
                        EXG2LOFF_Sens = thisShimmer.EXG2LOFF_Sens;                  % byte 6
                        EXGLOFF_Sens_Updated = bitand(EXG2LOFF_Sens,255-4);         % clear bits 3

                        % update the bytes
                        EXGConfig2_Updated = bitor(EXGConfig2_Updated, bitshift(detectionMode,6));    % Lead-off comparator power-down - bit 7 of byte 1  (0 = Lead-off comparators disabled (default), 1 = Lead-off comparators enabled)
                        EXGLoff_Updated = bitor(EXGLoff_Updated, 0);                                  % Lead-off frequency - bit 1 of byte 2 (0 = DC Current)
                        EXGCh1Set_Updated = bitor(EXGCh1Set_Updated, bitshift(1-detectionMode,7));    % Channel 1 power-down - bit 8 of byte 3 - (0 = Normal operation, 1 = Channel power-down)
                        EXGCh2Set_Updated = bitor(EXGCh2Set_Updated, bitshift(1-detectionMode,7));    % Channel 2 power-down - bit 8 of byte 4 - (0 = Normal operation, 1 = Channel power-down)
                        EXGRLD_Sens_Updated = EXG2RLD_Sens;                                           % byte 5 - no change for SENSOR_EXG2
                        EXGLOFF_Sens_Updated = bitor(EXGLOFF_Sens_Updated, detectionMode*4);          % Channel 2 lead-off detection positive inputs - bit 3 of byte 6
  
                    end
                    
                    clearreaddatabuffer(thisShimmer);                               % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_EXG_REGS_COMMAND);  % Send the SET_EXG_REGS_COMMAND to the Shimmer
                    writetocomport(thisShimmer, char(chipselect));                  % chipselect = 0/1 selects SENSOR_EXG1/SENSOR_EXG2
                    writetocomport(thisShimmer, char(1));                           % Start at byte 1
                    writetocomport(thisShimmer, char(6));                           % and write 6 bytes
                    writetocomport(thisShimmer, char(EXGConfig2_Updated));          % Write the updated ExG configuration byte to the Shimmer
                    writetocomport(thisShimmer, char(EXGLoff_Updated));             % Write the updated ExG configuration byte to the Shimmer
                    writetocomport(thisShimmer, char(EXGCh1Set_Updated));           % Write the updated ExG configuration byte to the Shimmer
                    writetocomport(thisShimmer, char(EXGCh2Set_Updated));           % Write the updated ExG configuration byte to the Shimmer
                    writetocomport(thisShimmer, char(EXGRLD_Sens_Updated));         % Write the updated ExG configuration byte to the Shimmer
                    writetocomport(thisShimmer, char(EXGLOFF_Sens_Updated));        % Write the updated ExG configuration byte to the Shimmer
                    
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT); % Wait for Acknowledgment from Shimmer
                    
                    
                    if (isWritten == false)
                        fprintf(strcat('Warning: writeexgleadoffdetectionmode - Set ExG Regs response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                    chipselect = chipselect + 1;
                end
                thisShimmer.setexgleadoffdetectioncurrent(4, 1);                    % set defaults, lead-off detection current = 22nA
                thisShimmer.setexgleadoffdetectioncurrent(4, 2);
                thisShimmer.setexgleadoffcomparatorthreshold(64,1);                 % set defaults, lead-off comparator threshold = Pos:90% - Neg:10%
                thisShimmer.setexgleadoffcomparatorthreshold(64,2);
                
                if (thisShimmer.EMGflag)                                            % enable/disable SENSOR_EXG2_24BIT / SENSOR_EXG2_16BIT in case EMG is enabled
                    enabledSensors = uint32(thisShimmer.EnabledSensors);
                    if (bitand(enabledSensors, hex2dec('10')))
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, detectionMode);
                    else % EMG 16BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, detectionMode);
                    end
                    
                    isWritten = writeenabledsensors(thisShimmer, uint32(enabledSensors)); % write updated enabled sensors (set/reset SENSOR_EXG2_24BIT or SENSOR_EXG2_16BIT)
                    
                    if (isWritten)
                        isRead = readenabledsensors(thisShimmer);                         % Following a succesful write, call the readenabledsensors function which updates the enabledSensors property with the current Shimmer enabled sensors setting
                        if (isRead)
                            isSet = (enabledSensors == thisShimmer.EnabledSensors);       % isSet will be equal to 1 if EnabledSensors property is equal to the requested setting
                        end
                    else
                        isSet = false;
                    end
                    
                    if (~isSet && bitand(enabledSensors, hex2dec('10')))                  % EMG 24BIT
                        disp('writeexgleadoffdetectionmode - SENSOR_EXG2_24BIT could not be set, lead-off detection won''t fully function.');
                    elseif (~isSet && bitand(enabledSensors, hex2dec('100000')))          % EMG 16BIT
                        disp('writeexgleadoffdetectionmode - SENSOR_EXG2_16BIT could not be set, lead-off detection won''t fully function.');
                    end
                end
            end
        end % function writeexgleadoffdetectionmode
         
        function isWritten = writerealtimeclock(thisShimmer,systemTimeTicks)
            % Writes 8bytes Real Time Clock value to Shimmer3
            if  (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                isWritten = false;     
                fprintf('Warning: writerealtimeclock - Command not available for this hardware version.\n');
            elseif (thisShimmer.FirmwareCompatibilityCode < 6)
                isWritten = false;     
                fprintf('Warning: writerealtimeclock - Command not available for this firmware version.\n');
            elseif (~strcmp(thisShimmer.State,'Connected'))
                isWritten = false;              
                fprintf(strcat('Warning: writerealtimeclock - Cannot set Real Time Clock for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            else                       

                clearreaddatabuffer(thisShimmer);                                                    % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_RWC_COMMAND);                            % Send the Set Real Time Clock Command to the Shimmer

                % write 8 bytes System Time consecutively to Comport - little Endian:                
                bytesLittleEndian = typecast(systemTimeTicks,'uint8');    
                
                for iByte = 1:8
                  writetocomport(thisShimmer,char(bytesLittleEndian(iByte))); 
                end
                          
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);                    % Wait for Acknowledgment from Shimmer     
            end
        end % function writerealtimeclock
        
        function isWritten = writemagrange(thisShimmer,magRange)
            % Writes Magnetometer range for Shimmer2r or Shimmer3 - in Connected state
            if (thisShimmer.FirmwareCompatibilityCode == 0)
                fprintf(strcat('Warning: writemagrange - Command not supported for this firmware version, please update firmware.\n'));
            else
                if (strcmp(thisShimmer.State,'Connected'))
                    
                    if ((thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3...
                            && ((magRange == 0) || (magRange == 1) || (magRange == 2) || (magRange == 3) || (magRange == 4) || (magRange == 5) || (magRange == 6)))...
                            || ((thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.HardwareCompatibilityCode < 2)...
                            && ((magRange == 7) || (magRange == 1) || (magRange == 2) || (magRange == 3) || (magRange == 4) || (magRange == 5) || (magRange == 6))))
                        
                        clearreaddatabuffer(thisShimmer);                                    % As a precaution always clear the read data buffer before a write
                        writetocomport(thisShimmer, thisShimmer.SET_MAG_GAIN_COMMAND);       % Send the SET_MAG_GAIN_COMMAND to the Shimmer
                        
                        writetocomport(thisShimmer, char(magRange));                         % Write the magRange char value to the Shimmer
                        isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);    % Wait for Acknowledgment from Shimmer
                        
                        if (isWritten == false)
                            fprintf(strcat('Warning: writemagrange - Set mag range response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        end
                    else
                        isWritten = false;
                        fprintf(strcat('Warning: writemagrange - Attempt to set mag range failed due to a request to set the range to an \n'));
                        fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        fprintf(strcat('For Shimmer2r: Valid range settings are 0 (0.7 gauss), 1 (1.0 gauss), 2 (1.5 gauss), 3 (2.0 gauss), 4 (3.2 gauss), 5 (3.8 gauss) and 6 (4.5 gauss).\n'));
                        fprintf(strcat('For Shimmer3 with LSM303DLHC: Valid range settings are 1 (1.3 gauss), 2 (1.9 gauss), 3 (2.5 gauss), 4 (4.0 gauss), 5 (4.7 gauss), 6 (5.6 gauss) and 7 (8.1 gauss).\n'));
                        fprintf(strcat('For Shimmer3 with LSM303AHTR: Fixed ranges of 49.152 gauss.\n'));
                    end
                    
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writemagrange - Cannot set mag range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                end
            end
        end % function writemagrange
        
        function isWritten = writegyrorange(thisShimmer,gyroRange)
            % Writes Gyroscope range to Shimmer3 - in Connected state
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                
                if ((gyroRange == 0) || (gyroRange == 1) || (gyroRange == 2) || (gyroRange == 3))
                    
                    clearreaddatabuffer(thisShimmer);                                             % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_MPU9150_GYRO_RANGE_COMMAND);      % Send the SET_MPU9150_GYRO_RANGE_COMMAND to the Shimmer
                    
                    writetocomport(thisShimmer, char(gyroRange));                                 % Write the gyroRange char value to the Shimmer
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);             % Wait for Acknowledgment from Shimmer
                    
                    if (isWritten == false)
                        fprintf(strcat('Warning: writegyrorange - Set gyro range response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writegyrorange - Attempt to set gyro range failed due to a request to set the range to an \n'));
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    fprintf(strcat('Valid range settings are 0 (250 dps), 1 (500 dps), 2 (1000 dps), 3 (2000 dps) \n'));
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writegyrorange - Cannot set gyro range for COM ',thisShimmer.ComPort,' as Shimmer is not connected\n'));
                fprintf('or Shimmer is not a Shimmer3.\n');
            end
            
        end % function writegyrorange
        
        function isWritten = writepressureresolution(thisShimmer,pressureRes)
            % Writes Pressure sensor resolution to Shimmer3 - in Connected state
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 2 )
                disp('Warning: writepressureresolution - Command not supported for this firmware version, please update firmware.')
            elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                
                if ((pressureRes == 0) || (pressureRes == 1) || (pressureRes == 2) || (pressureRes == 3))
                    
                    clearreaddatabuffer(thisShimmer);                                    % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_BMP180_PRES_RESOLUTION_COMMAND);
                    
                    writetocomport(thisShimmer, char(pressureRes));
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);
                    
                    if (isWritten == false)
                        fprintf(strcat('Warning: writepressureresolution - Set pressure resolution response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    else
                        thisShimmer.PressureResolution = pressureRes;
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writepressureresolution - Attempt to set pressure resolution failed due to a request to set the resolution to an \n'));
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    fprintf(strcat('Valid resolution settings are 0 ("Low"), 1 ("Standard"), 2 ("High"), 3 ("Very High" / "Ultra High".\n'));
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writepressureresolution - Cannot set pressure resolution for COM ',thisShimmer.ComPort,' as Shimmer is not connected\n'));
                fprintf('or Shimmer is not Shimmer3. Pressure resolution is only supported for Shimmer3.\n');
            end
            
        end % function writepressureresolution
                
        function isWritten = writemagrate(thisShimmer,magRate)
            % Writes Magnetometer data rate to Shimmer2r or Shimmer3 - in Connected state
            if (strcmp(thisShimmer.State,'Connected'))
                
                if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3...
                        && ((magRate == 0) || (magRate == 1) || (magRate == 2) || (magRate == 3) || (magRate == 4) || (magRate == 5) || (magRate == 6))...
                        || (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.HardwareCompatibilityCode < 2)...
                        && (magRate == 0) || (magRate == 1) || (magRate == 2) || (magRate == 3) || (magRate == 4) || (magRate == 5) || (magRate == 6)  || (magRate == 7)...
                        || (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.HardwareCompatibilityCode >= 2)...
                        && (magRate == 0) || (magRate == 1) || (magRate == 2) || (magRate == 3))
                    
                    clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_MAG_SAMPLING_RATE_COMMAND); % Send the Set Mag Rate Command to the Shimmer
                    
                    writetocomport(thisShimmer, char(magRate));                             % Write the mag rate char value to the Shimmer
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);       % Wait for Acknowledgment from Shimmer
                    
                    if (isWritten == false)
                        fprintf(strcat('Warning: writemagrate - Set mag rate response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writemagrate - Attempt to set mag rate failed due to a request to set the range to an \n'));
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    fprintf('For Shimmer2r: Valid rate settings are 0 (0.5 Hz), 1 (1.0 Hz), 2 (2.0 Hz), 3 (5.0 Hz), 4 (10.0 Hz), 5 (20.0 Hz) and 6 (50.0 Hz).\n');
                    fprintf('For Shimmer3 with LSM303DLHC: Valid rate settings are 0 (0.75Hz), 1 (1.5Hz), 2 (3Hz), 3 (7.5Hz), 4 (15Hz), 5 (30Hz), 6 (75Hz), 7 (220Hz).\n');
                    fprintf('For Shimmer3 with LSM303AHTR: Valid rate settings are 0 (10.0Hz), 1 (20.0Hz)), 2 (50.0Hz), 3 (100.0Hz).\n');
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writemagrate - Cannot set mag rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writemagrate      
        
        function isWritten = writeaccelrate(thisShimmer,accelRate)
            % Writes LSM303DLHC/LSM303AHTR accelerometer data rate for Shimmer3 - in Connected state
            if (strcmp(thisShimmer.State,'Connected'))
                if (((thisShimmer.HardwareCompatibilityCode < 2) && (accelRate == 1) || (accelRate == 2) || (accelRate == 3) || (accelRate == 4) || (accelRate == 5)...
                        || (accelRate == 6) || (accelRate == 7) || (accelRate == 9))...
                        || ((thisShimmer.HardwareCompatibilityCode >= 2) && (accelRate == 1) || (accelRate == 2) || (accelRate == 3) || (accelRate == 4) || (accelRate == 5)...
                        || (accelRate == 6) || (accelRate == 7) || (accelRate == 8) || (accelRate == 9) || (accelRate == 10)))
                                           
                    clearreaddatabuffer(thisShimmer);                                                     % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND);  % Send the Set Mag Rate Command to the Shimmer
                    
                    writetocomport(thisShimmer, char(accelRate));                                         % Write the mag rate char value to the Shimmer
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);                     % Wait for Acknowledgment from Shimmer
                    
                    if (isWritten == false)
                        fprintf(strcat('Warning: writeaccelrate - Set acc rate response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writeaccelrate - Attempt to set acc rate failed due to a request to set the range to an \n'));
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    if (thisShimmer.HardwareCompatibilityCode < 2)
                        fprintf(strcat('Valid rate settings are 1 (1.0 Hz), 2 (10.0 Hz), 3 (25.0 Hz), 4 (50.0 Hz), 5 (100.0 Hz), 6 (200.0 Hz), 7 (400.0 Hz), 9 (1344.0 Hz).\n'));
                    elseif (thisShimmer.HardwareCompatibilityCode >= 2)
                        fprintf(strcat('Valid rate settings are  1 (12.5 Hz), 2 (25.0 Hz), 3 (50.0 Hz), 4 (100.0 Hz), 5 (200.0 Hz), 6 (400.0 Hz), 7 (800.0 Hz), 8 (1600.0 Hz), 9 (3200.0Hz) and 10 (6400.0Hz).\n'));
                    end
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writeaccelrate - Cannot set accel rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeaccelrate
        
        function isWritten = writegyrorate(thisShimmer,gyroRate)
            % Writes MPU9150 gyroscope data rate for Shimmer3 - in Connected state
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                
                if (gyroRate>=0 && gyroRate<=255)
                    
                    clearreaddatabuffer(thisShimmer);                                           % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_MPU9150_SAMPLING_RATE_COMMAND); % Send the Set Gyro Rate Command to the Shimmer
                    
                    writetocomport(thisShimmer, char(gyroRate));                                % Write the gyroRate char value to the Shimmer
                    isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);           % Wait for acknowledgment from Shimmer
                    
                    if (isWritten == false)
                        fprintf(strcat('Warning: writegyrorate - Set gyro rate response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    end
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writegyrorate - Attempt to set gyro rate failed due to a request to set the range to an \n'));
                    fprintf(strcat('invalid setting for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    fprintf(strcat('Valid range settings are 0 - 255, see datasheet of MPU9150 for further info'));
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writegyrorate - Cannot set gyro rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected\n'));
                fprintf('or Shimmer is not a Shimmer3. Gyro rate is only supported for Shimmer3.\n');
            end
            
        end % function writegyrorate
        
        function isWritten = writeledblink(thisShimmer,blink)
            % Write Set Blink LED command to Shimmer - in Connected state
            if (thisShimmer.FirmwareIdentifier == 3)
                disp('Warning: setledblink - Not supported for LogAndStream.');
                
            elseif (strcmp(thisShimmer.State,'Connected')||strcmp(thisShimmer.State,'Streaming'))
                clearreaddatabuffer(thisShimmer);                                 % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_BLINK_LED);           % Send the Set Blink LED Command to the Shimmer
                writetocomport(thisShimmer, char(blink));                         % Write the blink char value to the Shimmer
                
                isWritten = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT); % Wait for Acknowledgment from Shimmer
                
                if (isWritten == false)
                    fprintf(strcat('Warning: writeledblink - Set led blink response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
                
                isWritten = true;
            else
                isWritten = false;
                fprintf(strcat('Warning: writeledblink - Cannot set LED blink command for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function writeledblink
        
        function isWritten = writeledblinkwhilestreaming(thisShimmer,blink) %this should be a private function only used while streaming internally by the driver, to set a warning led when the Shimmer device is low on battery
            % Write Set Blink LED command to Shimmer - in Streaming state
            if (thisShimmer.FirmwareIdentifier == 3)
                disp('Warning: setledblink - Not supported for LogAndStream.');
                
            elseif (thisShimmer.WriteLEDBLINKWAITFORACK==0)
                if (strcmp(thisShimmer.State,'Streaming'))
                    clearreaddatabuffer(thisShimmer);                                 % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.SET_BLINK_LED);           % Send the SET_BLINK_LED command to the Shimmer
                    writetocomport(thisShimmer, char(blink));                         % Write the blink char value to the Shimmer
                    
                    %%% Notify STREAMING that you are expecting an ACK in the
                    %%% data stream
                    
                    thisShimmer.WriteLEDBLINKWAITFORACK=thisShimmer.WriteLEDBLINKWAITFORACK+1;
                    isWritten = true;
                else
                    isWritten = false;
                    fprintf(strcat('Warning: writeledblinkwhilestreaming - Cannot set LED blink command for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                end
            else
                isWritten = false;
                fprintf(strcat('Warning: writeledblinkwhilestreaming - Cannot set LED blink command for COM ',thisShimmer.ComPort,' Shimmer as still awaiting ACK of last command.\n'));
            end
        end % function writeledblink while streaming
  
        function enabledSensors = determineenabledsensorsbytes(thisShimmer, settingsCellArray)
            % Determines which sensors should be enabled/disabled based on
            % the input settingsCellArray.
            enabledSensors = uint32(thisShimmer.EnabledSensors); 
            
            iSensor = 1;
            while iSensor < (length(settingsCellArray))
                
                
                sensorName = char(settingsCellArray(iSensor));
                enableBit = cell2mat(settingsCellArray(iSensor+1));
                
                switch sensorName
                    case('Accel')
                        if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ACCEL, enableBit);        % set the accel enabled setting to the value in enable bit
                            if (bitand(thisShimmer.EnabledSensors, thisShimmer.SENSOR_EXT_A0)>0 && ~strcmp(thisShimmer.ExternalBoard,'ExpBoard') && bitand(thisShimmer.EnabledSensors, thisShimmer.SENSOR_EXT_A7)>0 && enableBit==1)
                                disp('Warning: determineenabledsensorsbytes - Note that streaming accelerometer and battery monitoring data simultaneously may cause crosstalk to occur.'); % because of two adjacent ADC channels
                            end
                        else
                           enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_A_ACCEL, enableBit);        % set the accel enabled setting to the value in enable bit
                        end
                        
                    case('LowNoiseAccel')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                           enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_A_ACCEL, enableBit);        % set the accel enabled setting to the value in enable bit
                        else
                            if(enableBit)
                                disp('Warning: determineenabledsensorsbytes - Shimmer2/2r does not support Low Noise Accel.');
                            end
                        end
                        
                    case('WideRangeAccel')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_WR_ACCEL, enableBit);       % set the accel enabled setting to the value in enable bit
                        else
                            if(enableBit)
                                disp('Warning: determineenabledsensorsbytes - Shimmer2/2r does not support Wide Range Accel.');
                            end
                        end
                        
                    case('AlternativeAccel')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                             enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MPU9150_ACCEL, enableBit);       % set the accel enabled setting to the value in enable bit
                             if(enableBit)
                                 disp('Warning: determineenabledsensorsbytes - Note that calibration of the Alternative Accelerometer is not supported for the moment, please use wide range accel or low noise accel.');
                             end
                        else
                            if(enableBit)
                                disp('Warning: determineenabledsensorsbytes - Shimmer2/2r does not support Alternative Accel');
                            end
                        end
                         
                    case('Gyro')
                        if ( thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && (strcmp(thisShimmer.InternalBoard,'Gyro') || strcmp(thisShimmer.InternalBoard,'9DOF')))
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, enableBit);        % set the Gyro enabled setting to the value in enable bit
                            if ( enableBit == 1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable the SENSOR_ECG
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable the SENSOR_EMG
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);             % disable the GSR
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);          % disable the Strain Gauge
                            end
                        elseif( thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 )
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, enableBit);        % set the Gyro enabled setting to the value in enable bit
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the Gyro sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to Gyro or 9DOF.\n'));
                            end
                        end
                        
                    case('Mag')
                        if ( thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && (strcmp(thisShimmer.InternalBoard,'Mag') || strcmp(thisShimmer.InternalBoard,'9DOF')))
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, enableBit);         % set the Mag enabled setting to the value in enable bit
                            if ( enableBit == 1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable the SENSOR_ECG
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable the SENSOR_EMG
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);             % disable the GSR
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);          % disable the Strain Gauge
                            end
                        elseif( thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 )
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, enableBit);         % set the Gyro enabled setting to the value in enable bit
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the Mag sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to Mag or 9DOF.\n'));
                            end
                        end
                        
                    case('AlternativeMag')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MPU9150_MAG, enableBit); % set the accel enabled setting to the value in enable bit
                            if(enableBit)
                                disp('Warning: determineenabledsensorsbytes - Note that calibration of the Alternative Magnetometer is not supported for the moment, please use Mag');
                            end
                        else
                            if(enableBit)
                                disp('Warning: determineenabledsensorsbytes - Shimmer2/2r does not support Alternative Mag');
                            end
                        end
                        
                    case({'ECG', 'ECG 24BIT'})
                        if (strcmp(thisShimmer.InternalBoard,'ECG') || strcmp(thisShimmer.InternalBoard,'EXG'))
                            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, enableBit);     % set the SENSOR_EXG1_24BIT enabled setting to the value in enable bit 
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, enableBit);     % set the SENSOR_EXG2_24BIT enabled setting to the value in enable bit 
                                if ( enableBit == 1)
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, 0);         % disable SENSOR_EXG1_16BIT
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);         % disable SENSOR_EXG2_16BIT
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);             % disable Int A1
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);            % disable Int A12
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);            % disable Int A13
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);            % disable Int A14
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                % disable the GSR
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);         % disable the Bridge Amplifier
                                    thisShimmer.setdefaultecgparameters;                                                       % set default parameters
                                end
                                thisShimmer.EMGflag = false;
                                thisShimmer.ECGflag = enableBit;
                            else % Shimmer 2r
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, enableBit);            % set the SENSOR_ECG enabled setting to the value in enable bit
                                if ( enableBit == 1)
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                % disable the GSR
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);             % disable the Strain Gauge
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);               % disable the Gyro
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);                % disable the Mag
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);                % disable EMG
                                end
                            end
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the ECG sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to ECG or EXG.\n'));
                            end
                        end
                        
                    case({'EMG', 'EMG 24BIT'})  
                        if (strcmp(thisShimmer.InternalBoard,'EMG') || strcmp(thisShimmer.InternalBoard,'EXG'))
                            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, enableBit);     % set the SENSOR_EXG1_24BIT enabled setting to the value in enable bit 
                                if ( enableBit == 1)
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);         % disable SENSOR_EXG2_24BIT 
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, 0);         % disable SENSOR_EXG1_16BIT
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);         % disable SENSOR_EXG2_16BIT   
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);             % disable Int A1
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);            % disable Int A12
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);            % disable Int A13
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);            % disable Int A14
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                % disable the GSR
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);         % disable the Bridge Amplifier                           
                                    thisShimmer.setdefaultemgparameters;                                                       % set default parameters
                                end
                                thisShimmer.EMGflag = enableBit;
                                thisShimmer.ECGflag = false;
                            else % Shimmer 2r
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, enableBit);            % set the SENSOR_EMG enabled setting to the value in enable bit 
                                if ( enableBit == 1)
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                % disable the GSR
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);             % disable the Strain Gauge                           
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);               % disable the Gyro
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);                % disable the Mag
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);                % disable ECG
                                end
                            end
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EMG sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to EMG or EXG.\n'));
                            end
                        end
                        
                    case('ECG 16BIT') 
                        if ((strcmp(thisShimmer.InternalBoard,'ECG') || strcmp(thisShimmer.InternalBoard,'EXG')) && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, enableBit);        % set the SENSOR_EXG1_16BIT enabled setting to the value in enable bit 
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, enableBit);        % set the SENSOR_EXG2_16BIT enabled setting to the value in enable bit 
                            if ( enableBit == 1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, 0);            % disable SENSOR_EXG1_24BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);            % disable SENSOR_EXG2_24BIT   
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);                % disable Int A1
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);               % disable Int A12
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);               % disable Int A13
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);               % disable Int A14
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                   % disable the GSR
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);            % disable the Bridge Amplifier                             
                                thisShimmer.setdefaultecgparameters;                                                          % set default parameters
                                thisShimmer.EMGflag = false;
                                thisShimmer.ECGflag = enableBit;
                            end
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the ECG 16BIT sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to ECG or EXG.\n'));
                                fprintf(strcat('Warning: determineenabledsensorsbytes - ECG 16BIT is only supported on Shimmer3.\n'));
                            end
                        end
                    
                    case('EMG 16BIT') 
                        if ((strcmp(thisShimmer.InternalBoard,'EMG') || strcmp(thisShimmer.InternalBoard,'EXG')) && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, enableBit);        % set the SENSOR_EXG1_16BIT enabled setting to the value in enable bit 
                            if ( enableBit == 1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);            % disable SENSOR_EXG2_16BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, 0);            % disable SENSOR_EXG1_24BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);            % disable SENSOR_EXG2_24BIT   
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);                % disable Int A1
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);               % disable Int A12
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);               % disable Int A13
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);               % disable Int A14
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                   % disable the GSR
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);            % disable the Bridge Amplifier                             
                                thisShimmer.setdefaultemgparameters;                                                          % set default parameters
                            end
                            thisShimmer.EMGflag = enableBit;
                            thisShimmer.ECGflag = false;
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EMG 16BIT sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to EMG or EXG.\n'));
                                fprintf(strcat('Warning: determineenabledsensorsbytes - EMG 16BIT is only supported on Shimmer3.\n'));
                            end
                        end
                        
                    case({'EXG1', 'EXG1 24BIT'})  
                        if ((strcmp(thisShimmer.InternalBoard,'ECG') || strcmp(thisShimmer.InternalBoard,'EMG') || strcmp(thisShimmer.InternalBoard,'EXG')) && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)  
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, enableBit);        % set the SENSOR_EXG1_24BIT enabled setting to the value in enable bit 
                            if ( enableBit == 1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, 0);            % disable SENSOR_EXG1_16BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);            % disable SENSOR_EXG2_16BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);                % disable Int A1
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);               % disable Int A12
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);               % disable Int A13
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);               % disable Int A14
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                   % disable the GSR
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);            % disable the Bridge Amplifier     
                                fprintf(strcat('Warning: determineenabledsensorsbytes - EXG1 is enabled but configuration parameters must be separately specified.\n'));
                            end
                            thisShimmer.EMGflag = false;
                            thisShimmer.ECGflag = false;
                            if (bitget(enabledSensors, thisShimmer.SENSOR_EXG2_24BIT))
                                fprintf(strcat('Warning: determineenabledsensorsbytes - EXG2 (24BIT) may be enabled from a previous configuration; it should be explicitly disabled if not required.\n'));
                            end
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && enableBit==1)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - EXG1 (24BIT) is only supported on Shimmer3.\n'));
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EXG1 sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to EXG, ECG or EMG.\n'));
                            end
                        end
                        
                    case({'EXG2', 'EXG2 24BIT'})  
                        if ((strcmp(thisShimmer.InternalBoard,'ECG') || strcmp(thisShimmer.InternalBoard,'EMG') || strcmp(thisShimmer.InternalBoard,'EXG')) && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)  
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, enableBit);        % set the SENSOR_EXG1_24BIT enabled setting to the value in enable bit 
                            if ( enableBit == 1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, 0);            % disable SENSOR_EXG1_16BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);            % disable SENSOR_EXG2_16BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);                % disable Int A1
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);               % disable Int A12
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);               % disable Int A13
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);               % disable Int A14
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                   % disable the GSR
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);            % disable the Bridge Amplifier                         
                                fprintf(strcat('Warning: determineenabledsensorsbytes - EXG2 is enabled but configuration parameters must be separately specified.\n'));
                            end
                            thisShimmer.EMGflag = false;
                            thisShimmer.ECGflag = false;
                            if (bitget(enabledSensors, thisShimmer.SENSOR_EXG1_24BIT))
                                fprintf(strcat('Warning: determineenabledsensorsbytes - EXG1 (24BIT) may be enabled from a previous configuration; it should be explicitly disabled if not required.\n'));
                            end
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && enableBit==1)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - EXG2 (24BIT) is only supported on Shimmer3.\n'));
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EXG2 sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to EXG, ECG or EMG.\n'));
                            end
                        end
                        
                    case('EXG1 16BIT')  
                        if ((strcmp(thisShimmer.InternalBoard,'ECG') || strcmp(thisShimmer.InternalBoard,'EMG') || strcmp(thisShimmer.InternalBoard,'EXG')) && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)  
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, enableBit);        % set the SENSOR_EXG1_16BIT enabled setting to the value in enable bit 
                            if ( enableBit == 1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, 0);            % disable SENSOR_EXG1_24BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);            % disable SENSOR_EXG2_24BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);                % disable Int A1
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);               % disable Int A12
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);               % disable Int A13
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);               % disable Int A14
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                   % disable the GSR
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);            % disable the Bridge Amplifier                           
                                fprintf(strcat('Warning: determineenabledsensorsbytes - EXG1 is enabled but configuration parameters must be separately specified.\n'));
                            end
                            thisShimmer.EMGflag = false;
                            thisShimmer.ECGflag = false;
                            if (bitget(enabledSensors, thisShimmer.SENSOR_EXG2_16BIT))
                                fprintf(strcat('Warning: determineenabledsensorsbytes - EXG2 (16BIT) may be enabled from a previous configuration; it should be explicitly disabled if not required.\n'));
                            end
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && enableBit == 1)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - EXG1 16BIT is only supported on Shimmer3.\n'));
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EXG1 16BIT sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to EXG, ECG or EMG.\n'));
                            end
                        end
                        
                    case('EXG2 16BIT')  
                        if ((strcmp(thisShimmer.InternalBoard,'ECG') || strcmp(thisShimmer.InternalBoard,'EMG') || strcmp(thisShimmer.InternalBoard,'EXG')) && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)  
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, enableBit);      % set the SENSOR_EXG2_16BIT enabled setting to the value in enable bit 
                            if ( enableBit == 1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, 0);          % disable SENSOR_EXG1_24BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);          % disable SENSOR_EXG2_24BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);              % disable Int A1
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);             % disable Int A12
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);             % disable Int A13
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);             % disable Int A14
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                 % disable the GSR
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);          % disable the Bridge Amplifier                           
                                fprintf(strcat('Warning: determineenabledsensorsbytes - EXG2 is enabled but configuration parameters must be separately specified.\n'));
                            end
                            thisShimmer.EMGflag = false;
                            thisShimmer.ECGflag = false;
                            if (bitget(enabledSensors, thisShimmer.SENSOR_EXG1_16BIT))
                                fprintf(strcat('Warning: determineenabledsensorsbytes - EXG1 (16BIT) may be enabled from a previous configuration; it should be explicitly disabled if not required.\n'));
                            end
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && enableBit==1)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - EXG2 16BIT is only supported on Shimmer3.\n'));
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EXG2 16BIT sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to EXG, ECG or EMG.\n'));
                            end
                        end
                        
                    case('GSR')
                        if (strcmp(thisShimmer.InternalBoard,'GSR'))
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, enableBit);                     % set the GSR enabled setting to the value in enable bit
                            if ( enableBit ==1 )
                                if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);                    % disable the Gyro
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);                     % disable the Mag                                
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);                     % disable SENSOR_ECG
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);                     % disable SENSOR_ECG     
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);                  % disable the Strain Gauge
                                else
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);                  % disable SENSOR_INT_A1
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);                 % disable SENSOR_INT_A14                                
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, 0);              % disable SENSOR_EXG1_16BIT
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);              % disable SENSOR_EXG2_16BIT                                
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, 0);              % disable SENSOR_EXG1_24BIT
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);              % disable SENSOR_EXG2_24BIT  
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);              % disable the Bridge Amplifier
                                end
                                
                            end
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the GSR sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to GSR.\n'));
                            end
                        end
                        
                    case({'ExpBoard_A7', 'EXT A7'})
                        if (strcmp(thisShimmer.ExternalBoard,'ExpBoard') || strcmp(thisShimmer.ExternalBoard,'External'))
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A7, enableBit);                  % set the ExpBoard7 enabled setting to the value in enable bit
                            if ( enableBit == 1 && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_HEART, 0);                       % disable the HR
                            end
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EXT A7 sensor on Shimmer COM', thisShimmer.ComPort, ' the external board must be set to ExpBoard or External.\n'));
                            end
                        end
                        
                    case('ExpBoard_A0')
                        if ((strcmp(thisShimmer.ExternalBoard,'ExpBoard') || strcmp(thisShimmer.ExternalBoard,'External')) && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A0, enableBit);                  % set the ExpBoard0 enabled setting to the value in enable bit
                            if ( enableBit == 1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_HEART, 0);                       % disable the HR
                            end
                        elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && enableBit==1)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - EXT A0 channel is not available on Shimmer3; use EXT A6.\n'));
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EXT A0 sensor on Shimmer COM', thisShimmer.ComPort, ' the external board must be set to ExpBoard or External.\n'));
                            end
                        end

                    case('EXT A6')
                        if ((strcmp(thisShimmer.ExternalBoard,'ExpBoard') || strcmp(thisShimmer.ExternalBoard,'External')) && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A6, enableBit);      % set the ExpBoard6 enabled setting to the value in enable bit
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && enableBit==1)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - EXT A6 channel is not available on Shimmer2/2r; use EXT A0.\n'));
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EXT A6 sensor on Shimmer COM', thisShimmer.ComPort, ' the external board must be set to ExpBoard or External.\n'));
                            end
                        end
                        
                    case('EXT A15')
                        if ((strcmp(thisShimmer.ExternalBoard,'ExpBoard') || strcmp(thisShimmer.ExternalBoard,'External')) && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A15, enableBit);      % set the ExpBoard15 enabled setting to the value in enable bit
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && enableBit==1)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - EXT A15 channel is not available on Shimmer2/2r.\n'));
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the EXT A15 sensor on Shimmer COM', thisShimmer.ComPort, ' the external board must be set to ExpBoard or External.\n'));
                            end
                        end
                        
                    case('Strain Gauge')
                        if (strcmp(thisShimmer.InternalBoard,'Strain Gauge') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, enableBit);     % set the Strain Gauge enabled setting to the value in enable bit
                            if ( enableBit ==1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);            % disable the Gyro
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);             % disable the Mag
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable the ECG
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable the EMG
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);             % disable the GSR
                            end
                        elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && enableBit==1)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - Strain Gauge is not available on Shimmer3, use Bridge Amplifier instead.\n'));
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the Strain Gauge sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to Strain Gauge.\n'));
                            end
                        end
                        
                    case('Bridge Amplifier')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 4 && enableBit ==1)
                            fprintf('Warning: determineenabledsensorsbytes - Bridge Amplifier is not supported for this firmware version, please update firmware.\n');
                        elseif (strcmp(thisShimmer.InternalBoard,'Bridge Amplifier') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, enableBit);      % set the Bridge Amplifier enabled setting to the value in enable bit
                            if ( enableBit ==1)
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);             % disable Int A12
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);             % disable Int A13
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);             % disable Int A14
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, 0);          % disable SENSOR_EXG1_16BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);          % disable SENSOR_EXG2_16BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, 0);          % disable SENSOR_EXG1_24BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);          % disable SENSOR_EXG2_24BIT
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                 % disable the GSR
                            end
                        elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && enableBit==1)
                            fprintf('Warning: determineenabledsensorsbytes - Bridge Amplifier is not available for Shimmer2/2r, use Strain Gauge instead.\n');
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the Bridge Amplifier sensor on Shimmer COM', thisShimmer.ComPort, ' the internal board must be set to Bridge Amplifier.\n'));
                            end
                        end
                        
                    case('Heart Rate')
                        if (strcmp(thisShimmer.ExternalBoard,'Heart Rate') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_HEART, enableBit);      % set the HR enabled setting to the value in enable bit
                            if ( enableBit ==1) 
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A7, 0);               % disable the ExpBoard7
                                enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A0, 0);               % disable the ExpBoard0
                            end
                        elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && enableBit==1)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - Heart Rate is not available on Shimmer3.\n'));
                        else
                            if(enableBit)
                                fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the HR sensor on Shimmer COM', thisShimmer.ComPort, ' the external board must be set to Heart Rate.\n'));
                            end
                        end
                        
                    case('BattVolt')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_VBATT, enableBit);      % set the BattVolt enabled setting to the value in enable bit
                        else
                            if(~strcmp(thisShimmer.ExternalBoard,'ExpBoard'))
                            % Check if pmux is set to 1
                                if (thisShimmer.setpmux(1))
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A7, enableBit);      % set the ExpBoard7 enabled setting to the value in enable bit
                                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A0, enableBit);      % set the ExpBoard0 enabled setting to the value in enable bit
                                    if ( enableBit == 1 )
                                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_HEART, 0);           % disable the HR
                                    end
                                    
                                    % check if accel sensor is enabled, if it is display crosstalk warning
                                    if (bitand(thisShimmer.EnabledSensors, thisShimmer.SENSOR_ACCEL)>0 && ~strcmp(thisShimmer.ExternalBoard,'ExpBoard') && enableBit==1)
                                        disp('Warning: determineenabledsensorsbytes - Note that streaming Accelerometer and Battery Monitoring data simultaneously may cause crosstalk to occur.');
                                    end
                                else
                                    if(enableBit)
                                        fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the Battery Voltage sensor on Shimmer COM', thisShimmer.ComPort, ' setpmux commmand failed.\n'));
                                    end
                                end
                            else
                                if(enableBit)
                                    fprintf(strcat('Warning: determineenabledsensorsbytes - To enable the Battery Voltage sensor on Shimmer COM', thisShimmer.ComPort, ' the external board must be set to None.\n'));
                                end
                            end
                        end
                        
                    case('INT A1')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && ~strcmp(thisShimmer.InternalBoard,'EXG')...
                                && ~strcmp(thisShimmer.InternalBoard,'ECG') && ~strcmp(thisShimmer.InternalBoard,'EMG')...
                                && ~strcmp(thisShimmer.InternalBoard,'GSR'))
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, enableBit);              % enable internal ADC A1, only if SENSOR_GSR and ExG sensors are disabled
                        else
                            if(enableBit)
                                fprintf('Warning: determineenabledsensorsbytes - INT A1 option only supported on Shimmer3,\n');
                                fprintf('when ExG and GSR sensors are disabled.');
                            end
                        end
                        
                    case('INT A12')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && ~strcmp(thisShimmer.InternalBoard,'EXG')...
                                && ~strcmp(thisShimmer.InternalBoard,'ECG') && ~strcmp(thisShimmer.InternalBoard,'EMG')...
                                && ~strcmp(thisShimmer.InternalBoard,'Bridge Amplifier'))
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, enableBit);              % enable internal ADC A12, only if SENSOR_BRIDGE_AMP and ExG sensors are disabled
                        else
                            if(enableBit)
                                fprintf('Warning: determineenabledsensorsbytes - INT A12 option only supported on Shimmer3,\n');
                                fprintf('when ExG and Bridge Amplifier sensors are disabled.');
                            end
                        end
                        
                    case('INT A13')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && ~strcmp(thisShimmer.InternalBoard,'EXG')...
                                && ~strcmp(thisShimmer.InternalBoard,'ECG') && ~strcmp(thisShimmer.InternalBoard,'EMG')...
                                && ~strcmp(thisShimmer.InternalBoard,'Bridge Amplifier'))
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, enableBit);              % enable internal ADC A13, only if SENSOR_BRIDGE_AMP and ExG sensors are disabled
                        else
                            if(enableBit)
                                fprintf('Warning: determineenabledsensorsbytes - INT A13 option only supported on Shimmer3,\n');
                                fprintf('when ExG and Bridge Amplifier sensors are disabled.');
                            end
                        end
                        
                    case('INT A14')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && ~strcmp(thisShimmer.InternalBoard,'EXG')...
                                && ~strcmp(thisShimmer.InternalBoard,'ECG') && ~strcmp(thisShimmer.InternalBoard,'EMG')...
                                && ~strcmp(thisShimmer.InternalBoard,'Bridge Amplifier') && ~strcmp(thisShimmer.InternalBoard,'GSR'))
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, enableBit);              % enable internal ADC A14, only if SENSOR_BRIDGE_AMP, SENSOR_GSR and ExG sensors are disabled
                        else
                            if(enableBit)
                                fprintf('Warning: determineenabledsensorsbytes - INT A14 option only supported on Shimmer3,\n');
                                fprintf('when ExG, GSR and Bridge Amplifier sensors are disabled.');
                            end
                        end
                        
                    case('Pressure')
                        if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 2)
                            fprintf('Warning: determineenabledsensorsbytes - Pressure option is not supported for this firmware version.\n');
                        elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                            enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BMP180_PRESSURE, enableBit);
                        else
                            if(enableBit)
                                fprintf('Warning: determineenabledsensorsbytes - Pressure option only supported on Shimmer3.\n');
                            end
                        end
                        
                    otherwise
                        if(enableBit)
                            fprintf(strcat('Warning: determineenabledsensorsbytes - Attempt to enable unrecognised sensor on Shimmer COM', thisShimmer.ComPort, '.\n'));
                        end
                        
                end
                
                iSensor = iSensor + 2;
            end
                    
            enabledSensors = disableunavailablesensors(thisShimmer, enabledSensors);    % Update enabledSensors value to disable unavailable sensors based on daughter board settings
            
        end %function determineenabledsensorsbytes
              
        function enabledSensors = disableunavailablesensors(thisShimmer, enabledSensors)
            % Disables conflicting sensors based on input enabledSensors.
            internalBoard = char(thisShimmer.InternalBoard);
            externalBoard = char(thisShimmer.ExternalBoard);
                        
            switch internalBoard
                case ('None')
                    if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2 || thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2R)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);            % disable SENSOR_GYRO
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);             % disable SENSOR_MAG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable SENSOR_EMG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable SENSOR_ECG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);          % disable SENSOR_STRAIN
                    elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, 0);      % disable SENSOR_EXG1_24BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);      % disable SENSOR_EXG2_24BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, 0);      % disable SENSOR_EXG1_16BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);      % disable SENSOR_EXG2_16BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);      % disable SENSOR_BRIDGE_AMP
                    end
                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                 % disable SENSOR_GSR
                    
                    
                case ('Gyro')
                    if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2 || thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2R)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);             % disable SENSOR_MAG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable SENSOR_ECG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable SENSOR_EMG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);             % disable SENSOR_GSR
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);          % disable SENSOR_STRAIN
                    end
                    
                case ('Mag')
                    if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2 || thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2R)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);            % disable SENSOR_MAG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable SENSOR_ECG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable SENSOR_EMG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);             % disable SENSOR_GSR
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);          % disable SENSOR_STRAIN
                    end
                    
                case ('9DOF')
                    if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2 || thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2R)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable SENSOR_ECG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable SENSOR_EMG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);             % disable SENSOR_GSR
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);          % disable SENSOR_STRAIN
                    end
                    
                case ('ECG')
                    if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2 || thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2R)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);            % disable SENSOR_GYRO
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);             % disable SENSOR_MAG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable the SENSOR_EMG for Shimmer2/2r; for Shimmer3 'ECG' both SENSOR_EXG1 and SENSOR_EXG2 need to be enabled.
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);          % disable SENSOR_STRAIN
                    elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);          % disable SENSOR_INT_A1
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);         % disable SENSOR_INT_A12
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);         % disable SENSOR_INT_A13
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);         % disable SENSOR_INT_A14
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);      % disable SENSOR_BRIDGE_AMP
                    end
                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                 % disable SENSOR_GSR
                                        
                case ('EMG')
                    if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2 || thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_2R)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);            % disable SENSOR_GYRO
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);             % disable SENSOR_MAG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable the SENSOR_ECG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);              % disable SENSOR_STRAIN
                    elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);          % disable SENSOR_INT_A1
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);         % disable SENSOR_INT_A12
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);         % disable SENSOR_INT_A13
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);         % disable SENSOR_INT_A14
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);      % disable SENSOR_EXG2_24BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);      % disable SENSOR_EXG2_16BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);      % disable SENSOR_BRIDGE_AMP
                    end
                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                 % disable SENSOR_GSR
                    
                case ('EXG')
                    if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);            % disable SENSOR_GYRO
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);             % disable SENSOR_MAG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);              % disable SENSOR_STRAIN
                    elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);          % disable SENSOR_INT_A1
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);         % disable SENSOR_INT_A12
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);         % disable SENSOR_INT_A13
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);         % disable SENSOR_INT_A14
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);              % disable SENSOR_BRIDGE_AMP
                    end
                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);                 % disable SENSOR_GSR
                    
                case ('GSR')
                    if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);            % disable SENSOR_GYRO
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);             % disable SENSOR_MAG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable the SENSOR_ECG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable the SENSOR_EMG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_STRAIN, 0);          % disable SENSOR_STRAIN
                    elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, 0);      % disable SENSOR_EXG1_24BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);      % disable SENSOR_EXG2_24BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, 0);      % disable SENSOR_EXG1_16BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);      % disable SENSOR_EXG2_16BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A1, 0);          % disable SENSOR_INT_A1
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);         % disable SENSOR_INT_A14
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_BRIDGE_AMP, 0);      % disable SENSOR_BRIDGE_AMP
                    end
                                
                case ('Strain Gauge')
                    if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GYRO, 0);            % disable SENSOR_GYRO
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_MAG, 0);             % disable SENSOR_MAG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_ECG, 0);             % disable the SENSOR_ECG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EMG, 0);             % disable the SENSOR_EMG
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);             % disable SENSOR_GSR
                    end
                    
                case ('Bridge Amplifier')
                    if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A12, 0);         % disable SENSOR_INT_A12
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A13, 0);         % disable SENSOR_INT_A13
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_INT_A14, 0);         % disable SENSOR_INT_A14
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_24BIT, 0);      % disable SENSOR_EXG1_24BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_24BIT, 0);      % disable SENSOR_EXG2_24BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG1_16BIT, 0);      % disable SENSOR_EXG1_16BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXG2_16BIT, 0);      % disable SENSOR_EXG2_16BIT
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_GSR, 0);             % disable SENSOR_GSR
                    end
            end
                      
            
            switch externalBoard
                case ('None')
                    if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_HEART, 0);               % disable SENSOR_HEART
                    end
                    if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 && thisShimmer.getpmux==0)           % if BattVolt is enabled don't do this
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A7, 0);          % disable SENSOR_EXT_A7
                        enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A0, 0);          % disable SENSOR_EXT_A1
                    end
                case ('ExpBoard')
                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_HEART, 0);               % disable SENSOR_HEART
                    
                case ('Heart Rate')
                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A7, 0);              % disable SENSOR_EXT_A7
                    enabledSensors = bitset(uint32(enabledSensors), thisShimmer.SENSOR_EXT_A0, 0);              % disable SENSOR_EXT_A1
            end
            
        end % function disableunavailablesensors
        
        function isWritten = writeenabledsensors (thisShimmer,enabledSensors)
            % Writes the enabled sensors bytes to Shimmer - in Connected state
            % Based on input enabledSensors this function writes
            % enabledSensorsLowByte and enabledSensorsHighByte for
            % Shimmer2/2r. For Shimmer3 it writes enabledSensorsLowByte,
            % enabledSensorsHighByte and enabledSensorsHigherByte.
            if (strcmp(thisShimmer.State,'Connected'))
                enabledSensorsLowByte = bitand(enabledSensors,255);                 % Extract the lower byte          % previously enabledSensorsLowByte = uint8(enabledSensors);
                enabledSensorsHighByte = bitand(bitshift(enabledSensors,-8),255);   % Extract the higher byte
               
                clearreaddatabuffer(thisShimmer);                                   % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.SET_SENSORS_COMMAND);       % Send the Set Sensors Command to the Shimmer
                writetocomport(thisShimmer, char(enabledSensorsLowByte));           % Write the enabled sensors lower byte value to the Shimmer
                writetocomport(thisShimmer, char(enabledSensorsHighByte));          % Write the enabled sensors higher byte value to the Shimmer
                if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    enabledSensorsHigherByte = bitand(bitshift(enabledSensors,-16),255);   % Extract the higher byte
                    writetocomport(thisShimmer, char(enabledSensorsHigherByte));           % Write the enabled sensors higher byte value to the Shimmer
                end
                isWritten = waitforack(thisShimmer, 8);                             % Wait for Acknowledgment from Shimmer
                if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                  inquiry(thisShimmer);                                             % get inquiry response of the Shimmer
                end
                if (isWritten == false)
                    fprintf(strcat('Warning: writeenabledsensors - Set enabled sensors response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                end
                
            else
                isWritten = false;
                fprintf(strcat('Warning: writeenabledsensors - Cannot set enabled sensors for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
         
        end % function writeenabledsensors
        
    end % methods (Access = 'private')
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Get Status Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = 'private')
        

 
        function isRead = readsamplingrate(thisShimmer)
            % Sends the GET_SAMPLING_RATE_COMMAND to Shimmer - in Connected state 
            % Receives the sampling rate and updates the SamplingRate property.
            if (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_SAMPLING_RATE_COMMAND);     % Send the Get Sampling Rate Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);  % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, inf);     % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.SAMPLING_RATE_RESPONSE)
                            if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                                if shimmerResponse(2) == 255                                % samplingRate == 0 is a special case, refer to 'Sampling Rate Table.txt' for more details
                                    thisShimmer.SamplingRate = 0;
                                else
                                    thisShimmer.SamplingRate = 1024 / double(shimmerResponse(2));   % Refer to 'Sampling Rate Table.txt' for more details
                                end
                            else
                               thisShimmer.SamplingRate = 32768/(bitand(double(shimmerResponse(2)),255)+bitshift(double(shimmerResponse(3)),8));
                            end
                            isRead = true;
                        else
                            thisShimmer.SamplingRate = 'Nan';              % Set the SamplingRate to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readsamplingrate - Get sampling rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.SamplingRate = 'Nan';              % Set the SamplingRate to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readsamplingrate - Get sampling rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.SamplingRate = 'Nan';                      % Set the SamplingRate to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readsamplingrate - Get sampling rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readsamplingrate - Cannot get sampling rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readsamplingrate
        
        function isRead = readbaudrate(thisShimmer)
            % Sends the GET_BT_COMMS_BAUD_RATE to Shimmer - in Connected state 
            % Receives the baud rate and updates the BaudRate property.
            if ~strcmp(thisShimmer.State,'Connected')
                fprintf(strcat('Warning: readbaudrate - Cannot set baud rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                isRead = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: readbaudrate - This function is only supported for Shimmer3.');
                isRead = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 5) 
                disp('Warning: readbaudrate - This function is not supported in this firmware version, please update firmware.');
                isRead = false;
            else
                clearreaddatabuffer(thisShimmer);                                       % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_BT_COMMS_BAUD_RATE);        % Send the Set Baud Rate Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);  % Wait for Acknowledgment from Shimmer
                if ~(isAcknowledged == true)
                    thisShimmer.BaudRate = 'Nan';                                       % Set the property to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readbaudrate - Get Baud Rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                else
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);     % Read the 2 bytes response from the realterm buffer
                    
                    if ( ~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.BT_COMMS_BAUD_RATE_RESPONSE) )
                        thisShimmer.BaudRate = shimmerResponse(2);                      % Update property BaudRate
                        isRead = true;
                    else
                        thisShimmer.BaudRate = 'Nan';                                   % Set the  to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readbaudrate - Get Baud Rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                end
            end
        end  % function readbaudrate
        
        function isRead = readfirmwareversion(thisShimmer)
            % Sends the GET_FW_VERSION_COMMAND to Shimmer - in Connected state 
            % Receives the firmware version and updates the corresponding properties.
            if (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_FW_VERSION_COMMAND);          % Send the Get Firmware Version Command to the Shimmer device
                
                isAcknowledged = waitforacknofeedback(thisShimmer, 1);     % Wait for Acknowledgment from Shimmer , 1 second is used to speed up the initializing process when detecting the firmware
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 7);        % Read the 7 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.FW_VERSION_RESPONSE)
                            
                            thisShimmer.FirmwareIdentifier = bitshift(shimmerResponse(3),8)+shimmerResponse(2);
                            thisShimmer.FirmwareMajorVersion = bitshift(shimmerResponse(5),8)+shimmerResponse(4);
                            thisShimmer.FirmwareMinorVersion = shimmerResponse(6);
                            thisShimmer.FirmwareInternal = shimmerResponse(7);
                            
                            if(thisShimmer.FirmwareIdentifier == 1)        % BtStream firmware identified
                                thisShimmer.FullFirmwareName = ['BtStream v', num2str(thisShimmer.FirmwareMajorVersion),'.',num2str(thisShimmer.FirmwareMinorVersion),'.', num2str(thisShimmer.FirmwareInternal)];
                                disp(['Firmware version ' thisShimmer.FullFirmwareName ' Detected for Shimmer COM' thisShimmer.ComPort]);
                                if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion == 1)
                                    thisShimmer.FirmwareCompatibilityCode = 1;
                                elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion == 2)
                                    thisShimmer.FirmwareCompatibilityCode = 2;
                                elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion == 3)
                                    thisShimmer.FirmwareCompatibilityCode = 3;
                                elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion == 4)
                                    thisShimmer.FirmwareCompatibilityCode = 4;
                                elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion >= 8)
                                    thisShimmer.FirmwareCompatibilityCode = 7;
                                elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion >= 5)
                                    thisShimmer.FirmwareCompatibilityCode = 5;
                                elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3 )
                                    thisShimmer.FirmwareCompatibilityCode = 1;
                                else
                                    thisShimmer.FirmwareCompatibilityCode = 0;
                                end
                                
                            elseif(thisShimmer.FirmwareIdentifier == 3)    % LogAndStream firmware identified
                                thisShimmer.FullFirmwareName = ['LogAndStream v', num2str(thisShimmer.FirmwareMajorVersion),'.',num2str(thisShimmer.FirmwareMinorVersion),'.', num2str(thisShimmer.FirmwareInternal)];
                                disp(['Firmware version ' thisShimmer.FullFirmwareName ' Detected for Shimmer COM' thisShimmer.ComPort]);
                                if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion == 1)
                                    thisShimmer.FirmwareCompatibilityCode = 3;
                                elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion >= 6)
                                    thisShimmer.FirmwareCompatibilityCode = 6;
                                elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion == 2)
                                    thisShimmer.FirmwareCompatibilityCode = 4;
                                elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareMajorVersion == 0 && thisShimmer.FirmwareMinorVersion >= 3)
                                    thisShimmer.FirmwareCompatibilityCode = 5;
                                else
                                    thisShimmer.FirmwareCompatibilityCode = 0;
                                end
                                
                            else
                                thisShimmer.FullFirmwareName = 'Unknown';
                                disp(['Warning: readfirmwareversion - Firmware version unknown for Shimmer COM' thisShimmer.ComPort]);
                                thisShimmer.FirmwareCompatibilityCode = 0;
                            end
                            isRead = true;
                            
                        else
                            thisShimmer.FirmwareMajorVersion = 'Nan';                % Set the FirmwareMajorVersion to 'Nan' to indicate unknown
                            thisShimmer.FirmwareMinorVersion = 'Nan';                % Set the FirmwareMinorVersion to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readfirmwareversion - Get firmware version command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.FirmwareMajorVersion = 'Nan';                % Set the FirmwareMajorVersion to 'Nan' to indicate unknown
                        thisShimmer.FirmwareMinorVersion = 'Nan';                % Set the FirmwareMinorVersion to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readfirmwareversion - Get firmware version command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                       fprintf(strcat('Warning: readfirmwareversion - Get firmware version command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                       isRead = false;
                else % if no ack means that the Shimmer is running an old firmware
                    thisShimmer.FirmwareIdentifier = 0;
                    thisShimmer.FirmwareMajorVersion = 0;
                    thisShimmer.FirmwareMinorVersion = 1;
                    thisShimmer.FirmwareInternal = 0;
                    thisShimmer.FullFirmwareName = 'BoilerPlate v0.1.0';
                    thisShimmer.FirmwareCompatibilityCode = 0;
                    fprintf(strcat('Warning: readfirmwareversion - Unknown Firmware version; assuming BoilerPlate 0.1.0 for Shimmer Com',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readfirmwareversion - Cannot get firmware version for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readfirmwareversion       
        
        function isRead = dummyreadfirmwareversion(thisShimmer)
            % Function should only be used immediately after connection as
            % a dummy command to flush the buffers and ensure correct
            % communication.
            
            if (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                                         % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_FW_VERSION_COMMAND);          % Send the Get FW version Command to the Shimmer
                
                isAcknowledged = waitforacknofeedback(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgement from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 7);        % Read the 2 byte response from the realterm buffer  
                    isRead = true;
                end
            end            
        end % function dummyreadfirmwareversion
                     
        function isRead = readaccelrange(thisShimmer)
            % Sends the GET_ACCEL_RANGE_COMMAND to Shimmer - in Connected state 
            % Receives the accel range and updates the AccelRange property.
            if (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_ACCEL_RANGE_COMMAND);          % Send the Set Accel Range Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);        % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.ACCEL_RANGE_RESPONSE)
                            thisShimmer.AccelRange = shimmerResponse(2);
                            isRead = true;
                        else
                            thisShimmer.AccelRange = 'Nan';                % Set the AccelRange to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readaccelrange - Get accel range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.AccelRange = 'Nan';                % Set the AccelRange to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readaccelrange - Get accel range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.AccelRange = 'Nan';                        % Set the AccelRange to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readaccelrange - Get accel range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readaccelrange - Cannot get accel range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readaccelrange
        
        function isRead = readaccelhrmode(thisShimmer)
            % Sends the GET_LSM303DLHC_ACCEL_HRMODE_COMMAND to Shimmer3 - in Connected state 
            % Receives the High Resolution mode setting and updates the AccelWideRangeHRMode property.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                               % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_LSM303DLHC_ACCEL_HRMODE_COMMAND);   % Send GET_LSM303DLHC_ACCEL_HRMODE_COMMAND to Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);          % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);             % Read the 2 byte response from the realterm buffer
                    
                    if (~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.LSM303DLHC_ACCEL_HRMODE_RESPONSE))
                        thisShimmer.AccelWideRangeHRMode = shimmerResponse(2);
                        isRead = true;
                    else
                        thisShimmer.AccelWideRangeHRMode = 'Nan';                               % Set the AccelWideRangeHRMode to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readaccelhrmode - Get Accel HR mode command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.AccelWideRangeHRMode = 'Nan';                                   % Set the AccelWideRangeHRMode to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readaccelhrmode - Get Accel HR mode command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: readaccelhrmode - Accel HR mode is not available for Shimmer2/2r.');
            else
                isRead = false;
                fprintf(strcat('Warning: readaccelhrmode - Cannot get Accel HR mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readaccelhrmode
        
        function isRead = readaccellpmode(thisShimmer)
            % Sends the GET_LSM303DLHC_ACCEL_LPMODE_COMMAND to Shimmer3 - in Connected state 
            % Receives the Low Power mode setting and updates the AccelWideRangeLPMode property.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                             % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_LSM303DLHC_ACCEL_LPMODE_COMMAND); % Send GET_LSM303DLHC_ACCEL_LPMODE_COMMAND to Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);        % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);           % Read the 2 byte response from the realterm buffer
                    
                    if (~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.LSM303DLHC_ACCEL_LPMODE_RESPONSE))
                        thisShimmer.AccelWideRangeLPMode = shimmerResponse(2);
                        if (thisShimmer.AccelWideRangeLPMode)
                            thisShimmer.AccelWideRangeDataRate = 2;
                            if (thisShimmer.SamplingRate <= 1)
                                thisShimmer.AccelWideRangeDataRate = 1;
                            end
                        end
                        isRead = true;
                    else
                        thisShimmer.AccelWideRangeLPMode = 'Nan';                             % Set the AccelWideRangeLPMode to 'Nan' to indicate unknown
                        thisShimmer.AccelWideRangeDataRate = 'Nan';
                        fprintf(strcat('Warning: readaccellpmode - Get Accel LP mode command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.AccelWideRangeLPMode = 'Nan';                        % Set the AccelWideRangeLPMode to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readaccellpmode - Get Accel LP mode command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: readaccellpmode - Accel LP mode is not available for Shimmer2/2r.');
            else
                isRead = false;
                fprintf(strcat('Warning: readaccellpmode - Cannot get Accel LP mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readaccellpmode
                
        function isRead = readconfigbyte0(thisShimmer)     
            % Sends the GET_CONFIG_BYTE0_COMMAND to Shimmer2/2r - in Connected state 
            % Receives Config Byte 0 and updates the ConfigByte0 property.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                clearreaddatabuffer(thisShimmer);                                           % As a precaution always clear the read data buffer before a write                
                writetocomport(thisShimmer, thisShimmer.GET_CONFIG_BYTE0_COMMAND);          % Send the Get Config Byte0 Command to the Shimmer                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);      % Wait for Acknowledgment from Shimmer                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);         % Read the 2 byte response from the realterm buffer                    
                    if ~isempty(shimmerResponse)
                        if (shimmerResponse(1) == thisShimmer.CONFIG_BYTE0_RESPONSE)
                            thisShimmer.ConfigByte0 = shimmerResponse(2);
                            isRead = true;
                        else
                            thisShimmer.ConfigByte0 = 'Nan';                                % Set the ConfigByte0 to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readconfigbyte0 - Get config byte0 command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.ConfigByte0 = 'Nan';                                    % Set the ConfigByte0 to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readconfigbyte0 - Config byte 0 command response not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.ConfigByte0 = 'Nan';                                        % Set the ConfigByte0 to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readconfigbyte0 - Config byte 0 command response not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
            elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                isRead = false;
                fprintf(strcat('Warning: readconfigbyte0 - This command is not available for Shimmer3; please use readconfigbytes.\n'));
            else
                isRead = false;
                fprintf(strcat('Warning: readconfigbyte0 - Cannot get config byte0 for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
        end % function readconfigbyte0
               
        function isRead = readconfigbytes(thisShimmer)     
            % Sends the GET_CONFIG_BYTE0_COMMAND to Shimmer3 - in Connected state 
            % Receives Config Byte 0, Config Byte 1, Config Byte 2 and Config Byte 3
            % and updates the corresponding properties.
            if (strcmp(thisShimmer.State,'Connected'))
                clearreaddatabuffer(thisShimmer);                                           % As a precaution always clear the read data buffer before a write                
                writetocomport(thisShimmer, thisShimmer.GET_CONFIG_BYTE0_COMMAND);          % Send GET_CONFIG_BYTE0_COMMAND to Shimmer3 to get config bytes byte0, byte1, byte2, byte3.              
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);      % Wait for Acknowledgment from Shimmer                
                if (isAcknowledged == true)
                    if(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);     % Read the 2 byte response from the realterm buffer    
                    else
                        [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 5);     % Read the 5 byte response from the realterm buffer   
                    end
                    if ~isempty(shimmerResponse)
                        if (shimmerResponse(1) == thisShimmer.CONFIG_BYTE0_RESPONSE)
                            thisShimmer.ConfigByte0 = shimmerResponse(2);
                            if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                                thisShimmer.ConfigByte1 = shimmerResponse(3);
                                thisShimmer.ConfigByte2 = shimmerResponse(4);
                                thisShimmer.ConfigByte3 = shimmerResponse(5);
                            end
                            isRead = true;
                        else
                            thisShimmer.ConfigByte0 = 'Nan';                                % Set the ConfigByte0 to 'Nan' to indicate unknown
                            thisShimmer.ConfigByte1 = 'Nan';                                % Set the ConfigByte1 to 'Nan' to indicate unknown
                            thisShimmer.ConfigByte2 = 'Nan';                                % Set the ConfigByte2 to 'Nan' to indicate unknown
                            thisShimmer.ConfigByte3 = 'Nan';                                % Set the ConfigByte3 to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readconfigbytes - Get config byte0 command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.ConfigByte0 = 'Nan';                                    % Set the ConfigByte0 to 'Nan' to indicate unknown
                        thisShimmer.ConfigByte1 = 'Nan';                                    % Set the ConfigByte1 to 'Nan' to indicate unknown
                        thisShimmer.ConfigByte2 = 'Nan';                                    % Set the ConfigByte2 to 'Nan' to indicate unknown
                        thisShimmer.ConfigByte3 = 'Nan';                                    % Set the ConfigByte3 to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readconfigbytes - Get config byte0 command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.ConfigByte0 = 'Nan';                                        % Set the ConfigByte0 to 'Nan' to indicate unknown                        
                    thisShimmer.ConfigByte1 = 'Nan';                                        % Set the ConfigByte1 to 'Nan' to indicate unknown
                    thisShimmer.ConfigByte2 = 'Nan';                                        % Set the ConfigByte2 to 'Nan' to indicate unknown
                    thisShimmer.ConfigByte3 = 'Nan';                                        % Set the ConfigByte3 to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readconfigbytes - Get config byte0 command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
            else
                isRead = false;
                fprintf(strcat('Warning: readconfigbytes - Cannot get config byte0 for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
        end % function readconfigbytes
               
        function isRead = readbuffersize(thisShimmer)
            % Sends the GET_BUFFER_SIZE_COMMAND to Shimmer - in Connected state 
            % Receives Buffer Size and updates BufferSize property.
            % Currently not supported for Shimmer3.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                           % As a precaution always clear the read data buffer before a write
                
                writetocomport(thisShimmer, thisShimmer.GET_BUFFER_SIZE_COMMAND);          % Send the Get Buffer Size Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);      % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);         % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.BUFFER_SIZE_RESPONSE)
                            thisShimmer.BufferSize = shimmerResponse(2);
                            isRead = true;
                        else
                            thisShimmer.BufferSize = 'Nan';               % Set the Buffer Size to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readbuffersize - Get buffer size command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.BufferSize = 'Nan';               % Set the Buffer Size to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readbuffersize - Get buffer size command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.BufferSize = 'Nan';                       % Set the Buffer Size to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readbuffersize - Get buffer size command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
            elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                isRead = false;
                disp('Warning: readbuffersize - Buffer size is currently not configurable for Shimmer3.');
            else
                isRead = false;
                fprintf(strcat('Warning: readbuffersize - Cannot get buffer size for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readbuffersize
               
        function isRead = readgsrrange(thisShimmer)
            % Sends the GET_GSR_RANGE_COMMAND to Shimmer - in Connected state 
            % Receives GSR range and updates the GsrRange property. 
            if (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_GSR_RANGE_COMMAND);            % Send the Get GSR Range Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);        % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.GSR_RANGE_RESPONSE)
                            thisShimmer.GsrRange = shimmerResponse(2);
                            isRead = true;
                        else
                            thisShimmer.GsrRange = 'Nan';                  % Set the GsrRange to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readgsrrange - Get gsr range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.GsrRange = 'Nan';                  % Set the GsrRange to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readgsrrange - Get gsr range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.GsrRange = 'Nan';                        % Set the GsrRange to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readgsrrange - Get gsr range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readgsrrange - Cannot get gsr range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readgsrrange
        
        function isRead = readmagrange(thisShimmer)
            % Sends the GET_MAG_GAIN_COMMAND to Shimmer - in Connected state 
            % Receives Magnetometer range and updates the MagRange property. 
            if (thisShimmer.FirmwareCompatibilityCode == 0)
                fprintf(strcat('Warning: setmagrange - Command not supported for this firmware version, please update firmware.\n'));
            elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.HardwareCompatibilityCode == 2)
                fprintf(strcat('Warning: setmagrange - Fixed mag range of 49.152 gauss.\n'));
            else
                if (strcmp(thisShimmer.State,'Connected'))
                    
                    clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.GET_MAG_GAIN_COMMAND);             % Send the Get Mag Range Command to the Shimmer
                    
                    isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                    
                    if (isAcknowledged == true)
                        [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);        % Read the 2 byte response from the realterm buffer
                        
                        if ~isempty(shimmerResponse)
                            
                            if (shimmerResponse(1) == thisShimmer.MAG_GAIN_RESPONSE)
                                thisShimmer.MagRange = shimmerResponse(2);
                                isRead = true;
                            else
                                thisShimmer.MagRange = 'Nan';                  % Set the MagRange to 'Nan' to indicate unknown
                                fprintf(strcat('Warning: readmagrange - Get mag range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                isRead = false;
                            end
                        else
                            thisShimmer.MagRange = 'Nan';                  % Set the MagRange to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readmagrange - Get mag range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.MagRange = 'Nan';                        % Set the MagRange to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readmagrange - Get mag range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                    
                else
                    isRead = false;
                    fprintf(strcat('Warning: readmagrange - Cannot get mag range for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                end
            end
        end % function readmagrange
             
        function isRead = readgyrorange(thisShimmer)
            % Sends the GET_MPU9150_GYRO_RANGE_COMMAND to Shimmer3 - in Connected state 
            % Receives MPU9150 - Gyroscope range and updates the GyroRange property. 
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_MPU9150_GYRO_RANGE_COMMAND);            % Send the Get Mag Range Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);        % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.MPU9150_GYRO_RANGE_RESPONSE)
                            thisShimmer.GyroRange = shimmerResponse(2);
                            isRead = true;
                        else
                            thisShimmer.GyroRange = 'Nan';                  % Set the GyroRange to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readgyrorange - Get gyro range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.GyroRange = 'Nan';                  % Set the GyroRange to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readgyrorange - Get gyro range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.GyroRange = 'Nan';                        % Set the GyroRange to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readgyrorange - Get gyro range command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readgyrorange - Cannot get gyro range for COM ',thisShimmer.ComPort,' as Shimmer is not connected\n'));
                fprintf('or Shimmer is not a Shimmer3.\n')
            end
            
        end % function readgyrorange
        
        function isRead = readpressureresolution(thisShimmer)
            % Sends the GET_BMP180_PRES_RESOLUTION_COMMAND to Shimmer3 - in Connected state 
            % Receives BMP180/BMP280 - Pressure sensor resolution and updates the PressureResolution property. 
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 2 )
                disp('Warning: readpressureresolution - Command not supported for this firmware version, please update firmware.')
            elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                               % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_BMP180_PRES_RESOLUTION_COMMAND);    % Send GET_BMP180_PRES_RESOLUTION_COMMAND to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);          % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);             % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.BMP180_PRES_RESOLUTION_RESPONSE)
                        thisShimmer.PressureResolution = shimmerResponse(2);
                        isRead = true;
                        
                    else
                        thisShimmer.PressureResolution = 'Nan';                  % Set the Pressure Resolution to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readpressureresolution - Get pressure resolution command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.PressureResolution = 'Nan';                        % Set the Pressure Resolution to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readpressureresolution - Get pressure resolution command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
            elseif(thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                isRead = false;
                disp('Warning: readpressureresolution - Pressure sensor is not available on Shimmer2/2r.');
            else
                isRead = false;
                fprintf(strcat('Warning: readpressureresolution - Cannot get pressure resolution for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readpressureresolution
        
        function isRead = readpressuresensorcalibrationcoefficients(thisShimmer)
            %READPRESSURESENSORCALIBRATIONCOEFFICIENTS - retrieve
            %calibration coefficients for the Pressure Sensor (BMP180/BMP280) on the
            %Shimmer3 
            %
            %
            %   ISREAD = READPRESSURESENSORCALIBRATIONCOEFFICIENTS retrieves pressure
            %   sensor coefficients
            %
            %   SYNOPSIS: isRead = thisShimmer.readpressuresensorcalibrationcoefficients
            %
            %   OUTPUT: isRead - isRead is true if calibratrion
            %                    coefficients are successfully retrieved
            %                    isRead is false means if retrieval of calibration 
            %                       coefficients is not successful.
            %
            %   EXAMPLE: isRead =
            %   shimmer1.readpressuresensorcalibrationcoefficients()
            %
            
            if strcmp(thisShimmer.getstate,'Connected')
                if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 2 )
                    disp('Warning: readpressuresensorcalibrationcoefficients - Command not supported for this firmware version, please update firmware.')
                    isRead = false;
                elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    if thisShimmer.HardwareCompatibilityCode < 2
                        nBytes = 23;
                        getCalCoCommand = thisShimmer.GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND;
                    elseif thisShimmer.HardwareCompatibilityCode == 2
                        nBytes = 25;
                        getCalCoCommand = thisShimmer.GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND;
                    else
                        % assuming BMP280
                        nBytes = 25;
                        getCalCoCommand = thisShimmer.GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND;
                    end
                    clearreaddatabuffer(thisShimmer);                                                       % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer,getCalCoCommand);                                            % Send command to get pressure sensor calibration coefficients
                    isStarted = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);                       % Wait for Acknowledgment from Shimmer
                    
                    if(~isStarted)
                        fprintf(strcat('Warning: readpressuresensorcalibrationcoefficients - Get calibration coefficients command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    else
                        serialData = [];
                        nIterations = 0;
                        while(length(serialData) < nBytes && nIterations < 4)
                            [tempSerialData, isFileOpen] = readdatabuffer(thisShimmer, inf);                % Read all available serial data from the com port
                            serialData = [serialData; tempSerialData];
                            pause(.2);
                            nIterations = nIterations + 1;
                        end
                        if (~isempty(serialData) && serialData(1) == thisShimmer.BMP180_CALIBRATION_COEFFICIENTS_RESPONSE)
                            thisShimmer.AC1 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(2)),8)+double(serialData(3)),16);
                            thisShimmer.AC2 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(4)),8)+double(serialData(5)),16);
                            thisShimmer.AC3 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(6)),8)+double(serialData(7)),16);
                            thisShimmer.AC4 = bitshift(double(serialData(8)),8)+double(serialData(9));
                            thisShimmer.AC5 = bitshift(double(serialData(10)),8)+double(serialData(11));
                            thisShimmer.AC6 = bitshift(double(serialData(12)),8)+double(serialData(13));
                            thisShimmer.B1 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(14)),8)+double(serialData(15)),16);
                            thisShimmer.B2 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(16)),8)+double(serialData(17)),16);
                            thisShimmer.MB = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(18)),8)+double(serialData(19)),16);
                            thisShimmer.MC = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(20)),8)+double(serialData(21)),16);
                            thisShimmer.MD = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(22)),8)+double(serialData(23)),16);
                            isRead = true;
                        elseif (~isempty(serialData) && serialData(1) == thisShimmer.BMP280_CALIBRATION_COEFFICIENTS_RESPONSE)
                            thisShimmer.DIG_T1 = bitshift(double(serialData(3)),8)+double(serialData(2));
                            thisShimmer.DIG_T2 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(5)),8)+double(serialData(4)),16);
                            thisShimmer.DIG_T3 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(7)),8)+double(serialData(6)),16);
                            thisShimmer.DIG_P1 = bitshift(double(serialData(9)),8)+double(serialData(8));
                            thisShimmer.DIG_P2 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(11)),8)+double(serialData(10)),16);
                            thisShimmer.DIG_P3 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(13)),8)+double(serialData(12)),16);
                            thisShimmer.DIG_P4 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(15)),8)+double(serialData(14)),16);
                            thisShimmer.DIG_P5 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(17)),8)+double(serialData(16)),16);
                            thisShimmer.DIG_P6 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(19)),8)+double(serialData(18)),16);
                            thisShimmer.DIG_P7 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(21)),8)+double(serialData(20)),16);
                            thisShimmer.DIG_P8 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(23)),8)+double(serialData(22)),16);
                            thisShimmer.DIG_P9 = calculatetwoscomplement(thisShimmer,bitshift(double(serialData(25)),8)+double(serialData(24)),16);
                            isRead = true;
                        else
                            fprintf(strcat('Warning: readpressuresensorcalibrationcoefficients - Get calibration coefficients command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    end
                else
                    disp('Warning: readpressuresensorcalibrationcoefficients - Pressure sensor only supported on Shimmer3');
                    isRead = false;
                end
            else
                isRead = false;
                fprintf(strcat('Warning: readpressuresensorcalibrationcoefficients - Cannot get pressure sensor calibaration coefficients for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
        end    % read pressure sensor calibration coefficients
        
        
        
        
        function IDByteArray = readexpansionboardidbytes(thisShimmer, numBytes, offset)
            %READEXPANSIONBOARDIDBYTES - retrieves expansion board id bytes
            %
            %   ISREAD = READEXPANSIONBOARDIDBYTES retrieves expansion
            %   board id bytes
            %
            %   SYNOPSIS: isRead = thisShimmer.readexpansionboardidbytes
            %
            %   OUTPUT: isRead - isRead is true if expansion board id bytes
            %                    are successfully retrieved
            %                    isRead is false means if retrieval of
            %                    expansion board id bytes is not
            %                    successful.
            %
            %   EXAMPLE: isRead = shimmer1.readexpansionboardidbytes()
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: readexpansionboardidbytes - Cannot get Expansion Board ID bytes for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                IDByteArray = 'Nan';
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: readexpansionboardidbytes - Only supported on Shimmer3');
                IDByteArray = 'Nan';
            elseif (thisShimmer.FirmwareCompatibilityCode < 5) 
                disp('Warning: readexpansionboardidbytes - Command not supported for this firmware version, please update firmware.')
                IDByteArray = 'Nan';
            else
                clearreaddatabuffer(thisShimmer);                                        % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer,thisShimmer.GET_DAUGHTER_CARD_ID_COMMAND);    % Send command to get expansion board ID bytes
                writetocomport(thisShimmer, char(numBytes));                             % Send numBytes value
                writetocomport(thisShimmer, char(offset));                               % Send offset value
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                
                if(~isAcknowledged)
                    fprintf(strcat('Warning: readexpansionboardidbytes - Get Expansion Board ID bytes command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    IDByteArray = 'Nan';
                else
                    serialData = [];
                    [serialData, isFileOpen] = readdatabuffer(thisShimmer, inf);         % Read all available serial data from the com port
                    if (~isempty(serialData) && serialData(1)==thisShimmer.DAUGHTER_CARD_ID_RESPONSE)
                        for numByte = 1:numBytes
                            IDByteArray(numByte) = serialData(numByte+2);                % First two bytes are DAUGHTER_CARD_ID_RESPONSE and numBytes.
                        end
                    else
                        fprintf(strcat('Warning: readexpansionboardidbytes - Get Expansion Board ID bytes command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        IDByteArray = 'Nan';
                    end
                end
            end
        end    % read expansion board ID bytes

        
        function isRead = readexgconfiguration(thisShimmer,chipIdentifier)  % chipIdentifier selects SENSOR_EXG1 or SENSOR_EXG2
            % Sends the GET_EXG_REGS_COMMAND to Shimmer3 - in Connected state 
            % Receives (all) ExG configuration bytes and updates the corresponding properties. 
            if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: readexgconfiguration - Command not supported for this firmware version, please update firmware.')
                isRead = false;
            elseif (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && (chipIdentifier == 1 || chipIdentifier == 2))
                
                clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_EXG_REGS_COMMAND);
                writetocomport(thisShimmer, char(chipIdentifier-1));                       % char(0) selects SENSOR_EXG1, char(1) selects SENSOR_EXG2
                writetocomport(thisShimmer, char(0));
                writetocomport(thisShimmer, char(10));
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    serialData = [];
                    nIterations = 0;
                    while(length(serialData) < 12 && nIterations < 4 )                                       % Read the 12 byte response from the realterm buffer
                        [tempSerialData, isFileOpen] = readdatabuffer(thisShimmer, inf);   % Read all available serial data from the com port
                        serialData = [serialData; tempSerialData];
                        pause(.2);
                        nIterations = nIterations + 1;
                    end
                    shimmerResponse = serialData(1:12);
                    ExG1Failed = false;
                    ExG2Failed = false;
                    if ~isempty(shimmerResponse)
                        if (chipIdentifier==1)
                            if (shimmerResponse(1) == thisShimmer.EXG_REGS_RESPONSE)
                                thisShimmer.EXG1Config1 = shimmerResponse(3);
                                thisShimmer.EXG1Config2 = shimmerResponse(4);
                                thisShimmer.EXG1Loff = shimmerResponse(5);
                                thisShimmer.EXG1Ch1Set = shimmerResponse(6);
                                thisShimmer.EXG1Ch2Set = shimmerResponse(7);
                                thisShimmer.EXG1RLD_Sens = shimmerResponse(8);
                                thisShimmer.EXG1LOFF_Sens = shimmerResponse(9);
                                thisShimmer.EXG1LOFF_Stat = shimmerResponse(10);
                                thisShimmer.EXG1Resp1 = shimmerResponse(11);
                                thisShimmer.EXG1Resp2 = shimmerResponse(12);
                                thisShimmer.EXG1CH1Gain = convertEXGGain(thisShimmer, bitshift(bitand(112,thisShimmer.EXG1Ch1Set),-4));
                                thisShimmer.EXG1CH2Gain = convertEXGGain(thisShimmer, bitshift(bitand(112,thisShimmer.EXG1Ch2Set),-4));
                                thisShimmer.EXG1Rate = bitand(thisShimmer.EXG1Config1,7);
                                isRead = true;
                            else
                                ExG1Failed = true;
                            end
                        elseif (chipIdentifier==2)
                            if (shimmerResponse(1) == thisShimmer.EXG_REGS_RESPONSE)
                                thisShimmer.EXG2Config1 = shimmerResponse(3);
                                thisShimmer.EXG2Config2 = shimmerResponse(4);
                                thisShimmer.EXG2Loff = shimmerResponse(5);
                                thisShimmer.EXG2Ch1Set = shimmerResponse(6);
                                thisShimmer.EXG2Ch2Set = shimmerResponse(7);
                                thisShimmer.EXG2RLD_Sens = shimmerResponse(8);
                                thisShimmer.EXG2LOFF_Sens = shimmerResponse(9);
                                thisShimmer.EXG2LOFF_Stat = shimmerResponse(10);
                                thisShimmer.EXG2Resp1 = shimmerResponse(11);
                                thisShimmer.EXG2Resp2 = shimmerResponse(12);
                                thisShimmer.EXG2CH1Gain = convertEXGGain(thisShimmer, bitshift(bitand(112,thisShimmer.EXG2Ch1Set),-4));
                                thisShimmer.EXG2CH2Gain = convertEXGGain(thisShimmer, bitshift(bitand(112,thisShimmer.EXG2Ch2Set),-4));
                                thisShimmer.EXG2Rate = bitand(thisShimmer.EXG2Config1,7);
                                isRead = true;
                            else
                                ExG2Failed = true;
                            end
                        end
                    else
                        if (chipIdentifier == 1)
                            ExG1Failed = true;
                        elseif (chipIdentifier == 2)
                            ExG2Failed = true;
                        end
                    end
                else
                    if (chipIdentifier == 1)
                        ExG1Failed = true;
                    elseif (chipIdentifier == 2)
                        ExG2Failed = true;
                    end
                    fprintf(strcat('Warning: readexgconfiguration - Get EXG settings command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                if(ExG1Failed == true)
                    thisShimmer.EXG1Config1 = 'Nan';
                    thisShimmer.EXG1Config2 = 'Nan';
                    thisShimmer.EXG1Loff = 'Nan';
                    thisShimmer.EXG1Ch1Set = 'Nan';
                    thisShimmer.EXG1Ch2Set = 'Nan';
                    thisShimmer.EXG1RLD_Sens = 'Nan';
                    thisShimmer.EXG1LOFF_Sens = 'Nan';
                    thisShimmer.EXG1LOFF_Stat = 'Nan';
                    thisShimmer.EXG1Resp1 = 'Nan';
                    thisShimmer.EXG1Resp2 = 'Nan';
                    thisShimmer.EXG1CH1Gain = 'Nan';
                    thisShimmer.EXG1CH2Gain = 'Nan';
                    thisShimmer.EXG1Rate = 'Nan';
                    fprintf(strcat('Warning: readexgconfiguration - Get EXG settings command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                if(ExG2Failed == true)
                    thisShimmer.EXG2Config1 = 'Nan';
                    thisShimmer.EXG2Config2 = 'Nan';
                    thisShimmer.EXG2Loff = 'Nan';
                    thisShimmer.EXG2Ch1Set = 'Nan';
                    thisShimmer.EXG2Ch2Set = 'Nan';
                    thisShimmer.EXG2RLD_Sens = 'Nan';
                    thisShimmer.EXG2LOFF_Sens = 'Nan';
                    thisShimmer.EXG2LOFF_Stat = 'Nan';
                    thisShimmer.EXG2Resp1 = 'Nan';
                    thisShimmer.EXG2Resp2 = 'Nan';
                    thisShimmer.EXG2CH1Gain = 'Nan';
                    thisShimmer.EXG2CH2Gain = 'Nan';
                    thisShimmer.EXG2Rate = 'Nan';
                    fprintf(strcat('Warning: readexgconfiguration - Get EXG settings command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                isRead = false;
                disp('Warning: readexgconfiguration - ExG configuration is not supported for Shimmer2/2r.');
            elseif(~(chipIdentifier == 1 || chipIdentifier == 2))
                isRead = false;
                disp('Warning: readexgconfiguration - Invalid chip identifier (please use 1 (Chip1) or 2 (Chip2).');
            else
                isRead = false;
                fprintf(strcat('Warning: readexgconfiguration - Cannot get EXG settings for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readEXGconfiguration
        
        function isRead = readmagrate(thisShimmer)
            % Sends the GET_MAG_SAMPLING_RATE_COMMAND to Shimmer - in Connected state 
            % Receives Magetometer data rate and updates the MagRate property. 
            if (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_MAG_SAMPLING_RATE_COMMAND);    % Send the Get Mag Rate Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);        % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.MAG_SAMPLING_RATE_RESPONSE)
                            thisShimmer.MagRate = shimmerResponse(2);
                            isRead = true;
                        else
                            thisShimmer.MagRate = 'Nan';                  % Set the MagRate to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readmagrate - Get mag rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.MagRate = 'Nan';                  % Set the MagRate to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readmagrate - Get mag rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.MagRate = 'Nan';                        % Set the MagRate to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readmagrate - Get mag rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readmagrate - Cannot get mag rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readmagrate
                        
        function isRead = readinternalexppower(thisShimmer)
            % Sends the GET_INTERNAL_EXP_POWER_ENABLE_COMMAND to Shimmer3 - in Connected state 
            % Receives Internal Expansion Power settingand updates the InternalExpPower property. 
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.FirmwareCompatibilityCode >= 2)
                
                clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_INTERNAL_EXP_POWER_ENABLE_COMMAND);            
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);        % Read the 2 byte response from the realterm buffer
                    
                    if (~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.INTERNAL_EXP_POWER_ENABLE_RESPONSE))
                            thisShimmer.InternalExpPower = shimmerResponse(2);
                            isRead = true;
                    else
                        thisShimmer.InternalExpPower = 'Nan';                  % Set the InternalExpPower to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readinternalexppower - Get internal exp power command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.InternalExpPower = 'Nan';                        % Set the InternalExpPower to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readinternalexppower - Get internal exp power command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)    
                disp('Warning: readinternalexppower - Internal exp power is not available for Shimmer2/2r.');
            elseif (thisShimmer.FirmwareCompatibilityCode < 2)
                disp('Warning: readinternalexppower - Internal exp power is not supported for this firmware version, please update firmware.');
            else
                isRead = false;
                fprintf(strcat('Warning: readinternalexppower - Cannot get internal exp power for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readinternalexppower
        
        function isRead = readgyrorate(thisShimmer)
            % Sends the GET_MPU9150_SAMPLING_RATE_COMMAND to Shimmer3 - in Connected state 
            % Receives MPU9150 - Gyroscope data rate and updates the GyroRate property. 
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                           % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_MPU9150_SAMPLING_RATE_COMMAND); % Send the Get Gyro Rate Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);      % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);         % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.MPU9150_SAMPLING_RATE_RESPONSE)
                            thisShimmer.GyroRate = shimmerResponse(2);
                            isRead = true;
                        else
                            thisShimmer.GyroRate = 'Nan';                  % Set the GyroRate to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readgyrorate - Get gyro rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.GyroRate = 'Nan';                  % Set the GyroRate to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readgyrorate - Get gyro rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.GyroRate = 'Nan';                        % Set the GyroRate to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readgyrorate - Get gyro rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readgyrorate - Cannot get gyro rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected\n'));
                fprintf('or Shimmer is not a Shimmer3. Gyro rate is only supported for Shimmer3.\n');
            end
            
        end % function readgyrorate
        
        function isRead = readaccelrate(thisShimmer)
            % Sends the GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND to Shimmer3 - in Connected state 
            % Receives LSM303DLHC/LSM303AHTR - Accelerometer data rate and updates the AccelWideRangeDataRate property.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                                     % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND);  % Send the Get Accel Rate Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);                % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);                   % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.LSM303DLHC_ACCEL_SAMPLING_RATE_RESPONSE)
                            thisShimmer.AccelWideRangeDataRate = shimmerResponse(2);
                            isRead = true;
                        else
                            thisShimmer.AccelWideRangeDataRate = 'Nan';                  % Set the AccelWideRangeDataRate to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readaccelrate - Get accel rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.AccelWideRangeDataRate = 'Nan';                  % Set the AccelWideRangeDataRate to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readaccelrate - Get accel rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.AccelWideRangeDataRate = 'Nan';                        % Set the AccelWideRangeDataRate to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readaccelrate - Get accel rate command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readaccelrate - Cannot get accel rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                fprintf('or Shimmer is not a Shimmer3. Read accel rate is only supported for Shimmer3.\n');
            end
            
        end % function readaccelrate
        
        function isRead = readexgrate(thisShimmer, chipIdentifier) % function readexgrate
            % Sends the GET_EXG_REGS_COMMAND to Shimmer3 - in Connected state 
            % Receives ExG data rate and updates the corresponding properties.
            if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: readexgrate - ExG rate is not supported for this firmware version, please update firmware.');
                isRead = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                fprintf('Warning: readexgrate - ExG rate is not supported for Shimmer2/2r.\n');
                isRead = false;
            elseif((chipIdentifier == 1 || chipIdentifier == 2) && thisShimmer.FirmwareCompatibilityCode >= 3)

                if (strcmp(thisShimmer.State,'Connected'))
                    clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer, thisShimmer.GET_EXG_REGS_COMMAND);             % Send the GET_EXG_REGS_COMMAND to the Shimmer
                    writetocomport(thisShimmer, char(chipIdentifier-1));                       % char(0) selects SENSOR_EXG1, char(1) selects SENSOR_EXG2
                    writetocomport(thisShimmer, char(0));                                      % Start at byte 0.
                    writetocomport(thisShimmer, char(1));                                      % Read one byte.
                    isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                    if (chipIdentifier == 1) 
                        % SENSOR_EXG1
                        if (isAcknowledged == true)
                            [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 3);        % Read the 3 bytes response from the realterm buffer

                            if ( ~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.EXG_REGS_RESPONSE) )
                                    thisShimmer.EXG1Config1 = shimmerResponse(3);       % Update property EXG1Config1
                                    thisShimmer.EXG1Rate = bitand(thisShimmer.EXG1Config1,7);%Update property EXG1Rate
                                    isRead = true;
                            else
                                thisShimmer.EXG1Config1 = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                fprintf(strcat('Warning: readexgrate - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                isRead = false;
                            end
                        else
                            thisShimmer.EXG1Config1 = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readexgrate - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        % SENSOR_EXG2
                        if (isAcknowledged == true)
                            [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 3);        % Read the 3 bytes response from the realterm buffer

                            if (~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.EXG_REGS_RESPONSE))
                                    thisShimmer.EXG2Config1 = shimmerResponse(3);       % Update property EXG2Config1
                                    thisShimmer.EXG2Rate = bitand(thisShimmer.EXG2Config1,7);%Update property EXG2Rate
                                    isRead = true;
                            else
                                thisShimmer.EXG2Config1 = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                fprintf(strcat('Warning: readexgrate - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                isRead = false;
                            end
                        else
                            thisShimmer.EXG2Config1 = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readexgrate - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    end
                else
                    isRead = false;
                    fprintf(strcat('Warning: readexgrate - Cannot get exg rate for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                end
            else
                isRead = false;
                fprintf(strcat('Warning: readexgrate - Invalid chip selection.\n'));
            end
            
        end % function readexgrate
        
        function isRead = readexggain(thisShimmer, chipIdentifier, channelIdentifier) % function readexggain
            % Sends the GET_EXG_REGS_COMMAND to Shimmer3 - in Connected state 
            % Receives ExG gain and updates the corresponding properties.
            if(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: readexggain - ExG gain is not supported for this firmware version, please update firmware.');
                isRead = false;
            elseif (thisShimmer.FirmwareCompatibilityCode >= 3)
                if (channelIdentifier == 1)
                    % channelIdentifier 1
                    if (chipIdentifier == 1 || chipIdentifier == 2)
                        if (strcmp(thisShimmer.State,'Connected'))
                            clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                            writetocomport(thisShimmer, thisShimmer.GET_EXG_REGS_COMMAND);             % Send the GET_EXG_REGS_COMMAND to the Shimmer
                            writetocomport(thisShimmer, char(chipIdentifier-1));                       % char(0) selects SENSOR_EXG1, char(1) selects SENSOR_EXG2
                            writetocomport(thisShimmer, char(3));                                      % Start at byte 3.
                            writetocomport(thisShimmer, char(1));                                      % Read one byte.
                            isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                            if (chipIdentifier == 1)
                                % SENSOR_EXG1
                                if (isAcknowledged == true)
                                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 3);    % Read the 3 bytes response from the realterm buffer
                                    if (~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.EXG_REGS_RESPONSE))
                                        thisShimmer.EXG1Ch1Set = shimmerResponse(3);
                                        thisShimmer.EXG1CH1Gain = convertEXGGain(thisShimmer, bitshift(bitand(112,thisShimmer.EXG1Ch1Set),-4)); % Update property EXG1CH1Gain
                                        isRead = true;
                                    else
                                        thisShimmer.EXG1CH1Gain = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                        thisShimmer.EXG1Ch1Set = 'Nan';
                                        fprintf(strcat('Warning: readexggain - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                        isRead = false;
                                    end
                                else
                                    thisShimmer.EXG1CH1Gain = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                    thisShimmer.EXG1Ch1Set = 'Nan';
                                    fprintf(strcat('Warning: readexggain - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                    isRead = false;
                                end
                            else
                                % SENSOR_EXG2
                                if (isAcknowledged == true)
                                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 3);        % Read the 3 bytes response from the realterm buffer
                                    
                                    if (~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.EXG_REGS_RESPONSE))
                                        thisShimmer.EXG2Ch1Set = shimmerResponse(3);
                                        thisShimmer.EXG2CH1Gain = convertEXGGain(thisShimmer, bitshift(bitand(112,thisShimmer.EXG2Ch1Set),-4)); % Update property EXG1CH1Gain
                                        isRead = true;
                                    else
                                        thisShimmer.EXG2CH1Gain = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                        thisShimmer.EXG2Ch1Set = 'Nan';
                                        fprintf(strcat('Warning: readexggain - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                        isRead = false;
                                    end
                                else
                                    thisShimmer.EXG2CH1Gain = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                    thisShimmer.EXG2Ch1Set = 'Nan';
                                    fprintf(strcat('Warning: readexggain - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                    isRead = false;
                                end
                            end
                        else
                            isRead = false;
                            fprintf(strcat('Warning: readexggain - Cannot get exg gain for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                        end
                    else
                        isRead = false;
                        fprintf(strcat('Warning: readexggain - Invalid chip selection.\n'));
                    end
                elseif (channelIdentifier == 2)
                    % channelIdentifier 2
                    if(chipIdentifier == 1 || chipIdentifier == 2)
                        if (strcmp(thisShimmer.State,'Connected'))
                            clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                            writetocomport(thisShimmer, thisShimmer.GET_EXG_REGS_COMMAND);             % Send the GET_EXG_REGS_COMMAND to the Shimmer
                            writetocomport(thisShimmer, char(chipIdentifier-1));                       % char(0) selects SENSOR_EXG1, char(1) selects SENSOR_EXG2
                            writetocomport(thisShimmer, char(4));                                      % Start at byte 4.
                            writetocomport(thisShimmer, char(1));                                      % Read byte 4.
                            isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                            if (chipIdentifier == 1)
                                % SENSOR_EXG1
                                if (isAcknowledged == true)
                                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 3);        % Read the 3 bytes response from the realterm buffer
                                    
                                    if (~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.EXG_REGS_RESPONSE))
                                        thisShimmer.EXG1Ch2Set = shimmerResponse(3);
                                        thisShimmer.EXG1CH2Gain = convertEXGGain(thisShimmer, bitshift(bitand(112,thisShimmer.EXG1Ch2Set),-4)); % Update property EXG1CH2Gain
                                        isRead = true;
                                    else
                                        thisShimmer.EXG1CH2Gain = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                        thisShimmer.EXG1Ch2Set = 'Nan';
                                        fprintf(strcat('Warning: readexggain - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                        isRead = false;
                                    end
                                else
                                    thisShimmer.EXG1CH2Gain = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                    thisShimmer.EXG1Ch2Set = 'Nan';
                                    fprintf(strcat('Warning: readexggain - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                    isRead = false;
                                end
                            else
                                % SENSOR_EXG2
                                if (isAcknowledged == true)
                                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 3);        % Read the 3 bytes response from the realterm buffer
                                    
                                    if (~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.EXG_REGS_RESPONSE))
                                        thisShimmer.EXG2Ch2Set = shimmerResponse(3);
                                        thisShimmer.EXG2CH2Gain = convertEXGGain(thisShimmer, bitshift(bitand(112,thisShimmer.EXG2Ch2Set),-4)); % Update property EXG2CH2Gain
                                        isRead = true;
                                    else
                                        thisShimmer.EXG2CH2Gain = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                        thisShimmer.EXG2Ch2Set = 'Nan';
                                        fprintf(strcat('Warning: readexggain - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                        isRead = false;
                                    end
                                else
                                    thisShimmer.EXG2CH2Gain = 'Nan';                  % Set the  to 'Nan' to indicate unknown
                                    thisShimmer.EXG2Ch2Set = 'Nan';
                                    fprintf(strcat('Warning: readexggain - Get exg regs command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                                    isRead = false;
                                end
                            end
                        else
                            isRead = false;
                            fprintf(strcat('Warning: readexggain - Cannot get exg gain for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                        end
                    else
                        isRead = false;
                        fprintf(strcat('Warning: readexggain - Invalid chip selection.\n'));
                    end
                else
                    isRead = false;
                    fprintf(strcat('Warning: readexggain - Invalid channel selection.\n'));
                end
            else
                fprintf('Warning: readexggain - Command is not supported for Shimmer2/2r.\n');
            end
        end % function readexggain
        
        function isRead = readexgreferenceelectrodeconfiguration(thisShimmer)
            % READEXGREFERENCEELECTRODECONFIGURATION - Reads the ExG Reference
            % Electrode Configuration
            %
            %
            %   OUTPUT: isRead - isRead is true if exg reference electrode
            %                    configuration is successfully retrieved
            %                    isRead is false means if retrieval is not
            %                    successful.
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: readexgreferenceelectrodeconfiguration - Cannot get ExG reference electrode configuration for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                thisShimmer.EXG1RLD_Sens = 'Nan';
                thisShimmer.EXGReferenceElectrodeConfiguration = 'Nan';
                isRead = false;
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: readexgreferenceelectrodeconfiguration - Only supported on Shimmer3');
                thisShimmer.EXG1RLD_Sens = 'Nan';
                thisShimmer.EXGReferenceElectrodeConfiguration = 'Nan';
                isRead = false;
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: readexgreferenceelectrodeconfiguration - Command not supported for this firmware version, please update firmware.')
                thisShimmer.EXG1RLD_Sens = 'Nan';
                thisShimmer.EXGReferenceElectrodeConfiguration = 'Nan';
                isRead = false;
            else
                clearreaddatabuffer(thisShimmer);                                        % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer,thisShimmer.GET_EXG_REGS_COMMAND);            % Send command to get ExG register values
                writetocomport(thisShimmer, char(0));                                    % char(0) selects SENSOR_EXG1
                writetocomport(thisShimmer, char(5));                                    % Start at byte 7.
                writetocomport(thisShimmer, char(1));                                    % Read 1 byte.
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                
                if(~isAcknowledged)
                    fprintf(strcat('Warning: readexgreferenceelectrodeconfiguration - Get ExG registers command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    thisShimmer.EXG1RLD_Sens = 'Nan';
                    thisShimmer.EXGReferenceElectrodeConfiguration = 'Nan';
                    isRead = false;
                else
                    serialData = [];
                    [serialData, isFileOpen] = readdatabuffer(thisShimmer, inf);         % Read all available serial data from the com port
                    if (~isempty(serialData) && serialData(1)==thisShimmer.EXG_REGS_RESPONSE)
                        % Update properties
                        thisShimmer.EXG1RLD_Sens = serialData(3);                    
                        thisShimmer.EXGReferenceElectrodeConfiguration = bitand(thisShimmer.EXG1RLD_Sens,15);
                        isRead = true;
                    else
                        fprintf(strcat('Warning: readexgreferenceelectrodeconfiguration - Get ExG registers command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        thisShimmer.EXG1RLD_Sens = 'Nan';
                        thisShimmer.EXGReferenceElectrodeConfiguration = 'Nan';
                        isRead = false;
                    end
                end
            end
        end % readexgreferenceelectrodeconfiguration
        
        function isRead = readexgleadoffdetectionmode(thisShimmer)
            % READEXGLEADOFFDETECTIONMODE - Reads the ExG lead-off
            % detection mode
            %
            %   OUTPUT: isRead - isRead is true if exg lead-off detection
            %                    mode is successfully retrieved.
            %                    isRead is false means if retrieval is not
            %                    successful.
            %
            if (~strcmp(thisShimmer.getstate,'Connected') || thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3...
                    || thisShimmer.FirmwareCompatibilityCode < 3)
                thisShimmer.EXG1Loff = 'Nan';
                thisShimmer.EXG2Loff = 'Nan';
                thisShimmer.EXG1Config2 = 'Nan';
                thisShimmer.EXG2Config2 = 'Nan';
                thisShimmer.EXG1RLD_Sens = 'Nan';
                thisShimmer.EXG1LOFF_Sens = 'Nan';
                thisShimmer.EXG2LOFF_Sens = 'Nan';
                thisShimmer.EXG1Ch1Set = 'Nan';
                thisShimmer.EXG1Ch2Set = 'Nan';
                thisShimmer.EXG2Ch1Set = 'Nan';
                thisShimmer.EXG2Ch2Set = 'Nan';
                thisShimmer.EXG1FLEAD_OFF = 'Nan'; % Lead-off frequency
                thisShimmer.EXG2FLEAD_OFF = 'Nan'; % Lead-off frequency
                thisShimmer.EXG1PDB_LOFF_COMP = 'Nan'; % Lead-off comparator power-down
                thisShimmer.EXG2PDB_LOFF_COMP = 'Nan'; % Lead-off comparator power-down
                thisShimmer.EXG1RLD_LOFF_SENSE = 'Nan'; % RLD lead-off sense function
                thisShimmer.EXG1LOFF2P = 'Nan'; % Channel 2 lead-off detection positive inputs
                thisShimmer.EXG1LOFF1N = 'Nan'; % Channel 1 lead-off detection negative inputs
                thisShimmer.EXG1LOFF1P = 'Nan'; % Channel 1 lead-off detection positive inputs
                thisShimmer.EXG2LOFF2P = 'Nan'; % Channel 2 lead-off detection positive inputs
                thisShimmer.EXG1PD1 = 'Nan'; % Channel 1 power-down
                thisShimmer.EXG2PD1 = 'Nan'; % Channel 1 power-down
                thisShimmer.EXG1PD2 = 'Nan'; % Channel 2 power-down
                thisShimmer.EXG2PD2 = 'Nan'; % Channel 2 power-down
                thisShimmer.EXGLeadOffDetectionMode = 'Nan';
                isRead = false;
            end
            
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: readexgleadoffdetectionmode - Cannot get ExG lead-off detection mode for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: readexgleadoffdetectionmode - Only supported on Shimmer3');
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: readexgleadoffdetectionmode - Command not supported for this firmware version, please update firmware.')
            else
                chipselect = 0;
                while chipselect < 2;                                                        % Get ExG register values for both SENSOR_EXG1 and SENSOR_EXG2
                    clearreaddatabuffer(thisShimmer);                                        % As a precaution always clear the read data buffer before a write
                    writetocomport(thisShimmer,thisShimmer.GET_EXG_REGS_COMMAND);            % Send command to get ExG register values
                    writetocomport(thisShimmer, char(chipselect));                           % chipselect = 0/1 selects SENSOR_EXG1/SENSOR_EXG2
                    writetocomport(thisShimmer, char(0));                                    % Start at byte 0.
                    writetocomport(thisShimmer, char(10));                                   % Read 10 bytes
                    isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                    
                    if(~isAcknowledged)
                        fprintf(strcat('Warning: readexgleadoffdetectionmode - Get ExG registers command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        if chipselect == 0  %SENSOR_EXG1
                            thisShimmer.EXG1Loff = 'Nan';
                            thisShimmer.EXG1FLEAD_OFF = 'Nan'; % Lead-off frequency
                            thisShimmer.EXG1Config2 = 'Nan';
                            thisShimmer.EXG1PDB_LOFF_COMP = 'Nan'; % Lead-off comparator power-down
                            thisShimmer.EXG1RLD_Sens = 'Nan';
                            thisShimmer.EXG1RLD_LOFF_SENSE = 'Nan'; % RLD lead-off sense function
                            thisShimmer.EXG1LOFF_Sens = 'Nan';
                            thisShimmer.EXG1LOFF2P = 'Nan'; % Channel 2 lead-off detection positive inputs
                            thisShimmer.EXG1LOFF1N = 'Nan'; % Channel 1 lead-off detection negative inputs
                            thisShimmer.EXG1LOFF1P = 'Nan'; % Channel 1 lead-off detection positive inputs
                            thisShimmer.EXG1Ch1Set = 'Nan';
                            thisShimmer.EXG1Ch2Set = 'Nan';
                            thisShimmer.EXG1PD1 = 'Nan'; % Channel 1 power-down
                            thisShimmer.EXG1PD2 = 'Nan'; % Channel 2 power-down
                            isRead = false;
                        elseif chipselect == 1 %SENSOR_EXG2
                            thisShimmer.EXG2Loff = 'Nan';
                            thisShimmer.EXG2FLEAD_OFF = 'Nan'; % Lead-off frequency
                            thisShimmer.EXG2Config2 = 'Nan';
                            thisShimmer.EXG2PDB_LOFF_COMP = 'Nan'; % Lead-off comparator power-down
                            thisShimmer.EXG2LOFF_Sens = 'Nan';
                            thisShimmer.EXG2LOFF2P = 'Nan'; % Channel 2 lead-off detection positive inputs
                            thisShimmer.EXG2Ch1Set = 'Nan';
                            thisShimmer.EXG2Ch2Set = 'Nan';
                            thisShimmer.EXG2PD1 = 'Nan'; % Channel 1 power-down
                            thisShimmer.EXG2PD2 = 'Nan'; % Channel 2 power-down
                            isRead = false;
                        end
                    else
                        serialData = [];
                        nIterations = 0;
                        while(length(serialData) < 12 && nIterations < 4)                      % Read the 12 byte response from the realterm buffer
                            [tempSerialData, isFileOpen] = readdatabuffer(thisShimmer, inf);   % Read all available serial data from the com port
                            serialData = [serialData; tempSerialData];
                            pause(.2);
                            nIterations = nIterations + 1;
                        end
                        shimmerResponse = serialData(1:12);
                        if (~isempty(serialData) && serialData(1)==thisShimmer.EXG_REGS_RESPONSE)
                            % Update properties
                            if chipselect == 0 %SENSOR_EXG1
                                thisShimmer.EXG1Loff = shimmerResponse(5);       
                                thisShimmer.EXG1FLEAD_OFF = bitand(thisShimmer.EXG1Loff,1); % Lead-off frequency
                                thisShimmer.EXG1Config2 = shimmerResponse(4); 
                                thisShimmer.EXG1PDB_LOFF_COMP = bitshift(bitand(thisShimmer.EXG1Config2,64),-6); % Lead-off comparator power-down
                                thisShimmer.EXG1RLD_Sens = shimmerResponse(8); 
                                thisShimmer.EXG1RLD_LOFF_SENSE = bitshift(bitand(thisShimmer.EXG1RLD_Sens,16),-4); % RLD lead-off sense function
                                thisShimmer.EXG1LOFF_Sens = shimmerResponse(9); 
                                thisShimmer.EXG1LOFF2P = bitshift(bitand(thisShimmer.EXG1LOFF_Sens,4),-2); % Channel 2 lead-off detection positive inputs
                                thisShimmer.EXG1LOFF1N = bitshift(bitand(thisShimmer.EXG1LOFF_Sens,2),-1); % Channel 1 lead-off detection negative inputs
                                thisShimmer.EXG1LOFF1P = bitand(thisShimmer.EXG1LOFF_Sens,1); % Channel 1 lead-off detection positive inputs
                                thisShimmer.EXG1Ch1Set = shimmerResponse(6); 
                                thisShimmer.EXG1Ch2Set = shimmerResponse(7); 
                                thisShimmer.EXG1PD1 = bitshift(bitand(thisShimmer.EXG1Ch1Set,128),-7); % Channel 1 power-down
                                thisShimmer.EXG1PD2 = bitshift(bitand(thisShimmer.EXG1Ch2Set,128),-7); % Channel 2 power-down
                                isRead = true;
                            elseif chipselect == 1 %SENSOR_EXG2
                                thisShimmer.EXG2Loff = shimmerResponse(5); 
                                thisShimmer.EXG2FLEAD_OFF = bitand(thisShimmer.EXG2Loff,1); % Lead-off frequency
                                thisShimmer.EXG2Config2 = shimmerResponse(4);
                                thisShimmer.EXG2PDB_LOFF_COMP = bitshift(bitand(thisShimmer.EXG2Config2,64),-6); % Lead-off comparator power-down
                                thisShimmer.EXG2LOFF_Sens = shimmerResponse(9); 
                                thisShimmer.EXG2LOFF2P = bitshift(bitand(thisShimmer.EXG2LOFF_Sens,4),-2); % Channel 2 lead-off detection positive inputs
                                thisShimmer.EXG2Ch1Set = shimmerResponse(6);
                                thisShimmer.EXG2Ch2Set = shimmerResponse(7);
                                thisShimmer.EXG2PD1 = bitshift(bitand(thisShimmer.EXG2Ch1Set,128),-7); % Channel 1 power-down
                                thisShimmer.EXG2PD2 = bitshift(bitand(thisShimmer.EXG2Ch2Set,128),-7); % Channel 2 power-down
                                isRead = true;
                            end
                            if (chipselect ==1 && thisShimmer.EXG1FLEAD_OFF == 0 && thisShimmer.EXG2FLEAD_OFF == 0 && thisShimmer.EXG1PDB_LOFF_COMP == 1 ...      % EXGLeadOffDetectionMode = 'DC Current'
                                    && thisShimmer.EXG2PDB_LOFF_COMP == 1 && thisShimmer.EXG1RLD_LOFF_SENSE == 1 && thisShimmer.EXG1LOFF2P == 1 ...
                                    && thisShimmer.EXG1LOFF1N == 1 && thisShimmer.EXG1LOFF1P == 1 && thisShimmer.EXG2LOFF2P == 1 ...
                                    && thisShimmer.EXG1PD1 == 0 && thisShimmer.EXG2PD1 == 0 && thisShimmer.EXG1PD2 == 0 && thisShimmer.EXG2PD2 == 0)
                                thisShimmer.EXGLeadOffDetectionMode = 1;
                            elseif (chipselect ==1 && thisShimmer.EXG1PDB_LOFF_COMP == 0 ...                                                                      % EXGLeadOffDetectionMode = 'Off'
                                    && thisShimmer.EXG2PDB_LOFF_COMP == 0 && thisShimmer.EXG1RLD_LOFF_SENSE == 0 && thisShimmer.EXG1LOFF2P == 0 ...
                                    && thisShimmer.EXG1LOFF1N == 0 && thisShimmer.EXG1LOFF1P == 0 && thisShimmer.EXG2LOFF2P == 0)
                                thisShimmer.EXGLeadOffDetectionMode = 0;
                            else                                                                                                                % EXGLeadOffDetectionMode = 'Unknown'
                                thisShimmer.EXGLeadOffDetectionMode = 'Nan';
                            end
                        else
                            fprintf(strcat('Warning: readexgleadoffdetectionmode - Get ExG registers command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            if chipselect == 0 %SENSOR_EXG1
                                thisShimmer.EXG1Loff = 'Nan';
                                thisShimmer.EXG1FLEAD_OFF = 'Nan'; % Lead-off frequency
                                thisShimmer.EXG1Config2 = 'Nan';
                                thisShimmer.EXG1PDB_LOFF_COMP = 'Nan'; % Lead-off comparator power-down
                                thisShimmer.EXG1RLD_Sens = 'Nan';
                                thisShimmer.EXG1RLD_LOFF_SENSE = 'Nan'; % RLD lead-off sense function
                                thisShimmer.EXG1LOFF_Sens = 'Nan';
                                thisShimmer.EXG1LOFF2P = 'Nan'; % Channel 2 lead-off detection positive inputs
                                thisShimmer.EXG1LOFF1N = 'Nan'; % Channel 1 lead-off detection negative inputs
                                thisShimmer.EXG1LOFF1P = 'Nan'; % Channel 1 lead-off detection positive inputs
                                thisShimmer.EXG1Ch1Set = 'Nan';
                                thisShimmer.EXG1Ch2Set = 'Nan';
                                thisShimmer.EXG1PD1 = 'Nan'; % Channel 1 power-down
                                thisShimmer.EXG1PD2 = 'Nan'; % Channel 2 power-down
                                isRead = false;
                            elseif chipselect == 1 %SENSOR_EXG2
                                thisShimmer.EXG2Loff = 'Nan';
                                thisShimmer.EXG2FLEAD_OFF = 'Nan'; % Lead-off frequency
                                thisShimmer.EXG2Config2 = 'Nan';
                                thisShimmer.EXG2PDB_LOFF_COMP = 'Nan'; % Lead-off comparator power-down
                                thisShimmer.EXG2LOFF_Sens = 'Nan';
                                thisShimmer.EXG2LOFF2P = 'Nan'; % Channel 2 lead-off detection positive inputs
                                thisShimmer.EXG2Ch1Set = 'Nan';
                                thisShimmer.EXG2Ch2Set = 'Nan';
                                thisShimmer.EXG2PD1 = 'Nan'; % Channel 1 power-down
                                thisShimmer.EXG2PD2 = 'Nan'; % Channel 2 power-down
                                isRead = false;
                            end
                        end
                    end
                    chipselect = chipselect + 1;
                end
            end
        end % readexgleadoffdetectionmode

        function isRead = readexgleadoffdetectioncurrent(thisShimmer, chipIdentifier)
            % READEXGLEADOFFDETECTIONCURRENT - Reads the ExG lead-off
            % detection current
            %
            %   INPUT chipIdentifier - chipIdentifier is 1 or 2 to select
            %                          SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: isRead - isRead is true if ExG lead-off
            %                    detection current is successfully retrieved
            %
            %                    isRead is false means if retrieval is not
            %                    successful.
            %
            if (~strcmp(thisShimmer.getstate,'Connected') || (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)...
                    || thisShimmer.FirmwareCompatibilityCode < 3)
                if (chipIdentifier == 1)
                    thisShimmer.EXG1Loff = 'Nan';
                    thisShimmer.EXG1ILEAD_OFF = 'Nan';
                    isRead = false;
                elseif (chipIdentifier == 2)
                    thisShimmer.EXG2Loff = 'Nan';
                    thisShimmer.EXG2ILEAD_OFF = 'Nan';
                    isRead = false;
                else
                    disp('Warning: readexgleadoffdetectioncurrent - Invalid value for chipIdentifier.')
                    isRead = false;
                end
            end
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: readexgleadoffdetectioncurrent - Cannot get ExG lead-off detection current for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: readexgleadoffdetectioncurrent - Only supported on Shimmer3');
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: readexgleadoffdetectioncurrent - Command not supported for this firmware version, please update firmware.')
            elseif ~(chipIdentifier == 1 || chipIdentifier == 2)
                disp('Warning: readexgleadoffdetectioncurrent - Invalid value for chipIdentifier.')
            else
                clearreaddatabuffer(thisShimmer);                                        % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer,thisShimmer.GET_EXG_REGS_COMMAND);            % Send command to get ExG register values
                writetocomport(thisShimmer, char(chipIdentifier-1));                     % Selects SENSOR_EXG1/SENSOR_EXG2
                writetocomport(thisShimmer, char(2));                                    % Start at byte 2.
                writetocomport(thisShimmer, char(1));                                    % Read 1 byte.
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                
                if(~isAcknowledged)
                    fprintf(strcat('Warning: readexgleadoffdetectioncurrent - Get ExG registers command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    if (chipIdentifier == 1)
                        thisShimmer.EXG1Loff = 'Nan';
                        thisShimmer.EXG1ILEAD_OFF = 'Nan';
                        isRead = false;
                    elseif (chipIdentifier == 2)
                        thisShimmer.EXG2Loff = 'Nan';
                        thisShimmer.EXG2ILEAD_OFF = 'Nan';
                        isRead = false;
                    end
                else
                    serialData = [];
                    [serialData, isFileOpen] = readdatabuffer(thisShimmer, inf);         % Read all available serial data from the com port
                    if (~isempty(serialData) && serialData(1)==thisShimmer.EXG_REGS_RESPONSE)
                        % Update properties
                        if (chipIdentifier == 1)
                            thisShimmer.EXG1Loff = serialData(3);
                            thisShimmer.EXG1ILEAD_OFF = bitand(thisShimmer.EXG1Loff,12);
                            isRead = true;
                        elseif (chipIdentifier == 2)
                            thisShimmer.EXG2Loff = serialData(3);
                            thisShimmer.EXG2ILEAD_OFF = bitand(thisShimmer.EXG2Loff,12);
                            isRead = true;
                        end
                    else
                        fprintf(strcat('Warning: readexgleadoffdetectioncurrent - Get ExG registers command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        if (chipIdentifier == 1)
                            thisShimmer.EXG1Loff = 'Nan';
                            thisShimmer.EXG1ILEAD_OFF = 'Nan';
                            isRead = false;
                        elseif (chipIdentifier == 2)
                            thisShimmer.EXG2Loff = 'Nan';
                            thisShimmer.EXG2ILEAD_OFF = 'Nan';
                            isRead = false;
                        end
                    end
                end
                
            end
        end % readexgleadoffdetectioncurrent
        
        function isRead = readexgleadoffcomparatorthreshold(thisShimmer, chipIdentifier)
            % READEXGLEADOFFCOMPARATORTHRESHOLD - Reads the ExG lead-off
            % comparator threshold
            %
            %   INPUT chipIdentifier - chipIdentifier is 1 or 2 to select
            %                          SENSOR_EXG1 or SENSOR_EXG2
            %
            %   OUTPUT: isRead - isRead is true if ExG lead-off
            %                    comparator threshold is successfully retrieved
            %
            %                    isRead is false means if retrieval is not
            %                    successful.
            %
            if (~strcmp(thisShimmer.getstate,'Connected') || (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)...
                    || thisShimmer.FirmwareCompatibilityCode < 3)
                if (chipIdentifier == 1)
                    thisShimmer.EXG1Loff = 'Nan';
                    thisShimmer.EXG1COMP_TH = 'Nan';
                    isRead = false;
                elseif (chipIdentifier == 2)
                    thisShimmer.EXG2Loff = 'Nan';
                    thisShimmer.EXG2COMP_TH = 'Nan';
                    isRead = false;
                else
                    disp('Warning: readexgleadoffcomparatorthreshold - Invalid value for chipIdentifier.')
                    isRead = false;
                end
            end
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: readexgleadoffcomparatorthreshold - Cannot get ExG lead-off comparator threshold for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: readexgleadoffcomparatorthreshold - Only supported on Shimmer3');
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: readexgleadoffcomparatorthreshold - Command not supported for this firmware version, please update firmware.')
            elseif ~(chipIdentifier == 1 || chipIdentifier == 2)
                disp('Warning: readexgleadoffcomparatorthreshold - Invalid value for chipIdentifier.')
            else
                clearreaddatabuffer(thisShimmer);                                        % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer,thisShimmer.GET_EXG_REGS_COMMAND);            % Send command to get ExG register values
                writetocomport(thisShimmer, char(chipIdentifier-1));                     % Selects SENSOR_EXG1/SENSOR_EXG2
                writetocomport(thisShimmer, char(2));                                    % Start at byte 2.
                writetocomport(thisShimmer, char(1));                                    % Read 1 byte.
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                
                if(~isAcknowledged)
                    fprintf(strcat('Warning: readexgleadoffcomparatorthreshold - Get ExG registers command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    if (chipIdentifier == 1)
                        thisShimmer.EXG1Loff = 'Nan';
                        thisShimmer.EXG1COMP_TH = 'Nan';
                        isRead = false;
                    elseif (chipIdentifier == 2)
                        thisShimmer.EXG2Loff = 'Nan';
                        thisShimmer.EXG2COMP_TH = 'Nan';
                        isRead = false;
                    end
                else
                    serialData = [];
                    [serialData, isFileOpen] = readdatabuffer(thisShimmer, inf);         % Read all available serial data from the com port
                    if (~isempty(serialData) && serialData(1)==thisShimmer.EXG_REGS_RESPONSE)
                        % Update properties
                        if (chipIdentifier == 1)
                            thisShimmer.EXG1Loff = serialData(3);
                            thisShimmer.EXG1COMP_TH = bitand(thisShimmer.EXG1Loff,224);
                            isRead = true;
                        elseif (chipIdentifier == 2)
                            thisShimmer.EXG2Loff = serialData(3);
                            thisShimmer.EXG2COMP_TH = bitand(thisShimmer.EXG2Loff,224);
                            isRead = true;
                        end
                    else
                        fprintf(strcat('Warning: readexgleadoffcomparatorthreshold - Get ExG registers command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        if (chipIdentifier == 1)
                            thisShimmer.EXG1Loff = 'Nan';
                            thisShimmer.EXG1COMP_TH = 'Nan';
                            isRead = false;
                        elseif (chipIdentifier == 2)
                            thisShimmer.EXG2Loff = 'Nan';
                            thisShimmer.EXG2COMP_TH = 'Nan';
                            isRead = false;
                        end
                    end
                end
                
            end
        end % readexgleadoffcomparatorthreshold

                                 
        function isRead = readrealtimeclock(thisShimmer) % function readrealtimeclock
            % READREALTIMECLOCK - Reads the Real Time Clock setting from
            % the Shimmer3 and updates the class properties accordingly.
            %
            %
            %   OUTPUT: isRead - isRead is true if Real Time Clock setting
            %                    is successfully retrieved.
            %
            %                    isRead is false means if retrieval is not
            %                    successful.
            %
            if ~strcmp(thisShimmer.getstate,'Connected')
                fprintf(strcat('Warning: readrealtimeclock - Cannot get Real Time Clock setting for COM',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
                thisShimmer.RealTimeClockMilliseconds = 'Nan';
                thisShimmer.RealTimeClockTicks = 'Nan';
                isRead = 'false';
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                fprintf('Warning: readrealtimeclock - Only supported on Shimmer3.\n');
                thisShimmer.RealTimeClockMilliseconds = 'Nan';
                thisShimmer.RealTimeClockTicks = 'Nan';
                isRead = 'false';
            elseif (thisShimmer.FirmwareCompatibilityCode < 6)
                fprintf('Warning: readrealtimeclock - Command not supported for this firmware version, please update firmware.\n');
                thisShimmer.RealTimeClockMilliseconds = 'Nan';
                thisShimmer.RealTimeClockTicks = 'Nan';
                isRead = 'false';
            else
                clearreaddatabuffer(thisShimmer);                                        % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer,thisShimmer.GET_RWC_COMMAND);                 % Send command to get Real Time Clock value
  
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);   % Wait for Acknowledgment from Shimmer
                
                if(~isAcknowledged)
                    fprintf(strcat('Warning: readrealtimeclock - Get Real Time Clock command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    thisShimmer.RealTimeClockMilliseconds = 'Nan';
                    thisShimmer.RealTimeClockTicks = 'Nan';
                    isRead = 'false';
                else
                    serialData = [];
                    [serialData, isFileOpen] = readdatabuffer(thisShimmer, 9);           % Read all available serial data from the com port
                    if (~isempty(serialData) && serialData(1)==thisShimmer.RWC_RESPONSE)
                        % Update class properties
                        responseBytesLittleEndian = serialData(2:9);
                        thisShimmer.RealTimeClockTicks = typecast(responseBytesLittleEndian,'uint64');
                        thisShimmer.RealTimeClockMilliseconds = double(thisShimmer.RealTimeClockTicks)/32.768;
                        isRead = 'true';
                    else
                        fprintf(strcat('Warning: readrealtimeclock - Get Real Time Clock command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        thisShimmer.RealTimeClockMilliseconds = 'Nan';
                        thisShimmer.RealTimeClockTicks = 'Nan';
                        isRead = 'false';
                    end
                end
                
            end
        end % function readrealtimeclock

        
        function isRead = readledblink(thisShimmer)
            % Sends the GET_BLINK_LED to Shimmer - in Connected state 
            % Receives the Blink LED setting and updates the BlinkLED property.
            if (thisShimmer.FirmwareIdentifier == 3)
                disp('Warning: setledblink - Not supported for LogAndStream.');
                
            elseif (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                                           % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_BLINK_LED);                     % Send the GET_BLINK_LED Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);      % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 2);         % Read the 2 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.BLINK_LED_RESPONSE)
                            thisShimmer.BlinkLED = shimmerResponse(2);
                            isRead = true;
                        else
                            thisShimmer.BlinkLED = 'Nan';                  % Set the BlinkLED to 'Nan' to indicate unknown
                            fprintf(strcat('Warning: readledblink - Get LED Blink command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.BlinkLED = 'Nan';                  % Set the BlinkLED to 'Nan' to indicate unknown
                        fprintf(strcat('Warning: readledblink - Get LED Blink command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.BlinkLED = 'Nan';                        % Set the BlinkLED to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readledblink - Get LED Blink command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readledblink - Cannot get LED Blink for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readledblink
        
        function isRead = reademgcalibrationparameters(thisShimmer)
            % Sends the GET_EMG_CALIBRATION_COMMAND to Shimmer - in Connected state 
            % Receives EMG calibration parameters and updates the corresponding properties. 
            % This function is currently only available for Shimmer2/2r.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_EMG_CALIBRATION_COMMAND);            % Send the GET_EMG_CALIBRATION_COMMAND Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 5);        % Read the 5 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.EMG_CALIBRATION_RESPONSE)
                            if shimmerResponse(3)~= 255                        % Test for stored calibration data on the SD card
                                thisShimmer.EMGOffset=bitshift(double(shimmerResponse(2)),8)+double(shimmerResponse(3));
                                thisShimmer.EMGGain=bitshift(double(shimmerResponse(4)),8)+double(shimmerResponse(5));
                                
                            else
                                str = sprintf('Warning: reademgcalibrationparameters - Calibration parameters for EMG not found for Shimmer on Com Port %s, default values will be used', thisShimmer.ComPort);
                                disp(str);
                                
                                thisShimmer.EMGOffset = 2060;
                                thisShimmer.EMGGain = 750;
                            end
                            isRead = true;
                        else
                            thisShimmer.EMGOffset = 2060;
                            thisShimmer.EMGGain = 750;
                            fprintf(strcat('Warning: reademgcalibrationparameters - Get EMG calibration parameters command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'. default values will be used.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.EMGOffset = 2060;
                        thisShimmer.EMGGain = 750;
                        fprintf(strcat('Warning: reademgcalibrationparameters - Get EMG calibration parameters command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'. default values will be used.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.EMGOffset = 2060;
                    thisShimmer.EMGGain = 750;
                    fprintf(strcat('Warning: reademgcalibrationparameters - Get EMG calibration parameters command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'. default values will be used.\n'));
                    isRead = false;
                end
                
             elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3) 
                isRead = false;
                disp('Warning: reademgcalibrationparameters - EMG calibration parameters not currently available for Shimmer3.');
           else
                isRead = false;
                fprintf(strcat('Warning: reademgcalibrationparameters - Cannot get EMG calibration parameters for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function reademgcalibrationparameters
        
        function isRead = readecgcalibrationparameters(thisShimmer)
            % Sends the GET_ECG_CALIBRATION_COMMAND to Shimmer - in Connected state 
            % Receives ECG calibration parameters and updates the corresponding properties. 
            % This function is currently only available for Shimmer2/2r.
            if (strcmp(thisShimmer.State,'Connected') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                
                clearreaddatabuffer(thisShimmer);                                          % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.GET_ECG_CALIBRATION_COMMAND);      % Send GET_ECG_CALIBRATION_COMMAND to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);     % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, 9);        % Read the 5 byte response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.ECG_CALIBRATION_RESPONSE)
                            if shimmerResponse(3)~= 255                        % Test for stored calibration data on the SD card  
                                thisShimmer.ECGLALLOffset=bitshift(double(shimmerResponse(2)),8)+double(shimmerResponse(3));
                                thisShimmer.ECGLALLGain=bitshift(double(shimmerResponse(4)),8)+double(shimmerResponse(5));
                                thisShimmer.ECGRALLOffset=bitshift(double(shimmerResponse(6)),8)+double(shimmerResponse(7));
                                thisShimmer.ECGRALLGain=bitshift(double(shimmerResponse(8)),8)+double(shimmerResponse(9));
                                
                            else
                                str = sprintf('Warning: readecgcalibrationparameters - Calibration parameters for ECG not found for Shimmer on Com Port %s, default values will be used', thisShimmer.ComPort);
                                disp(str);
                                
                                thisShimmer.ECGLALLOffset = 2060;
                                thisShimmer.ECGLALLGain = 175;
                                thisShimmer.ECGRALLOffset = 2060;
                                thisShimmer.ECGRALLGain = 175;
                            end
                            isRead = true;
                        else
                            thisShimmer.ECGLALLOffset = 2060;
                            thisShimmer.ECGLALLGain = 175;
                            thisShimmer.ECGRALLOffset = 2060;
                            thisShimmer.ECGRALLGain = 175;
                            fprintf(strcat('Warning: readecgcalibrationparameters - Get ECG calibration parameters command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'. default values will be used.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.ECGLALLOffset = 2060;
                        thisShimmer.ECGLALLGain = 175;
                        thisShimmer.ECGRALLOffset = 2060;
                        thisShimmer.ECGRALLGain = 175;
                        fprintf(strcat('Warning: readecgcalibrationparameters - Get ECG calibration parameters command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'. default values will be used.\n'));
                        isRead = false;
                    end
                else
                    thisShimmer.ECGLALLOffset = 2060;
                    thisShimmer.ECGLALLGain = 175;
                    thisShimmer.ECGRALLOffset = 2060;
                    thisShimmer.ECGRALLGain = 175;
                    fprintf(strcat('Warning: readecgcalibrationparameters - Get ECG calibration parameters command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'. default values will be used.\n'));
                    isRead = false;
                end
            elseif(thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3) 
                isRead = false;
                disp('Warning: readecgcalibrationparameters - ECG calibration parameters not currently available for Shimmer3.');
            else
                isRead = false;
                fprintf(strcat('Warning: readecgcalibrationparameters - Cannot get ECG calibration parameters for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readecgcalibrationparameters
        
        function isRead = readenabledsensors(thisShimmer)
            % Calls the inquiry function and updates the EnabledSensors property.
            if (strcmp(thisShimmer.State,'Connected'))
                
                clearreaddatabuffer(thisShimmer);                          % As a precaution always clear the read data buffer before a write
                isRead = inquiry(thisShimmer);                             % Send an inquiry command to the Shimmer
                                                
                if (~isRead)
                    thisShimmer.EnabledSensors = 'Nan';                    % Set the EnabledSensors to 'Nan' to indicate unknown
                    fprintf(strcat('Warning: readenabledsensors - inquiry command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
                
            else
                isRead = false;
                fprintf(strcat('Warning: readenabledsensors - Cannot get enabled sensors for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readenabledsensors
                
        function isRead = readshimmerversion(thisShimmer)
            % Sends the GET_SHIMMER_VERSION_COMMAND_NEW (or GET_SHIMMER_VERSION_COMMAND) to Shimmer - in Connected state 
            % Receives the Shimmer Version and updates the ShimmerVersion property. 
            if (strcmp(thisShimmer.State,'Connected'))
                    clearreaddatabuffer(thisShimmer);                                         % As a precaution always clear the read data buffer before a write

                    writetocomport(thisShimmer, thisShimmer.GET_SHIMMER_VERSION_COMMAND_NEW);     % Send the new Get Shimmer Version Command to the Shimmer                    
                    isAcknowledged = waitforack(thisShimmer, 2);    % Wait for Acknowledgment from Shimmer (user short timeout to speed up initialisation)
                    if (~isAcknowledged)
                        writetocomport(thisShimmer, thisShimmer.GET_SHIMMER_VERSION_COMMAND); % Send the deprecated Get Shimmer Version Command to the Shimmer
                        isAcknowledged = waitforack(thisShimmer, 2);
                    end
                    
                    if (isAcknowledged == true)
                        [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, inf);     % Read response from the realterm buffer
                        
                        if (~isempty(shimmerResponse) && (shimmerResponse(1) == thisShimmer.SHIMMER_VERSION_RESPONSE))    
                            thisShimmer.ShimmerVersion = shimmerResponse(2);
                        else
                            thisShimmer.ShimmerVersion = 'NaN';
                            fprintf(strcat('Warning: readshimmerversion - Shimmer version command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        thisShimmer.ShimmerVersion = 'NaN';
                        fprintf(strcat('Warning: readshimmerversion - Shimmer version command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
            else
                thisShimmer.ShimmerVersion = 'NaN';
                isRead = false;
                fprintf(strcat('Warning: readshimmerversion - Cannot get Shimmer version reponse for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
            
        end % function readshimmerversion
              
        function isRead = inquiry(thisShimmer)
            % Sends the INQUIRY_COMMAND to Shimmer - in Connected state 
            % Receives Inquire Response and call the function
            % parseinquiryresponse with the just received Inquire Response 
            % as input argument.
            if (strcmp(thisShimmer.State,'Connected'))
                clearreaddatabuffer(thisShimmer);                                         % As a precaution always clear the read data buffer before a write
                writetocomport(thisShimmer, thisShimmer.INQUIRY_COMMAND);                 % Send the Inquiry Command to the Shimmer
                
                isAcknowledged = waitforack(thisShimmer, thisShimmer.DEFAULT_TIMEOUT);    % Wait for Acknowledgment from Shimmer
                
                if (isAcknowledged == true)
                    [shimmerResponse, isFileOpen] = readdatabuffer(thisShimmer, inf);     % Read Inquiry Command response from the realterm buffer
                    
                    if ~isempty(shimmerResponse)
                        
                        if (shimmerResponse(1) == thisShimmer.INQUIRY_RESPONSE)
                            parseinquiryresponse(thisShimmer, shimmerResponse);
                            isRead = true;
                        else
                            fprintf(strcat('Warning: inquiry - Inquiry command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                            isRead = false;
                        end
                    else
                        fprintf(strcat('Warning: inquiry - Inquiry command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                        isRead = false;
                    end
                    
                else
                    fprintf(strcat('Warning: inquiry - Inquiry command response expected but not returned for Shimmer COM',thisShimmer.ComPort,'.\n'));
                    isRead = false;
                end
            else
                isRead = false;
                fprintf(strcat('Warning: inquiry - Cannot get inquiry reponse for COM ',thisShimmer.ComPort,' as Shimmer is not connected.\n'));
            end
        end % function inquiry
                
        function parseinquiryresponse(thisShimmer, inquiryResponse)
            % Parses the Inquiry Response inquiryResponse and updates
            % properties accordingly.
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                thisShimmer.SamplingRate = 32768.0 /double(int32(256*uint16(inquiryResponse(3)))+int32(inquiryResponse(2)));
                nChannels = inquiryResponse(8);
                thisShimmer.BufferSize = inquiryResponse(9);                                        % Buffer size is currently not used
                nIterations = 0;
                while(length(inquiryResponse) < 9+nChannels && nIterations < 4)
                    [tempResponse, isFileOpen] = readdatabuffer(thisShimmer, inf);     % Read Inquiry Command response from the realterm buffer
                    inquiryResponse = [inquiryResponse; tempResponse];
                    nIterations = nIterations + 1;
                end
                signalIDArray = inquiryResponse(10:9+nChannels);
                thisShimmer.ConfigByte0 = double(inquiryResponse(4));                               % ConfigByte0
                thisShimmer.ConfigByte1 = double(inquiryResponse(5));                               % ConfigByte1
                thisShimmer.ConfigByte2 = double(inquiryResponse(6));                               % ConfigByte2
                thisShimmer.ConfigByte3 = double(inquiryResponse(7));                               % ConfigByte3              
                thisShimmer.AccelWideRangeHRMode = bitand(thisShimmer.ConfigByte0,1);               % High Resolution mode LSM303DLHC/LSM303AHTR
                thisShimmer.AccelWideRangeLPMode = bitand(bitshift(thisShimmer.ConfigByte0,-1),1);  % Low Power mode LSM303DLHC/LSM303AHTR
                thisShimmer.AccelRange = bitand(bitshift(thisShimmer.ConfigByte0,-2),3); 
                thisShimmer.AccelWideRangeDataRate = bitand(bitshift(thisShimmer.ConfigByte0,-4),15);   
                thisShimmer.GyroRate = bitand(thisShimmer.ConfigByte1,255);                         % MPU9150 sampling rate 
                thisShimmer.GyroRange = bitand(thisShimmer.ConfigByte2,3);
                thisShimmer.MagRate =  bitand(bitshift(thisShimmer.ConfigByte2,-2),7);
                thisShimmer.MagRange = bitand(bitshift(thisShimmer.ConfigByte2,-5),7); 
                thisShimmer.InternalExpPower = bitand(thisShimmer.ConfigByte3,1);
                thisShimmer.GsrRange = bitand(bitshift(thisShimmer.ConfigByte3,-1),7);
                thisShimmer.PressureResolution = bitand(bitshift(thisShimmer.ConfigByte3,-4),3);
                interpretdatapacketformat(thisShimmer,signalIDArray);      
            else
                thisShimmer.SamplingRate = 1024.0 / double(inquiryResponse(2));
                thisShimmer.AccelRange = inquiryResponse(3);
                thisShimmer.ConfigByte0 = inquiryResponse(4);
                nChannels = inquiryResponse(5);                                                     % The number of Channels aka number of signals being transmitted
                thisShimmer.BufferSize = inquiryResponse(6);                                        % Buffer size is currently not used
                signalIDArray = inquiryResponse(7:6+nChannels);
                interpretdatapacketformat(thisShimmer,signalIDArray);
            end
            
        end % parseinquiryresponse
        
        
        function determinehwcompcode(thisShimmer)
            % Checks Expansion Board ID and sets 'HardwareCompatibilityCode' accordingly
            % HardwareCompatibilityCode = 1:
            %  - BMP180 ('Pressure' & 'Temperature')
            %  - MPU9150 ('Gyro')
            %  - LSM303DLHC ('Wide Range Accel', 'Mag')
            %  - KXRB5-2042 ('Low Noise Accel')
            %
            % HardwareCompatibilityCode = 2:
            %  - BMP280 ('Pressure' & 'Temperature')
            %  - MPU9250 ('Gyro')
            %  - LSM303AHTR ('Wide Range Accel', 'Mag')
            %  - KXTC9-2050 ('Low Noise Accel')
            IDByteArray = readexpansionboardidbytes(thisShimmer, 3, 0);    % call readexpansionboardidbytes with numBytes = 3 and offset = 0
            IDByte0 = IDByteArray(1);
            IDByte1 = IDByteArray(2);
            IDByte2 = IDByteArray(3);
            
            thisShimmer.HardwareCompatibilityCode = 1;
            if (IDByte0 == 8 || IDByte0 == 14 || IDByte0 == 37)
                % Old 'not-unified' expansion board - check 'special
                % revision'
                if IDByte2 == 171
                    thisShimmer.HardwareCompatibilityCode = 2;
                end
            elseif IDByte0 == 31
                % IMU
                if IDByte1 >= 6
                    thisShimmer.HardwareCompatibilityCode = 2;
                end
            elseif (IDByte0 == 36 || IDByte0 == 38)
                % PROTO3 Mini/PROTO3 Deluxe 
                if IDByte1 < 3
                    % check 'special revision' for 'not-unified'
                    % expansion board
                    if IDByte2 == 171
                        thisShimmer.HardwareCompatibilityCode = 2;
                    end
                else
                    thisShimmer.HardwareCompatibilityCode = 2;
                end
            elseif IDByte0 == 47
                % ExG
                if IDByte1 >= 3
                    thisShimmer.HardwareCompatibilityCode = 2;
                end
            elseif IDByte0 == 48
                % GSR+
                if IDByte1 >= 3
                    thisShimmer.HardwareCompatibilityCode = 2;
                end
            elseif IDByte0 == 49
                % Bridge Amplifier+
                if IDByte1 >= 2
                    thisShimmer.HardwareCompatibilityCode = 2;
                end
            elseif IDByte0 == 59
                % Shimmer Development
                thisShimmer.HardwareCompatibilityCode = 2;
            end
        end % determinehwcompcode
        
        function interpretdatapacketformat(thisShimmer,signalIDArray)
            % Is called by the function parseinquiryresponse and interprets
            % the data packet format based on the input signalIDArray.
            enabledSensors = 0;                                            % Enabled/Disabled Sensors bitmap
            nBytesDataPacket = 1;                                          % Initially Number of Bytes in Data Packet = 1 (Packet Type byte)

            % Get Data Packet Format values for Timestamp
            signalNameArray(1) = cellstr('Timestamp');                     % Cell array containing the names of the signal in each data channel
            
                % Timestamp value is of type unsigned 16bit
                if (thisShimmer.FirmwareCompatibilityCode < 6)
                    nBytesDataPacket = nBytesDataPacket+2;                 % Increment Number of Bytes in Data Packet
                    signalDataTypeArray(1) = cellstr('u16');               % Cell array containing the data type of the signal in each data channel
                else
                    nBytesDataPacket = nBytesDataPacket+3;                 % Three byte timestamp has been introduced with FirmwareCompatibilityCode == 6
                    signalDataTypeArray(1) = cellstr('u24');               % Cell array containing the data type of the signal in each data channel
                end
            
                
            % Get Data Packet Format values for other enabled data signals
            for i = 1:length(signalIDArray)
                
                hexSignalID=dec2hex(signalIDArray(i));                     % Extract signalID(i) in hex formnat
                if (thisShimmer.ShimmerVersion==thisShimmer.SHIMMER_3)
                    switch hexSignalID
                        case ('0')
                            signalNameArray(i+1) = cellstr('Low Noise Accelerometer X');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('80'));
                        case ('1')
                            signalNameArray(i+1) = cellstr('Low Noise Accelerometer Y');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('80'));
                        case ('2')
                            signalNameArray(i+1) = cellstr('Low Noise Accelerometer Z');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('80'));
                        case ('3')
                            signalNameArray(i+1) = cellstr('Battery Voltage');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('2000'));
                        case ('4')
                            signalNameArray(i+1) = cellstr('Wide Range Accelerometer X');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('1000'));
                        case ('5')
                            signalNameArray(i+1) = cellstr('Wide Range Accelerometer Y');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('1000'));
                        case ('6')
                            signalNameArray(i+1) = cellstr('Wide Range Accelerometer Z');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('1000'));
                        case ('7')
                            signalNameArray(i+1) = cellstr('Magnetometer X');
                            if thisShimmer.HardwareCompatibilityCode < 2
                                signalDataTypeArray(i+1) = cellstr('i16*');
                            else
                                signalDataTypeArray(i+1) = cellstr('i16');
                            end
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('20'));
                        case ('8')
                            signalNameArray(i+1) = cellstr('Magnetometer Y');
                            if thisShimmer.HardwareCompatibilityCode < 2
                                signalDataTypeArray(i+1) = cellstr('i16*');
                            else
                                signalDataTypeArray(i+1) = cellstr('i16');
                            end
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('20'));
                        case ('9')
                            signalNameArray(i+1) = cellstr('Magnetometer Z');
                            if thisShimmer.HardwareCompatibilityCode < 2
                                signalDataTypeArray(i+1) = cellstr('i16*');
                            else
                                signalDataTypeArray(i+1) = cellstr('i16');
                            end
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('20'));
                        case ('A')
                            signalNameArray(i+1) = cellstr('Gyroscope X');
                            signalDataTypeArray(i+1) = cellstr('i16*');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('40'));
                        case ('B')
                            signalNameArray(i+1) = cellstr('Gyroscope Y');
                            signalDataTypeArray(i+1) = cellstr('i16*');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('40'));
                        case ('C')
                            signalNameArray(i+1) = cellstr('Gyroscope Z');
                            signalDataTypeArray(i+1) = cellstr('i16*');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('40'));
                        case ('D')
                            signalNameArray(i+1) = cellstr('External ADC A7');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('2'));
                        case ('E')
                            signalNameArray(i+1) = cellstr('External ADC A6');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('1'));
                        case ('F')
                            signalNameArray(i+1) = cellstr('External ADC A15');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('800'));
                        case ('10')
                            signalNameArray(i+1) = cellstr('Internal ADC A1');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('400'));
                        case ('11')
                            signalNameArray(i+1) = cellstr('Internal ADC A12');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('200'));
                        case ('12')
                            signalNameArray(i+1) = cellstr('Internal ADC A13');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('100'));
                        case ('13')
                            signalNameArray(i+1) = cellstr('Internal ADC A14');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('800000'));
                        case ('14')
                            signalNameArray(i+1) = cellstr('Alternative Accelerometer X');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('400000'));
                        case ('15')
                            signalNameArray(i+1) = cellstr('Alternative Accelerometer Y');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('400000'));
                        case ('16')
                            signalNameArray(i+1) = cellstr('Alternative Accelerometer Z');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('400000'));
                        case ('17')
                            signalNameArray(i+1) = cellstr('Alternative Magnetometer X');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('200000'));
                        case ('18')
                            signalNameArray(i+1) = cellstr('Alternative Magnetometer Y');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('200000'));
                        case ('19')
                            signalNameArray(i+1) = cellstr('Alternative Magnetometer Z');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('200000'));
                        case ('1A')
                            signalNameArray(i+1) = cellstr('Temperature');
                            signalDataTypeArray(i+1) = cellstr('u16*');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('40000'));
                        case ('1B')
                            signalNameArray(i+1) = cellstr('Pressure');
                            signalDataTypeArray(i+1) = cellstr('u24*');
                            nBytesDataPacket=nBytesDataPacket+3;
                            enabledSensors = bitor(enabledSensors,hex2dec('40000'));
                        case ('1C')
                            signalNameArray(i+1) = cellstr('GSR Raw');
                            signalDataTypeArray(i+1) = cellstr('u16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('04'));
                        case ('1D')
                            signalNameArray(i+1) = cellstr('EXG1 STA');
                            signalDataTypeArray(i+1) = cellstr('u8');
                            nBytesDataPacket=nBytesDataPacket+1;
                        case ('1E')
                            signalNameArray(i+1) = cellstr('EXG1 CH1');
                            signalDataTypeArray(i+1) = cellstr('i24*');
                            nBytesDataPacket=nBytesDataPacket+3;
                            enabledSensors = bitor(enabledSensors,hex2dec('10'));
                        case ('1F')
                            signalNameArray(i+1) = cellstr('EXG1 CH2');
                            signalDataTypeArray(i+1) = cellstr('i24*');
                            nBytesDataPacket=nBytesDataPacket+3;
                            enabledSensors = bitor(enabledSensors,hex2dec('10'));
                        case ('20')
                            signalNameArray(i+1) = cellstr('EXG2 STA');
                            signalDataTypeArray(i+1) = cellstr('u8');
                            nBytesDataPacket=nBytesDataPacket+1;
                        case ('21')
                            signalNameArray(i+1) = cellstr('EXG2 CH1'); 
                            signalDataTypeArray(i+1) = cellstr('i24*');
                            nBytesDataPacket=nBytesDataPacket+3;
                            enabledSensors = bitor(enabledSensors,hex2dec('08'));
                        case ('22')
                            signalNameArray(i+1) = cellstr('EXG2 CH2');
                            signalDataTypeArray(i+1) = cellstr('i24*');
                            nBytesDataPacket=nBytesDataPacket+3;
                            enabledSensors = bitor(enabledSensors,hex2dec('08'));
                        case ('23')
                            signalNameArray(i+1) = cellstr('EXG1 CH1 16BIT');
                            signalDataTypeArray(i+1) = cellstr('i16*');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('100000'));
                        case ('24')
                            signalNameArray(i+1) = cellstr('EXG1 CH2 16BIT');
                            signalDataTypeArray(i+1) = cellstr('i16*');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('100000'));
                        case ('25')
                            signalNameArray(i+1) = cellstr('EXG2 CH1 16BIT');
                            signalDataTypeArray(i+1) = cellstr('i16*');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('080000'));
                        case ('26')
                            signalNameArray(i+1) = cellstr('EXG2 CH2 16BIT');
                            signalDataTypeArray(i+1) = cellstr('i16*');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('080000'));
                        case ('27')
                            signalNameArray(i+1) = cellstr('Bridge Amplifier High');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('8000'));
                        case ('28')
                            signalNameArray(i+1) = cellstr('Bridge Amplifier Low');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('8000'));
                        otherwise
                            signalNameArray(i+1) = cellstr(hexSignalID);       % Default values for unrecognised data signal
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                    end
                else
                    switch hexSignalID
                        case ('0')
                            signalNameArray(i+1) = cellstr('Accelerometer X');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('80'));
                        case ('1')
                            signalNameArray(i+1) = cellstr('Accelerometer Y');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('80'));
                        case ('2')
                            signalNameArray(i+1) = cellstr('Accelerometer Z');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('80'));
                        case ('3')
                            signalNameArray(i+1) = cellstr('Gyroscope X');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('40'));
                        case ('4')
                            signalNameArray(i+1) = cellstr('Gyroscope Y');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('40'));
                        case ('5')
                            signalNameArray(i+1) = cellstr('Gyroscope Z');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('40'));
                        case ('6')
                            signalNameArray(i+1) = cellstr('Magnetometer X');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('20'));
                        case ('7')
                            signalNameArray(i+1) = cellstr('Magnetometer Y');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('20'));
                        case ('8')
                            signalNameArray(i+1) = cellstr('Magnetometer Z');
                            signalDataTypeArray(i+1) = cellstr('i16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('20'));
                        case ('9')
                            signalNameArray(i+1) = cellstr('ECG RA-LL');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('10'));
                        case ('A')
                            signalNameArray(i+1) = cellstr('ECG LA-LL');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('10'));
                        case ('B')
                            signalNameArray(i+1) = cellstr('GSR Raw');
                            signalDataTypeArray(i+1) = cellstr('u16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('04'));
                        case ('C')
                            signalNameArray(i+1) = cellstr('GSR Res');
                            signalDataTypeArray(i+1) = cellstr('u16');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('04'));
                        case ('D')
                            signalNameArray(i+1) = cellstr('EMG');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('08'));
                        case ('E')
                            signalNameArray(i+1) = cellstr('ExpBoard A0');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('01'));
                        case ('F')
                            signalNameArray(i+1) = cellstr('ExpBoard A7');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('02'));
                        case ('10')
                            signalNameArray(i+1) = cellstr('Strain Gauge High');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('8000'));
                        case ('11')
                            signalNameArray(i+1) = cellstr('Strain Gauge Low');
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;
                            enabledSensors = bitor(enabledSensors,hex2dec('8000'));
                        case ('12')
                            signalNameArray(i+1) = cellstr('Heart Rate');
                            signalDataTypeArray(i+1) = cellstr('u8');
                            nBytesDataPacket=nBytesDataPacket+1;
                            if (thisShimmer.FirmwareCompatibilityCode > 0) %only supported on BTStream
                                signalDataTypeArray(i+1) = cellstr('u16');
                                nBytesDataPacket=nBytesDataPacket+1;
                            end

                            enabledSensors = bitor(enabledSensors,hex2dec('4000'));
                        otherwise
                            signalNameArray(i+1) = cellstr(hexSignalID);       % Default values for unrecognised data signal
                            signalDataTypeArray(i+1) = cellstr('u12');
                            nBytesDataPacket=nBytesDataPacket+2;

                    end
                end  % switch signalID
                
            end
            
            thisShimmer.SignalNameArray = signalNameArray;
            thisShimmer.SignalDataTypeArray = signalDataTypeArray;
            thisShimmer.nBytesDataPacket = nBytesDataPacket;
            thisShimmer.EnabledSensors = enabledSensors;
            
        end  % function interpretdatapacketformat
                
        function [offsetVector,alignmentMatrix,sensitivityMatrix] = retrievekinematicparameters(thisShimmer,serialData)
            % Is called by getcalibrationparameters. Retrieves kinematic 
            % parameters: offset vector, alignment matrix and
            % sensitivity matrix - from input serialData.
            offsetVector(1,1)=bitor(bitshift(uint16(serialData(1)),8),uint16(serialData(2)));
            offsetVector(2,1)=bitor(bitshift(uint16(serialData(3)),8),uint16(serialData(4)));
            offsetVector(3,1)=bitor(bitshift(uint16(serialData(5)),8),uint16(serialData(6)));
            offsetVector=calculatetwoscomplement(thisShimmer,offsetVector,16);
            
            SMatrix(1)=bitor(bitshift(uint16(serialData(7)),8),uint16(serialData(8)));
            SMatrix(2)=bitor(bitshift(uint16(serialData(9)),8),uint16(serialData(10)));
            SMatrix(3)=bitor(bitshift(uint16(serialData(11)),8),uint16(serialData(12)));
            sensitivityMatrix=diag(SMatrix);
            sensitivityMatrix=calculatetwoscomplement(thisShimmer,sensitivityMatrix,16);
            
            alignmentMatrix=(calculatetwoscomplement(thisShimmer,double(reshape(serialData(13:end),3,3)),8)./100)';
            
        end % function retrievekinematicparameters
        
    end %methods (Access = 'private')
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Data Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = 'private')
        
        function [accelData,signalName,signalFormat,signalUnit] = getacceldata(thisShimmer, dataMode, parsedData)
            % Get accelerometer data from input parsedData.
            %
            % This is for backward compatability should anyone have used
            % 'Accelerometer X' as a signal name. For Shimmer3 the
            % the signal 'Accelerometer X' is decided based on the selected accel range of
            % the wide range accelerometer. 
            if (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                    iAccelXShimmer = thisShimmer.getsignalindex('Accelerometer X');            % Determine the column index of the Accelerometer X-axis signal
                    iAccelYShimmer = thisShimmer.getsignalindex('Accelerometer Y');            % Determine the column index of the Accelerometer Y-axis signal
                    iAccelZShimmer = thisShimmer.getsignalindex('Accelerometer Z');            % Determine the column index of the Accelerometer Z-axis signal
                else
                    if (thisShimmer.getaccelrange ==0 && bitand(thisShimmer.EnabledSensors, hex2dec('80'))>0)
                        iAccelXShimmer = thisShimmer.getsignalindex('Low Noise Accelerometer X');            % Determine the column index of the Accelerometer X-axis signal
                        iAccelYShimmer = thisShimmer.getsignalindex('Low Noise Accelerometer Y');            % Determine the column index of the Accelerometer Y-axis signal
                        iAccelZShimmer = thisShimmer.getsignalindex('Low Noise Accelerometer Z');            % Determine the column index of the Accelerometer Z-axis signal
                    elseif (bitand(thisShimmer.EnabledSensors, hex2dec('1000'))>0)
                        iAccelXShimmer = thisShimmer.getsignalindex('Wide Range Accelerometer X');            % Determine the column index of the Accelerometer X-axis signal
                        iAccelYShimmer = thisShimmer.getsignalindex('Wide Range Accelerometer Y');            % Determine the column index of the Accelerometer Y-axis signal
                        iAccelZShimmer = thisShimmer.getsignalindex('Wide Range Accelerometer Z');            % Determine the column index of the Accelerometer Z-axis signal
                    elseif (bitand(thisShimmer.EnabledSensors, hex2dec('80'))>0)
                        iAccelXShimmer = thisShimmer.getsignalindex('Low Noise Accelerometer X');            % Determine the column index of the Accelerometer X-axis signal
                        iAccelYShimmer = thisShimmer.getsignalindex('Low Noise Accelerometer Y');            % Determine the column index of the Accelerometer Y-axis signal
                        iAccelZShimmer = thisShimmer.getsignalindex('Low Noise Accelerometer Z');            % Determine the column index of the Accelerometer Z-axis signal
                    end
                end
                if strcmp(dataMode,'a')
                    accelUncalibratedData=double(parsedData(:,[iAccelXShimmer iAccelYShimmer iAccelZShimmer]));
                    if ((thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.getaccelrange ==0 && bitand(thisShimmer.EnabledSensors, hex2dec('80'))>0) || thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                        accelCalibratedData = thisShimmer.calibrateinertialsensordata(accelUncalibratedData,thisShimmer.AccelCalParametersAM,thisShimmer.AccelCalParametersSM,thisShimmer.AccelCalParametersOV);
                    elseif (bitand(thisShimmer.EnabledSensors, hex2dec('1000'))>0)
                        accelCalibratedData = thisShimmer.calibrateinertialsensordata(accelUncalibratedData,thisShimmer.DAccelCalParametersAM,thisShimmer.DAccelCalParametersSM,thisShimmer.DAccelCalParametersOV);
                    else
                        accelCalibratedData = thisShimmer.calibrateinertialsensordata(accelUncalibratedData,thisShimmer.AccelCalParametersAM,thisShimmer.AccelCalParametersSM,thisShimmer.AccelCalParametersOV);
                    end
                    accelData=[accelUncalibratedData accelCalibratedData];
                    signalName{1}='Accelerometer X';
                    signalName{2}='Accelerometer Y';
                    signalName{3}='Accelerometer Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    if thisShimmer.DefaultAccelCalibrationParameters==true
                        signalName{4}='Accelerometer X';
                        signalName{5}='Accelerometer Y';
                        signalName{6}='Accelerometer Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='m/(s^2) *';  % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{5}='m/(s^2) *';
                        signalUnit{6}='m/(s^2) *';
                    else
                        signalName{4}='Accelerometer X';
                        signalName{5}='Accelerometer Y';
                        signalName{6}='Accelerometer Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='m/(s^2)';
                        signalUnit{5}='m/(s^2)';
                        signalUnit{6}='m/(s^2)';
                    end
                    
                elseif strcmp(dataMode,'u')
                    accelUncalibratedData=double(parsedData(:,[iAccelXShimmer iAccelYShimmer iAccelZShimmer]));
                    accelData=accelUncalibratedData;
                    signalName{1}='Accelerometer X';
                    signalName{2}='Accelerometer Y';
                    signalName{3}='Accelerometer Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    
                elseif strcmp(dataMode,'c')
                    accelUncalibratedData=double(parsedData(:,[iAccelXShimmer iAccelYShimmer iAccelZShimmer]));
                    if ((thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.getaccelrange ==0) || thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                        accelCalibratedData = thisShimmer.calibrateinertialsensordata(accelUncalibratedData,thisShimmer.AccelCalParametersAM,thisShimmer.AccelCalParametersSM,thisShimmer.AccelCalParametersOV);
                    elseif (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.getaccelrange ~=0)
                        accelCalibratedData = thisShimmer.calibrateinertialsensordata(accelUncalibratedData,thisShimmer.DAccelCalParametersAM,thisShimmer.DAccelCalParametersSM,thisShimmer.DAccelCalParametersOV);
                    end
                    accelData=accelCalibratedData;
                    if thisShimmer.DefaultAccelCalibrationParameters==true
                        signalName{1}='Accelerometer X';
                        signalName{2}='Accelerometer Y';
                        signalName{3}='Accelerometer Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='m/(s^2) *';  % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{2}='m/(s^2) *';
                        signalUnit{3}='m/(s^2) *';
                    else
                        signalName{1}='Accelerometer X';
                        signalName{2}='Accelerometer Y';
                        signalName{3}='Accelerometer Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='m/(s^2)';
                        signalUnit{2}='m/(s^2)';
                        signalUnit{3}='m/(s^2)';
                    end
                else
                    disp('Wrong data mode specified');
                end
            else
                accelData = [];
                fprintf(strcat('Warning: getacceldata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
        end
        
        function [accelData,signalName,signalFormat,signalUnit] = getlownoiseacceldata(thisShimmer, dataMode, parsedData)
             % Get low noise accelerometer data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3) % Shimmer must be in a Shimmer3 in Streaming state
                
                iAccelXShimmer = thisShimmer.getsignalindex('Low Noise Accelerometer X');            % Determine the column index of the Accelerometer X-axis signal
                iAccelYShimmer = thisShimmer.getsignalindex('Low Noise Accelerometer Y');            % Determine the column index of the Accelerometer Y-axis signal
                iAccelZShimmer = thisShimmer.getsignalindex('Low Noise Accelerometer Z');            % Determine the column index of the Accelerometer Z-axis signal
                if strcmp(dataMode,'a')
                    accelUncalibratedData=double(parsedData(:,[iAccelXShimmer iAccelYShimmer iAccelZShimmer]));
                    accelCalibratedData = thisShimmer.calibrateinertialsensordata(accelUncalibratedData,thisShimmer.AccelCalParametersAM,thisShimmer.AccelCalParametersSM,thisShimmer.AccelCalParametersOV);
                    accelData=[accelUncalibratedData accelCalibratedData];
                    signalName{1}='Low Noise Accelerometer X';
                    signalName{2}='Low Noise Accelerometer Y';
                    signalName{3}='Low Noise Accelerometer Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    if thisShimmer.DefaultAccelCalibrationParameters==true
                        signalName{4}='Low Noise Accelerometer X';
                        signalName{5}='Low Noise Accelerometer Y';
                        signalName{6}='Low Noise Accelerometer Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='m/(s^2) *';  % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{5}='m/(s^2) *';
                        signalUnit{6}='m/(s^2) *';
                    else
                        signalName{4}='Low Noise Accelerometer X';
                        signalName{5}='Low Noise Accelerometer Y';
                        signalName{6}='Low Noise Accelerometer Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='m/(s^2)';
                        signalUnit{5}='m/(s^2)';
                        signalUnit{6}='m/(s^2)';
                    end
                    
                elseif strcmp(dataMode,'u')
                    accelUncalibratedData=double(parsedData(:,[iAccelXShimmer iAccelYShimmer iAccelZShimmer]));
                    accelData=accelUncalibratedData;
                    signalName{1}='Low Noise Accelerometer X';
                    signalName{2}='Low Noise Accelerometer Y';
                    signalName{3}='Low Noise Accelerometer Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    
                elseif strcmp(dataMode,'c')
                    accelUncalibratedData=double(parsedData(:,[iAccelXShimmer iAccelYShimmer iAccelZShimmer]));
                    accelCalibratedData = thisShimmer.calibrateinertialsensordata(accelUncalibratedData,thisShimmer.AccelCalParametersAM,thisShimmer.AccelCalParametersSM,thisShimmer.AccelCalParametersOV);
                    accelData=accelCalibratedData;
                    if thisShimmer.DefaultAccelCalibrationParameters==true
                        signalName{1}='Low Noise Accelerometer X';
                        signalName{2}='Low Noise Accelerometer Y';
                        signalName{3}='Low Noise Accelerometer Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='m/(s^2) *';  % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{2}='m/(s^2) *';
                        signalUnit{3}='m/(s^2) *';
                    else
                        signalName{1}='Low Noise Accelerometer X';
                        signalName{2}='Low Noise Accelerometer Y';
                        signalName{3}='Low Noise Accelerometer Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='m/(s^2)';
                        signalUnit{2}='m/(s^2)';
                        signalUnit{3}='m/(s^2)';
                    end
                else
                    disp('Warning: getlownoiseacceldata - Wrong data mode specified');
                end
            else
                accelData = [];
                fprintf(strcat('Warning: getlownoiseacceldata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming or not a Shimmer3'));
            end
        end
        
        function [accelData,signalName,signalFormat,signalUnit] = getwiderangeacceldata(thisShimmer, dataMode, parsedData)
             % Get wide range accelerometer data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)  % Shimmer must be in a Shimmer3 in Streaming state
                
                iAccelXShimmer = thisShimmer.getsignalindex('Wide Range Accelerometer X');            % Determine the column index of the Accelerometer X-axis signal
                iAccelYShimmer = thisShimmer.getsignalindex('Wide Range Accelerometer Y');            % Determine the column index of the Accelerometer Y-axis signal
                iAccelZShimmer = thisShimmer.getsignalindex('Wide Range Accelerometer Z');            % Determine the column index of the Accelerometer Z-axis signal
                if strcmp(dataMode,'a')
                    accelUncalibratedData=double(parsedData(:,[iAccelXShimmer iAccelYShimmer iAccelZShimmer]));
                    accelCalibratedData = thisShimmer.calibrateinertialsensordata(accelUncalibratedData,thisShimmer.DAccelCalParametersAM,thisShimmer.DAccelCalParametersSM,thisShimmer.DAccelCalParametersOV);
                    accelData=[accelUncalibratedData accelCalibratedData];
                    signalName{1}='Wide Range Accelerometer X';
                    signalName{2}='Wide Range Accelerometer Y';
                    signalName{3}='Wide Range Accelerometer Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    if thisShimmer.DefaultDAccelCalibrationParameters==true
                        signalName{4}='Wide Range Accelerometer X';
                        signalName{5}='Wide Range Accelerometer Y';
                        signalName{6}='Wide Range Accelerometer Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='m/(s^2) *';  % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{5}='m/(s^2) *';
                        signalUnit{6}='m/(s^2) *';
                    else
                        signalName{4}='Wide Range Accelerometer X';
                        signalName{5}='Wide Range Accelerometer Y';
                        signalName{6}='Wide Range Accelerometer Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='m/(s^2)';
                        signalUnit{5}='m/(s^2)';
                        signalUnit{6}='m/(s^2)';
                    end
                    
                elseif strcmp(dataMode,'u')
                    accelUncalibratedData=double(parsedData(:,[iAccelXShimmer iAccelYShimmer iAccelZShimmer]));
                    accelData=accelUncalibratedData;
                    signalName{1}='Wide Range Accelerometer X';
                    signalName{2}='Wide Range Accelerometer Y';
                    signalName{3}='Wide Range Accelerometer Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    
                elseif strcmp(dataMode,'c')
                    accelUncalibratedData=double(parsedData(:,[iAccelXShimmer iAccelYShimmer iAccelZShimmer]));
                    accelCalibratedData = thisShimmer.calibrateinertialsensordata(accelUncalibratedData,thisShimmer.DAccelCalParametersAM,thisShimmer.DAccelCalParametersSM,thisShimmer.DAccelCalParametersOV);
                    accelData=accelCalibratedData;
                    if thisShimmer.DefaultDAccelCalibrationParameters==true
                        signalName{1}='Wide Range Accelerometer X';
                        signalName{2}='Wide Range Accelerometer Y';
                        signalName{3}='Wide Range Accelerometer Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='m/(s^2) *';  % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{2}='m/(s^2) *';
                        signalUnit{3}='m/(s^2) *';
                    else
                        signalName{1}='Wide Range Accelerometer X';
                        signalName{2}='Wide Range Accelerometer Y';
                        signalName{3}='Wide Range Accelerometer Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='m/(s^2)';
                        signalUnit{2}='m/(s^2)';
                        signalUnit{3}='m/(s^2)';
                    end
                else
                    disp('Warning: getwiderangeacceldata - Wrong data mode specified');
                end
            else
                accelData = [];
                fprintf(strcat('Warning: getwiderangeacceldata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming or not a Shimmer3'));
            end
        end
        
        function [magData,signalName,signalFormat,signalUnit] = getmagdata(thisShimmer, dataMode, parsedData)
             % Get magnetometer data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                iMagXShimmer = thisShimmer.getsignalindex('Magnetometer X');            % Determine the column index of the Magnetometer X-axis signal
                iMagYShimmer = thisShimmer.getsignalindex('Magnetometer Y');            % Determine the column index of the Magnetometer Y-axis signal
                iMagZShimmer = thisShimmer.getsignalindex('Magnetometer Z');            % Determine the column index of the Magnetometer Z-axis signal
                
                if strcmp(dataMode,'a')
                    magUncalibratedData=double(parsedData(:,[iMagXShimmer iMagYShimmer iMagZShimmer]));
                    magCalibratedData = thisShimmer.calibrateinertialsensordata(magUncalibratedData,thisShimmer.MagneCalParametersAM,thisShimmer.MagneCalParametersSM,thisShimmer.MagneCalParametersOV);
                    magData=[magUncalibratedData magCalibratedData];
                    signalName{1}='Magnetometer X';
                    signalName{2}='Magnetometer Y';
                    signalName{3}='Magnetometer Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    if thisShimmer.DefaultMagneCalibrationParameters==true
                        signalName{4}='Magnetometer X';
                        signalName{5}='Magnetometer Y';
                        signalName{6}='Magnetometer Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='local flux *'; % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{5}='local flux *';
                        signalUnit{6}='local flux *';
                    else
                        signalName{4}='Magnetometer X';
                        signalName{5}='Magnetometer Y';
                        signalName{6}='Magnetometer Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='local flux';
                        signalUnit{5}='local flux';
                        signalUnit{6}='local flux';
                    end
                    
                elseif strcmp(dataMode,'u')
                    magUncalibratedData=double(parsedData(:,[iMagXShimmer iMagYShimmer iMagZShimmer]));
                    magData=magUncalibratedData;
                    signalName{1}='Magnetometer X';
                    signalName{2}='Magnetometer Y';
                    signalName{3}='Magnetometer Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    
                elseif strcmp(dataMode,'c')
                    if thisShimmer.DefaultMagneCalibrationParameters==true
                        signalName{1}='Magnetometer X';
                        signalName{2}='Magnetometer Y';
                        signalName{3}='Magnetometer Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='local flux *'; % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{2}='local flux *';
                        signalUnit{3}='local flux *';
                    else
                        signalName{1}='Magnetometer X';
                        signalName{2}='Magnetometer Y';
                        signalName{3}='Magnetometer Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='local flux';
                        signalUnit{2}='local flux';
                        signalUnit{3}='local flux';
                    end
                    
                    magUncalibratedData=double(parsedData(:,[iMagXShimmer iMagYShimmer iMagZShimmer]));
                    magCalibratedData = thisShimmer.calibrateinertialsensordata(magUncalibratedData,thisShimmer.MagneCalParametersAM,thisShimmer.MagneCalParametersSM,thisShimmer.MagneCalParametersOV);
                    magData=magCalibratedData;
                    
                else
                    disp('Warning: getmagdata - Wrong data mode specified');
                end
            else
                magData = [];
                fprintf(strcat('Warning: getmagdata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
        end
                        
        function [gyroData, signalName, signalFormat, signalUnit] = getgyrodata(thisShimmer, dataMode, parsedData)
             % Get gyroscope data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                iGyroXShimmer = thisShimmer.getsignalindex('Gyroscope X');            % Determine the column index of the Gyroscope X-axis signal
                iGyroYShimmer = thisShimmer.getsignalindex('Gyroscope Y');            % Determine the column index of the Gyroscope Y-axis signal
                iGyroZShimmer = thisShimmer.getsignalindex('Gyroscope Z');            % Determine the column index of the Gyroscope Z-axis signal
                
                if strcmp(dataMode,'a')
                    gyroUncalibratedData=double(parsedData(:,[iGyroXShimmer iGyroYShimmer iGyroZShimmer]));
                    gyroCalibratedData = thisShimmer.calibrateinertialsensordata(gyroUncalibratedData,thisShimmer.GyroCalParametersAM,thisShimmer.GyroCalParametersSM,thisShimmer.GyroCalParametersOV);
                    gyroData=[gyroUncalibratedData gyroCalibratedData];
                    signalName{1}='Gyroscope X';
                    signalName{2}='Gyroscope Y';
                    signalName{3}='Gyroscope Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    if thisShimmer.DefaultGyroCalibrationParameters==true
                        signalName{4}='Gyroscope X';
                        signalName{5}='Gyroscope Y';
                        signalName{6}='Gyroscope Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='degrees/s *'; % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{5}='degrees/s *';
                        signalUnit{6}='degrees/s *';
                    else
                        signalName{4}='Gyroscope X';
                        signalName{5}='Gyroscope Y';
                        signalName{6}='Gyroscope Z';
                        signalFormat{4}='CAL';
                        signalFormat{5}='CAL';
                        signalFormat{6}='CAL';
                        signalUnit{4}='degrees/s'; % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{5}='degrees/s';
                        signalUnit{6}='degrees/s';
                    end
                elseif strcmp(dataMode,'u')
                    gyroUncalibratedData=double(parsedData(:,[iGyroXShimmer iGyroYShimmer iGyroZShimmer]));
                    gyroData=gyroUncalibratedData;
                    signalName{1}='Gyroscope X';
                    signalName{2}='Gyroscope Y';
                    signalName{3}='Gyroscope Z';
                    signalFormat{1}='RAW';
                    signalFormat{2}='RAW';
                    signalFormat{3}='RAW';
                    signalUnit{1}='no units';
                    signalUnit{2}='no units';
                    signalUnit{3}='no units';
                    
                elseif strcmp(dataMode,'c')
                    gyroUncalibratedData=double(parsedData(:,[iGyroXShimmer iGyroYShimmer iGyroZShimmer]));
                    gyroCalibratedData = thisShimmer.calibrateinertialsensordata(gyroUncalibratedData,thisShimmer.GyroCalParametersAM,thisShimmer.GyroCalParametersSM,thisShimmer.GyroCalParametersOV);
                    gyroData=gyroCalibratedData;
                    if thisShimmer.DefaultGyroCalibrationParameters==true
                        signalName{1}='Gyroscope X';
                        signalName{2}='Gyroscope Y';
                        signalName{3}='Gyroscope Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='degrees/s *'; % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{2}='degrees/s *';
                        signalUnit{3}='degrees/s *';
                    else
                        signalName{1}='Gyroscope X';
                        signalName{2}='Gyroscope Y';
                        signalName{3}='Gyroscope Z';
                        signalFormat{1}='CAL';
                        signalFormat{2}='CAL';
                        signalFormat{3}='CAL';
                        signalUnit{1}='degrees/s'; % *indicates that default calibration parameters were used to calibrate the sensor data
                        signalUnit{2}='degrees/s';
                        signalUnit{3}='degrees/s';
                    end
                else
                    disp('Warning: getgyrodata - Wrong data mode specified');
                end
            else
                gyroData = [];
                fprintf(strcat('Warning: getgyrodata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
        end
        
        function [quaternionData,signalName,signalFormat,signalUnit] = getquaterniondata(thisShimmer, dataMode, accelCalibratedData, gyroCalibratedData, magCalibratedData)
            % Get quaternion data from calibrated accelerometer, gyroscope and magnetometer data.
            if (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                if strcmp(dataMode, 'c')
                    
                    quaternionData = thisShimmer.updatequaternion(accelCalibratedData, gyroCalibratedData, magCalibratedData);
                    
                    signalName{1}='Quaternion 0';
                    signalName{2}='Quaternion 1';
                    signalName{3}='Quaternion 2';
                    signalName{4}='Quaternion 3';
                    signalFormat{1}='CAL';
                    signalFormat{2}='CAL';
                    signalFormat{3}='CAL';
                    signalFormat{4}='CAL';
                    signalUnit{1}='normalised quaternion';
                    signalUnit{2}='normalised quaternion';
                    signalUnit{3}='normalised quaternion';
                    signalUnit{4}='normalised quaternion';
                else
                    disp('Warning: getquaterniondata - Wrong data mode specified');
                end
            else
                quaternionData = [];
                fprintf(strcat('Warning: getquaterniondata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
        end % function getquaterniondata
        
        function [ecgData,signalName,signalFormat,signalUnit] = getecgdata(thisShimmer, dataMode, parsedData)
            % Gets ECG data from input parsedData for Shimmer2/2r.
            if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                if (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state

                    iECGRAShimmer = thisShimmer.getsignalindex('ECG RA-LL');            % Determine the column index of the ECG RA-LL signal
                    iECGLAShimmer = thisShimmer.getsignalindex('ECG LA-LL');            % Determine the column index of the ECG LA-LL signal

                    if strcmp(dataMode,'a')
                        ecgUncalibratedData=double(parsedData(:,[iECGRAShimmer iECGLAShimmer]));
                        ecgCalibratedData(:,1) = thisShimmer.calibrateu12ADCValue(ecgUncalibratedData(:,1),thisShimmer.ECGRALLOffset,3,thisShimmer.ECGRALLGain);
                        ecgCalibratedData(:,2) = thisShimmer.calibrateu12ADCValue(ecgUncalibratedData(:,2),thisShimmer.ECGLALLOffset,3,thisShimmer.ECGLALLGain);
                        ecgData=[ecgUncalibratedData ecgCalibratedData];
                        signalName{1}='ECG RA-LL';
                        signalFormat{1}='RAW';
                        signalUnit{1}='no units';
                        signalName{2}='ECG LA-LL';
                        signalFormat{2}='RAW';
                        signalUnit{2}='no units';
                        signalName{3}='ECG RA-LL';
                        signalFormat{3}='CAL';
                        signalName{4}='ECG LA-LL';
                        signalFormat{4}='CAL';

                        if (thisShimmer.DefaultECGCalibrationParameters==true)
                            signalUnit{3}='millivolts *';
                            signalUnit{4}='millivolts *';
                        else
                            signalUnit{3}='millivolts';
                            signalUnit{4}='millivolts';
                        end

                    elseif strcmp(dataMode,'u')
                        ecgUncalibratedData=double(parsedData(:,[iECGRAShimmer iECGLAShimmer]));
                        ecgData=ecgUncalibratedData;
                        signalName{1}='ECG RA-LL';
                        signalFormat{1}='RAW';
                        signalUnit{1}='no units';
                        signalName{2}='ECG LA-LL';
                        signalFormat{2}='RAW';
                        signalUnit{2}='no units';

                    elseif strcmp(dataMode,'c')
                        ecgUncalibratedData=double(parsedData(:,[iECGRAShimmer iECGLAShimmer]));
                        ecgCalibratedData(:,1) = thisShimmer.calibrateu12ADCValue(ecgUncalibratedData(:,1),thisShimmer.ECGRALLOffset,3,thisShimmer.ECGRALLGain);
                        ecgCalibratedData(:,2) = thisShimmer.calibrateu12ADCValue(ecgUncalibratedData(:,2),thisShimmer.ECGLALLOffset,3,thisShimmer.ECGLALLGain);
                        ecgData=[ecgCalibratedData];
                        signalName{1}='ECG RA-LL';
                        signalFormat{1}='CAL';
                        signalName{2}='ECG LA-LL';
                        signalFormat{2}='CAL';
                        if (thisShimmer.DefaultECGCalibrationParameters==true)
                            signalUnit{1}='millivolts *';
                            signalUnit{2}='millivolts *';
                        else
                            signalUnit{1}='millivolts';
                            signalUnit{2}='millivolts';
                        end

                    else
                        disp('Wrong data mode specified');
                    end
                else
                    ecgData = [];
                    signalName = [];
                    signalFormat = [];
                    signalUnit = [];
                    fprintf(strcat('Warning: getecgdata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
                end
            else % Shimmer3
                disp('Warning: getecgdata - getecgdata() is not a valid method for Shimmer3. Please use getexgdata().')
            end
        end
        
        function [emgData, signalName, signalFormat, signalUnit] = getemgdata(thisShimmer, dataMode, parsedData)
            % Gets EMG data from input parsedData for Shimmer2/2r.
            if (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                if ( thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                
                    iEMGShimmer = thisShimmer.getsignalindex('EMG');            % Determine the column index of the EMG signal

                    if strcmp(dataMode,'a')
                        emgUncalibratedData=double(parsedData(:,iEMGShimmer));
                        [emgCalibratedData] = thisShimmer.calibrateu12ADCValue(emgUncalibratedData,thisShimmer.EMGOffset,3,thisShimmer.EMGGain);
                        emgData=[emgUncalibratedData emgCalibratedData];
                        signalName{1}='EMG';
                        signalFormat{1}='RAW';
                        signalUnit{1}='no units';
                        signalName{2}='EMG';
                        signalFormat{2}='CAL';
                        if (thisShimmer.DefaultEMGCalibrationParameters==true)
                            signalUnit{2}='millivolts *';
                        else
                            signalUnit{2}='millivolts';
                        end

                    elseif strcmp(dataMode,'u')
                        emgUncalibratedData=double(parsedData(:,iEMGShimmer));
                        emgData=emgUncalibratedData;
                        signalName{1}='EMG';
                        signalFormat{1}='RAW';
                        signalUnit{1}='no units';
                    elseif strcmp(dataMode,'c')
                        emgUncalibratedData=double(parsedData(:,iEMGShimmer));
                        [emgCalibratedData] = thisShimmer.calibrateu12ADCValue(emgUncalibratedData,2060,3,750);
                        emgData=emgCalibratedData;
                        signalName{1}='EMG';
                        signalFormat{1}='CAL';
                        if (thisShimmer.DefaultEMGCalibrationParameters==true)
                            signalUnit{1}='millivolts *';
                        else
                            signalUnit{1}='millivolts';
                        end
                    else
                        disp('Wrong data mode specified');
                    end
                else
                    emgData = [];
                    fprintf(strcat('Warning: getemgdata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
                end
            else % Shimmer3
                disp('Warning: getemgdata - getemgdata() is not a valid method for Shimmer3. Please use getexgdata().')
            end
        end
        
        function [exgData,signalName,signalFormat,signalUnit] = getexgdata(thisShimmer, dataMode, parsedData)
            % Gets ExG data from input parsedData for Shimmer3.
            if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                disp('Warning: getexgdata - getexgdata() is not valid for Shimmer2/2r; please use getecgdata() or getemgdata().');
            elseif (thisShimmer.FirmwareCompatibilityCode < 3)
                disp('Warning: getexgdata - getexgdata() is not supported by FW versions earlier than BtStream v0.3; please update the FW.');
            else
                if ~(strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                    exgData = [];
                    signalName = [];
                    signalFormat = [];
                    signalUnit = [];
                    fprintf(strcat('Warning: getexgdata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming.'));
                else
                    if ~(bitand(thisShimmer.EnabledSensors, hex2dec('08')) || ...
                            bitand(thisShimmer.EnabledSensors, hex2dec('10')) || ... 
                            bitand(thisShimmer.EnabledSensors, hex2dec('080000')) || ...
                            bitand(thisShimmer.EnabledSensors, hex2dec('100000')))
                        exgData = [];
                        signalName = [];
                        signalFormat = [];
                        signalUnit = [];
                        fprintf(strcat('Warning: getexgdata - ExG sensors are not enabled.'));
                    else
%                         if (thisShimmer.EXG1Ch1Set == 64 && thisShimmer.EXG1Ch2Set == 64 && thisShimmer.EXG2Ch1Set == 64 && thisShimmer.EXG2Ch2Set == 71) % ECG: default parameters 
                        if (thisShimmer.ECGflag) % ECG: default parameters 
                            defaultECG = true;
                            defaultEMG = false;
%                         elseif (thisShimmer.EXG1Ch1Set == 105 && thisShimmer.EXG1Ch2Set == 96 && thisShimmer.EXG2Ch1Set == 129 && thisShimmer.EXG2Ch2Set == 129) % EMG: default parameters   
                        elseif (thisShimmer.EMGflag) % EMG: default parameters   
                            defaultECG = false;
                            defaultEMG = true;
                        else
                            defaultECG = false;
                            defaultEMG = false;
                        end
              
                        % find which signals are available and set signal
                        % info accordingly 
                        i = 1; % raw
                        j = 1; % calibrated
                        
                        if ~isempty(thisShimmer.getsignalindex('EXG1 STA'))
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG1 STA');
                            uncalSignalName{i}='EXG1 STA';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            
                            if (thisShimmer.EXGLeadOffDetectionMode==1)    % For calibrated data: output parsed lead-off status bits
                                calIndices(j) = thisShimmer.getsignalindex('EXG1 STA');
                                if defaultECG
                                    calIndices(j+1:j+3)= calIndices(j);
                                    calSignalName{j}='Lead-off ECG LL';
                                    calSignalName{j+1}='Lead-off ECG LA';
                                    calSignalName{j+2}='Lead-off ECG RA';
                                    calSignalName{j+3}='Lead-off ECG RLD';
                                    for k=0:3
                                        calSignalFormat{j+k} = 'RAW';
                                    end
                                    for k=0:3
                                        calSignalUnit{j+k}= 'no units';
                                    end
                                    gainArray(j:j+3) = 0;
                                    res16bitArray(j:j+3) = false;
                                    statusArray(j) = 1;
                                    statusArray(j+1) = 4;
                                    statusArray(j+2) = 8;
                                    statusArray(j+3) = 16;
                                    j = j + 4;
                                elseif defaultEMG
                                    calIndices(j+1:j+3)= calIndices(j);
                                    calSignalName{j}='Lead-off EMG CH1-Neg';
                                    calSignalName{j+1}='Lead-off EMG CH2-Pos';
                                    calSignalName{j+2}='Lead-off EMG CH2-Neg';
                                    calSignalName{j+3}='Lead-off EMG Ref';
                                    for k=0:3
                                        calSignalFormat{j+k} = 'RAW';
                                    end
                                    for k=0:3
                                        calSignalUnit{j+k}= 'no units';
                                    end
                                    gainArray(j:j+3) = 0;
                                    res16bitArray(j:j+3) = false;
                                    statusArray(j) = 1;
                                    statusArray(j+1) = 4;
                                    statusArray(j+2) = 8;
                                    statusArray(j+3) = 16;
                                    j= j + 4;
                                else
                                    calSignalName{j}='EXG1 STA';
                                    calSignalFormat{j}='RAW';
                                    calSignalUnit{j}='no units';
                                    gainArray(j) = 0;
                                    res16bitArray(j) = false;
                                    statusArray(j)=true;
                                    j = j + 1;
                                end
                            end
                        end
                        if (bitand(thisShimmer.EnabledSensors, hex2dec('10')))
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG1 CH1');            
                            uncalSignalName{i}='EXG1 CH1';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            calIndices(j) = thisShimmer.getsignalindex('EXG1 CH1');
                            gainArray(j) = thisShimmer.EXG1CH1Gain;
                            res16bitArray(j) = false;
                            statusArray(j)=false;
                            if defaultECG
                                calSignalName{j}='ECG LL-RA';
                            elseif defaultEMG
                                calSignalName{j}='EMG CH1';
                            else
                                calSignalName{j}='EXG1 CH1';
                            end
                            calSignalFormat{j}='CAL';
                            calSignalUnit{j}='millivolts *';
                            j = j + 1;
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG1 CH2');  
                            uncalSignalName{i}='EXG1 CH2';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            calIndices(j) = thisShimmer.getsignalindex('EXG1 CH2'); 
                            gainArray(j) = thisShimmer.EXG1CH2Gain;
                            res16bitArray(j) = false;
                            statusArray(j)=false;
                            if defaultECG
                                calSignalName{j}='ECG LA-RA';
                            elseif defaultEMG
                                calSignalName{j}='EMG CH2';
                            else
                                calSignalName{j}='EXG1 CH2';
                            end
                            calSignalFormat{j}='CAL';
                            calSignalUnit{j}='millivolts *';
                            j = j + 1;
                        elseif(bitand(thisShimmer.EnabledSensors, hex2dec('100000')))
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG1 CH1 16BIT');             
                            uncalSignalName{i}='EXG1 CH1 16BIT';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            calIndices(j) = thisShimmer.getsignalindex('EXG1 CH1 16BIT');            
                            gainArray(j) = thisShimmer.EXG1CH1Gain*2;
                            res16bitArray(j) = true;
                            statusArray(j)=false;
                            if defaultECG
                                calSignalName{j}='ECG LL-RA';
                            elseif defaultEMG
                                calSignalName{j}='EMG CH1';
                            else
                                calSignalName{j}='EXG1 CH1';
                            end
                            calSignalFormat{j}='CAL';
                            calSignalUnit{j}='millivolts *';
                            j = j + 1;
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG1 CH2 16BIT');  
                            uncalSignalName{i}='EXG1 CH2 16BIT';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            calIndices(j) = thisShimmer.getsignalindex('EXG1 CH2 16BIT'); 
                            gainArray(j) = thisShimmer.EXG1CH2Gain*2;
                            res16bitArray(j) = true;
                            statusArray(j)=false;
                            if defaultECG
                                calSignalName{j}='ECG LA-RA';
                            elseif defaultEMG
                                calSignalName{j}='EMG CH2';
                            else
                                calSignalName{j}='EXG1 CH2';
                            end
                            calSignalFormat{j}='CAL';
                            calSignalUnit{j}='millivolts *';
                            j = j + 1;
                        end
                        if ~isempty(thisShimmer.getsignalindex('EXG2 STA'))
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG2 STA');
                            uncalSignalName{i}='EXG2 STA';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            
                            if (thisShimmer.EXGLeadOffDetectionMode==1)    % For calibrated data: output parsed lead-off status bits
                                calIndices(j) = thisShimmer.getsignalindex('EXG2 STA');
                                if defaultECG
                                    calSignalName{j}='Lead-off ECG Vx';
                                    calSignalFormat{j}='RAW';
                                    calSignalUnit{j}='no units';
                                    gainArray(j) = 0;
                                    res16bitArray(j) = false;
                                    statusArray(j) = 4;
                                    j = j + 1;
                                elseif defaultEMG
                                    calSignalName{j}='Lead-off EMG CH1-Pos';
                                    calSignalFormat{j}='RAW';
                                    calSignalUnit{j}='no units';
                                    gainArray(j) = 0;
                                    res16bitArray(j) = false;
                                    statusArray(j) = 4;
                                    j = j + 1;
                                else
                                    calSignalName{j}='EXG2 STA';
                                    calSignalFormat{j}='RAW';
                                    calSignalUnit{j}='no units';
                                    gainArray(j) = 0;
                                    res16bitArray(j) = false;
                                    statusArray(j)=true;
                                    j = j + 1;
                                end
                            end
                        end
                        if (bitand(thisShimmer.EnabledSensors, hex2dec('08'))&&~thisShimmer.EMGflag)
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG2 CH1');            
                            uncalSignalName{i}='EXG2 CH1';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            calIndices(j) = thisShimmer.getsignalindex('EXG2 CH1');            
                            gainArray(j) = thisShimmer.EXG2CH1Gain;
                            res16bitArray(j) = false;
                            statusArray(j)=false;
                            if defaultECG
                                calSignalName{j}='ECG RESP';
                            else
                                calSignalName{j}='EXG2 CH1';
                            end
                            calSignalFormat{j}='CAL';
                            calSignalUnit{j}='millivolts *';
                            j = j + 1;
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG2 CH2');  
                            uncalSignalName{i}='EXG2 CH2';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            calIndices(j) = thisShimmer.getsignalindex('EXG2 CH2'); 
                            gainArray(j) = thisShimmer.EXG2CH2Gain;
                            res16bitArray(j) = false;
                            statusArray(j)=false;
                            if defaultECG
                                calSignalName{j}='ECG Vx-RL';
                            else
                                calSignalName{j}='EXG2 CH2';
                            end
                            calSignalFormat{j}='CAL';
                            calSignalUnit{j}='millivolts *';
                            j = j + 1;
                        elseif(bitand(thisShimmer.EnabledSensors, hex2dec('080000'))&&~thisShimmer.EMGflag)
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG2 CH1 16BIT');            
                            uncalSignalName{i}='EXG2 CH1 16BIT';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            calIndices(j) = thisShimmer.getsignalindex('EXG2 CH1 16BIT');            
                            gainArray(j) = thisShimmer.EXG2CH1Gain*2;
                            res16bitArray(j) = true;
                            statusArray(j)=false;
                            if defaultECG
                                calSignalName{j}='ECG RESP';
                            else
                                calSignalName{j}='EXG2 CH1';
                            end
                            calSignalFormat{j}='CAL';
                            calSignalUnit{j}='millivolts *';
                            j = j + 1;
                            uncalIndices(i) = thisShimmer.getsignalindex('EXG2 CH2 16BIT');  
                            uncalSignalName{i}='EXG2 CH2 16BIT';
                            uncalSignalFormat{i}='RAW';
                            uncalSignalUnit{i}='no units';
                            i = i + 1;
                            calIndices(j) = thisShimmer.getsignalindex('EXG2 CH2 16BIT'); 
                            gainArray(j) = thisShimmer.EXG2CH2Gain*2;
                            res16bitArray(j) = true;
                            statusArray(j)=false;
                            if defaultECG
                                calSignalName{j}='ECG Vx-RL';
                            else
                                calSignalName{j}='EXG2 CH2';
                            end
                            calSignalFormat{j}='CAL';
                            calSignalUnit{j}='millivolts *';
                            j = j + 1;
                        end

                        % create uncalibrated data array
                        numUncalSignals = length(uncalIndices);
                        exgUncalibratedData=double(parsedData(:,uncalIndices));

                        % create calibrated data array
                        numCalSignals = length(calIndices);
                        for j = 1:numCalSignals
                            if (statusArray(j)>=1)
                                exgCalibratedData(:,j)=bitand(parsedData(:,calIndices(j)),statusArray(j))/statusArray(j); % extract lead-off detection bits 
                            elseif (~res16bitArray(j))
                                exgCalibratedData(:,j)=thisShimmer.calibratei24ADCValue(double(parsedData(:,calIndices(j))),0,2.42,gainArray(j)); % calibrate the data
                            else
                                exgCalibratedData(:,j)=thisShimmer.calibratei16ADCValue(double(parsedData(:,calIndices(j))),0,2.42,gainArray(j)); % calibrate the data
                            end                                
                        end

                        % collate signal information
                        if strcmp(dataMode,'a')
                            exgData=[exgUncalibratedData exgCalibratedData];
                        elseif strcmp(dataMode,'u')
                            exgData=exgUncalibratedData;
                            numCalSignals = 0;
                        elseif strcmp(dataMode,'c')
                            exgData=exgCalibratedData;
                            numUncalSignals = 0;
                        end
                                
                        for i = 1:numUncalSignals
                            signalName{i} = uncalSignalName{i};
                            signalFormat{i} = uncalSignalFormat{i};
                            signalUnit{i} = uncalSignalUnit{i};
                        end
                        for i = 1:numCalSignals
                            signalName{i + numUncalSignals} = calSignalName{i};
                            signalFormat{i + numUncalSignals} = calSignalFormat{i};
                            signalUnit{i + numUncalSignals} = calSignalUnit{i};
                        end
                    end
                end
            end
        end
                
        function [adcData,signalName,signalFormat,signalUnit] = getadcdata(thisShimmer, dataMode,parsedData)
            % Gets AD-converter data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && ~thisShimmer.GetADCFlag)                   % Shimmer must be in a Streaming state. Only supported for Shimmer3
                iExtAdcA7 = thisShimmer.getsignalindex('External ADC A7');
                iExtAdcA6 = thisShimmer.getsignalindex('External ADC A6');
                iExtAdcA15 = thisShimmer.getsignalindex('External ADC A15');
                iIntAdcA1 = thisShimmer.getsignalindex('Internal ADC A1');
                iIntAdcA12 = thisShimmer.getsignalindex('Internal ADC A12');
                iIntAdcA13 = thisShimmer.getsignalindex('Internal ADC A13');
                iIntAdcA14 = thisShimmer.getsignalindex('Internal ADC A14');
                iAll = [iExtAdcA7 iExtAdcA6 iExtAdcA15 iIntAdcA1 iIntAdcA12 iIntAdcA13 iIntAdcA14];
                uncalibratedData = double(parsedData(:,iAll));
                calibratedData = thisShimmer.calibrateu12ADCValue(uncalibratedData,0,3,1);
                tempi = 1;
                if strcmp(dataMode,'a')
                    adcData = [uncalibratedData calibratedData];
                    if (~isempty(iExtAdcA7))
                        signalName{tempi}='External ADC A7';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iExtAdcA6))
                        signalName{tempi}='External ADC A6';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iExtAdcA15))
                        signalName{tempi}='External ADC A15';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA1))
                        signalName{tempi}='Internal ADC A1';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA12))
                        signalName{tempi}='Internal ADC A12';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA13))
                        signalName{tempi}='Internal ADC A13';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA14))
                        signalName{tempi}='Internal ADC A14';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iExtAdcA7))
                        signalName{tempi}='External ADC A7';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iExtAdcA6))
                        signalName{tempi}='External ADC A6';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iExtAdcA15))
                        signalName{tempi}='External ADC A15';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA1))
                        signalName{tempi}='Internal ADC A1';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA12))
                        signalName{tempi}='Internal ADC A12';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA13))
                        signalName{tempi}='Internal ADC A13';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA14))
                        signalName{tempi}='Internal ADC A14';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                elseif strcmp(dataMode,'u')
                    adcData = uncalibratedData;
                     if (~isempty(iExtAdcA7))
                        signalName{tempi}='External ADC A7';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iExtAdcA6))
                        signalName{tempi}='External ADC A6';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iExtAdcA15))
                        signalName{tempi}='External ADC A15';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA1))
                        signalName{tempi}='Internal ADC A1';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA12))
                        signalName{tempi}='Internal ADC A12';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA13))
                        signalName{tempi}='Internal ADC A13';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA14))
                        signalName{tempi}='Internal ADC A14';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                elseif strcmp(dataMode,'c')
                    adcData = calibratedData;
                    if (~isempty(iExtAdcA7))
                        signalName{tempi}='External ADC A7';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iExtAdcA6))
                        signalName{tempi}='External ADC A6';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iExtAdcA15))
                        signalName{tempi}='External ADC A15';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA1))
                        signalName{tempi}='Internal ADC A1';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA12))
                        signalName{tempi}='Internal ADC A12';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA13))
                        signalName{tempi}='Internal ADC A13';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    if (~isempty(iIntAdcA14))
                        signalName{tempi}='Internal ADC A14';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                else
                    disp('Warning: getadcdata - Wrong data mode specified');
                end
            else
                adcData = [];
                signalName = [];
                signalFormat = [];
                signalUnit = [];
                if ~thisShimmer.GetADCFlag         % Do not show warnings if getadcdata flag is set.
                    fprintf(strcat('Warning: getadcdata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming\n'));
                    fprintf('or Shimmer is not Shimmer3. Currently only supported for Shimmer3.\n');
                end
            end
        end
                
        function [expBoardData,signalName,signalFormat,signalUnit] = getexpboarddata(thisShimmer, dataMode,parsedData)
            % Get External Expansion Board data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming'))                   % Shimmer must be in a Streaming state
               
                
                if (bitand(thisShimmer.EnabledSensors,1)==1 && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)        % SENSOR_ExpBoard_A0 = 0x0001
                    iExpBA0Shimmer = thisShimmer.getsignalindex('ExpBoard A0');    % Determine the column index of the ExpBoard_A0 signal
                elseif (bitand(thisShimmer.EnabledSensors,1)==1 && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)   % SENSOR_ExpBoard_A6 = 0x0001
                    iExpBA6Shimmer = thisShimmer.getsignalindex('External ADC A6');    % Determine the column index of the ExpBoard_A6 signal, External ADC A6 on Shimmer3
                else
                    iExpBA0Shimmer = [];
                    iExpBA6Shimmer = [];
                end
                
                if (bitand(thisShimmer.EnabledSensors,2)==2 && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)                % SENSOR_ExpBoard_A7 = 0x0002
                    iExpBA7Shimmer = thisShimmer.getsignalindex('ExpBoard A7');    % Determine the column index of the ExpBoard_A7 signal
                elseif (bitand(thisShimmer.EnabledSensors,2)==2 && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    iExpBA7Shimmer = thisShimmer.getsignalindex('External ADC A7'); % Determine the column index of the ExpBoard_A7 signal, External ADC A7 on Shimmer3
                else
                    iExpBA7Shimmer = [];
                end
                tempi=1;
                if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                    iExpBShimmer=[iExpBA0Shimmer iExpBA7Shimmer];
                else
                    iExpBShimmer=[iExpBA6Shimmer iExpBA7Shimmer];
                end
                if strcmp(dataMode,'a')
                    expBoardUncalibratedData=double(parsedData(:,iExpBShimmer));
                    [expBoardCalibratedData] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData,0,3,1);
                    expBoardData=[expBoardUncalibratedData expBoardCalibratedData];
                    
                    if (bitand(thisShimmer.EnabledSensors,1)==1 && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        signalName{tempi}='ExpBoard A0'; % Shimmer2/2r
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    else
                        signalName{tempi}='ExpBoard A6'; % Shimmer3
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    if bitand(thisShimmer.EnabledSensors,2)==2 
                        signalName{tempi}='ExpBoard A7';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if (bitand(thisShimmer.EnabledSensors,1)==1 && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        signalName{tempi}='ExpBoard A0'; % Shimmer2/2r
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    else
                        signalName{tempi}='ExpBoard A6'; % Shimmer3
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    if bitand(thisShimmer.EnabledSensors,2)==2
                        signalName{tempi}='ExpBoard A7';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    
                elseif strcmp(dataMode,'u')
                    expBoardUncalibratedData=double(parsedData(:,iExpBShimmer));
                    expBoardData=expBoardUncalibratedData;
                    tempi=1;
                    if (bitand(thisShimmer.EnabledSensors,1)==1 && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        signalName{tempi}='ExpBoard A0';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    else
                        signalName{tempi}='ExpBoard A6'; % Shimmer3
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    if bitand(thisShimmer.EnabledSensors,2)==2
                        signalName{tempi}='ExpBoard A7';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                elseif strcmp(dataMode,'c')
                    expBoardUncalibratedData=double(parsedData(:,iExpBShimmer));
                    [expBoardCalibratedData] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData,0,3,1);
                    expBoardData=expBoardCalibratedData;
                    tempi=1;
                    if (bitand(thisShimmer.EnabledSensors,1)==1 && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                        signalName{tempi}='ExpBoard A0';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    else
                        signalName{tempi}='ExpBoard A6';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    if bitand(thisShimmer.EnabledSensors,2)==2
                        signalName{tempi}='ExpBoard A7';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                else
                    disp('Warning: getexpboarddata - Wrong data mode specified');
                end
            else
                expBoardData = [];
                fprintf(strcat('Warning: getexpboarddata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
        end
        
        function [battVoltData,signalName,signalFormat,signalUnit] = getbattvoltdata(thisShimmer, dataMode,parsedData)
            % Gets Shimmer battery voltage data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming'))                                 % Shimmer must be in a Streaming state
                
                if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                    iExpBA7Shimmer = thisShimmer.getsignalindex('Battery Voltage');    % Determine the column index of the ExpBoard_A7 signal
                    iExpBA0Shimmer = [];
                else
                    if bitand(thisShimmer.EnabledSensors,1)==1                         % SENSOR_ExpBoard_A0 = 0x0001
                        iExpBA0Shimmer = thisShimmer.getsignalindex('ExpBoard A0');    % Determine the column index of the ExpBoard_A0 signal
                    else
                        iExpBA0Shimmer = [];
                    end
                    
                    if bitand(thisShimmer.EnabledSensors,2)==2                         % SENSOR_ExpBoard_A7 = 0x0002
                        iExpBA7Shimmer = thisShimmer.getsignalindex('ExpBoard A7');    % Determine the column index of the ExpBoard_A7 signal
                    else
                        iExpBA7Shimmer = [];
                    end
                end
                tempi=1;
                iExpBShimmer=[iExpBA0Shimmer iExpBA7Shimmer];
                
                %check battery voltage and send led alert if low
                expBoardUncalibratedData=double(parsedData(:,iExpBShimmer));
                if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                    [expBoardCalibratedData(:,1)] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData(:,1),0,3,1)*1.988; % as the voltage is divided multiply it by Z1=698 ; Z2=706 ; Vout=Z2/(Z2+Z1)*Vin
                    [expBoardCalibratedData(:,2)] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData(:,2),0,3,1)*2; % as the voltage is divided multiply it by two
                    battVoltData=expBoardCalibratedData;
                    if (thisShimmer.FirmwareCompatibilityCode == 0) %only supported on BTStream
                        disp('getbattvoltdata - Command not supported for this firmware version, please update firmware.');
                    else
                        if (mean(battVoltData(:,2))<thisShimmer.BattVoltLimit*1000 && thisShimmer.BlinkLED~=1)
                            thisShimmer.setledblinkwhilestreaming(1);
                        end
                        if (mean(battVoltData(:,2))>(thisShimmer.BattVoltLimit*1000+100) && thisShimmer.BlinkLED~=0)
                            thisShimmer.setledblinkwhilestreaming(0);
                        end
                    end
                else
                    [expBoardCalibratedData(:,1)] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData(:,1),0,3,1)*2; % as the voltage is divided multiply it by two
                    battVoltData=expBoardCalibratedData;
                    if (thisShimmer.FirmwareIdentifier ~= 3 && mean(battVoltData(:,1))<thisShimmer.BattVoltLimit*1000 && thisShimmer.BlinkLED~=1)
                        thisShimmer.setledblinkwhilestreaming(1);
                    end
                    if (thisShimmer.FirmwareIdentifier ~= 3 && mean(battVoltData(:,1))>(thisShimmer.BattVoltLimit*1000+100) && thisShimmer.BlinkLED~=0)
                        thisShimmer.setledblinkwhilestreaming(0);
                    end
                end
                

                
                if strcmp(dataMode,'a')
                    expBoardUncalibratedData=double(parsedData(:,iExpBShimmer));
                     if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                         [expBoardCalibratedData(:,1)] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData(:,1),0,3,1)*1.988; % as the voltage is divided multiply it by Z1=698 ; Z2=706 ; Vout=Z2/(Z2+Z1)*Vin
                         [expBoardCalibratedData(:,2)] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData(:,2),0,3,1)*2; % as the voltage is divided multiply it by two
                     else
                         [expBoardCalibratedData(:,1)] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData(:,1),0,3,1)*2; % as the voltage is divided multiply it by two
                     end
                    battVoltData=[expBoardUncalibratedData expBoardCalibratedData];
                    
                    if ~isempty(iExpBA0Shimmer)
                        signalName{tempi}='VSenseReg';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    if ~isempty(iExpBA7Shimmer)
                        signalName{tempi}='VSenseBatt';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                    if ~isempty(iExpBA0Shimmer)
                        signalName{tempi}='VSenseReg';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    if ~isempty(iExpBA7Shimmer)
                        signalName{tempi}='VSenseBatt';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                    
                elseif strcmp(dataMode,'u')
                    expBoardUncalibratedData=double(parsedData(:,iExpBShimmer));
                    battVoltData=expBoardUncalibratedData;
                    tempi=1;
                    if ~isempty(iExpBA0Shimmer)
                        signalName{tempi}='VSenseReg';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    if ~isempty(iExpBA7Shimmer)
                        signalName{tempi}='VSenseBatt';
                        signalFormat{tempi}='RAW';
                        signalUnit{tempi}='no units';
                        tempi=tempi+1;
                    end
                    
                elseif strcmp(dataMode,'c')
                    expBoardUncalibratedData=double(parsedData(:,iExpBShimmer));
                    if (thisShimmer.ShimmerVersion ~= thisShimmer.SHIMMER_3)
                        [expBoardCalibratedData(:,1)] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData(:,1),0,3,1)*1.988; % as the voltage is divided multiply it by Z1=698 ; Z2=706 ; Vout=Z2/(Z2+Z1)*Vin
                        [expBoardCalibratedData(:,2)] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData(:,2),0,3,1)*2; % as the voltage is divided multiply it by two
                    else
                        [expBoardCalibratedData(:,1)] = thisShimmer.calibrateu12ADCValue(expBoardUncalibratedData(:,1),0,3,1)*2; % as the voltage is divided multiply it by two
                    end
                    battVoltData=expBoardCalibratedData;
                    tempi=1;
                    if ~isempty(iExpBA0Shimmer)
                        signalName{tempi}='VSenseReg';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    if ~isempty(iExpBA7Shimmer)
                        signalName{tempi}='VSenseBatt';
                        signalFormat{tempi}='CAL';
                        signalUnit{tempi}='millivolts';
                        tempi=tempi+1;
                    end
                    
                else
                    disp('Warning: getbattvoltdata - Wrong data mode specified');
                end
            else
                battVoltData = [];
                fprintf(strcat('Warning: getbattvoltdata - Cannot get battery voltage data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
        end
                            
        function [pressureData, signalName, signalFormat, signalUnit] = getpressuredata(thisShimmer, dataMode, parsedData)
            % Gets Shimmer3 BMP180/BMP280 - Pressure sensor data from input parsedData.
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 2 )
                disp('Warning: getpressuredata - Command not supported for this firmware version, please update firmware.')
            elseif (strcmp(thisShimmer.State,'Streaming') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3) % Shimmer must be in a Streaming state
                iPressure = thisShimmer.getsignalindex('Pressure');                                               % Determine the column index of the BMP180/BMP280 Pressure signal
                iTemperature = thisShimmer.getsignalindex('Temperature');                                         % Determine the column index of the BMP180/BMP280 Temperature signal
                pressureUncalibratedData=double(parsedData(:,iPressure));
                temperatureUncalibratedData=double(parsedData(:,iTemperature));
                if thisShimmer.HardwareCompatibilityCode < 2
                    pressureUncalibratedData=bitshift(pressureUncalibratedData,-(8-thisShimmer.PressureResolution));
                elseif thisShimmer.HardwareCompatibilityCode == 2
                    temperatureUncalibratedData = bitshift(temperatureUncalibratedData,4);
                    pressureUncalibratedData = bitshift(pressureUncalibratedData,-4);
                end
                if strcmp(dataMode,'a')
                    [pressureCalibratedData] = thisShimmer.calibratepressuredata(pressureUncalibratedData,temperatureUncalibratedData);
                    pressureData=[pressureUncalibratedData temperatureUncalibratedData pressureCalibratedData];
                    signalName{1}='Pressure';
                    signalFormat{1}='RAW';
                    signalUnit{1}='no units';
                    signalName{2}='Temperature';
                    signalFormat{2}='RAW';
                    signalUnit{2}='no units';
                    signalName{3}='Pressure';
                    signalFormat{3}='CAL';
                    signalName{4}='Temperature';
                    signalFormat{4}='CAL';
                    signalUnit{4}='Celsius';
                    if (thisShimmer.DefaultPressureCalibrationParameters==true)
                        signalUnit{3}='Pa'; % eventually when there is a self calibration method * can be inserted
                    else
                        signalUnit{3}='Pa';
                    end
                elseif strcmp(dataMode,'u')
                    pressureData=[pressureUncalibratedData temperatureUncalibratedData];
                    signalName{1}='Pressure';
                    signalFormat{1}='RAW';
                    signalUnit{1}='no units';
                    signalName{2}='Temperature';
                    signalFormat{2}='RAW';
                    signalUnit{2}='no units';
                elseif strcmp(dataMode,'c')
                    [pressureCalibratedData] = thisShimmer.calibratepressuredata(pressureUncalibratedData,temperatureUncalibratedData);
                    pressureData=pressureCalibratedData;
                    signalName{1}='Pressure';
                    signalFormat{1}='CAL';
                    if (thisShimmer.DefaultPressureCalibrationParameters==true)
                        signalUnit{1}='Pa *';
                    else
                        signalUnit{1}='Pa';
                    end
                    signalName{2}='Temperature';
                    signalFormat{2}='CAL';
                    signalUnit{2}='Celsius';
                else
                    disp('Warning: getpressuredata - Wrong data mode specified');
                end
            else
                pressureData = [];
                fprintf(strcat('Warning: getpressuredata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming,\n'));
                fprintf('or Shimmer is not a Shimmer3. Pressure is only supported on Shimmmer3.\n');
            end
        end
              
        function [strainGaugeData,signalName,signalFormat,signalUnit] = getstraingaugedata(thisShimmer, dataMode, parsedData)
            % Gets Shimmer2/2r Strain Gauge data from input parsedData.
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3)
                fprintf('Warning: getstraingaugedata - Strain Gauge is not supported for Shimmer3, see setenabledsensors.\n');
            elseif (strcmp(thisShimmer.State,'Streaming')) % Shimmer must be in a Streaming state
                
                iSGHighShimmer = thisShimmer.getsignalindex('Strain Gauge High');                            % Determine the column index of the Strain Gauge High signal
                iSGLowShimmer = thisShimmer.getsignalindex('Strain Gauge Low');                              % Determine the column index of the Strain Gauge High signal
                if strcmp(dataMode,'a')
                    strainGaugeHighUncalibratedData=double(parsedData(:,iSGHighShimmer));
                    strainGaugeLowUncalibratedData=double(parsedData(:,iSGLowShimmer));
                    strainGaugeHighCalibratedData = thisShimmer.calibrateu12ADCValue(strainGaugeHighUncalibratedData,60,3,551);
                    strainGaugeLowCalibratedData = thisShimmer.calibrateu12ADCValue(strainGaugeLowUncalibratedData,1950,3,183.7);
                    strainGaugeData=[strainGaugeHighUncalibratedData strainGaugeLowUncalibratedData strainGaugeHighCalibratedData  strainGaugeLowCalibratedData];
                    signalName{1}='Strain Gauge High';
                    signalFormat{1}='RAW';
                    signalUnit{1}='no units';
                    
                    signalName{2}='Strain Gauge Low';
                    signalFormat{2}='RAW';
                    signalUnit{2}='no units';
                    
                    signalName{3}='Strain Gauge High';
                    signalFormat{3}='CAL';
                    signalUnit{3}='millivolts';
                    
                    signalName{4}='Strain Gauge Low';
                    signalFormat{4}='CAL';
                    signalUnit{4}='millivolts';
                elseif strcmp(dataMode,'u')
                    strainGaugeHighUncalibratedData=double(parsedData(:,iSGHighShimmer));
                    strainGaugeLowUncalibratedData=double(parsedData(:,iSGLowShimmer));
                    strainGaugeData=[strainGaugeHighUncalibratedData strainGaugeLowUncalibratedData];
                    signalName{1}='Strain Gauge High';
                    signalFormat{1}='RAW';
                    signalUnit{1}='no units';
                    signalName{2}='Strain Gauge Low';
                    signalFormat{2}='RAW';
                    signalUnit{2}='no units';
                    
                elseif strcmp(dataMode,'c')
                    strainGaugeHighUncalibratedData=double(parsedData(:,iSGHighShimmer));
                    strainGaugeLowUncalibratedData=double(parsedData(:,iSGLowShimmer));
                    strainGaugeHighCalibratedData = thisShimmer.calibrateu12ADCValue(strainGaugeHighUncalibratedData,60,3,551);
                    strainGaugeLowCalibratedData = thisShimmer.calibrateu12ADCValue(strainGaugeLowUncalibratedData,1950,3,183.7);
                    strainGaugeData=[strainGaugeHighCalibratedData  strainGaugeLowCalibratedData];
                    signalName{1}='Strain Gauge High';
                    signalFormat{1}='CAL';
                    signalUnit{1}='millivolts';
                    
                    signalName{2}='Strain Gauge Low';
                    signalFormat{2}='CAL';
                    signalUnit{2}='millivolts';
                else
                    disp('Warning: getstraingaugedata - Wrong data mode specified');
                end
            else
                strainGaugeData = [];
                fprintf(strcat('Warning: getstraingaugedata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming.\n'));
            end
        end
        
        function [bridgeAmplifierData,signalName,signalFormat,signalUnit] = getbridgeamplifierdata(thisShimmer, dataMode, parsedData)
            % Gets Shimmer3 Bridge Amplifier data from input parsedData.
            if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                fprintf('Warning: getbridgeamplifierdata - Bridge Amplifier is not supported for Shimmer2/2r, see setenabledsensors.\n');
            elseif ( thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 4)
                fprintf('Warning: getbridgeamplifierdata - Bridge Amplifier is not supported for this firmware version, please update firmware.\n');
            elseif (strcmp(thisShimmer.State,'Streaming') && thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3) % Shimmer must be in a Streaming state
                
                iBAHighShimmer = thisShimmer.getsignalindex('Bridge Amplifier High');                            % Determine the column index of the Bridge Amplifier High signal
                iBALowShimmer = thisShimmer.getsignalindex('Bridge Amplifier Low');                              % Determine the column index of the Bridge Amplifier High signal
                if strcmp(dataMode,'a')
                    bridgeAmplifierHighUncalibratedData=double(parsedData(:,iBAHighShimmer));
                    bridgeAmplifierLowUncalibratedData=double(parsedData(:,iBALowShimmer));
                    bridgeAmplifierHighCalibratedData = thisShimmer.calibrateu12ADCValue(bridgeAmplifierHighUncalibratedData,60,3,551);
                    bridgeAmplifierLowCalibratedData = thisShimmer.calibrateu12ADCValue(bridgeAmplifierLowUncalibratedData,1950,3,183.7);
                    bridgeAmplifierData=[bridgeAmplifierHighUncalibratedData bridgeAmplifierLowUncalibratedData bridgeAmplifierHighCalibratedData  bridgeAmplifierLowCalibratedData];
                    signalName{1}='Bridge Amplifier High';
                    signalFormat{1}='RAW';
                    signalUnit{1}='no units';
                    
                    signalName{2}='Bridge Amplifier Low';
                    signalFormat{2}='RAW';
                    signalUnit{2}='no units';
                    
                    signalName{3}='Bridge Amplifier High';
                    signalFormat{3}='CAL';
                    signalUnit{3}='millivolts';
                    
                    signalName{4}='Bridge Amplifier Low';
                    signalFormat{4}='CAL';
                    signalUnit{4}='millivolts';
                elseif strcmp(dataMode,'u')
                    bridgeAmplifierHighUncalibratedData=double(parsedData(:,iBAHighShimmer));
                    bridgeAmplifierLowUncalibratedData=double(parsedData(:,iBALowShimmer));
                    bridgeAmplifierData=[bridgeAmplifierHighUncalibratedData bridgeAmplifierLowUncalibratedData];
                    signalName{1}='Bridge Amplifier High';
                    signalFormat{1}='RAW';
                    signalUnit{1}='no units';
                    signalName{2}='Bridge Amplifier Low';
                    signalFormat{2}='RAW';
                    signalUnit{2}='no units';
                    
                elseif strcmp(dataMode,'c')
                    bridgeAmplifierHighUncalibratedData=double(parsedData(:,iBAHighShimmer));
                    bridgeAmplifierLowUncalibratedData=double(parsedData(:,iBALowShimmer));
                    bridgeAmplifierHighCalibratedData = thisShimmer.calibrateu12ADCValue(bridgeAmplifierHighUncalibratedData,60,3,551);
                    bridgeAmplifierLowCalibratedData = thisShimmer.calibrateu12ADCValue(bridgeAmplifierLowUncalibratedData,1950,3,183.7);
                    bridgeAmplifierData=[bridgeAmplifierHighCalibratedData  bridgeAmplifierLowCalibratedData];
                    signalName{1}='Bridge Amplifier High';
                    signalFormat{1}='CAL';
                    signalUnit{1}='millivolts';
                    
                    signalName{2}='Bridge Amplifier Low';
                    signalFormat{2}='CAL';
                    signalUnit{2}='millivolts';
                else
                    disp('Warning: getbridgeamplifierdata - Wrong data mode specified');
                end
            else
                bridgeAmplifierData = [];
                fprintf(strcat('Warning: getbridgeamplifierdata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming.\n'));
            end
        end

        
        function [heartRateData, signalName, signalFormat, signalUnit] = getheartratedata(thisShimmer,parsedData)
            % Gets Shimmer2/2r Heart Rate data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming') && thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)                     % Shimmer must be in a Streaming state
        
                iHeartRateShimmer = thisShimmer.getsignalindex('Heart Rate');           % Determine the column index of the heart rate signal
                heartRateData = double(parsedData(:,iHeartRateShimmer));
                if (thisShimmer.FirmwareCompatibilityCode > 0)                          %only supported on BTStream
                    for i=1:length(heartRateData)
                        if (heartRateData(i)==0)
                            heartRateData(i)=thisShimmer.HRLastKnown;
                        else
                            heartRateData(i)=round(1024/heartRateData(i)*60);
                            thisShimmer.HRLastKnown=heartRateData(i);
                        end
                    end
                end
                signalName{1}='HeartRate';
                signalFormat{1}='CAL';
                signalUnit{1}='BPM';
            else
                heartRateData = [];
                fprintf(strcat('Warning: getheartratedata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming,\n'));
                fprintf('or Shimmer is a Shimmer3. Heartrate is not supported on Shimmmer3.\n');
            end
        end
        
        function [gsrData,signalName,signalFormat,signalUnit] = getgsrdata(thisShimmer, dataMode,parsedData)
            % Gets GSR data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                
               
                iGSRShimmer = thisShimmer.getsignalindex('GSR Raw');       % Determine the column index of the GSR signal
                gsrRange=thisShimmer.GsrRange;
                newGSRRange=gsrRange;
                for i=1:length(parsedData(:,iGSRShimmer))
                    if gsrRange==4
                        newGSRRange=round(bitand(49152,uint16(parsedData(i,iGSRShimmer)))/16383);
                        
                    end
                    if (gsrRange==0 || newGSRRange==0)
                        %                        p1=6.5995E-9; %polynomial fit
                        %                        p2=-6.895E-5;
                        %                        p3=2.699E-1;
                        %                        p4=-4.769835E+2;
                        %                        p5=3.403513341E+5;
                            p1=0.0373; %linear fit
                            p2=-24.9915;

                    elseif (gsrRange==1 || newGSRRange==1)
                        %                         p1=1.3569627E-8;
                        %                         p2=-1.650399E-4;
                        %                         p3=7.54199E-1;
                        %                         p4=-1.5726287856E+3;
                        %                         p5=1.367507927E+6;
                            p1=0.0054;
                            p2=-3.5194;
                     
                    elseif (gsrRange==2 || newGSRRange==2)
                        %                         p1=2.550036498E-8;
                        %                         p2=-3.3136E-4;
                        %                         p3=1.6509426597E+0;
                        %                         p4=-3.833348044E+3;
                        %                         p5=3.8063176947E+6;
                            p1=0.0015;
                            p2=-1.0163;
                     
                    elseif (gsrRange==3  || newGSRRange==3)
                        %                         p1=3.7153627E-7;
                        %                         p2=-4.239437E-3;
                        %                         p3=1.7905709E+1;
                        %                         p4=-3.37238657E+4;
                        %                         p5=2.53680446279E+7;
                            p1=4.5580e-04;
                            p2=-0.3014;
                     
                    end
                    
                    if strcmp(dataMode,'a')
                        gsrUncalibratedData(i,1)=parsedData(i,iGSRShimmer);
                        gsrCalibratedData(i,1) = thisShimmer.calibrategsrdatalinearfunction(parsedData(i,iGSRShimmer),p1,p2);
                        gsrData=[gsrUncalibratedData gsrCalibratedData];
                        signalName{1}='GSR';
                        signalFormat{1}='RAW';
                        signalUnit{1}='no units';
                        signalName{2}='GSR';
                        signalFormat{2}='CAL';
                        signalUnit{2}='kohms';
                        
                    elseif strcmp(dataMode,'u')
                        gsrUncalibratedData(i,1)=parsedData(i,iGSRShimmer);
                        gsrData=gsrUncalibratedData;
                        signalName{1}='GSR';
                        signalFormat{1}='RAW';
                        signalUnit{1}='no units';
                        
                    elseif strcmp(dataMode,'c')
                        gsrUncalibratedData(i,1)=parsedData(i,iGSRShimmer);
                        gsrCalibratedData(i,1) = thisShimmer.calibrategsrdatalinearfunction(parsedData(i,iGSRShimmer),p1,p2);
                        gsrData=gsrCalibratedData;
                        signalName{1}='GSR';
                        signalFormat{1}='CAL';
                        signalUnit{1}='kohms';
                        
                    else
                        disp('Warning: getgsrdata - Wrong data mode specified');
                    end
                    
                end
            else
                gsrData = [];
                fprintf(strcat('Warning: getgsrdata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
        end
        
        function [calibratedData] = calibrateu12ADCValue(thisShimmer,uncalibratedData,offset,vRefP,gain)
            % Calibration of 12BIT ADC data - calibrates input uncalibratedData with parameters:
            % offset,vRefP,gain.
            calibratedData=(uncalibratedData-offset)*(((vRefP*1000)/gain)/4095);
                      
        end %function calibrateu12ADCValue
        
        function [calibratedData] = calibratei24ADCValue(thisShimmer,uncalibratedData,offset,vRefP,gain)
            % Calibration of 24BIT ADC data - calibrates input uncalibratedData with parameters:
            % offset,vRefP,gain.
            calibratedData=(uncalibratedData-offset)*(((vRefP*1000)/gain)/(power(2,23)-1));
                      
        end %function calibratei24ADCValue
        
        function [calibratedData] = calibratei16ADCValue(thisShimmer,uncalibratedData,offset,vRefP,gain)
            % Calibration of 16BIT ADC data - calibrates input uncalibratedData with parameters:
            % offset,vRefP,gain.
            calibratedData=(uncalibratedData-offset)*(((vRefP*1000)/gain)/(power(2,15)-1));
                      
        end %function calibratei16ADCValue
        
        function [gsrCalibratedData] = calibrategsrdata(thisShimmer,gsrUncalibratedData,p1,p2,p3,p4,p5)
            % Calibration of GSR data - calibrates input gsrUncalibratedData with parameters:
            % p1,p2,p3,p4,p5.
            %
            % y = p1*x^4 + p2*x^3 + p3*x^2 + p4*x + p5; Resistance = y;
            % x = uncalibratedGSRData
            
            gsrUncalibratedData = bitand(uint16(gsrUncalibratedData),4095);
            gsrUncalibratedData = double(gsrUncalibratedData);
            gsrCalibratedData=(p1*power(gsrUncalibratedData,4)+p2*power(gsrUncalibratedData,3)+p3*power(gsrUncalibratedData,2)+p4*gsrUncalibratedData+p5)/1000;          
            
        end %function calibrateGSRdata
        
        function [gsrCalibratedData] = calibrategsrdatalinearfunction(thisShimmer,gsrUncalibratedData,p1,p2)
            % Calibration of GSR data - calibrates input gsrUncalibratedData with parameters:
            % p1,p2.
            %
            %y = p1*x^4 + p2*x^3 + p3*x^2 + p4*x + p5; Resistance = y;
            %x = uncalibratedGSRData
            
            gsrUncalibratedData = bitand(uint16(gsrUncalibratedData),4095); % remove the two most significant bits which are used to indicate the mode
            gsrUncalibratedData = double(gsrUncalibratedData);
            gsrCalibratedData=(1/(p1*gsrUncalibratedData+p2))*1000; %in kohms
            
            
        end %function calibrateGSRdatalinearfunction
        
        function CalibratedData = calibrateinertialsensordata(thisShimmer,UncalibratedData,R,K,B)
            % Calibration of inertial sensor data - calibrates input UncalibratedData with parameters:
            % R,K,B.
            %
            % Based on the theory outlined by Ferraris F, Grimaldi U, and Parvis M.
            % in "Procedure for effortless in-field calibration of three-axis rate gyros and accelerometers" Sens. Mater. 1995; 7: 311-30.
            % For a multiple samples of 3 axis data......
            %C = [R^(-1)] .[K^(-1)] .([U]-[B])
            
            %where.....
            %[C] -> [3 x n] Calibrated Data Matrix
            %[U] -> [3 x n] Uncalibrated Data Matrix
            %[B] ->  [3 x n] Offset Vector Matrix
            %[R] -> [3x3] Alignment Matrix
            %[K] -> [3x3] Sensitivity Matrix
            %n = Number of Samples
            CalibratedData=((R^-1)*(K^-1)*(UncalibratedData'-B*ones(1,length(UncalibratedData(:,1)))))';
            
        end
        
        function calibratedData = calibratepressuredata(thisShimmer, UP, UT)
            % Calibration of Shimmer3 BMP180/BMP280 Pressure sensor data - with
            % parameters: UP, UT.
            %
            % Algorithm from BMP180 datasheet and BMP280 datasheet,
            % respectively.
            calibratedData = [];
            if (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                fprintf('Warning: calibratepressuredata - Calibrate pressure data is not supported for Shimmer2/2r.\n')
            elseif (thisShimmer.FirmwareCompatibilityCode < 2)
                fprintf('Warning: calibratepressuredata - Calibrate pressure data is not supported for this firmware version, please update firmware.\n')
            elseif (thisShimmer.HardwareCompatibilityCode < 2)
                X1 = (UT - thisShimmer.AC6) * thisShimmer.AC5 / 32768;
                X2 = (thisShimmer.MC * 2048 ./ (X1 + thisShimmer.MD));
                B5 = X1 + X2;
                T = (B5 + 8) / 16;
                
                B6 = B5 - 4000;
                X1 = (thisShimmer.B2 * (power(B6,2)/ 4096)) / 2048;
                X2 = thisShimmer.AC2 .* B6 / 2048;
                X3 = X1 + X2;
                B3 = (((thisShimmer.AC1 * 4 + X3)*bitshift(1,thisShimmer.PressureResolution) + 2)) / 4;
                X1 = thisShimmer.AC3 .* B6 / 8192;
                X2 = (thisShimmer.B1 .* (power(B6,2)./ 4096)) / 65536;
                X3 = ((X1 + X2) + 2) / 4;
                B4 = thisShimmer.AC4 .* (X3 + 32768) / 32768;
                B7 = (UP - B3) * bitshift(50000,-thisShimmer.PressureResolution);
                if B7 < 2147483648 %0x80000000
                    p = (B7 * 2) ./ B4;
                else
                    p = (B7 ./ B4) * 2;
                end
                X1 = ((p / 256.0) .* (p / 256.0) * 3038) / 65536;
                X2 = (-7357 * p) / 65536;
                p = p +( (X1 + X2 + 3791) / 16);
                
                calibratedData(:,2) = T/10;
                calibratedData(:,1) = p;
            elseif (thisShimmer.HardwareCompatibilityCode == 2)
                adc_T = UT;
                adc_P = UP;
                               
                var1 = ((adc_T)/16384.0 - thisShimmer.DIG_T1/1024.0) * thisShimmer.DIG_T2;
                var2 = (((adc_T)/131072.0 - thisShimmer.DIG_T1/8192.0) .* (adc_T/131072.0 - thisShimmer.DIG_T1/8192.0)) * thisShimmer.DIG_T3;
                t_fine = var1 + var2;
                T = t_fine / 5120.0;
                
                
                var1 = (t_fine/2.0) - 64000.0;
                var2 = var1 .* var1 * thisShimmer.DIG_P6 / 32768.0;
                var2 = var2 + var1 * thisShimmer.DIG_P5 * 2.0;
                var2 = (var2/4.0)+(thisShimmer.DIG_P4 * 65536.0);
                var1 = (thisShimmer.DIG_P3 * var1 .* var1 / 524288.0 + thisShimmer.DIG_P2 * var1) / 524288.0;
                var1 = (1.0 + var1 / 32768.0)*thisShimmer.DIG_P1;
                if (var1 == 0.0)
                    var1 = eps; % avoid division by zero
                end
                p = 1048576.0 - adc_P;
                p = (p - (var2 / 4096.0)) * 6250.0 ./ var1;
                var1 = thisShimmer.DIG_P9 * p .* p / 2147483648.0;
                var2 = p * thisShimmer.DIG_P8 / 32768.0;
                p = p + (var1 + var2 + thisShimmer.DIG_P7) / 16.0;
                calibratedData(:,2) = T;
                calibratedData(:,1) = p;
            end
        end
        
        function noMotion = nomotiondetect(thisShimmer)
            % Detects if Shimmer is not moving.
            stdGyroBuffer = std(thisShimmer.GyroBuffer);
            if(max(stdGyroBuffer) <= thisShimmer.GyroMotionThreshold)
                noMotion = true;
            else
                noMotion = false;
            end
        end % function nomotiondetect

        function [] = estimategyrooffset(thisShimmer)
            % Estimates Gyroscope offset.
            meanGyroBuffer = mean(thisShimmer.GyroBuffer,1);
            
            thisShimmer.GyroCalParametersOV = meanGyroBuffer';
        end % function estimategyrooffset
        
        function [timeStampData, signalName,signalFormat,signalUnit]=gettimestampdata(thisShimmer,dataMode,parsedData)
            % Gets Time Stamp data from input parsedData.
            if (strcmp(thisShimmer.State,'Streaming'))                     % Shimmer must be in a Streaming state
                
                if strcmp(dataMode,'a')
                    uncalibratedTimeStampData = parsedData(:,1);
                    calibratedTimeStampData=thisShimmer.calibratetimestampdata(uncalibratedTimeStampData);
                    timeStampData=[uncalibratedTimeStampData calibratedTimeStampData];
                    signalName{1}='Time Stamp';
                    signalFormat{1}='RAW';
                    signalUnit{1}='no units';
                    signalName{2}='Time Stamp';
                    signalFormat{2}='CAL';
                    signalUnit{2}='milliseconds';
                elseif strcmp(dataMode,'u')
                    uncalibratedTimeStampData = parsedData(:,1);
                    timeStampData=uncalibratedTimeStampData;
                    signalName{1}='Time Stamp';
                    signalFormat{1}='RAW';
                    signalUnit{1}='no units';
                elseif strcmp(dataMode,'c')
                    uncalibratedTimeStampData = parsedData(:,1);
                    calibratedTimeStampData=thisShimmer.calibratetimestampdata(uncalibratedTimeStampData);
                    timeStampData=calibratedTimeStampData;
                    signalName{1}='Time Stamp';
                    signalFormat{1}='CAL';
                    signalUnit{1}='milliseconds';
                else
                    disp('Warning: gettimestampdata - Wrong data mode specified');
                end
            else
                timeStampData = [];
                fprintf(strcat('Warning: gettimestampdata - Cannot get data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
        end
        
        function timeStampCalibratedData = calibratetimestampdata(thisShimmer,uncalibratedTimeStampData)
            % Calibration of Time Stamp data - calibrates input uncalibratedTimeStampData.
            if (thisShimmer.FirmwareCompatibilityCode < 6)
                contUncalibratedTimeStampData = [thisShimmer.LastUncalibratedLoopTimeStamp; uncalibratedTimeStampData + thisShimmer.nClockOverflows * 65536]; % shift in LastUncalibratedLoopTimeStamp  and add clock overflow offsets of previous iterations
            else
                contUncalibratedTimeStampData = [thisShimmer.LastUncalibratedLoopTimeStamp; uncalibratedTimeStampData + thisShimmer.nClockOverflows * 16777216]; % shift in LastUncalibratedLoopTimeStamp  and add clock overflow offsets of previous iterations
            end
            for i=1:length(contUncalibratedTimeStampData)-1
                if (contUncalibratedTimeStampData(i+1) < contUncalibratedTimeStampData(i))
                    if (thisShimmer.FirmwareCompatibilityCode < 6)
                        contUncalibratedTimeStampData(i+1:end) = contUncalibratedTimeStampData(i+1:end)+65536; % add offset for each clock overflow within current iteration
                    else
                        contUncalibratedTimeStampData(i+1:end) = contUncalibratedTimeStampData(i+1:end)+16777216; % add offset for each clock overflow within current iteration
                    end
                    thisShimmer.nClockOverflows = thisShimmer.nClockOverflows + 1;
                end
            end
            
            timeStampCalibratedData=contUncalibratedTimeStampData(2:end)/32768*1000; % omit last timestamp of previous iteration and convert to ms: C=(U/32768*1000) C=calibrated U=uncalibrated
            
            thisShimmer.LastUncalibratedLoopTimeStamp=contUncalibratedTimeStampData(end);
            
        end %function calibratetimestampdata

        function quaternionData = updatequaternion(thisShimmer, accelCalibratedData, gyroCalibratedData, magCalibratedData)
            % Updates quaternion data based on accelerometer, gyroscope and
            % magnetometer data inputs: accelCalibratedData,
            % gyroCalibratedData and magCalibratedData.
            %
            % Implementation of MARG algorithm from:
            % Madgwick, S.O.H.; Harrison, A. J L; Vaidyanathan, R., "Estimation of IMU and MARG orientation using a gradient descent algorithm,"
            % Rehabilitation Robotics (ICORR), 2011 IEEE International Conference on , vol., no., pp.1,7, June 29 2011-July 1 2011, doi: 10.1109/ICORR.2011.5975346
            
            numSamples = size(accelCalibratedData,1);
            quaternionData = zeros(numSamples,4);
            
            % Normalise accelerometer and magnetometer data.
            accelMagnitude = sqrt(sum(accelCalibratedData.^2,2));
            magMagnitude = sqrt(sum(magCalibratedData.^2,2));
            
            if(min(accelMagnitude) ~= 0 && min(magMagnitude) ~= 0)
                accelNormalised = accelCalibratedData./repmat(accelMagnitude,1,3);
                magNormalised = magCalibratedData./repmat(magMagnitude,1,3);
                
                previousQuaternion = thisShimmer.LastQuaternion;
                
                iSample = 1;
                
                while(iSample <= numSamples)
                    q1 = previousQuaternion(1);
                    q2 = previousQuaternion(2);
                    q3 = previousQuaternion(3);
                    q4 = previousQuaternion(4);
                    
                    ax = accelNormalised(iSample,1);
                    ay = accelNormalised(iSample,2);
                    az = accelNormalised(iSample,3);
                    mx = magNormalised(iSample,1);
                    my = magNormalised(iSample,2);
                    mz = magNormalised(iSample,3);
                    gx = gyroCalibratedData(iSample,1)/180*pi;
                    gy = gyroCalibratedData(iSample,2)/180*pi;
                    gz = gyroCalibratedData(iSample,3)/180*pi;
                    
                    q1q1 = q1*q1;
                    q2q2 = q2*q2;
                    q3q3 = q3*q3;
                    q4q4 = q4*q4;
                    
                    twoq1 = 2*q1;
                    twoq2 = 2*q2;
                    twoq3 = 2*q3;
                    twoq4 = 2*q4;
                    
                    q1q2 = q1*q2;
                    q1q3 = q1*q3;
                    q1q4 = q1*q4;
                    q2q3 = q2*q3;
                    q2q4 = q2*q4;
                    q3q4 = q3*q4;
                    
                    % Calculate reference direction of Earth's magnetic
                    % field.
                    hx = mx*(q1q1 + q2q2 - q3q3-q4q4) + 2*my*(q2q3 - q1q4) + 2*mz*(q2q4 + q1q3);
                    hy = 2*mx*(q1q4 + q2q3) + my*(q1q1 - q2q2 + q3q3 - q4q4) * 2*mz*(q3q4 - q1q2);
                    twobx = sqrt(hx^2 + hy^2); % horizontal component
                    twobz = 2*mx*(q2q4 - q1q3) + 2*my*(q1q2 + q3q4) + mz*(q1q1 - q2q2 - q3q3 + q4q4); % vertical component
                    
                    % Calculate corrective step for gradient descent
                    % algorithm.
                    s1 = -twoq3 * (2*(q2q4 - q1q3) - ax) + ...
                        twoq2*(2*(q1q2 + q3q4) - ay) - ...
                        twobz*q3*(twobx*(0.5 - q3q3 - q4q4) + twobz*(q2q4 - q1q3) - mx) + ...
                        (-twobx*q4 + twobz*q2)*(twobx*(q2q3 - q1q4) + twobz*(q1q2 + q3q4) - my) + ...
                        twobx*q3*(twobx*(q1q3 + q2q4) + twobz*(0.5 - q2q2 - q3q3) - mz);
                    
                    s2 = twoq4*(2*(q2q4 - q1q3) - ax) + ...
                        twoq1*(2*(q1q2 + q3q4) - ay) - ...
                        4*q2*(1 - 2*(q2q2 + q3q3) - az) + ...
                        twobz*q4*(twobx*(0.5 - q3q3 - q4q4) + twobz*(q2q4 - q1q3) - mx) + ...
                        (twobx*q3 + twobz*q1)*(twobx*(q2q3 - q1q4) + twobz*(q1q2 + q3q4) - my) + ...
                        (twobx*q4 - twobz*twoq2)*(twobx*(q1q3 + q2q4) + twobz*(0.5 - q2q2 - q3q3) - mz);
                    
                    s3 = -twoq1*(2*(q2q4 - q1q3) - ax) + ...
                        twoq4*(2*(q1q2 + q3q4) - ay) - ...
                        4*q3*(1 - 2*(q2q2 + q3q3) - az) + ...
                        (-twobx*twoq3 - twobz*q1)*(twobx*(0.5 - q3q3 - q4q4) + twobz*(q2q4 - q1q3) - mx) + ...
                        (twobx * q2 + twobz * q4)*(twobx*(q2q3 - q1q4) + twobz*(q1q2 + q3q4) - my) + ...
                        (twobx * q1 - twobz*twoq3)*(twobx*(q1q3 + q2q4) + twobz*(0.5 - q2q2 - q3q3) - mz);
                    
                    s4 = twoq2 * (2.0 * (q2q4 - q1q3) - ax) + ...
                        twoq3 * (2*(q1q2 + q3q4) - ay) + ...
                        (-twobx * twoq4 + twobz * q2) * (twobx * (0.5 - q3q3 - q4q4) + twobz * (q2q4 - q1q3) - mx) + ...
                        (-twobx * q1 + twobz * q3) * (twobx * (q2q3 - q1q4) + twobz * (q1q2 + q3q4) - my) + ...
                        twobx * q2 * (twobx * (q1q3 + q2q4) + twobz * (0.5 - q2q2 - q3q3) - mz);
                    
                    sNormalised = [s1, s2, s3, s4]/sqrt(sum([s1 s2 s3 s4].^2));
                    
                    % Rate of change from gyro values...
                    dqdt(1) = 0.5 * (-q2 * gx - q3 * gy - q4 * gz);
                    dqdt(2) = 0.5 * ( q1 * gx - q4 * gy + q3 * gz);
                    dqdt(3) = 0.5 * ( q4 * gx + q1 * gy - q2 * gz);
                    dqdt(4) = 0.5 * (-q3 * gx + q2 * gy + q1 * gz);
                    
                    % ...plus rate of change from gradient descent step.
                    beta = 0.5;
                    dqdt = dqdt - beta*sNormalised;
                    
                    tempQuaternion = previousQuaternion + dqdt./thisShimmer.SamplingRate;
                    quaternionData(iSample, :) = tempQuaternion/(sqrt(sum(tempQuaternion.^2)));
                    
                    previousQuaternion = quaternionData(iSample, :);
                    thisShimmer.LastQuaternion = previousQuaternion;
                    iSample = iSample + 1;
                end
            end
            
        end %function updatequaternion
        
        function [parsedData,systemTime] = capturedata(thisShimmer)
            % Reads data from the serial buffer, frames and parses these data.
            parsedData=[];
            systemTime = 0;
            
            if (strcmp(thisShimmer.State,'Streaming'))                        % TRUE if the Shimmer is in a Streaming state
                
                serialData = [];
                
                [serialData, isFileOpen] = readdatabuffer(thisShimmer, inf);  % Read all available serial data from the com port
                
                if (not(isempty(serialData)))
                    
                     if (thisShimmer.EnableTimestampUnix)  
                         systemTime = clock;
                     end
                   
                    framedData = framedata(thisShimmer, serialData);
                    parsedData = parsedata(thisShimmer, framedData);
         
                end
            else
                fprintf(strcat('Warning: capturedata - Cannot capture data as COM ',thisShimmer.ComPort,' Shimmer is not Streaming'));
            end
            
        end %function captureData
        
        function framedData = framedata(thisShimmer, serialData)
            serialData = cat(1,thisShimmer.SerialDataOverflow,serialData);                              % append serial data from previous function call to new serial data
            framedData = [];
            iDataRow = 1;
            skip = 0;
            indexFirstDataPacket = find(serialData == thisShimmer.DATA_PACKET_START_BYTE,1);            % find first data packet start byte
            indexFirstAckResponse = find(serialData == thisShimmer.ACK_RESPONSE,1);                     % find first ack response byte
            
            if (isempty(indexFirstDataPacket) && isempty(indexFirstAckResponse))                        % check whether data packet start byte or ack response is found first in serial data
                thisShimmer.SerialDataOverflow = serialData;                                            % use serial data in next function call in case no data packet start byte or ack response is found
                framedData = [];
                skip = 1;
            elseif (isempty(indexFirstAckResponse))
                currentIndex = indexFirstDataPacket;                                                    % start at first data packet start byte
                ackFirst = 0;
            elseif (isempty(indexFirstDataPacket) || indexFirstDataPacket > indexFirstAckResponse)
                currentIndex = indexFirstAckResponse;                                                   % start at first ack response byte
                ackFirst = 1;
            else
                currentIndex = indexFirstDataPacket;                                                    % start at first data packet start byte
                ackFirst = 0;
            end
            
            if ~skip
                if ~(ackFirst)                                                                          % data packet start byte is found before ack response is found
                    exitLoop = 0;
                    endOfPacket = 0;
                    while (currentIndex <= length(serialData) && ~exitLoop)                             % loop through serialData
                        if (length(serialData(currentIndex:end)) >= (thisShimmer.nBytesDataPacket))                                                          % check if there are enough bytes for a full packet
                            if (length(serialData(currentIndex:end)) == (thisShimmer.nBytesDataPacket))                                                      % check if there are exactly enough bytes for one packet - it is the last packet
                                framedData(iDataRow,1:thisShimmer.nBytesDataPacket) = serialData(currentIndex:currentIndex-1+thisShimmer.nBytesDataPacket);  % add current packet to framedData
                                currentIndex = length(serialData);
                                endOfPacket = 1;
                            elseif (serialData(currentIndex+thisShimmer.nBytesDataPacket)==thisShimmer.ACK_RESPONSE)                                         % next byte after nBytesDataPacket bytes is 0xFF
                                framedData(iDataRow,1:thisShimmer.nBytesDataPacket) = serialData(currentIndex:currentIndex-1+thisShimmer.nBytesDataPacket);  % add current packet to framedData
                                thisShimmer.SerialDataOverflow = serialData(currentIndex+thisShimmer.nBytesDataPacket:end);                                  % use serial data in next function call
                                exitLoop = 1;                                                                                                                % exit while loop
                            elseif (serialData(currentIndex+thisShimmer.nBytesDataPacket)==thisShimmer.DATA_PACKET_START_BYTE)                               % next byte after nBytesDataPacket bytes is 0x00
                                framedData(iDataRow,1:thisShimmer.nBytesDataPacket) = serialData(currentIndex:currentIndex-1+thisShimmer.nBytesDataPacket);  % add current packet to framedData
                                currentIndex = currentIndex+thisShimmer.nBytesDataPacket;                                                                    % update index to next data packet start byte
                                iDataRow = iDataRow + 1;
                            else                                                                        % error, discard first byte and pass rest to overflow - not last packet and first byte after nBytesDataPacket is not 0x00 or 0xFF
                                thisShimmer.SerialDataOverflow = serialData(currentIndex+1:end);        % use serial data - apart from first byte - in next function call
                                exitLoop = 1;                                                           % exit while loop
                            end
                        else                                                                            % not enough bytes for a full packet
                            if (endOfPacket)
                                thisShimmer.SerialDataOverflow = [];                                    % all serial data is used in current function call
                            else
                                thisShimmer.SerialDataOverflow = serialData(currentIndex:end);          % use serial data in next function call
                            end
                            exitLoop = 1;                                                               % exit while loop
                        end
                    end
                else                                                                                    % ack response byte is found before data packet start byte is found
                    if (currentIndex == length(serialData))
                        thisShimmer.SerialDataOverflow = serialData(currentIndex:end);                  % use serial data in next function call
                    elseif (serialData(currentIndex+1) == thisShimmer.ACK_RESPONSE)                     % check if next byte is also acknowledgement byte
                        thisShimmer.SerialDataOverflow = serialData(currentIndex+1:end);                % discard acknowledgement byte and pass rest to overflow for next function call
                    elseif (serialData(currentIndex+1) == thisShimmer.DATA_PACKET_START_BYTE)           % check if next byte is data start byte
                        thisShimmer.SerialDataOverflow = serialData(currentIndex+1:end);                % discard acknowledgement byte and pass rest to overflow for next function call
                    elseif (serialData(currentIndex+1) ==  thisShimmer.INSTREAM_CMD_RESPONSE)
                        if (currentIndex <= length(serialData(currentIndex:end))-3)                     % check if enough bytes for a response
                            if (serialData(currentIndex+2) ==  thisShimmer.STATUS_RESPONSE)
                                statusResponse = serialData(currentIndex+3);
                                thisShimmer.IsSensing = bitand(statusResponse,2)/2;                     % update property
                                thisShimmer.IsDocked = bitand(statusResponse,1);                        % update property
                                thisShimmer.IsSDLogging = bitand(statusResponse,8)/8;                   % update property
                                thisShimmer.IsStreaming = bitand(statusResponse,16)/16;                 % update property
                                disp(strcat('Sensing: ',num2str(thisShimmer.IsSensing)));
                                disp(strcat('Docked: ',num2str(thisShimmer.IsDocked)));
                                disp(strcat('SDLogging: ',num2str(thisShimmer.IsSDLogging)));
                                disp(strcat('Streaming: ',num2str(thisShimmer.IsStreaming)));
                                thisShimmer.SerialDataOverflow = serialData(currentIndex+4:end);        % pass rest of bytes to overflow
                            elseif (serialData(currentIndex+2) ==  thisShimmer.DIR_RESPONSE)
                                directoryNameLength = double(serialData(currentIndex+3));               % length of SD Directory Name
                                if (currentIndex <= length(serialData(currentIndex:end))-3-directoryNameLength)      % check if enough bytes for full response
                                    directoryName = serialData(currentIndex+4:currentIndex+4+directoryNameLength);
                                    thisShimmer.SdCardDirectoryName = native2unicode(directoryName', 'US-ASCII');    % Convert bytes to ASCII string
                                    disp(strcat('SD Card Directory Name: ',thisShimmer.SdCardDirectoryName));
                                    thisShimmer.SerialDataOverflow = serialData(currentIndex+3+directoryNameLength+1:end);        % pass rest of bytes to overflow
                                else                                                                    % not enough bytes to get the full directory name
                                    thisShimmer.SerialDataOverflow = serialData(currentIndex:end);      % discard acknowledgement byte and pass rest to overflow for next function call
                                end
                            elseif (serialData(currentIndex+2) ==  thisShimmer.VBATT_RESPONSE)
                                
                                if (currentIndex <= length(serialData(currentIndex:end))-3-2)      % check if enough bytes for full response
                                    battAdcValue = uint32(uint16(serialData(currentIndex+3+1))*256 + uint16(serialData(currentIndex+3)));                % battery ADC value
                                    batteryVoltage = calibrateu12ADCValue(thisShimmer,battAdcValue,0,3.0,1.0)*1.988; % calibrate 12-bit ADC value with offset = 0; vRef=3.0; gain=1.0
                                    fprintf(['Battery Voltage: ' num2str(batteryVoltage) '[mV]' '\n']);
                                    thisShimmer.LatestBatteryVoltageReading = batteryVoltage;
                                    thisShimmer.SerialDataOverflow = serialData(currentIndex+5+1:end);        % pass rest of bytes to overflow
                                else                                                                    % not enough bytes to get the full directory name
                                    thisShimmer.SerialDataOverflow = serialData(currentIndex:end);      % discard acknowledgement byte and pass rest to overflow for next function call
                                end
                                
                            else                                                                        % (0xFF, 0x8A, ~0x71 && ~0x88)
                                thisShimmer.SerialDataOverflow = serialData(currentIndex+1:end);        % discard acknowledgement byte and pass rest to overflow for next function call
                            end
                        else                                                                            % not enough bytes for full response
                            thisShimmer.SerialDataOverflow = serialData(currentIndex:end);              % discard acknowledgement byte and pass rest to overflow for next function call
                        end
                    else                                                                                % is not last byte, 0xFF, 0x00, 0x8A - assume error occurred - discard first byte and pass rest to overflow for next function call
                        thisShimmer.SerialDataOverflow = serialData(currentIndex+1:end);                % discard acknowledgement byte and pass rest to overflow for next function call
                    end
                end
            end
        end % function framedata
        

                
        function parsedData = parsedata(thisShimmer, framedData)
            % Parses framed data from input framedData.
            if ~isempty(framedData)                                       % TRUE if framedData is not empty
                
                iColFramedData=2;                                         % Start at 2nd column, 1st colum is data packet identifier
                
                for iColParsedData=1:length(thisShimmer.SignalDataTypeArray)
                    
                    dataType=char(thisShimmer.SignalDataTypeArray(iColParsedData));
                    
                    switch dataType
                        case('u24')
                            lsbArray(:,1) = uint32(framedData(:,iColFramedData + 0));                % Extract the least significant byte and convert to unsigned 32bit
                            msbArray(:,1) = uint32(256*uint32(framedData(:,iColFramedData + 1)));    % Extract the most significant byte and convert to unsigned 32bit
                            xmsbArray(:,1)= uint32(65536*uint32(framedData(:,iColFramedData + 2)));  % Extract the most significant byte and convert to unsigned 32bit
                            parsedData(:,iColParsedData) = int32(xmsbArray+uint32(msbArray) + uint32(lsbArray));  % Convert to signed 32bit integer to enable creation of array of all data
                            iColFramedData = iColFramedData + 3;                                     % Increment column offset by 3 bytes for next column of sensor data                           
                        case('u8')
                            lsbArray(:,1) = uint8(framedData(:,iColFramedData));                     % Extract the column of bytes of interest
                            parsedData(:,iColParsedData) = int32(lsbArray);                          % Convert to signed 32bit integer to enable creation of array of all data
                            iColFramedData = iColFramedData + 1;                                     % Increment column offset by 1 byte for next column of sensor data
                        case('i8')
                            lsbArray(:,1) = int8(framedData(:,iColFramedData));                      % Extract the column of bytes of interest
                            parsedData(:,iColParsedData) = int32(lsbArray);                          % Convert to signed 32bit integer to enable creation of array of all data
                            iColFramedData = iColFramedData + 1;                                     % Increment column offset by 1 byte for next column of sensor data
                        case('u12')
                            lsbArray(:,1) = uint16(framedData(:,iColFramedData));                    % Extract the least significant byte and convert to unsigned 16bit
                            msbArray(:,1) = uint16(bitand(15,framedData(:,iColFramedData + 1)));     % Extract the most significant byte, set 4 MSBs to 0 and convert to unsigned 16bit
                            parsedData(:,iColParsedData) = int32(256*msbArray + lsbArray);           % Convert to signed 32bit integer to enable creation of array of all data
                            iColFramedData = iColFramedData + 2;                                     % Increment column offset by 2 bytes for next column of sensor data
                        case('u16')
                            lsbArray(:,1) = uint16(framedData(:,iColFramedData));                    % Extract the least significant byte and convert to unsigned 16bit
                            msbArray(:,1) = uint16(framedData(:,iColFramedData + 1));                % Extract the most significant byte and convert to unsigned 16bit
                            parsedData(:,iColParsedData) = int32(256*msbArray + lsbArray);           % Convert to signed 32bit integer to enable creation of array of all data
                            iColFramedData = iColFramedData + 2;                                     % Increment column offset by 2 bytes for next column of sensor data
                        case('u16*')
                            lsbArray(:,1) = uint16(framedData(:,iColFramedData + 1));                % Extract the least significant byte and convert to unsigned 16bit
                            msbArray(:,1) = uint16(framedData(:,iColFramedData + 0));                % Extract the most significant byte and convert to unsigned 16bit
                            parsedData(:,iColParsedData) = int32(256*msbArray + lsbArray);           % Convert to signed 32bit integer to enable creation of array of all data
                            iColFramedData = iColFramedData + 2;      
                        case('i16')
                            lsbArray(:,1) = uint16(framedData(:,iColFramedData));                    % Extract the least significant byte and convert to unsigned 16bit
                            msbArray(:,1) = uint16(framedData(:,iColFramedData + 1));                % Extract the most significant byte and convert to unsigned 16bit
                            parsedData(:,iColParsedData) = int32(256*msbArray + lsbArray);           % Convert to signed 32bit integer to enable creation of array of all data
                            iColFramedData = iColFramedData + 2;                                     % Increment column offset by 2 bytes for next column of sensor data
                            parsedData(:,iColParsedData) = calculatetwoscomplement(thisShimmer,cast(parsedData(:,iColParsedData),'uint16'),16);
                        case('i16*')
                            lsbArray(:,1) = uint16(framedData(:,iColFramedData+1));                  % Extract the least significant byte and convert to unsigned 16bit
                            msbArray(:,1) = uint16(framedData(:,iColFramedData));                    % Extract the most significant byte and convert to unsigned 16bit
                            parsedData(:,iColParsedData) = int32(256*msbArray + lsbArray);           % Convert to signed 32bit integer to enable creation of array of all data
                            iColFramedData = iColFramedData + 2;                                     % Increment column offset by 2 bytes for next column of sensor data
                            parsedData(:,iColParsedData) = calculatetwoscomplement(thisShimmer,cast(parsedData(:,iColParsedData),'uint16'),16);
                        case('i16>') % int16 to 12 bits
                            lsbArray(:,1) = uint16(framedData(:,iColFramedData));                    % Extract the least significant byte and convert to unsigned 16bit
                            msbArray(:,1) = uint16(framedData(:,iColFramedData + 1));                % Extract the most significant byte and convert to unsigned 16bit
                            parsedData(:,iColParsedData) = int32(256*msbArray + lsbArray);           % Convert to signed 32bit integer to enable creation of array of all data
                            iColFramedData = iColFramedData + 2;                                     % Increment column offset by 2 bytes for next column of sensor data
                            parsedData(:,iColParsedData) = calculatetwoscomplement(thisShimmer,cast(parsedData(:,iColParsedData),'uint16'),16);
                            parsedData(:,iColParsedData) = parsedData(:,iColParsedData)/16;
                        case('u24*') %bytes reverse order
                            lsbArray(:,1) = uint32(framedData(:,iColFramedData + 2));                % Extract the least significant byte and convert to unsigned 32bit
                            msbArray(:,1) = uint32(256*uint32(framedData(:,iColFramedData + 1)));    % Extract the most significant byte and convert to unsigned 32bit
                            xmsbArray(:,1)= uint32(65536*uint32(framedData(:,iColFramedData + 0)));  % Extract the most significant byte and convert to unsigned 32bit
                            parsedData(:,iColParsedData) = int32(xmsbArray+uint32(msbArray) + uint32(lsbArray));
                            iColFramedData = iColFramedData + 3;
                        case('i24*') 
                            lsbArray(:,1) = uint32(framedData(:,iColFramedData + 2));                % Extract the least significant byte and convert to unsigned 32bit
                            msbArray(:,1) = uint32(256*uint32(framedData(:,iColFramedData + 1)));    % Extract the most significant byte and convert to unsigned 32bit
                            xmsbArray(:,1)= uint32(65536*uint32(framedData(:,iColFramedData + 0)));  % Extract the most significant byte and convert to unsigned 32bit
                            parsedData(:,iColParsedData) = uint32(xmsbArray+uint32(msbArray) + uint32(lsbArray));  % Convert to signed 32bit integer to enable creation of array of all data
                            parsedData(:,iColParsedData) = calculatetwoscomplement(thisShimmer,cast(parsedData(:,iColParsedData),'uint32'),24);
                            iColFramedData = iColFramedData + 3;                                     % Increment column offset by 2 bytes for next column of sensor data
                    end
                end
                
            else
                
                parsedData = [];                                          % Return empty array
                
            end
        end % function parsedata
        
        function exgGain = convertEXGGain(thisShimmer,gainsetting) 
            % Converts ExG gain setting from input gainsetting to exgGain.
            if (thisShimmer.ShimmerVersion == thisShimmer.SHIMMER_3 && thisShimmer.FirmwareCompatibilityCode < 3)
                fprintf(strcat('Warning: convertEXGGain - Command not supported for this firmware version, please update firmware.\n'));
            elseif (thisShimmer.ShimmerVersion < thisShimmer.SHIMMER_3)
                fprintf(strcat('Warning: convertEXGGain - Command not supported for Shimmer2/2r.\n'));
            else
                if (gainsetting ==0)
                    exgGain=6;
                elseif (gainsetting ==1)
                    exgGain=1;
                elseif (gainsetting ==2)
                    exgGain=2;
                elseif (gainsetting ==3)
                    exgGain=3;
                elseif (gainsetting ==4)
                    exgGain=4;
                elseif (gainsetting ==5)
                    exgGain=8;
                elseif (gainsetting ==6)
                    exgGain=12;
                else
                    exgGain = 'Nan';
                end
                
            end
        end % function exgGain
        
        function newData = calculatetwoscomplement(thisShimmer,signedData,bitLength)
            % Calculates the two's complement of input signedData.
            newData=double(signedData);
            for i=1:numel(signedData)
                %if (signedData(i)>=bitshift(1,bitLength-1))
                    %deprecated method from 2013
                    %newData(i)=-double((bitcmp(signedData(i),bitLength)+1));
                    %
                     if (bitLength==24)
                        if (signedData(i)>=bitshift(1,bitLength-1))
                            newData(i)=-(double( bitxor(signedData(i),(bitshift(1,bitLength)-1)))+1);
                        end
                     elseif (bitLength==16)
                        if (signedData(i)>=bitshift(1,bitLength-1))
                            newData(i)=-(double(bitcmp(uint16(signedData(i))))+1);
                        end
                    elseif (bitLength==8)
                      if (signedData(i)>=bitshift(1,bitLength-1))
                            newData(i)=-(double(bitcmp(uint8(signedData(i))))+1);
                      end
                    else
                        disp('Warning: calculatetwoscomplement - BitLength not supported for twocomplement method');
                    end
            end
            
        end %function calculatetwoscomplement
        
        function isAcknowledged = waitforack(thisShimmer, timeout)
            % Reads serial data buffer until either an acknowledgement is
            % received or time out from input timeout is exceeded.
            serialData = [];
            timeCount = 0;
            waitPeriod = 0.1;                                              % Period in seconds the while loop waits between iterations
            
            elapsedTime = 0;                                               % Reset to 0
            tic;                                                           % Start timer
            
            while (isempty(serialData) && (elapsedTime < timeout))         % Keep reading serial port data until new data arrives OR elapsedTime exceeds timeout
                [serialData, isFileOpen] = readdatabuffer(thisShimmer, 1); % Read a single byte of serial data from the com port
                pause(waitPeriod);                                         % Wait 0.1 of a second
                timeCount = timeCount + waitPeriod;                        % Timeout is used to exit while loop after 5 seconds
                
                elapsedTime = elapsedTime + toc;                           % Stop timer and add to elapsed time
                tic;                                                       % Start timer
                
            end
            
            elapsedTime = elapsedTime + toc;                               % Stop timer
            
            if (not(isempty(serialData)))                                  % TRUE if a byte value was received
                if serialData(1) == thisShimmer.ACK_RESPONSE               % TRUE if the byte value was the acknowledge command
                    isAcknowledged = true;
                else
                    isAcknowledged = false;
                end
            else
                isAcknowledged = false;
                fprintf(strcat('Warning: waitforack - Timed-out on wait for acknowledgement byte on Shimmer COM',thisShimmer.ComPort,'.\n'));
            end
            
        end %function waitforack
               
        function isAcknowledged = waitforacknofeedback(thisShimmer, timeout)
            % Reads serial data buffer until either an acknowledgement is
            % received or time out from input timeout is exceeded.
            % (Feedback message from waitforack function is not included.)
            serialData = [];
            timeCount = 0;
            waitPeriod = 0.1;                                              % Period in seconds the while loop waits between iterations
            
            elapsedTime = 0;                                               % Reset to 0
            tic;                                                           % Start timer
            
            while (isempty(serialData) && (elapsedTime < timeout))        % Keep reading serial port data until new data arrives OR elapsedTime exceeds timeout
                [serialData, isFileOpen] = readdatabuffer(thisShimmer, 1); % Read a single byte of serial data from the com port
                pause(waitPeriod);                                         % Wait 0.1 of a second
                timeCount = timeCount + waitPeriod;                        % Timeout is used to exit while loop after 5 seconds
                
                elapsedTime = elapsedTime + toc;                           % Stop timer and add to elapsed time
                tic;                                                       % Start timer
                
            end
            
            elapsedTime = elapsedTime + toc;                               % Stop timer
            
            if (not(isempty(serialData)))                                  % TRUE if a byte value was received
                if serialData(1) == thisShimmer.ACK_RESPONSE               % TRUE if the byte value was the acknowledge command
                    isAcknowledged = true;
                else
                    isAcknowledged = false;
                end
            else
                isAcknowledged = false;
                
            end
            
        end %function waitforacknofeedback
        
    end %methods (Access = 'private')
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Realterm Abstraction Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = 'private')
        
        function isInitialised = initialiserealterm(thisShimmer)
            % Initialises Realterm buffer.
            thisShimmer.Hrealterm = actxserver('realterm.realtermintf');   % Start Realterm as a server
            thisShimmer.Hrealterm.baud = 9600;
            thisShimmer.Hrealterm.TimerPeriod = 10000;                     % Set time-out to 10 seconds       
            thisShimmer.Hrealterm.Port = thisShimmer.ComPort;              % Assign the Shimmer Com Port number to the realterm server
            thisShimmer.Hrealterm.caption = strcat('Matlab Shimmer Realterm Server COM',thisShimmer.ComPort);   % Assign a title to the realterm server window
            thisShimmer.Hrealterm.windowstate = 1;                         % Minimise realterm server window
            realtermBufferDirectory = strcat(pwd,'\realtermBuffer');       % Define directory for realtermBuffer
            
            if ~(exist(realtermBufferDirectory,'dir'))                     % if realtermBuffer directory does not exist then create it
                mkdir(realtermBufferDirectory);
            end
            
            thisShimmer.Hrealterm.CaptureFile=strcat(realtermBufferDirectory,'\matlab_data_COM',thisShimmer.ComPort,'.dat');    % define realterm buffer file name
            isInitialised = true;
        end % function initialiserealterm
                
        function isOpen = opencomport(thisShimmer)
            % Open COM Port.
            initialiserealterm(thisShimmer);                               % Define and open realterm server
            try
                thisShimmer.Hrealterm.PortOpen = true;                     % Open the COM Port
            catch
                fprintf(strcat('Warning: opencomport - Unable to open Com Port ',thisShimmer.ComPort,'.\n'));
            end
            
            if(thisShimmer.Hrealterm.PortOpen~=0)                          % TRUE if the COM Port opened OK
                invoke(thisShimmer.Hrealterm,'startcapture');              % Enable realtime buffer
                thisShimmer.FilePointer = 0;                               % Set FilePointer to start of file
                isOpen=1;
            else
                disconnect(thisShimmer);                                   % TRUE if COM Port didnt open close realterm server
                isOpen=0;
            end
        end % opencomport
                
        function isCleared = clearreaddatabuffer(thisShimmer)
            % The buffer isnt really cleared, all available data is read
            [bufferedData, isCleared] = readdatabuffer(thisShimmer, inf);   % so that file pointer is set to end of file
            
        end % function clearserialportbuffer
                
        function isOpen = closecomport(thisShimmer)
            % Close COM Port.
            thisShimmer.Hrealterm.PortOpen=0;                            % Close the COM Port
            
            if(thisShimmer.Hrealterm.PortOpen~=0)                        % TRUE if the COM Port is still open
                isOpen=1;
                fprintf(strcat('Warning: closecomport - Unable to close COM',thisShimmer.ComPort,'.\n'));
            else
                isOpen=0;
                closerealterm(thisShimmer);
            end
        end % function closecomport
                
        function isClosed = closerealterm(thisShimmer)
            % Close the Realterm server.
            invoke(thisShimmer.Hrealterm,'stopcapture');
            isClosed = true;
            try                                                           % Try to close realterm server
                invoke(thisShimmer.Hrealterm,'close'); delete(thisShimmer.Hrealterm);
            catch
                isClosed = false;
                fprintf(strcat('Warning: closerealterm - Unable to close realterm for COM',thisShimmer.ComPort,'.'))
            end; % try
            
        end % function closerealterm
                
        function void = writetocomport(thisShimmer, charValue)
            % Writes the value of input charValue to the COM Port.
            invoke(thisShimmer.Hrealterm, 'PutChar', charValue); % Send the charValue to the Com Port
        end % function writetocomport
        
        function [bufferedData, didFileOpen] = readdatabuffer(thisShimmer, nValues)
            % Reads data from the Realterm data buffer.
            bufferedData=[];
            
            fileId = fopen(thisShimmer.Hrealterm.CaptureFile, 'r');        % Open file with read only permission
            if (fileId ~= -1)                                              % TRUE if file was opened successfully
                didFileOpen = true;                                        % Set isFileOpen to 1 to indicate that the file was opened
                fseek(fileId,thisShimmer.FilePointer,'bof');               % Set file pointer to value stored from previous fread
                bufferedData=fread(fileId, nValues, '*uint8');             % Read data from the realterm data buffer
                thisShimmer.FilePointer = thisShimmer.FilePointer + length(bufferedData);  % Update FilePointer value to position of last value read
                fclose(fileId);
            else
                didFileOpen = false;                                             % Set isFileOpen to 0 to indicate that the file failed to open
                fprintf(strcat('Warning: readdatabuffer - Cannot open realterm capture file for COM',thisShimmer.ComPort,'.\n'));
            end
            
        end %function readserialport
        
        
    end %methods (Access = 'private')
    
end %classdef shimmer



