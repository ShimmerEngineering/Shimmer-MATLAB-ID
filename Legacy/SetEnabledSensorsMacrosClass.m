classdef SetEnabledSensorsMacrosClass < handle
    
    % SETENABLEDSENSORSMACROSCLASS - Defines macros for setenabledsensors
    % 
    % SETENABLEDSENSORSMACROSCLASS assigns user friendly variable names 
    % to the sensor names in string format that are valid inputs
    % for ShimmerHandleClass.setenabledsensors:
    %
    %         ACCEL='Accel';                  % Accelerometer; for Shimmer3 Low Noise Accelerometer will be selected.
    %         LNACCEL='LowNoiseAccel';        % Low Noise Accelerometer for Shimmer3
    %         WRACCEL='WideRangeAccel';       % Wide Range Accelerometer for Shimmer3
    %         ALTACCEL='AlternativeAccel';    % MPU9150 Accelerometer for Shimmer3
    %         GYRO = 'Gyro';                  % Gyroscope
    %         MAG = 'Mag';                    % Magnetometer
    %         ALTMAG = 'AlternativeMag';      % MPU9150 Magnetometer for Shimmer3
    %         ECG = 'ECG';                    % ECG
    %         ECG24BIT ='ECG 24BIT';          % ECG 24BIT for Shimmer3
    %         ECG16BIT ='ECG 16BIT';          % ECG 16BIT for Shimmer3
    %         EMG = 'EMG';                    % EMG
    %         EMG24BIT = 'EMG 24BIT';         % EMG 24BIT for Shimmer3
    %         EMG16BIT = 'EMG 16BIT';         % EMG 16BIT for Shimmer3
    %         EXG1 = 'EXG1';                  % EXG1 for Shimmer3
    %         EXG124BIT = 'EXG1 24BIT';       % EXG1 24BIT for Shimmer3
    %         EXG116BIT = 'EXG1 16BIT';       % EXG1 16BIT for Shimmer3
    %         EXG2 = 'EXG2';                  % EXG2 for Shimmer3
    %         EXG224BIT = 'EXG2 24BIT';       % EXG2 24BIT for Shimmer3
    %         EXG216BIT = 'EXG2 16BIT';       % EXG2 16BIT for Shimmer3
    %         GSR = 'GSR';                    % GSR
    %         EXPA0 = 'ExpBoard_A0';          % External Expansion Board A0 for Shimmer2r
    %         EXPA7 = 'ExpBoard_A7';          % External Expansion Board A7 for Shimmer2r
    %         EXTA7 = 'EXT A7';               % External ADC A7 for Shimmer3
    %         EXTA6 = 'EXT A6';               % External ADC A6 for Shimmer3
    %         EXTA15 = 'EXT A15';             % External ADC A15 for Shimmer3
    %         STRAIN = 'Strain Gauge';        % Strain Gauge for Shimmer2r
    %         BRIDGE = 'Bridge Amplifier';    % Bridge Amplifier for Shimmer3
    %         HEART = 'Heart Rate';           % Heart Rate for Shimmer2r
    %         BATT = 'BattVolt';              % Battery Voltage
    %         INTA1 = 'INT A1';               % Internal ADC for Shimmer3
    %         INTA12 = 'INT A12';             % Internal ADC for Shimmer3
    %         INTA13 = 'INT A13';             % Internal ADC for Shimmer3
    %         INTA14 = 'INT A14';             % Internal ADC for Shimmer3
    %         PRESSURE = 'Pressure';          % BMP180 Pressure (and Temperature) for Shimmer3
    %
    % See also ShimmerHandleClass.setenabledsensors
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties (Constant = true)
        ACCEL='Accel';                  % Accelerometer; for Shimmer3 Low Noise Accelerometer will be selected.
        LNACCEL='LowNoiseAccel';        % Low Noise Accelerometer for Shimmer3
        WRACCEL='WideRangeAccel';       % Wide Range Accelerometer for Shimmer3
        ALTACCEL='AlternativeAccel';    % MPU9150 Accelerometer for Shimmer3
        GYRO = 'Gyro';                  % Gyroscope
        MAG = 'Mag';                    % Magnetometer
        ALTMAG = 'AlternativeMag';      % MPU9150 Magnetometer for Shimmer3
        ECG = 'ECG';                    % ECG
        ECG24BIT ='ECG 24BIT';          % ECG 24BIT for Shimmer3
        ECG16BIT ='ECG 16BIT';          % ECG 16BIT for Shimmer3
        EMG = 'EMG';                    % EMG
        EMG24BIT = 'EMG 24BIT';         % EMG 24BIT for Shimmer3
        EMG16BIT = 'EMG 16BIT';         % EMG 16BIT for Shimmer3
        EXG1 = 'EXG1';                  % EXG1 for Shimmer3
        EXG124BIT = 'EXG1 24BIT';       % EXG1 24BIT for Shimmer3
        EXG116BIT = 'EXG1 16BIT';       % EXG1 16BIT for Shimmer3
        EXG2 = 'EXG2';                  % EXG2 for Shimmer3
        EXG224BIT = 'EXG2 24BIT';       % EXG2 24BIT for Shimmer3
        EXG216BIT = 'EXG2 16BIT';       % EXG2 16BIT for Shimmer3
        GSR = 'GSR';                    % GSR
        EXPA0 = 'ExpBoard_A0';          % External Expansion Board A0 for Shimmer2r
        EXPA7 = 'ExpBoard_A7';          % External Expansion Board A7 for Shimmer2r
        EXTA7 = 'EXT A7';               % External ADC A7 for Shimmer3
        EXTA6 = 'EXT A6';               % External ADC A6 for Shimmer3
        EXTA15 = 'EXT A15';             % External ADC A15 for Shimmer3
        STRAIN = 'Strain Gauge';        % Strain Gauge for Shimmer2r
        BRIDGE = 'Bridge Amplifier';    % Bridge Amplifier for Shimmer3
        HEART = 'Heart Rate';           % Heart Rate for Shimmer2r
        BATT = 'BattVolt';              % Battery Voltage
        INTA1 = 'INT A1';               % Internal ADC for Shimmer3
        INTA12 = 'INT A12';             % Internal ADC for Shimmer3
        INTA13 = 'INT A13';             % Internal ADC for Shimmer3
        INTA14 = 'INT A14';             % Internal ADC for Shimmer3
        PRESSURE = 'Pressure';          % BMP180 Pressure (and Temperature) for Shimmer3
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Constructor Method
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        
        function theseMacros = SetEnabledSensorsMacrosClass
            
            % SETENABLEDSENSORSMACROSCLASS - Constructor for the class
            %
            %   SETENABLEDSENSORSMACROSCLASS creates a handle to an
            %   instance of the SetEnabledSensorsMacrosClass. 
            %
            %   SYNOPSIS: theseMacros = SetEnabledSensorsMacrosClass
            %
            %   OUTPUT: theseMacros - handle to the instance of the
            %                                SetEnabledSensorsMacrosClass
            %
            %   EXAMPLE: theseMacros = SetEnabledSensorsMacrosClass;
            %
        
        end % function SetEnabledSensorsMacrosClass  
        
    end % methods (Constructor)
end