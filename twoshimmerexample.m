function void = twoshimmerexample(comPort, comPort2 , captureDuration)
%PLOTANDWRITEEXAMPLE - Demonstrate basic features of ShimmerHandleClass
%
%  PLOTANDWRITEEXAMPLE(COMPORT, CAPTUREDURATION, FILENAME) plots 3 
%  accelerometer signals, 3 gyroscope signals and 3 magnetometer signals,
%  from the Shimmer paired with COMPORT. The function 
%  will stream data for a fixed duration of time defined by the constant 
%  CAPTUREDURATION. The function also writes the data in a tab ddelimited 
%  format to the file defined in FILENAME.
%
%  SYNOPSIS: plotandwriteexample(comPort, captureDuration, fileName)
%
%  INPUT: comPort - String value defining the COM port number for Shimmer
%  INPUT: captureDuration - Numerical value defining the period of time 
%                           (in seconds) for which the function will stream 
%                           data from  the Shimmers.
%  INPUT : fileName - String value defining the name of the file that data 
%                     is written to in a comma delimited format.
%  OUTPUT: none
%
%  EXAMPLE: plotandwriteexample('COM3', 30, 'testdata.dat')
%
%  See also ShimmerDeviceHandler

%% definitions

deviceHandler = ShimmerDeviceHandler();                                   % Define a handler 
firsttime = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerDevice Handler
NO_SAMPLES_IN_PLOT = 500;                                                  % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations
numSamples = 0;

addpath('./Resources/')                                                    % directory containing supporting functions

%%
deviceHandler.bluetoothManager.connectShimmerThroughCommPort(comPort);
deviceHandler.bluetoothManager.connectShimmerThroughCommPort(comPort2);
cleaner1 = onCleanup(@() deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).disconnect());  % Ensure disconnection on cleanup
cleaner2 = onCleanup(@() deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).disconnect());  % Ensure disconnection on cleanup
pause(10);
% Ensure disconnection happens properly even if the workspace is cleared or the script is interrupted

if deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).isConnected() && deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).isConnected()
    shimmerClone = deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).deepClone();
    shimmerClone.setSamplingRateShimmer(51.2);
    
    shimmerClone.disableAllSensors();                                      % Disables all currently enabled sensors
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);           % Resets configuration on enabled and derived sensors
    
    sensorIds = javaArray('java.lang.Integer', 3);
    sensorIds(1) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_ANALOG_ACCEL);
    sensorIds(2) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_MPU9X50_GYRO);
    sensorIds(3) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_LSM303_MAG);
    hwid = shimmerClone.getHardwareVersionParsed();
    if hwid.equals('Shimmer3R')
        sensorIds(1) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_LSM6DSV_ACCEL_LN);
        sensorIds(2) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_LSM6DSV_GYRO);
        sensorIds(3) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_LIS2MDL_MAG);
    end
    shimmerClone.setSensorIdsEnabled(sensorIds);

    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).configureFromClone(shimmerClone);

    %%

    shimmer2Clone = deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).deepClone();
    shimmer2Clone.setSamplingRateShimmer(51.2);
    
    shimmer2Clone.disableAllSensors();                                      % Disables all currently enabled sensors
    shimmer2Clone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);           % Resets configuration on enabled and derived sensors

    sensorIds = javaArray('java.lang.Integer', 3);
    sensorIds(1) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_ANALOG_ACCEL);
    sensorIds(2) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_MPU9X50_GYRO);
    sensorIds(3) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_LSM303_MAG);
    hwid = shimmer2Clone.getHardwareVersionParsed();
    if hwid.equals('Shimmer3R')
        sensorIds(1) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_LSM6DSV_ACCEL_LN);
        sensorIds(2) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_LSM6DSV_GYRO);
        sensorIds(3) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_LIS2MDL_MAG);
    end


    shimmer2Clone.setSensorIdsEnabled(sensorIds);

    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmer2Clone, commType);
    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).configureFromClone(shimmer2Clone);


    pause(20);
    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).startStreaming()
    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).startStreaming()
    
        elapsedTime = 0;                                                   % Reset to 0    
        tic;                                                               % Start timer
        toggle = true;
        while (elapsedTime < captureDuration)            
                      
            pause(DELAY_PERIOD);    
            % Pause for this period of time on each iteration to allow data to arrive in the buffer
            comPortUsed = comPort;
            if (toggle)
                data = deviceHandler.obj.receiveData(comPort);                                  % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            else
                comPortUsed=comPort2;
                data = deviceHandler.obj.receiveData(comPort2);                                  % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            end
            toggle = ~toggle;
            if (isempty(data))
                continue;
            end
            newData = data(1);
            signalNameArray = data(2);
            signalFormatArray = data(3);
            signalUnitArray = data(4);     
            
            signalNameCellArray = cell(numel(signalNameArray), 1);     
            for i = 1:numel(signalNameArray)
                signalNameCellArray{i} = char(signalNameArray(i));         % Convert each Java string to a MATLAB char array
            end
            
            signalFormatCellArray = cell(numel(signalFormatArray), 1);     
            for i = 1:numel(signalFormatArray)
                signalFormatCellArray{i} = char(signalFormatArray(i));     % Convert each Java string to a MATLAB char array
            end
            
            signalUnitCellArray = cell(numel(signalUnitArray), 1);     
            for i = 1:numel(signalUnitArray)
                signalUnitCellArray{i} = char(signalUnitArray(i));         % Convert each Java string to a MATLAB char array
            end
            
            if(~isempty(signalNameCellArray))
                chIndex(1) = find(ismember(signalNameCellArray, 'Timestamp')); % Get signal indices
                chIndex(2) = find(ismember(signalNameCellArray, 'Accel_LN_X'));
                chIndex(3) = find(ismember(signalNameCellArray, 'Accel_LN_Y'));
                chIndex(4) = find(ismember(signalNameCellArray, 'Accel_LN_Z'));
                chIndex(5) = find(ismember(signalNameCellArray, 'Gyro_X'));
                chIndex(6) = find(ismember(signalNameCellArray, 'Gyro_Y'));
                chIndex(7) = find(ismember(signalNameCellArray, 'Gyro_Z'));
                chIndex(8) = find(ismember(signalNameCellArray, 'Mag_X'));
                chIndex(9) = find(ismember(signalNameCellArray, 'Mag_Y'));
                chIndex(10) = find(ismember(signalNameCellArray, 'Mag_Z'));
            end

            if ~isempty(newData)                                           % TRUE if new data has arrived
                % Example numeric column
                colData = newData(:, chIndex(2));   % 10×1 column
                
                % Replicate the string into a cell array with same number of rows
                nRows = size(colData, 1);
                comPortCol = repmat({comPortUsed}, nRows, 1);  
                
                % Combine numeric and string columns
                combined = [num2cell(colData), comPortCol];
                
                % Display
                disp('Data + COM Port Label:');
                disp(combined);

            end
            
            elapsedTime = elapsedTime + toc;                               % Stop timer and add to elapsed time
            tic;                                                           % Start timer           
            
        end  
        
        elapsedTime = elapsedTime + toc;                                   % Stop timer
        fprintf('The percentage of received packets: %d \n',deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).getPacketReceptionRateCurrent()); % Detect loss packets
        fprintf('The percentage of received packets: %d \n',deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).getPacketReceptionRateCurrent()); % Detect loss packets
        deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).stopStreaming();                                       % Stop data streaming                                                       % Stop data streaming             
        deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).stopStreaming();                                       % Stop data streaming                                                       % Stop data streaming             
    
    
    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).disconnect();
    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).disconnect();
    
end

       
end
