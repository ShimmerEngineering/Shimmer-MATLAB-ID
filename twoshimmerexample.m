function void = twoshimmerexample(comPort, comPort2, captureDuration)
%TWOSHIMMEREXAMPLE - Demonstrate streaming from two Shimmers simultaneously
%
%  TWOSHIMMEREXAMPLE(COMPORT, COMPORT2, CAPTUREDURATION) connects to and
%  streams accelerometer, gyroscope and magnetometer data from the two
%  Shimmers paired with COMPORT and COMPORT2. The function will stream
%  data for a fixed duration of time defined by the constant
%  CAPTUREDURATION, printing information on the data received from each
%  device to the console.
%
%  SYNOPSIS: twoshimmerexample(comPort, comPort2, captureDuration)
%
%  INPUT: comPort - String value defining the COM port number for the
%                   first Shimmer
%  INPUT: comPort2 - String value defining the COM port number for the
%                    second Shimmer
%  INPUT: captureDuration - Numerical value defining the period of time
%                           (in seconds) for which the function will stream
%                           data from  the Shimmers.
%  OUTPUT: none
%
%  EXAMPLE: twoshimmerexample('COM3', 'COM4', 30)
%
%  See also ShimmerDeviceHandler

%% definitions

deviceHandler = ShimmerDeviceHandler();                                   % Define a handler
configuredcom1 = 0;
configuredcom2 = 0;
CP1 = comPort;
CP2 = comPort2;

% Note: this constant is only relevant to this examplescript and is not used
% by the ShimmerDevice Handler
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations

addpath('./Resources/')                                                    % directory containing supporting functions

%%
deviceHandler.bluetoothManager.setVerbose(false);
deviceHandler.bluetoothManager.connectShimmerThroughCommPort(comPort);
deviceHandler.bluetoothManager.connectShimmerThroughCommPort(comPort2);
cleaner1 = onCleanup(@() deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).disconnect());  % Ensure disconnection on cleanup
cleaner2 = onCleanup(@() deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).disconnect());  % Ensure disconnection on cleanup
addlistener(deviceHandler, 'DeviceConnected', @(src,evt) onConnected(src, evt));
addlistener(deviceHandler, 'DeviceDisconnected',    @(src,evt) disp("Script: Disconnected"));
addlistener(deviceHandler, 'DeviceConnectionLost',  @(src,evt) disp("Script: Lost connection"));

while(isempty(deviceHandler.obj.receiveData(comPort)) || isempty(deviceHandler.obj.receiveData(comPort2)))                                  % we wait here for both devices to start streaming
    pause(0.1);
end
elapsedTime = 0;                                                   % Reset to 0
tic;
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

fprintf('The percentage of received packets: %.2f \n',deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).getPacketReceptionRateOverall()); % Detect loss packets
fprintf('The percentage of received packets: %.2f \n',deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).getPacketReceptionRateOverall()); % Detect loss packets
deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).stopStreaming();                                       % Stop data streaming                                                       % Stop data streaming
deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).stopStreaming();                                       % Stop data streaming                                                       % Stop data streaming


deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).disconnect();
deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort2).disconnect();

    function onConnected(deviceHandler, evt)
         if (strcmp(CP1,evt.ComPort))
            disp("Script: Connected");
            if (configuredcom1==1) % a connected state is also triggered after configuring, so this differentiates the two
                deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(evt.ComPort).startStreaming();
                return
            end

            shimmerClone = deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(evt.ComPort).deepClone();
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
            deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(evt.ComPort).configureFromClone(shimmerClone);
            configuredcom1 = configuredcom1 + 1;
        end
        if (strcmp(CP2,evt.ComPort))
            disp("Script: Connected");
            if (configuredcom2==1) % a connected state is also triggered after configuring, so this differentiates the two
                deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(evt.ComPort).startStreaming();
                return
            end
            shimmer2Clone = deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(evt.ComPort).deepClone();
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
            deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(evt.ComPort).configureFromClone(shimmer2Clone);
            configuredcom2 = configuredcom2 + 1;
        end

    end


end


