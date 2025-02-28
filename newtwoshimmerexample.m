function newtwoshimmerexample(comPort1, comPort2, captureDuration)

%NEWTWOSHIMMEREXAMPLE - Demonstrate basic features of ShimmerHandleClass
%
%  NEWTWOSHIMMEREXAMPLE(COMPORT1, COMPORT2, CAPTUREDURATION) plots the 3
%  accelerometer signals from the Shimmer paired with COMPORT1 and plots
%  the 3 accelerometer signals from the Shimmer paired with COMPORT2. The
%  function will stream data for a fixed duration of time defined by the
%  constant CAPTUREDURATION.
%
%  SYNOPSIS: newtwoshimmerexample(comPort1, comPort2, captureDuration)
%
%  INPUT: comPort1 - string value defining the COM port numbers for Shimmer 1
%  INPUT: comPort2 - string value defining the COM port numbers for Shimmer 2
%  INPUT: captureDuration - numerical value defining the period of time 
%                        (in seconds) for which the function will stream 
%                        data from  the Shimmers.
%  
%  EXAMPLE: newtwoshimmerexample('COM7','COM8',30)
%
%  See also newplotandwriteexample ShimmerDeviceHandler


% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 500;                                                  % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations

%%

shimmer1 = ShimmerDeviceHandler(comPort1);                                   % Define shimmer1 as a ShimmerHandle Class instance with comPort1
shimmer2 = ShimmerDeviceHandler(comPort2);                                   % Define shimmer2 as a ShimmerHandle Class instance with comPort2

[success1, obj1] = shimmer1.connect();
[success2, obj2] = shimmer2.connect();

pause(20);

if (success1 && success2)                                                  % TRUE if both shimmer 1 and shimmer 2 connect
    
    sensorIds = javaArray('java.lang.Integer', 1);
    sensorIds(1) = java.lang.Integer(obj1.sensorClass.SHIMMER_ANALOG_ACCEL);
    
    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    
    % Define settings for shimmer1
    shimmerClone = obj1.shimmer.deepClone();
    shimmerClone.setSamplingRateShimmer(51.2);
    
    shimmerClone.disableAllSensors();                                      % Disables all currently enabled sensors
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);           % Resets configuration on enabled and derived sensors

    shimmerClone.setSensorIdsEnabled(sensorIds);

    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    obj1.shimmer.configureFromClone(shimmerClone);

    pause(20);
    
    % Define settings for shimmer2
    shimmerClone = obj2.shimmer.deepClone();
    shimmerClone.setSamplingRateShimmer(51.2);
    
    shimmerClone.disableAllSensors();                                      % Disables all currently enabled sensors
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);           % Resets configuration on enabled and derived sensors

    shimmerClone.setSensorIdsEnabled(sensorIds);

    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    obj2.shimmer.configureFromClone(shimmerClone);
    
    pause(20);
    
    if (shimmer1.start && shimmer2.start)                                  % TRUE if both shimmers start streaming
        
        calibDataShimmer1 = []; 
        calibDataShimmer2 = [];
        
        h.figure1=figure('Name','Shimmer 1');                              % Create a handle to figure for plotting data from shimmer1
        h.figure2=figure('Name','Shimmer 2');                              % Create a handle to figure for plotting data from shimmer2
        
        elapsedTime = 0;                                                   % Reset to 0
        
        tic;                                                               % Start timer
        
        while (elapsedTime < captureDuration)            
                      
            pause(DELAY_PERIOD);                                           % Pause for this period of time on each iteration to allow data to arrive in the buffer
            
            
            % Read and plot data for shimmer1
            data = obj1.obj.receiveData();
            calibratedData = data(1);
            signalName = data(2);
            signalFormat = data(3);
            signalUnit = data(4);     
            
            calibDataShimmer1 = [calibDataShimmer1; calibratedData];       % Read the uncalibrated data for shimmer1 and add to previous data   
            
            signalNameCellArray = cell(numel(signalName), 1);     
            for i = 1:numel(signalName)
                signalNameCellArray{i} = char(signalName(i));              % Convert each Java string to a MATLAB char array
            end
            
            if(length(calibDataShimmer1) > NO_SAMPLES_IN_PLOT)                 
               calibDataShimmer1=calibDataShimmer1((length(calibDataShimmer1) - NO_SAMPLES_IN_PLOT):end, :);   % Trim excess previous data from array for plotting purposes
            end
           
            if (isempty(calibDataShimmer1)~=1)
                if (obj1.shimmer.getHardwareVersion() == 3)                % get signal indices for Shimmer3
                    chIndex(1) = find(ismember(signalNameCellArray, 'Accel_LN_X'));
                    chIndex(2) = find(ismember(signalNameCellArray, 'Accel_LN_Y'));
                    chIndex(3) = find(ismember(signalNameCellArray, 'Accel_LN_Z'));
                elseif (obj1.shimmer.getHardwareVersion() < 3)                     % get signal indices for Shimmer2/2r
                    chIndex(1) = find(ismember(signalNameCellArray, 'Accel_X'));
                    chIndex(2) = find(ismember(signalNameCellArray, 'Accel_Y'));
                    chIndex(3) = find(ismember(signalNameCellArray, 'Accel_Z'));
                end
                accelDataShimmer1 = [calibDataShimmer1(:,chIndex(1)), calibDataShimmer1(:,chIndex(2)), calibDataShimmer1(:,chIndex(3))]; % Extract only the columns of accelerometer data
                set(0,'CurrentFigure',h.figure1);           
                plot(accelDataShimmer1);                                       % Plot the accelerometer data
                title('Shimmer 1 Accelerometer Data');                         % Add title to the plot
                % axis([0 NO_SAMPLES_IN_PLOT 0 4095]);                           % Define min and max values for axis , Shimmer 3 has an int16 for the digital accel so users can change accordingly
                legend(char(signalNameCellArray(chIndex(1))),char(signalNameCellArray(chIndex(2))),char(signalNameCellArray(chIndex(3))))                          % Add legend to plot

            end
            

            % Read and plot data for shimmer2       
            data = obj2.obj.receiveData();
            calibratedData = data(1);
            signalName = data(2);
            signalFormat = data(3);
            signalUnit = data(4);  
            
            calibDataShimmer2 = [calibDataShimmer2; calibratedData];       % Read the uncalibrated data for shimmer2 and add to previous data
            
            signalNameCellArray = cell(numel(signalName), 1);     
            for i = 1:numel(signalName)
                signalNameCellArray{i} = char(signalName(i));              % Convert each Java string to a MATLAB char array
            end
            
            if(length(calibDataShimmer2) > NO_SAMPLES_IN_PLOT)
               calibDataShimmer2 = calibDataShimmer2((length(calibDataShimmer2) - NO_SAMPLES_IN_PLOT):end, :);   % Trim excess previous data from array for plotting purposes
            end
                    
            if (isempty(calibDataShimmer2)~=1)
                if (obj2.shimmer.getHardwareVersion() == 3)                    % get signal indices for Shimmer3
                    chIndex2(1) = find(ismember(signalNameCellArray, 'Accel_LN_X'));
                    chIndex2(2) = find(ismember(signalNameCellArray, 'Accel_LN_Y'));
                    chIndex2(3) = find(ismember(signalNameCellArray, 'Accel_LN_Z'));
                elseif (obj2.shimmer.getHardwareVersion() < 3)                 % get indices for Shimmer2/2r
                    chIndex2(1) = find(ismember(signalNameCellArray, 'Accel_X'));
                    chIndex2(2) = find(ismember(signalNameCellArray, 'Accel_Y'));
                    chIndex2(3) = find(ismember(signalNameCellArray, 'Accel_Z'));
                end
                accelDataShimmer2 = [calibDataShimmer2(:,chIndex2(1)), calibDataShimmer2(:,chIndex2(2)), calibDataShimmer2(:,chIndex2(3))]; % Extract only the columns of gyroscope data
                set(0,'CurrentFigure',h.figure2);                     
                plot(accelDataShimmer2);                                   % Plot the gyroscope data
                title('Shimmer 2 Accelerometer Data');                     % Add title to the plot
                % axis([0 NO_SAMPLES_IN_PLOT 0 4095]);                       % Define min and max values for axis , Shimmer 3 has an int16 for the digital accel so users can change accordingly
                legend(char(signalNameCellArray(chIndex2(1))),char(signalNameCellArray(chIndex2(2))),char(signalNameCellArray(chIndex2(3))))                              % Add legend to plot
            end       
            elapsedTime = elapsedTime + toc;                               % Stop timer and add to elapsed time
            tic;                                                           % Start timer
            
        end  
        
        elapsedTime = elapsedTime + toc;                                   % Stop timer
        
        obj1.shimmer.stopStreaming();                                                     % Stop data streaming from shimmer1                                                    
        obj2.shimmer.stopStreaming();                                                     % Stop data streaming from shimmer2
        
    else
        
        obj1.shimmer.stopStreaming();                                                     % Stop data streaming from shimmer1 (if it has started streaming)                                                
        obj2.shimmer.stopStreaming();                                                     % Stop data streaming from shimmer2 (if it has started streaming)  
        
    end            
    
    obj1.shimmer.disconnect();                                                   % Disconnect from shimmer1
    obj2.shimmer.disconnect();                                                   % Disconnect from shimmer2    
    
end

clear all;                                                                 % Remove all variables from memory

