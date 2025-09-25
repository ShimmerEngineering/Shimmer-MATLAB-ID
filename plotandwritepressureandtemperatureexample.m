function void = plotandwritepressureandtemperatureexample(comPort, captureDuration, fileName)
%PLOTANDWRITEPRESSUREANDTEMPERATUREEXAMPLE - Plotting pressure and temperature data and write to file
%
%  plotandwritepressureandtemperatureexample(COMPORT, CAPTUREDURATION, FILENAME) 
%
%  SYNOPSIS: plotandwritepressureandtemperatureexample(comPort, captureDuration,
%  fileName)
%
%  INPUT: comPort - String value defining the COM port number for Shimmer
%
%  INPUT: captureDuration - Numerical value defining the period of time
%                           (in seconds) for which the function will stream
%                           data from  the Shimmers.
%  INPUT : fileName - String value defining the name of the file that data
%                     is written to in a comma delimited format.
%  OUTPUT: none
%
%  EXAMPLE: plotandwritepressureandtemperatureexample('7', 30, 'testdata.dat')
%
%  See also plotandwriteexample ShimmerDeviceHandler

%% definitions

deviceHandler = ShimmerDeviceHandler();                                   % Define a handler
firsttime = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 500;                                                  % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations
numSamples = 0;

addpath('./Resources/') 

%%
deviceHandler.bluetoothManager.connectShimmerThroughCommPort(comPort);
cleaner = onCleanup(@() deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).disconnect());  % Ensure disconnection on cleanup
pause(10);

if deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).isConnected()

    shimmerClone = deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).deepClone();
    shimmerClone.setSamplingRateShimmer(51.2);
    shimmerClone.disableAllSensors()
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0,0);
    
    sensorIds = javaArray('java.lang.Integer', 1);
    sensorIds(1) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_BMPX80_PRESSURE);
    hwid = shimmerClone.getHardwareVersionParsed();
    if hwid.equals('Shimmer3R')
        sensorIds(1) = java.lang.Integer(deviceHandler.sensorClass.SHIMMER_BMP390_PRESSURE);
    end
    shimmerClone.setSensorIdsEnabled(sensorIds);

    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).configureFromClone(shimmerClone);

    pause(20);

     deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).startStreaming()
        
        plotData = [];                                               
        timeStamp = [];
        
        h.figure1=figure('Name','Shimmer 1 signals');                      % Create a handle to figure for plotting data from shimmer
        set(h.figure1, 'Position', [100, 500, 800, 400]);
        
        elapsedTime = 0;                                                   % Reset to 0    
        tic;                                                               % Start timer
        
        while (elapsedTime < captureDuration)            
                      
            pause(DELAY_PERIOD);                                           % Pause for this period of time on each iteration to allow data to arrive in the buffer
            
            data = deviceHandler.obj.receiveData(comPort);                                  % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
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
                     chIndex(1) = find(startsWith(signalNameCellArray, 'Pressure'));
                     chIndex(2) = find(startsWith(signalNameCellArray, 'Temperature'));
            end
            
            if (firsttime==true && isempty(newData)~=1)
                firsttime = newWriteHeadersToFile(fileName,signalNameCellArray(chIndex),signalFormatCellArray(chIndex),signalUnitCellArray(chIndex));
            end
            
            if ~isempty(newData)                                           % TRUE if new data has arrived
                                
                filtredData = newData(:, chIndex);
                
                dlmwrite(fileName, double(filtredData), '-append', 'delimiter', '\t', 'precision', 16);
                            
                plotData = [plotData; newData];                            % Update the plotDataBuffer with the new data
                numPlotSamples = size(plotData,1);
                numSamples = numSamples + size(newData,1);
                timeStampNew = newData(:,4);                               % get timestamps
                timeStamp = [timeStamp; timeStampNew];
                
                if numSamples > NO_SAMPLES_IN_PLOT
                    plotData = plotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                end
                sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;

                set(0,'CurrentFigure',h.figure1);       
                signalIndex = chIndex(1);
                signalIndex2 = chIndex(2);
                subplot(2,1,1);                                            % Create subplot
                plot(sampleNumber, plotData(:,signalIndex));               % Plot data
                legend([char(signalFormatArray(signalIndex)) ' ' char(signalNameArray(signalIndex)) ' (' char(signalUnitArray(signalIndex)) ')']);   
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim('auto');
                
                subplot(2,1,2);  
                plot(sampleNumber, plotData(:,signalIndex2));              % Plot data
                legend([char(signalFormatArray(signalIndex2)) ' ' char(signalNameArray(signalIndex2)) ' (' char(signalUnitArray(signalIndex2)) ')']);   
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim('auto');
               
            end
            
            elapsedTime = elapsedTime + toc;                               % Stop timer and add to elapsed time
            tic;                                                           % Start timer           
            
        end  
        
        elapsedTime = elapsedTime + toc;                                   % Stop timer
        fprintf('The percentage of received packets: %d \n',deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).getPacketReceptionRateCurrent()); % Detect loss packets
        deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).stopStreaming();                                       % Stop data streaming                                                       % Stop data streaming
        deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).disconnect();
end