function void = newplotandwritepressureandtemperatureexample(comPort, captureDuration, fileName)
%% definitions

shimmer = ShimmerDeviceHandler(comPort);                                   % Define shimmer as a ShimmerHandle Class instance with comPort1
firsttime = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 500;                                                  % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations
numSamples = 0;

%%

[success, obj] = shimmer.connect();

if success
    
    if (obj.shimmer.getHardwareVersion() == 10)
        pressureSensorId = 43;
        pressureSensor = 'BMP390_Pressure';
        temperatureSensor = 'BMP390_Temperature';
    else
        pressureSensorId = 36;
        pressureSensor = 'Pressure_BMP180';
        temperatureSensor = 'Temperature_BMP180';
    end

    shimmerClone = obj.shimmer.deepClone();
    shimmerClone.setSamplingRateShimmer(51.2);
    shimmerClone.disableAllSensors()
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0,0);
    shimmerClone.setSensorEnabledState(pressureSensorId, true);

    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    obj.shimmer.configureFromClone(shimmerClone);

    btState = javaMethod('valueOf', 'com.shimmerresearch.bluetooth.ShimmerBluetooth$BT_STATE', 'CONFIGURING');
    obj.shimmer.operationStart(btState);
    pause(25);

    if shimmer.start()
        
        plotData = [];                                               
        timeStamp = [];
        
        h.figure1=figure('Name','Shimmer 1 signals');                      % Create a handle to figure for plotting data from shimmer
        set(h.figure1, 'Position', [100, 500, 800, 400]);
        
        elapsedTime = 0;                                                   % Reset to 0    
        tic;                                                               % Start timer
        
        while (elapsedTime < captureDuration)            
                      
            pause(DELAY_PERIOD);                                           % Pause for this period of time on each iteration to allow data to arrive in the buffer
            
            data = obj.obj.receiveData();                                  % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            newData = data(1);
            signalNameArray = data(2);
            signalFormatArray = data(3);
            signalUnitArray = data(4);  
            
            signalNameCellArray = cell(numel(signalNameArray), 1);     
            for i = 1:numel(signalNameArray)
                signalNameCellArray{i} = char(signalNameArray(i));         % Convert each Java string to a MATLAB char array
            end
            
            
            if (firsttime==true && isempty(newData)~=1)
                %firsttime = writeHeadersToFile(fileName,signalNameArray,signalFormatArray,signalUnitArray);
            end

            
            if ~isempty(newData)                                           % TRUE if new data has arrived
                                
                dlmwrite(fileName, newData, '-append', 'delimiter', '\t'); % Append the new data to the file in a tab delimited format
                            
                plotData = [plotData; newData];                            % Update the plotDataBuffer with the new data
                numPlotSamples = size(plotData,1);
                numSamples = numSamples + size(newData,1);
                timeStampNew = newData(:,4);                               % get timestamps
                timeStamp = [timeStamp; timeStampNew];
                
                 if numSamples > NO_SAMPLES_IN_PLOT
                        plotData = plotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                 end
                 sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;
                 
               
                 chIndex(1) = find(ismember(signalNameCellArray, pressureSensor));   % get signal indices
                 chIndex(2) = find(ismember(signalNameCellArray, temperatureSensor));% get signal indices

                             
                 
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
        fprintf('The percentage of received packets: %d \n',getpercentageofpacketsreceived(51.2,timeStamp)); % Detect loss packets
        obj.shimmer.stopStreaming();                                       % Stop data streaming                                                    
       
    end 
    obj.shimmer.disconnect();  
end