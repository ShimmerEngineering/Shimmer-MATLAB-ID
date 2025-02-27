function void = newplotandwritepressureandtemperatureexample(comPort, captureDuration, fileName)
%NEWPLOTANDWRITEPRESSUREANDTEMPERATUREEXAMPLE - Plotting pressure and temperature data and write to file
%
%  newplotandwritepressureandtemperatureexample(COMPORT, CAPTUREDURATION, FILENAME) 
%
%  SYNOPSIS: newplotandwritepressureandtemperatureexample(comPort, captureDuration,
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
%  EXAMPLE: newplotandwritepressureandtemperatureexample('7', 30, 'testdata.dat')
%
%  See also newplotandwriteexample ShimmerDeviceHandler

%% definitions

shimmer = ShimmerDeviceHandler(comPort);                                   % Define shimmer as a ShimmerHandle Class instance with comPort1
firsttime = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 500;                                                  % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations
numSamples = 0;

addpath('./Resources/') 

%%

[success, obj] = shimmer.connect();

if success

    shimmerClone = obj.shimmer.deepClone();
    shimmerClone.setSamplingRateShimmer(51.2);
    shimmerClone.disableAllSensors()
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0,0);
    
    sensorIds = javaArray('java.lang.Integer', 1);
    sensorIds(1) = java.lang.Integer(obj.sensorClass.SHIMMER_BMPX80_PRESSURE);

    shimmerClone.setSensorIdsEnabled(sensorIds);

    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    obj.shimmer.configureFromClone(shimmerClone);

    pause(20);

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
            
            signalFormatCellArray = cell(numel(signalFormatArray), 1);     
            for i = 1:numel(signalFormatArray)
                signalFormatCellArray{i} = char(signalFormatArray(i));     % Convert each Java string to a MATLAB char array
            end
            
            signalUnitCellArray = cell(numel(signalUnitArray), 1);     
            for i = 1:numel(signalUnitArray)
                signalUnitCellArray{i} = char(signalUnitArray(i));         % Convert each Java string to a MATLAB char array
            end
            
            if(~isempty(signalNameCellArray))
                 chIndex(1) = find(ismember(signalNameCellArray, 'Pressure_BMP180'));   % get signal indices
                 chIndex(2) = find(ismember(signalNameCellArray, 'Temperature_BMP180'));% get signal indices
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
        fprintf('The percentage of received packets: %d \n',obj.shimmer.getPacketReceptionRateCurrent()); % Detect loss packets
        obj.shimmer.stopStreaming();                                       % Stop data streaming                                                    
       
    end 
    obj.shimmer.disconnect();  
end