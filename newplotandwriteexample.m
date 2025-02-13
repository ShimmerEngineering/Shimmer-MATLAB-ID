function void = newplotandwriteexample(comPort, captureDuration, fileName)
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

    shimmer.enabledsensors(51.2, {'LowNoiseAccel', 'Mag', 'Gyro'})

    if shimmer.start()
        plotData = [];                                               
        timeStamp = [];

        elapsedTime = 0;                                                             
        tic;  

        plotData = [];                                               
        timeStamp = [];
        
        h.figure1=figure('Name','Shimmer 1 signals' );                     % Create a handle to figure for plotting data from shimmer
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
       
            
            if (firsttime==true && isempty(newData)~=1)
                % firsttime = writeHeadersToFile(fileName,signalNameArray,signalFormatArray,signalUnitArray);
            end
            
            
            if ~isempty(newData)                                           % TRUE if new data has arrived
                
                dlmwrite(fileName, newData, '-append', 'delimiter', '\t','precision',16); % Append the new data to the file in a tab delimited format
                
                plotData = [plotData; newData];                            % Update the plotDataBuffer with the new data
                numPlotSamples = size(plotData,1);
                numSamples = numSamples + size(newData,1);
                timeStampNew = newData(:,11);                              % get timestamps
                timeStamp = [timeStamp; timeStampNew];
                
                 if numSamples > NO_SAMPLES_IN_PLOT
                        plotData = plotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                 end
                 sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;
                 
                signalNameCellArray = cell(numel(signalNameArray), 1);     
                for i = 1:numel(signalNameArray)
                    signalNameCellArray{i} = char(signalNameArray(i));     % Convert each Java string to a MATLAB char array
                end
               
                chIndex(1) = find(ismember(signalNameCellArray, 'Timestamp'));  % Get signal indices
                chIndex(2) = find(ismember(signalNameCellArray, 'Accel_LN_X'));
                chIndex(3) = find(ismember(signalNameCellArray, 'Accel_LN_Y'));
                chIndex(4) = find(ismember(signalNameCellArray, 'Accel_LN_Z'));
                chIndex(5) = find(ismember(signalNameCellArray, 'Gyro_X'));
                chIndex(6) = find(ismember(signalNameCellArray, 'Gyro_Y'));
                chIndex(7) = find(ismember(signalNameCellArray, 'Gyro_Z'));
                chIndex(8) = find(ismember(signalNameCellArray, 'Mag_X'));
                chIndex(9) = find(ismember(signalNameCellArray, 'Mag_Y'));
                chIndex(10) = find(ismember(signalNameCellArray, 'Mag_Z'));
                              
                 
                set(0,'CurrentFigure',h.figure1);           
                subplot(2,2,1);                                            % Create subplot
                signalIndex = chIndex(1);
                plot(sampleNumber, plotData(:,signalIndex));               % Plot the time stamp data
                legend([char(signalFormatArray(signalIndex)) ' ' char(signalNameArray(signalIndex)) ' (' char(signalUnitArray(signalIndex)) ')']);  
                xlim([sampleNumber(1) sampleNumber(end)]);

                subplot(2,2,2);                                            % Create subplot
                signalIndex1 = chIndex(2);
                signalIndex2 = chIndex(3);
                signalIndex3 = chIndex(4);
                plot(sampleNumber, plotData(:,[signalIndex1 signalIndex2 signalIndex3]));                                 % Plot the accelerometer data
                legendName1 = [char(signalFormatArray(signalIndex1)) ' ' char(signalNameArray(signalIndex1)) ' (' char(signalUnitArray(signalIndex1)) ')'];
                legendName2 = [char(signalFormatArray(signalIndex2)) ' ' char(signalNameArray(signalIndex2)) ' (' char(signalUnitArray(signalIndex2)) ')'];
                legendName3 = [char(signalFormatArray(signalIndex3)) ' ' char(signalNameArray(signalIndex3)) ' (' char(signalUnitArray(signalIndex3)) ')'];
                legend(legendName1,legendName2,legendName3);               % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);

                subplot(2,2,3);                                            % Create subplot
                signalIndex1 = chIndex(5);
                signalIndex2 = chIndex(6);
                signalIndex3 = chIndex(7);
                plot(sampleNumber, plotData(:,[signalIndex1 signalIndex2 signalIndex3]));                                 % Plot the gyroscope data
                legendName1 = [char(signalFormatArray(signalIndex1)) ' ' char(signalNameArray(signalIndex1)) ' (' char(signalUnitArray(signalIndex1)) ')'];
                legendName2 = [char(signalFormatArray(signalIndex2)) ' ' char(signalNameArray(signalIndex2)) ' (' char(signalUnitArray(signalIndex2)) ')'];
                legendName3 = [char(signalFormatArray(signalIndex3)) ' ' char(signalNameArray(signalIndex3)) ' (' char(signalUnitArray(signalIndex3)) ')'];
                legend(legendName1,legendName2,legendName3);               % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);
                
             
                subplot(2,2,4);                                            % Create subplot
                signalIndex1 = chIndex(8);
                signalIndex2 = chIndex(9);
                signalIndex3 = chIndex(10);
                plot(sampleNumber, plotData(:,[signalIndex1 signalIndex2 signalIndex3]));                                 % Plot the magnetometer data
                legendName1 = [char(signalFormatArray(signalIndex1)) ' ' char(signalNameArray(signalIndex1)) ' (' char(signalUnitArray(signalIndex1)) ')'];
                legendName2 = [char(signalFormatArray(signalIndex2)) ' ' char(signalNameArray(signalIndex2)) ' (' char(signalUnitArray(signalIndex2)) ')'];
                legendName3 = [char(signalFormatArray(signalIndex3)) ' ' char(signalNameArray(signalIndex3)) ' (' char(signalUnitArray(signalIndex3)) ')'];
                legend(legendName1,legendName2,legendName3);               % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);
            end
            
            elapsedTime = elapsedTime + toc;                               % Stop timer and add to elapsed time
            tic;                                                           % Start timer           
            
        end  
        
        elapsedTime = elapsedTime + toc;                                   % Stop timer
        fprintf('The percentage of received packets: %d \n',obj.shimmer.getPacketReceptionRateCurrent()); % Detect loss packets
        obj.shimmer.stopStreaming();                                       % Stop data streaming                                                       % Stop data streaming             
    
    end
    obj.shimmer.disconnect();
    
end

        

