function void = plotandwritepressureandtemperatureexample(comPort, captureDuration, fileName)

%PLOTANDWRITEPRESSUREANDTEMPERATUREEXAMPLE - Plotting pressure and temperature data and write to file
%
%  PLOTANDWRITEPRESSUREANDTEMPERATUREEXAMPLE(COMPORT, CAPTUREDURATION, FILENAME) 

addpath('./Resources/')                                                    % directory containing supporting functions

%% definitions

shimmer = ShimmerHandleClass(comPort);                                     % Define shimmer as a ShimmerHandle Class instance with comPort1
SensorMacros = SetEnabledSensorsMacrosClass;                               % assign user friendly macros for setenabledsensors

firsttime = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 500;                                                  % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations
numSamples = 0;


%%

if (shimmer.connect)                                                       % TRUE if the shimmer connects
    
    % Define settings for shimmer
    shimmer.setsamplingrate(51.2);                                         % Set the shimmer sampling rate to 51.2Hz
    shimmer.disableallsensors;
    shimmer.setenabledsensors(SensorMacros.PRESSURE,1);                    % Enable BMP180 Pressure (and Temperature
  
  
          
    if (shimmer.start)                                                     % TRUE if the shimmer starts streaming

        plotData = [];                                               
        timeStamp = [];
        
        h.figure1=figure('Name','Shimmer 1 signals');                      % Create a handle to figure for plotting data from shimmer
        set(h.figure1, 'Position', [100, 500, 800, 400]);
        
        elapsedTime = 0;                                                   % Reset to 0    
        tic;                                                               % Start timer
        
        while (elapsedTime < captureDuration)            
                      
            pause(DELAY_PERIOD);                                           % Pause for this period of time on each iteration to allow data to arrive in the buffer
            
            [newData,signalNameArray,signalFormatArray,signalUnitArray] = shimmer.getdata('c');   % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            
            
            
            if (firsttime==true && isempty(newData)~=1)
                firsttime = writeHeadersToFile(fileName,signalNameArray,signalFormatArray,signalUnitArray);
            end

            
            if ~isempty(newData)                                           % TRUE if new data has arrived
                                
                dlmwrite(fileName, newData, '-append', 'delimiter', '\t'); % Append the new data to the file in a tab delimited format
                            
                plotData = [plotData; newData];                            % Update the plotDataBuffer with the new data
                numPlotSamples = size(plotData,1);
                numSamples = numSamples + size(newData,1);
                timeStampNew = newData(:,1);                               % get timestamps
                timeStamp = [timeStamp; timeStampNew];
                
                 if numSamples > NO_SAMPLES_IN_PLOT
                        plotData = plotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                 end
                 sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;
                 
               
                 chIndex(1) = find(ismember(signalNameArray, 'Pressure'));   % get signal indices
                 chIndex(2) = find(ismember(signalNameArray, 'Temperature'));% get signal indices

                             
                 
                set(0,'CurrentFigure',h.figure1);       
                signalIndex = chIndex(1);
                signalIndex2 = chIndex(2);
                subplot(2,1,1);                                            % Create subplot
                plot(sampleNumber, plotData(:,signalIndex));               % Plot data
                legend([signalFormatArray{signalIndex} ' ' signalNameArray{signalIndex} ' (' signalUnitArray{signalIndex} ')']);   
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim('auto');
                
                subplot(2,1,2);  
                plot(sampleNumber, plotData(:,signalIndex2));              % Plot data
                legend([signalFormatArray{signalIndex2} ' ' signalNameArray{signalIndex2} ' (' signalUnitArray{signalIndex2} ')']);   
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim('auto');
               
            end
            
            elapsedTime = elapsedTime + toc;                               % Stop timer and add to elapsed time
            tic;                                                           % Start timer           
            
        end  
        
        elapsedTime = elapsedTime + toc;                                   % Stop timer
        fprintf('The percentage of received packets: %d \n',shimmer.getpercentageofpacketsreceived(timeStamp)); % Detect loss packets
        shimmer.stop;                                                      % Stop data streaming                                                    
       
    end 
    
    shimmer.disconnect;                                                    % Disconnect from shimmer
        
end


