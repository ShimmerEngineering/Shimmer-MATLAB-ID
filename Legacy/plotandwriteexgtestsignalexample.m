function shimmer = plotandwriteexgtestsignalexample(comPort, captureDuration, fileName)
%PLOTANDWRITEEXGTESTSIGNALEXAMPLE - Plotting exg test signal and write to file
%    INPUT: comPort  - String value defining the COM port number for Shimmer
%   
%    INPUT: captureDuration  - Numerical value defining the capture duration
%
%    INPUT: fileName - String value defining the name of the data file
%
%    OUTPUT: shimmer  - Object of the ShimmerHandleClass
%
% Example for Shimmer3

addpath('./Resources/')                                                    % directory containing supporting functions


%% definitions

shimmer = ShimmerHandleClass(comPort);
SensorMacros = SetEnabledSensorsMacrosClass;                               % assign user friendly macros for setenabledsensors

fs = 512;                                                                  % sample rate in [Hz]
firsttime = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 5000;                                                 % Number of samples in the plot 
DELAY_PERIOD = 0.2;                                                        % Delay (in seconds) between data read operations
numSamples = 0;

%% settings

if (shimmer.connect)
    shimmer.setsamplingrate(fs);                                           % Select sampling rate
    shimmer.setinternalboard('EXG');                                       % Select internal expansion board; select 'EXG' to enable both SENSOR_EXG1 and SENSOR_EXG2 
    shimmer.disableallsensors;
    shimmer.setenabledsensors(SensorMacros.EXG1,1,SensorMacros.EXG2,1);    % Enable sensors EXG1 and EXG2
      
    chipidentifier = 1;  % Select SENSOR_EXG1
    shimmer.setexgtestsignalparameters(chipidentifier); % enable testsignal for ExG chip1 
    chipidentifier = 2;  % Select SENSOR_EXG2
    shimmer.setexgtestsignalparameters(chipidentifier); % enable testsignal for ExG chip1 

    
    if (shimmer.start)                                                     % TRUE if the shimmer starts streaming 
        
        plotData = [];                                               
        newData = [];
        timeStamp = [];
        
        h.figure1=figure('Name','Shimmer 1 EXG Testsignals');              % Create a handle to figure for plotting data from shimmer
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
                
                dlmwrite(fileName, newData, '-append', 'delimiter', '\t','precision',16); % Append the new data to the file in a tab delimited format
                            
                plotData = [plotData; newData];                            % Update the plotDataBuffer with the new data
                numPlotSamples = size(plotData,1);
                numSamples = numSamples + size(newData,1);
                timeStampNew = newData(:,1);                                   % get timestamps
                timeStamp = [timeStamp; timeStampNew];
                
                if numSamples > NO_SAMPLES_IN_PLOT
                    plotData = plotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                end
                sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;
                
                chIndex(1) = find(ismember(signalNameArray, 'EXG1 CH1'));
                chIndex(2) = find(ismember(signalNameArray, 'EXG1 CH2'));
                chIndex(3) = find(ismember(signalNameArray, 'EXG2 CH1'));
                chIndex(4) = find(ismember(signalNameArray, 'EXG2 CH2'));
                            
                set(0,'CurrentFigure',h.figure1);   
                subplot(2,2,1);                                        % Create subplot
                signalIndex = chIndex(1);
                plot(sampleNumber,plotData(:,signalIndex));            % Plot the exg testsignal for channel 1 of SENSOR_EXG1
                legendName1=[signalFormatArray{signalIndex} ' ' signalNameArray{signalIndex} ' (' signalUnitArray{signalIndex} ')'];  
                legend(legendName1);                                   % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim([-1.1 1.1]);

                subplot(2,2,2);                                        % Create subplot
                signalIndex = chIndex(2);
                plot(sampleNumber,plotData(:,signalIndex));            % Plot the exg testsignal for channel 2 of SENSOR_EXG1
                legendName1=[signalFormatArray{signalIndex} ' ' signalNameArray{signalIndex} ' (' signalUnitArray{signalIndex} ')'];  
                legend(legendName1);                                   % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim([-1.1 1.1]);

                subplot(2,2,3);                                        % Create subplot
                signalIndex = chIndex(3);
                plot(sampleNumber,plotData(:,signalIndex));            % Plot the exg testsignal for channel 1 of SENSOR_EXG1
                legendName1=[signalFormatArray{signalIndex} ' ' signalNameArray{signalIndex} ' (' signalUnitArray{signalIndex} ')'];  
                legend(legendName1);                                   % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim([-1.1 1.1]);

                subplot(2,2,4);                                        % Create subplot
                signalIndex = chIndex(4);
                plot(sampleNumber,plotData(:,signalIndex));            % Plot the exg testsignal for channel 2 of SENSOR_EXG1
                legendName1=[signalFormatArray{signalIndex} ' ' signalNameArray{signalIndex} ' (' signalUnitArray{signalIndex} ')'];  
                legend(legendName1);                                   % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim([-1.1 1.1]);
            end
            
            elapsedTime = elapsedTime + toc;                               % Update elapsedTime with the time that elapsed since starting the timer
            tic;                                                           % Start timer           
            
        end  
        
        elapsedTime = elapsedTime + toc;                                   % Update elapsedTime with the time that elapsed since starting the timer
        fprintf('The percentage of received packets: %d \n',shimmer.getpercentageofpacketsreceived(timeStamp)); % Detect loss packets
        shimmer.stop;                                                      % Stop data streaming                                                    
       
    end 
    
end
 
shimmer.disconnect;
  
   
 

 clear shimmer
end % plotexgtestsignalexample