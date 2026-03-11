function void = plotandwriteexample(comPort, captureDuration, fileName)
%PLOTANDWRITEEXAMPLE - Demonstrate basic features of ShimmerHandleClass
%
%  PLOTANDWRITEEXAMPLE(COMPORT, CAPTUREDURATION, FILENAME) plots 3 
%  accelerometer signals, 3 gyroscope signals and 3 magnetometer signals,
%  from the Shimmer paired with COMPORT. The function 
%  will stream data for a fixed duration of time defined by the constant 
%  CAPTUREDURATION. The function also writes the data in a tab ddelimited 
%  format to the file defined in FILENAME.
%  NOTE: This example uses the method 'getdata' which is a more advanced 
%  alternative to the 'getuncalibrateddata' method in the beta release. 
%  The user is advised to use the updated method 'getdata'.  
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
%  EXAMPLE: plotandwriteexample('7', 30, 'testdata.dat')
%
%  See also twoshimmerexample ShimmerHandleClass 

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
    shimmer.setinternalboard('9DOF');                                      % Set the shimmer internal daughter board to '9DOF'
    shimmer.disableallsensors;                                             % disable all sensors
    shimmer.setenabledsensors(SensorMacros.ACCEL,1,SensorMacros.MAG,1,...  % Enable the shimmer accelerometer, magnetometer, gyroscope and battery voltage monitor
        SensorMacros.GYRO,1);    
    shimmer.setaccelrange(0);                                              % Set the accelerometer range to 0 (+/- 1.5g) for Shimmer2r
    
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
                
                dlmwrite(fileName, newData, '-append', 'delimiter', '\t','precision',16); % Append the new data to the file in a tab delimited format
                
                plotData = [plotData; newData];                            % Update the plotDataBuffer with the new data
                numPlotSamples = size(plotData,1);
                numSamples = numSamples + size(newData,1);
                timeStampNew = newData(:,1);                               % get timestamps
                timeStamp = [timeStamp; timeStampNew];
                
                 if numSamples > NO_SAMPLES_IN_PLOT
                        plotData = plotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                 end
                 sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;
                 
               
                 chIndex(1) = find(ismember(signalNameArray, 'Time Stamp'));   % get signal indices
                 if (shimmer.ShimmerVersion == 3)                              % for Shimmer3
                     chIndex(2) = find(ismember(signalNameArray, 'Low Noise Accelerometer X'));
                     chIndex(3) = find(ismember(signalNameArray, 'Low Noise Accelerometer Y'));
                     chIndex(4) = find(ismember(signalNameArray, 'Low Noise Accelerometer Z'));
                 elseif (shimmer.ShimmerVersion < 3)                           % for Shimmer2/2r
                     chIndex(2) = find(ismember(signalNameArray, 'Accelerometer X'));
                     chIndex(3) = find(ismember(signalNameArray, 'Accelerometer Y'));
                     chIndex(4) = find(ismember(signalNameArray, 'Accelerometer Z'));
                 end
                 chIndex(5) = find(ismember(signalNameArray, 'Gyroscope X'));
                 chIndex(6) = find(ismember(signalNameArray, 'Gyroscope Y'));
                 chIndex(7) = find(ismember(signalNameArray, 'Gyroscope Z'));
                 chIndex(8) = find(ismember(signalNameArray, 'Magnetometer X'));
                 chIndex(9) = find(ismember(signalNameArray, 'Magnetometer Y'));
                 chIndex(10) = find(ismember(signalNameArray, 'Magnetometer Z'));
                              
                 
                set(0,'CurrentFigure',h.figure1);           
                subplot(2,2,1);                                            % Create subplot
                signalIndex = chIndex(1);
                plot(sampleNumber, plotData(:,signalIndex));               % Plot the time stamp data
                legend([signalFormatArray{signalIndex} ' ' signalNameArray{signalIndex} ' (' signalUnitArray{signalIndex} ')']);   
                xlim([sampleNumber(1) sampleNumber(end)]);

                subplot(2,2,2);                                            % Create subplot
                signalIndex1 = chIndex(2);
                signalIndex2 = chIndex(3);
                signalIndex3 = chIndex(4);
                plot(sampleNumber, plotData(:,[signalIndex1 signalIndex2 signalIndex3]));                                 % Plot the accelerometer data
                legendName1=[signalFormatArray{signalIndex1} ' ' signalNameArray{signalIndex1} ' (' signalUnitArray{signalIndex1} ')'];  
                legendName2=[signalFormatArray{signalIndex2} ' ' signalNameArray{signalIndex2} ' (' signalUnitArray{signalIndex2} ')'];  
                legendName3=[signalFormatArray{signalIndex3} ' ' signalNameArray{signalIndex3} ' (' signalUnitArray{signalIndex3} ')'];  
                legend(legendName1,legendName2,legendName3); % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);

                subplot(2,2,3);                                            % Create subplot
                signalIndex1 = chIndex(5);
                signalIndex2 = chIndex(6);
                signalIndex3 = chIndex(7);
                plot(sampleNumber, plotData(:,[signalIndex1 signalIndex2 signalIndex3]));                                 % Plot the gyroscope data
                legendName1=[signalFormatArray{signalIndex1} ' ' signalNameArray{signalIndex1} ' (' signalUnitArray{signalIndex1} ')'];  
                legendName2=[signalFormatArray{signalIndex2} ' ' signalNameArray{signalIndex2} ' (' signalUnitArray{signalIndex2} ')'];  
                legendName3=[signalFormatArray{signalIndex3} ' ' signalNameArray{signalIndex3} ' (' signalUnitArray{signalIndex3} ')'];  
                legend(legendName1,legendName2,legendName3); % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);
                
             
                subplot(2,2,4);                                            % Create subplot
                signalIndex1 = chIndex(8);
                signalIndex2 = chIndex(9);
                signalIndex3 = chIndex(10);
                plot(sampleNumber, plotData(:,[signalIndex1 signalIndex2 signalIndex3]));                                 % Plot the magnetometer data
                legendName1=[signalFormatArray{signalIndex1} ' ' signalNameArray{signalIndex1} ' (' signalUnitArray{signalIndex1} ')'];  
                legendName2=[signalFormatArray{signalIndex2} ' ' signalNameArray{signalIndex2} ' (' signalUnitArray{signalIndex2} ')'];  
                legendName3=[signalFormatArray{signalIndex3} ' ' signalNameArray{signalIndex3} ' (' signalUnitArray{signalIndex3} ')'];  
                legend(legendName1,legendName2,legendName3); % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);
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


