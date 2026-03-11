function void = ppgtoheartrateexample(comPort, PPGChannelNum, captureDuration, fileName) 
%PPGTOHEARTRATEEXAMPLE - Heart Rate from Photo Plethysmograph signal
%
%  PPGTOHEARTRATEEXAMPLE(COMPORT, PPGCHANNELNUM, CAPTUREDURATION, FILENAME)
%  plots Photo Plethysmograph and estimated Heart Rate from the Shimmer
%  paired with COMPORT. The function will stream data for a fixed duration
%  of time defined by the constant CAPTUREDURATION. The function also
%  writes the data in a tab delimited format to the file defined in
%  FILENAME.
%
%  SYNOPSIS: ppgtoheartrateexample(comPort, PPGChannelNum, captureDuration,
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
%  EXAMPLE: ppgtoheartrateexample('7', 13, 30, 'testdata.dat')
%
%  See also twoshimmerexample plotandwriteexample ShimmerHandleClass
%
% NOTE: To use the Java Shimmer Biophysical Processing Library in   
% conjunction with the MATLAB ID:
% Save the ShimmerBiophysicalProcessingLibrary_Rev_X_Y.jar file to
% C:\Program\Files\MATLAB\R2013b\java\jar (or the equivalent) on your PC and
% add the location of the ShimmerBiophysicalProcessingLibrary_Rev_X_Y.jar file
% to the JAVA dynamic class path:
%
% javaclasspath('C:\Program
% Files\MATLAB\R2013b\java\jar\ShimmerBiophysicalProcessingLibrary_Rev_X_Y.jar')
%
% NOTE: In this example the PPG data is pre-filtered using a second order
% Chebyshev LPF with corner freq 5Hz by using FilterClass.m
% 
% NOTE: If heartRate < 30 or heartRate > 215 or standard deviation of last
% X interbeat intervals > 100 (X = numberOfBeatsToAve) then -1 is returned.


%% definitions
shimmer = ShimmerHandleClass(comPort);                                     % Define shimmer as a ShimmerHandle Class instance with comPort1

fs = 204.8;                                                                % sample rate in [Hz] 
firsttime = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 2500;                                                 % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations
numSamples = 0;


%% filter settings
fclp = 5;                                                                  % corner frequency lowpassfilter [Hz]; 
nPoles = 2;                                                                % number of poles (HPF, LPF)
pbRipple = 0.5;                                                            % pass band ripple (%)
lpfPPG = FilterClass(FilterClass.LPF,fs,fclp,nPoles,pbRipple);             % lowpass filters for PPG channel

%% PPG2HR settings
numberOfBeatsToAve = 1;                                                    % the number of consecutive heart beats that are averaged to calculate the heart rate. Instantaneous heart rate is calculated after each detected pulse. So the last X instantaneous heart rates are averaged to give the output (where X is numberOfBeatsToAve)) must be >= 1.
useLastEstimate = 1;                                                       % true for repeating last valid estimate when invalid data is detected.
PPG2HR = com.shimmerresearch.biophysicalprocessing.PPGtoHRAlgorithm(fs,numberOfBeatsToAve,useLastEstimate);  % create PPG to Heart Rate object: Sampling Rate = 204.8Hz, Number of Beats to Average = 1 (minimum), Repeat last valid estimate when invalid data is detected.

%%
if (nargin ~=4)
    fprintf('Number of input arguments needs to be 4.\n');
    fprintf('See ''help ppgtoheartrateexample''.\n');
elseif (shimmer.connect)                                                       % TRUE if the shimmer connects
    
    % define settings for shimmer
    shimmer.setsamplingrate(fs);                                           % set the shimmer sampling rate
    PPGChannel = ['INT A' num2str(PPGChannelNum)];        
    shimmer.disableallsensors;                                             % disable all sensors
    shimmer.setenabledsensors(PPGChannel,1);                               % enable PPG Channel 
    shimmer.setinternalexppower(1);                                        % set internal expansion power
    
        
    if (shimmer.start)                                                     % TRUE if the shimmer starts streaming

        plotData = [];                                               
        timeStamp = [];
        filteredplotData = [];
        heartRate = [];
        storeData = [];
        
        h.figure1=figure('Name','Shimmer PPG and Heart Rate signals');     % create a handle to figure for plotting data from shimmer
        set(h.figure1, 'Position', [100, 500, 800, 400]);
        
        elapsedTime = 0;                                                   % reset to 0    
        tic;                                                               % start timer
        
        while (elapsedTime < captureDuration)            
                      
            pause(DELAY_PERIOD);                                           % pause for this period of time on each iteration to allow data to arrive in the buffer
            
            [newData,signalNameArray,~,~] = shimmer.getdata('c');   % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            
            if (firsttime==true && isempty(newData)~=1)
                tab = char(9);
                cal = 'CAL';
                signalNamesString=[char('Time Stamp'), char(9), char('PPG'), char(9), char('PPG Filtered'), char(9), char('Heart Rate')]; % create a single string, signalNamesString
                signalFormatsString =[cal, tab, cal, tab, cal, tab, cal, tab];
                signalUnitsString = ['milliseconds',tab,'mV',tab,'mV',tab,'BPM'];
                
                % write headers to file
                headerLines = {signalNamesString; signalFormatsString; signalUnitsString};
                fid = fopen(fileName, 'wt');
                for l = 1:numel(headerLines)
                    fprintf(fid, '%s\n',headerLines{l});
                end
                fclose(fid);
            end

            
            if ~isempty(newData)                                                            % TRUE if new data has arrived
                                  
                % get signal indices
                chIndex(1) = find(ismember(signalNameArray, 'Time Stamp'));
                chIndex(2) = find(ismember(signalNameArray, ['Internal ADC A' num2str(PPGChannelNum)]));   % PPG data output             
                PPGData = newData(:,chIndex(2));
                PPGDataFiltered = PPGData;
                PPGDataFiltered = lpfPPG.filterData(PPGDataFiltered);                       % filter with low pass filter   
                newheartRate = PPG2HR.ppgToHrConversion(PPGDataFiltered, newData(:,chIndex(1)));                   % compute Heart Rate from PPG data
                   
                plotData = [plotData; PPGData];                                             % update the plotDataBuffer with the new PPG data
                filteredplotData = [filteredplotData; PPGDataFiltered];                     % update the filteredplotData buffer with the new filtered PPG data
                heartRate = [heartRate; newheartRate];                                      % update the filteredHRData buffer with the new filtered Heart Rate data
                numPlotSamples = size(plotData,1);                          
                numSamples = numSamples + size(newData,1);
                timeStampNew = newData(:,chIndex(1));                                       % get timestamps
                timeStamp = [timeStamp; timeStampNew];
                
                newstoreData = [timeStampNew PPGData PPGDataFiltered newheartRate];
                storeData = [storeData; newstoreData];
                                 
                dlmwrite(fileName, storeData, '-append', 'delimiter', '\t','precision',16);                % append the new data to the file in a tab delimited format

                
                 if numSamples > NO_SAMPLES_IN_PLOT
                     
                        plotData = plotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                        filteredplotData = filteredplotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                        heartRate = heartRate(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                        
                 end
                 sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;
                

                % plotting the data
                set(0,'CurrentFigure',h.figure1);         
                subplot(3,1,1)
                plot(sampleNumber, plotData(:,1));                         % plot the PPG data
                legend('PPG (mV)'); 
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim('auto');                                          
                
                subplot(3,1,2)
                plot(sampleNumber, filteredplotData(:,1));                 % plot the filtered PPG data
                legend('Filtered PPG (mV)'); 
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim('auto');                                          
                
                subplot(3,1,3)
                plot(sampleNumber, heartRate);                             % plot the Heart Rate data
                legend('Heart Rate (BPM');   
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim('auto');           
                
            end
            
            elapsedTime = elapsedTime + toc;                               % stop timer and add to elapsed time
            tic;                                                           % start timer           
            
        end  
        
        elapsedTime = elapsedTime + toc;                                   % stop timer
        fprintf('The percentage of received packets: %d \n',shimmer.getpercentageofpacketsreceived(timeStamp)); % Detect loss packets
        shimmer.stop;                                                      % stop data streaming                                                    
       
    end 
    
    
    shimmer.disconnect;                                                    % disconnect from shimmer
        
end


