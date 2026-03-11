function void = ecgtoheartrateexample(comPort, captureDuration, fileName) 
%ECGTOHEARTRATEEXAMPLE - Heart Rate from electrocardiogram signal
%
%  ECGTOHEARTRATEEXAMPLE(COMPORT, CAPTUREDURATION, FILENAME)
%  plots ECG and estimated Heart Rate from the Shimmer
%  paired with COMPORT. The function will stream data for a fixed duration
%  of time defined by the constant CAPTUREDURATION. The function also
%  writes the data in a tab delimited format to the file defined in
%  FILENAME.
%
%  SYNOPSIS: ecgtoheartrateexample(comPort, captureDuration, fileName)
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
%  EXAMPLE: ecgtoheartrateexample('7', 30, 'testdata.dat')
%
%  See also twoshimmerexample plotandwriteexample ShimmerHandleClass
%  ppgtoheartrateexample
%
% NOTE: To use the Java Shimmer Biophysical Processing Library in   
% conjunction with the MATLAB ID:
% Save the ShimmerBiophysicalProcessingLibrary_Rev_X_Y.jar  file to
% C:\Program\Files\MATLAB\R2013b\java\jar (or the equivalent) on your PC and
% add the location of the ShimmerBiophysicalProcessingLibrary_Rev_X_Y.jar file
% to the JAVA dynamic class path:
%
% javaclasspath('C:\Program Files\MATLAB\R2013b\java\jar\ShimmerBiophysicalProcessingLibrary_Rev_X_Y.jar')
%
% NOTE: In this example the ECG data is pre-filtered using a second order
% Chebyshev HPF with corner freq 0.5Hz by using FilterClass.m
% 



%% definitions
shimmer = ShimmerHandleClass(comPort);                                     % Define shimmer as a ShimmerHandle Class instance with comPort1
SensorMacros = SetEnabledSensorsMacrosClass;                               % assign user friendly macros for setenabledsensors

fs = 512;                                                                  % sample rate in [Hz]     

firsttime = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 2500;                                                 % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations
numSamples = 0;


%% filter settings
    fm = 50;                                                               % mains frequency [Hz]
    fchp = 0.5;                                                            % corner frequency highpassfilter [Hz]; Shimmer recommends 0.5Hz for monitoring applications, 0.05Hz for diagnostic settings
    nPoles = 4;                                                            % number of poles (HPF, LPF)
    pbRipple = 0.5;                                                        % pass band ripple (%)
    
    HPF = true;                                                            % enable (true) or disable (false) highpass filter
    LPF = true;                                                            % enable (true) or disable (false) lowpass filter
    BSF = true;                                                            % enable (true) or disable (false) bandstop filter
    
    % highpass filters for ExG channels
    if (HPF)
        hpfexg1ch1 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
        hpfexg1ch2 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
        hpfexg2ch1 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
        hpfexg2ch2 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
    end
    % lowpass filters for ExG channels
    if (LPF)
        lpfexg1ch1 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
        lpfexg1ch2 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
        lpfexg2ch1 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
        lpfexg2ch2 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
    end
    % bandstop filters for ExG channels;
    % cornerfrequencies at +1Hz and -1Hz from mains frequency
    if (BSF)
        bsfexg1ch1 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
        bsfexg1ch2 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
        bsfexg2ch1 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
        bsfexg2ch2 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
    end


%% ECG2HR settings
ECG2HR = com.shimmerresearch.biophysicalprocessing.ECGtoHRAdaptive(fs);  % create ECG to Heart Rate object

%%

if (shimmer.connect)                                                       % TRUE if the shimmer connects
    
    % define settings for shimmer
    shimmer.setsamplingrate(fs);                                           % set the shimmer sampling rate
    shimmer.setinternalboard('ECG');                                       % Select internal expansion board; select 'ECG' to enable both SENSOR_EXG1 and SENSOR_EXG2
    shimmer.disableallsensors;                                             % Disable other sensors
    shimmer.setenabledsensors(SensorMacros.ECG,1)                          % Enable SENSOR_EXG1 and SENSOR_EXG2
        
    if (shimmer.start)                                                     % TRUE if the shimmer starts streaming

        plotData = [];                                               
        timeStamp = [];
        filteredplotData = [];
        heartRate = [];
        storeData = [];
               
        h.figure1=figure('Name','Shimmer ECG and Heart Rate signals');     % create a handle to figure for plotting data from shimmer
        set(h.figure1, 'Position', [100, 500, 800, 400]);
        
        elapsedTime = 0;                                                   % reset to 0    
        tic;                                                               % start timer
        
        while (elapsedTime < captureDuration)            
                      
            pause(DELAY_PERIOD);                                           % pause for this period of time on each iteration to allow data to arrive in the buffer
            
            [newData,signalNameArray,signalFormatArray,signalUnitArray] = shimmer.getdata('c');   % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
                        
            if (firsttime==true && isempty(newData)~=1) 
                tab = char(9);
                cal = 'CAL';
                signalNamesString =[char('Time Stamp'), tab, char('ECG LL-RA'), tab, char('ECG LL-RA Filtered'), tab, char('Heart Rate')]; % create a single string, signalNamesString    
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
                chIndex(2) = find(ismember(signalNameArray, 'ECG LL-RA'));                  % ECG output 'LL-RA'
                ECGData = newData(:,chIndex(2));
                ECGDataFiltered = ECGData;
                if (HPF)
                    ECGDataFiltered = hpfexg1ch1.filterData(ECGDataFiltered);               % filter with high pass filter
                end
                if (LPF)
                    ECGDataFiltered = lpfexg1ch1.filterData(ECGDataFiltered);               % filter with low pass filter
                end
                if (BSF)
                    ECGDataFiltered = bsfexg1ch1.filterData(ECGDataFiltered);               % filter with band stop filter
                end
                
                newheartRate = ECG2HR.ecgToHrConversion(ECGDataFiltered, newData(:,chIndex(1)));  % compute Heart Rate from ECG data
                

                plotData = [plotData; ECGData];                                             % update the plotDataBuffer with the new PPG data
                filteredplotData = [filteredplotData; ECGDataFiltered];                     % update the filteredplotData buffer with the new filtered PPG data
                heartRate = [heartRate; newheartRate];                                      % update the filteredHRData buffer with the new filtered Heart Rate data
                numPlotSamples = size(plotData,1);                          
                numSamples = numSamples + size(newData,1);
                timeStampNew = newData(:,chIndex(1));                                       % get timestamps
                timeStamp = [timeStamp; timeStampNew];
                
                newstoreData = [timeStampNew ECGData ECGDataFiltered newheartRate];
                storeData = [storeData; newstoreData];
                                 
                dlmwrite(fileName, storeData, '-append', 'delimiter', '\t', 'precision',16);                % append the new data to the file in a tab delimited format

                
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
                legend('ECG LL-RA (mV)', 'Location', 'West'); 
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim('auto');                                          
                
                subplot(3,1,2)
                plot(sampleNumber, filteredplotData(:,1));                 % plot the filtered PPG data
                legend('Filtered ECG LL-RA (mV)', 'Location', 'West'); 
                xlim([sampleNumber(1) sampleNumber(end)]);
                ylim('auto');                                          
                
                subplot(3,1,3)
                plot(sampleNumber, heartRate);                             % plot the Heart Rate data
                legend('Heart Rate (BPM', 'Location', 'West');   
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


