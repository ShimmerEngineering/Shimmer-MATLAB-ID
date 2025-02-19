function void = newppgtoheartrateexample(comPort, PPGChannelNum, captureDuration, fileName) 
%% definitions
shimmer = ShimmerDeviceHandler(comPort);                                   % Define shimmer as a ShimmerHandle Class instance with comPort1

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
[success, obj] = shimmer.connect();

if success
    
    shimmerClone = obj.shimmer.deepClone();
    shimmerClone.setSamplingRateShimmer(fs);
    
    shimmerClone.disableAllSensors();                                      % Disables all currently enabled sensors
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);           % Resets configuration on enabled and derived sensors
    
    sensorIds = javaArray('java.lang.Integer', 1);
    
    if PPGChannelNum == 13
        sensorIds(1) = java.lang.Integer(obj.sensorClass.HOST_PPG_A13);
    elseif PPGChannelNum == 12
        sensorIds(1) = java.lang.Integer(obj.sensorClass.HOST_PPG_A12);
    end
    
    shimmerClone.setSensorIdsEnabled(sensorIds);

    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    obj.shimmer.configureFromClone(shimmerClone);

    pause(20);
    
    if shimmer.start()
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

            data = obj.obj.receiveData();                                      % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            newData = data(1);
            signalNameArray = data(2);

            signalNameCellArray = cell(numel(signalNameArray), 1);     
            for i = 1:numel(signalNameArray)
                signalNameCellArray{i} = char(signalNameArray(i));         % Convert each Java string to a MATLAB char array
            end

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


            if ~isempty(newData)                                           % TRUE if new data has arrived

                % get signal indices
                chIndex(1) = find(ismember(signalNameCellArray, 'Timestamp'));
                chIndex(2) = find(ismember(signalNameCellArray, ['PPG_A' num2str(PPGChannelNum)]));   % PPG data output             
                PPGData = newData(:,chIndex(2));
                PPGDataFiltered = PPGData;
                PPGDataFiltered = lpfPPG.filterData(PPGDataFiltered);      % filter with low pass filter   
                newheartRate = PPG2HR.ppgToHrConversion(PPGDataFiltered, newData(:,chIndex(1)));                   % compute Heart Rate from PPG data

                plotData = [plotData; PPGData];                            % update the plotDataBuffer with the new PPG data
                filteredplotData = [filteredplotData; PPGDataFiltered];    % update the filteredplotData buffer with the new filtered PPG data
                heartRate = [heartRate; newheartRate];                     % update the filteredHRData buffer with the new filtered Heart Rate data
                numPlotSamples = size(plotData,1);                          
                numSamples = numSamples + size(newData,1);
                timeStampNew = newData(:,chIndex(1));                      % get timestamps
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
        fprintf('The percentage of received packets: %d \n',obj.shimmer.getPacketReceptionRateCurrent()); % Detect loss packets
        obj.shimmer.stopStreaming();                                           % stop data streaming          
        
    end
    obj.shimmer.disconnect();

end