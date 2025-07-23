function void = plotandwriteemgexample(comPort, captureDuration, fileName)
    %NEWPLOTANDWRITEEMGEXAMPLE - Plotting emg signal and write to file
    %  SYNOPSIS: newplotandwriteemgexample(comPort, captureDuration,
    %  fileName)
    %
    %  INPUT: comPort  - String value defining the COM port number for Shimmer
    %   
    %  INPUT: captureDuration  - Numerical value defining the capture duration
    %
    %  INPUT: fileName - String value defining the name of the data file
    %
    %  OUTPUT: shimmer  - Object of the ShimmerHandleClass
    %
	%  EXAMPLE: newplotandwriteemgexample('COM5', 30, 'testdata.dat')
    %
    %  See also newplotandwriteexample ShimmerDeviceHandler
    
    %% definitions
    shimmer = ShimmerDeviceHandler(comPort);                               % Define shimmer as a ShimmerHandle Class instance with comPort1

    fs = 512;                                                              % sample rate in [Hz]     

    firsttime = true;

    % Note: these constants are only relevant to this examplescript and are not used
    % by the ShimmerHandle Class
    NO_SAMPLES_IN_PLOT = 5000;                                             % Number of samples in the plot 
    DELAY_PERIOD = 0.2;                                                    % Delay (in seconds) between data read operations
    numSamples = 0;

    addpath('./Resources/')                                                % directory containing supporting functions

    %% settings

    % filtering settings
    fm = 50;                                                                   % mains frequency [Hz]
    fchp = 5;                                                                  % corner frequency highpassfilter [Hz]; Shimmer recommends 5Hz to remove DC-offset and movement artifacts
    nPoles = 4;                                                                % number of poles (HPF, LPF)
    pbRipple = 0.5;                                                            % pass band ripple (%)
    
    HPF = true;                                                                % enable (true) or disable (false) highpass filter
    LPF = true;                                                                % enable (true) or disable (false) lowpass filter
    BSF = true;                                                                % enable (true) or disable (false) bandstop filter
    
    % highpass filters for ExG channels
    if (HPF)
        hpfexg1ch1 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
        hpfexg1ch2 = FilterClass(FilterClass.HPF,fs,fchp,nPoles,pbRipple);
    end
    if (LPF)
        % lowpass filters for ExG channels
        lpfexg1ch1 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
        lpfexg1ch2 = FilterClass(FilterClass.LPF,fs,fs/2-1,nPoles,pbRipple);
    end
    if (BSF)
        % bandstop filters for ExG channels;
        % cornerfrequencies at +1Hz and -1Hz from mains frequency
        bsfexg1ch1 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
        bsfexg1ch2 = FilterClass(FilterClass.LPF,fs,[fm-1,fm+1],nPoles,pbRipple);
    end

%%

[success, obj] = shimmer.connect();

if success
    
    shimmerClone = obj.shimmer.deepClone();
    shimmerClone.setSamplingRateShimmer(51.2);
    
    shimmerClone.disableAllSensors();                                      % Disables all currently enabled sensors
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);           % Resets configuration on enabled and derived sensors
    
    sensorIds = javaArray('java.lang.Integer', 1);
    sensorIds(1) = java.lang.Integer(obj.sensorClass.HOST_EMG);

    shimmerClone.setSensorIdsEnabled(sensorIds);

    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    obj.shimmer.configureFromClone(shimmerClone);

    pause(20);
    
    if shimmer.start()

        plotData = [];                                               
        timeStamp = [];
        filteredplotData = [];

        h.figure1=figure('Name','Shimmer EMG signals');                    % Create a handle to figure for plotting data from shimmer
        set(h.figure1, 'Position', [100, 500, 800, 400]);

        elapsedTime = 0;                                                   % Reset to 0    
        tic;                                                               % Start timer
        
        while (elapsedTime < captureDuration)            
                      
            pause(DELAY_PERIOD);                                           % pause for this period of time on each iteration to allow data to arrive in the buffer
            
            data = obj.obj.receiveData();
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
                chIndex(1) = find(ismember(signalNameCellArray, 'EMG_CH1_24BIT'));
                chIndex(2) = find(ismember(signalNameCellArray, 'EMG_CH2_24BIT'));
            end

            if (firsttime==true && isempty(newData)~=1)
                firsttime = newWriteHeadersToFile(fileName,signalNameCellArray(chIndex),signalFormatCellArray(chIndex),signalUnitCellArray(chIndex));
            end

            if ~isempty(newData)                                           % TRUE if new data has arrived

                EMGData = newData(:,chIndex);
                EMGDataFiltered = EMGData;
                % filter the data
                if HPF % filter newData with highpassfilter to remove DC-offset
                    EMGDataFiltered(:,1) = hpfexg1ch1.filterData(EMGDataFiltered(:,1));
                    EMGDataFiltered(:,2) = hpfexg1ch2.filterData(EMGDataFiltered(:,2));
                end

                if BSF % filter highpassfiltered data with bandstopfilter to suppress mains interference
                    EMGDataFiltered(:,1) = bsfexg1ch1.filterData(EMGDataFiltered(:,1));
                    EMGDataFiltered(:,2) = bsfexg1ch2.filterData(EMGDataFiltered(:,2));
                end

                if LPF % filter bandstopfiltered data with lowpassfilter to avoid aliasing
                    EMGDataFiltered(:,1) = lpfexg1ch1.filterData(EMGDataFiltered(:,1));
                    EMGDataFiltered(:,2) = lpfexg1ch2.filterData(EMGDataFiltered(:,2));
                end

                dlmwrite(fileName, double(EMGDataFiltered), '-append', 'delimiter', '\t','precision',16); % Append the new data to the file in a tab delimited format

                plotData = [plotData; EMGData];                            % Update the plotData buffer with the new ECG data
                filteredplotData = [filteredplotData; EMGDataFiltered];    % Update the filteredplotData buffer with the new filtered ECG data
                numPlotSamples = size(plotData,1);
                numSamples = numSamples + size(newData,1);

                timeStampNew = newData(:,1);                                   % get timestamps
                timeStamp = [timeStamp; timeStampNew];

                if numSamples > NO_SAMPLES_IN_PLOT
                    plotData = plotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                    filteredplotData = filteredplotData(numPlotSamples-NO_SAMPLES_IN_PLOT+1:end,:);
                end
                sampleNumber = max(numSamples-NO_SAMPLES_IN_PLOT+1,1):numSamples;

                set(0,'CurrentFigure',h.figure1);   
                subplot(2,2,1);                                        % Create subplot
                signalIndex = chIndex(1);
                plot(sampleNumber,plotData(:,1));                      % Plot the ecg for channel 1 of SENSOR_EXG1
                legendName1 = [char(signalFormatArray(signalIndex)) ' ' char(signalNameArray(signalIndex)) ' (' char(signalUnitArray(signalIndex)) ')'];  
                legend(legendName1);                                   % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);

                subplot(2,2,2);                                        % Create subplot
                signalIndex = chIndex(2);
                plot(sampleNumber,plotData(:,2));                      % Plot the ecg for channel 2 of SENSOR_EXG1
                legendName1 = [char(signalFormatArray(signalIndex)) ' ' char(signalNameArray(signalIndex)) ' (' char(signalUnitArray(signalIndex)) ')'];  
                legend(legendName1);                                   % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);

                subplot(2,2,3);                                        % Create subplot
                signalIndex = chIndex(1);
                plot(sampleNumber,filteredplotData(:,1));              % Plot the filtered ecg for channel 1 of SENSOR_EXG1
                legendName1 = [char(signalFormatArray(signalIndex)) ' ' char(signalNameArray(signalIndex)) ' (' char(signalUnitArray(signalIndex)) ')']; 
                legend(legendName1);                                   % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);

                subplot(2,2,4);                                        % Create subplot
                signalIndex = chIndex(2);
                plot(sampleNumber,filteredplotData(:,2));              % Plot the filtered ecg for channel 2 of SENSOR_EXG1
                legendName1 = [char(signalFormatArray(signalIndex)) ' ' char(signalNameArray(signalIndex)) ' (' char(signalUnitArray(signalIndex)) ')'];  
                legend(legendName1);                                   % Add legend to plot
                xlim([sampleNumber(1) sampleNumber(end)]);

            end

            elapsedTime = elapsedTime + toc;                           % Update elapsedTime with the time that elapsed since starting the timer
            tic;                                                       % Start timer           

            end  

            elapsedTime = elapsedTime + toc;                               % Update elapsedTime with the time that elapsed since starting the timer
            fprintf('The percentage of received packets: %d \n',obj.shimmer.getPacketReceptionRateCurrent()); % Detect loss packets
            obj.shimmer.stopStreaming();                                                  % Stop data streaming                                                    

        end 

    end
    
    obj.shimmer.disconnect();
end

