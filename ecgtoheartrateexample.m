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
%  EXAMPLE: ecgtoheartrateexample('COM5', 30, 'testdata.dat')
%
%  See also plotandwriteexample ShimmerDeviceHandler
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

%% definitions
deviceHandler = ShimmerDeviceHandler();                                   % Define a handler
fs = 512;                                                                  % sample rate in [Hz]

firsttime = true;

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 2500;                                                 % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations
numSamples = 0;

%% filter settings
HPF = true;                                                            % enable (true) or disable (false) highpass filter
LPF = true;                                                            % enable (true) or disable (false) lowpass filter
BSF = true;                                                            % enable (true) or disable (false) bandstop filter

% highpass filters for ExG channels
if (HPF)
    hpfexg1ch1 = com.shimmerresearch.algorithms.Filter(com.shimmerresearch.algorithms.Filter.HIGH_PASS,fs,0.5);
end
% lowpass filters for ExG channels
if (LPF)
    lpfexg1ch1 = com.shimmerresearch.algorithms.Filter(com.shimmerresearch.algorithms.Filter.LOW_PASS,fs,51.2);
end
% bandstop filters for ExG channels;
% cornerfrequencies at +1Hz and -1Hz from mains frequency
if (BSF)
    bsfexg1ch1 = com.shimmerresearch.algorithms.Filter(com.shimmerresearch.algorithms.Filter.BAND_STOP,fs,[49 51]);
end


%% ECG2HR settings
ECG2HR = com.shimmerresearch.biophysicalprocessing.ECGtoHRAdaptive(fs);  % create ECG to Heart Rate object

%%

deviceHandler.bluetoothManager.connectShimmerThroughCommPort(comPort);
% Ensure disconnection happens properly even if the workspace is cleared or the script is interrupted
cleaner = onCleanup(@() deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).disconnect());  % Ensure disconnection on cleanup
pause(10);
if deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).isConnected()

    shimmerClone = deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).deepClone();
    shimmerClone.setSamplingRateShimmer(fs);

    shimmerClone.disableAllSensors();                                      % Disables all currently enabled sensors
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);           % Resets configuration on enabled and derived sensors

    sensorIds = javaArray('java.lang.Integer', 1);
    sensorIds(1) = java.lang.Integer(deviceHandler.sensorClass.HOST_ECG);

    shimmerClone.setSensorIdsEnabled(sensorIds);
    shimmerClone.setConfigValueUsingConfigLabel(java.lang.Integer(deviceHandler.sensorClass.HOST_ECG),'Resolution',java.lang.Integer(1));
    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).configureFromClone(shimmerClone);

    pause(20);

    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).startStreaming()

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

        data = deviceHandler.obj.receiveData(comPort);                                  % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
        if (isempty(data))
            continue;
        end
        newData = data(1);
        disp(size(newData));
        signalNameArray = data(2);
        signalFormatArray = data(3);
        signalUnitArray = data(4);

        signalNameCellArray = cell(numel(signalNameArray), 1);
        for i = 1:numel(signalNameArray)
            signalNameCellArray{i} = char(signalNameArray(i));         % Convert each Java string to a MATLAB char array
        end

        if (firsttime==true && isempty(newData)~=1)
            tab = char(9);
            cal = 'CAL';
            signalNamesString =[char('Time Stamp'), tab, char('ECG LA-RA'), tab, char('ECG LA-RA Filtered'), tab, char('Heart Rate')]; % create a single string, signalNamesString
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
            chIndex(1) = find(ismember(signalNameCellArray, 'Timestamp'));
            chIndex(2) = find(ismember(signalNameCellArray, 'ECG_LA-RA_24BIT'));                  % ECG output 'LA-RA'
            ECGData = newData(:,chIndex(2));
            ECGDataFiltered = ECGData;
            if (HPF)
                 for i = 1:length(ECGDataFiltered)
                    ECGDataFiltered(i) = hpfexg1ch1.filterData(ECGDataFiltered(i));  % Filter one sample at a time
                end

            end
            if (LPF)
                for i = 1:length(ECGDataFiltered)
                    ECGDataFiltered(i) = lpfexg1ch1.filterData(ECGDataFiltered(i));  % Filter one sample at a time
                end

            end
            if (BSF)
                for i = 1:length(ECGDataFiltered)
                    ECGDataFiltered(i) = bsfexg1ch1.filterData(ECGDataFiltered(i));  % Filter one sample at a time
                end
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
            legend('ECG LA-RA (mV)', 'Location', 'West');
            xlim([sampleNumber(1) sampleNumber(end)]);
            ylim('auto');

            subplot(3,1,2)
            plot(sampleNumber, filteredplotData(:,1));                 % plot the filtered PPG data
            legend('Filtered ECG LA-RA (mV)', 'Location', 'West');
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
    fprintf('The percentage of received packets: %d \n',deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).getPacketReceptionRateCurrent()); % Detect loss packets
    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).stopStreaming();                                                      % stop data streaming



    deviceHandler.bluetoothManager.getShimmerDeviceBtConnected(comPort).disconnect();                                                    % disconnect from shimmer

end


end