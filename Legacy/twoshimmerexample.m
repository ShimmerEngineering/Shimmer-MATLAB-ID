function twoshimmerexample(comPort1, comPort2, captureDuration)

%TWOSHIMMEREXAMPLE - Demonstrate basic features of ShimmerHandleClass
%
%  TWOSHIMMEREXAMPLE(COMPORT1, COMPORT2, CAPTUREDURATION) plots the 3
%  accelerometer signals from the Shimmer paired with COMPORT1 and plots
%  the 3 accelerometer signals from the Shimmer paired with COMPORT2. The
%  function will stream data for a fixed duration of time defined by the
%  constant CAPTUREDURATION. NOTE: This example uses the method 'getdata'
%  which is a more advanced alternative to the 'getuncalibrateddata' method
%  in the beta release. The user is advised to use the updated method
%  'getdata'.
%
%  SYNOPSIS: twoshimmerexample(comPort1, comPort2, captureDuration)
%
%  INPUT: comPort1 - string value defining the COM port numbers for Shimmer 1
%  INPUT: comPort2 - string value defining the COM port numbers for Shimmer 2
%  INPUT: captureDuration - numerical value defining the period of time 
%                        (in seconds) for which the function will stream 
%                        data from  the Shimmers.
%  
%  EXAMPLE: twoshimmerexample('7','8',30)
%
%  See also plotandwriteexample ShimmerHandleClass


% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
NO_SAMPLES_IN_PLOT = 500;                                                  % Number of samples that will be displayed in the plot at any one time
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations

%%

shimmer1 = ShimmerHandleClass(comPort1);                                   % Define shimmer1 as a ShimmerHandle Class instance with comPort1
shimmer2 = ShimmerHandleClass(comPort2);                                   % Define shimmer2 as a ShimmerHandle Class instance with comPort2
SensorMacros = SetEnabledSensorsMacrosClass;                               % assign user friendly macros for setenabledsensors


if (shimmer1.connect && shimmer2.connect)                                  % TRUE if both shimmer 1 and shimmer 2 connect
    
    % Define settings for shimmer1
    shimmer1.setinternalboard('None');                                     % Set the shimmer 1 internal daughter board to 'None'
    shimmer1.disableallsensors;                                             % disable all sensors
    shimmer1.setenabledsensors(SensorMacros.ACCEL,1);                      % Enable the shimmer 1 accelerometer
    shimmer1.setaccelrange(0);                                             % Set accelerometer range
    shimmer1.setsamplingrate(51.2);                                        % Set the shimmer 1 sampling rate to 51.2Hz
    
   
    
    % Define settings for shimmer2
    shimmer2.setinternalboard('None');                                     % Set the shimmer 2 internal daughter board to 'None'
    shimmer2.disableallsensors;                                             % disable all sensors
    shimmer2.setenabledsensors(SensorMacros.ACCEL,1);                      % Enable the shimmer 2 accelerometer 
    shimmer2.setaccelrange(0);                                             % Set accelerometer range
    shimmer2.setsamplingrate(51.2);                                        % Set the shimmer 2 sampling rate to 51.2Hz
    

    
    if (shimmer1.start && shimmer2.start)                                  % TRUE if both shimmers start streaming
        
        uncalibDataShimmer1 = []; 
        uncalibDataShimmer2 = [];
        
        h.figure1=figure('Name','Shimmer 1');                              % Create a handle to figure for plotting data from shimmer1
        h.figure2=figure('Name','Shimmer 2');                              % Create a handle to figure for plotting data from shimmer2
        
        elapsedTime = 0;                                                   % Reset to 0
        
        tic;                                                               % Start timer
        
        while (elapsedTime < captureDuration)            
                      
            pause(DELAY_PERIOD);                                           % Pause for this period of time on each iteration to allow data to arrive in the buffer
            
            
            % Read and plot data for shimmer1
            [uncalibratedData,signalName,signalFormat,signalUnit]=shimmer1.getdata('u');
            uncalibDataShimmer1 = [uncalibDataShimmer1; uncalibratedData];    % Read the uncalibrated data for shimmer1 and add to previous data
                 
            if(length(uncalibDataShimmer1) > NO_SAMPLES_IN_PLOT)                 
               uncalibDataShimmer1=uncalibDataShimmer1((length(uncalibDataShimmer1) - NO_SAMPLES_IN_PLOT):end, :);   % Trim excess previous data from array for plotting purposes
            end
           
            if (isempty(uncalibDataShimmer1)~=1)
                if (shimmer1.ShimmerVersion == 3)                              % get signal indices for Shimmer3
                    chIndex(1) = find(ismember(signalName, 'Low Noise Accelerometer X'));
                    chIndex(2) = find(ismember(signalName, 'Low Noise Accelerometer Y'));
                    chIndex(3) = find(ismember(signalName, 'Low Noise Accelerometer Z'));
                elseif (shimmer1.ShimmerVersion < 3)                           % get signal indices for Shimmer2/2r
                    chIndex(1) = find(ismember(signalName, 'Accelerometer X'));
                    chIndex(2) = find(ismember(signalName, 'Accelerometer Y'));
                    chIndex(3) = find(ismember(signalName, 'Accelerometer Z'));
                end
                accelDataShimmer1 = [uncalibDataShimmer1(:,chIndex(1)), uncalibDataShimmer1(:,chIndex(2)), uncalibDataShimmer1(:,chIndex(3))]; % Extract only the columns of accelerometer data
                set(0,'CurrentFigure',h.figure1);           
                plot(accelDataShimmer1);                                       % Plot the accelerometer data
                title('Shimmer 1 Accelerometer Data');                         % Add title to the plot
                axis([0 NO_SAMPLES_IN_PLOT 0 4095]);                           % Define min and max values for axis , Shimmer 3 has an int16 for the digital accel so users can change accordingly
                legend(char(signalName{chIndex(1)}),char(signalName{chIndex(2)}),char(signalName{chIndex(3)}))                          % Add legend to plot

            end
            

             % Read and plot data for shimmer2            
            [uncalibratedData,signalName,signalFormat,signalUnit]=shimmer2.getdata('u');
            uncalibDataShimmer2 = [uncalibDataShimmer2; uncalibratedData];     % Read the uncalibrated data for shimmer2 and add to previous data
            
            if(length(uncalibDataShimmer2) > NO_SAMPLES_IN_PLOT)
               uncalibDataShimmer2 = uncalibDataShimmer2((length(uncalibDataShimmer2) - NO_SAMPLES_IN_PLOT):end, :);   % Trim excess previous data from array for plotting purposes
            end
                    
            if (isempty(uncalibDataShimmer2)~=1)
                if (shimmer2.ShimmerVersion == 3)                              % get signal indices for Shimmer3
                    chIndex2(1) = find(ismember(signalName, 'Low Noise Accelerometer X'));
                    chIndex2(2) = find(ismember(signalName, 'Low Noise Accelerometer Y'));
                    chIndex2(3) = find(ismember(signalName, 'Low Noise Accelerometer Z'));
                elseif (shimmer2.ShimmerVersion < 3)                           % get indices for Shimmer2/2r
                    chIndex2(1) = find(ismember(signalName, 'Accelerometer X'));
                    chIndex2(2) = find(ismember(signalName, 'Accelerometer Y'));
                    chIndex2(3) = find(ismember(signalName, 'Accelerometer Z'));
                end
                accelDataShimmer2 = [uncalibDataShimmer2(:,chIndex2(1)), uncalibDataShimmer2(:,chIndex2(2)), uncalibDataShimmer2(:,chIndex2(3))]; % Extract only the columns of gyroscope data
                set(0,'CurrentFigure',h.figure2);                     
                plot(accelDataShimmer2);                                   % Plot the gyroscope data
                title('Shimmer 2 Accelerometer Data');                     % Add title to the plot
                axis([0 NO_SAMPLES_IN_PLOT 0 4095]);                       % Define min and max values for axis , Shimmer 3 has an int16 for the digital accel so users can change accordingly
                legend(char(signalName{chIndex2(1)}),char(signalName{chIndex2(2)}),char(signalName{chIndex2(3)}))                              % Add legend to plot
            end       
            elapsedTime = elapsedTime + toc;                               % Stop timer and add to elapsed time
            tic;                                                           % Start timer
            
        end  
        
        elapsedTime = elapsedTime + toc;                                   % Stop timer
        
        shimmer1.stop;                                                     % Stop data streaming from shimmer1                                                    
        shimmer2.stop;                                                     % Stop data streaming from shimmer2
        
    else
        
        shimmer1.stop;                                                     % Stop data streaming from shimmer1 (if it has started streaming)                                                
        shimmer2.stop;                                                     % Stop data streaming from shimmer2 (if it has started streaming)  
        
    end            
    
    shimmer1.disconnect;                                                   % Disconnect from shimmer1
    shimmer2.disconnect;                                                   % Disconnect from shimmer2    
    
end

clear all;                                                                 % Remove all variables from memory

