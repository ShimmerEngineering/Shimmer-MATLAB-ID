function neworientation3Dexample(comPort, captureDuration, fileName)
%NEWORIENTATION3DEXAMPLE - Demonstrate 3D orientation visualation and write to file
%
%  NEWORIENTATION3DEXAMPLE(COMPORT, CAPTUREDURATION, FILENAME) streams 3
%  accelerometer signals, 3 gyroscope signals and 3 magnetometer signals,
%  from the Shimmer paired with COMPORT, estimates the 3D orientation in
%  quaternion format and displays a 3D graphic to visualise the
%  orientation. The "Set" and "Reset" buttons on the graph display can be
%  used to change the viewpoint of the graph so that the visualisation of
%  the device matches the viewpoint of the user, relative to the physical
%  device. The function
%  will stream data for a fixed duration of time defined by the constant
%  CAPTUREDURATION. The function also writes the data in a tab delimited
%  format to the file defined in FILENAME.
%
%  SYNOPSIS: neworientation3Dexample(comPort, captureDuration, fileName)
%
%  INPUT: comPort - String value defining the COM port number for Shimmer
%  INPUT: captureDuration - Numerical value defining the period of time
%                           (in seconds) for which the function will stream
%                           data from  the Shimmers.
%  INPUT : fileName - String value defining the name of the file that data
%                     is written to in a comma delimited format.
%  OUTPUT: none
%
%  EXAMPLE: neworientation3Dexample('COM3', 30, 'testdata.dat')
%
%  See also newplotandwriteexample ShimmerDeviceHandler

newSignalName = {'Quat_Madge_9DOF_W', 'Quat_Madge_9DOF_X', 'Quat_Madge_9DOF_Y', 'Quat_Madge_9DOF_Z'};
newSignalFormat = {'CAL', 'CAL', 'CAL', 'CAL'};
newSignalUnit = {'no_units', 'no_units', 'no_units', 'no_units'};

addpath('./quaternion/')                                                   % directory containing quaternion functions
addpath('./Resources/')                                                    % directory containing supporting functions

shimmer = ShimmerDeviceHandler(comPort);                                   % Define shimmer as a ShimmerDevice Handler instance with comPort

DELAY_PERIOD = 0.2; 
firsttime = true;

[success, obj] = shimmer.connect();

if success                                                                 % TRUE if the shimmer connects
     
    shimmerClone = obj.shimmer.deepClone();
    shimmerClone.setSamplingRateShimmer(51.2);
    
    shimmerClone.disableAllSensors();                                      % Disables all currently enabled sensors
    shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);           % Resets configuration on enabled and derived sensors
    
    sensorIds = javaArray('java.lang.Integer', 3);
    sensorIds(1) = java.lang.Integer(obj.sensorClass.SHIMMER_ANALOG_ACCEL);
    sensorIds(2) = java.lang.Integer(obj.sensorClass.SHIMMER_MPU9X50_GYRO);
    sensorIds(3) = java.lang.Integer(obj.sensorClass.SHIMMER_LSM303_MAG);

    shimmerClone.setSensorIdsEnabled(sensorIds);

    commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
    com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
    obj.shimmer.configureFromClone(shimmerClone);

    pause(20);
    
    if shimmer.start()                                                     % TRUE if the shimmer starts streaming
        
        % initial viewpoint for 3D visualisation
        cameraUpVector = [0,1,0,0];
        cameraPosition = [0,0,0,1];

        % vertices of the shimmer object
        shimmer3d = struct('p1',[0.5,-1,0.2],'p2',[-0.5,-1,0.2],...
                       'p3',[-0.5,1,0.2],'p4',[0.5,1,0.2],...
                       'p5',[0.5,-1,-0.2],'p6',[-0.5,-1,-0.2],...
                       'p7',[-0.5,1,-0.2],'p8',[0.5,1,-0.2],...
                       'p9',[0.4,-0.9,0.3],'p10',[-0.4,-0.9,0.3],...
                       'p11',[-0.4,0.9,0.3],'p12',[0.4,0.9,0.3],...
                       'p13',[0.2,-1,0.05], 'p14',[0.2,-1,-0.05],...
                       'p15',[-0.2,-1,-0.05],'p16',[-0.2,-1,0.05]);
        shimmer3dRotated = struct('p1',[0,0,0,1],'p2',[0,0,0,1],...
                              'p3',[0,0,0,1],'p4',[0,0,0,1],...
                              'p5',[0,0,0,1],'p6',[0,0,0,1],...
                              'p7',[0,0,0,1],'p8',[0,0,0,1],...
                              'p9',[0,0,0,1],'p10',[0,0,0,1],...
                              'p11',[0,0,0,1],'p12',[0,0,0,1],...
                              'p13',[0,0,0,1], 'p14',[0,0,0,1],...
                              'p15',[0,0,0,1],'p16',[0,0,0,1]);
        
        allData = [];
        
        h.figure1=figure('Name','Shimmer 1');                              % Create a handle to figure for plotting data from shimmer
        
        uicontrol('Style', 'pushbutton', 'String', 'Set',...
            'Position', [20 20 50 20],...
            'Callback', {@setaxes});                                       % Pushbutton to set the viewpoint
        
        uicontrol('Style', 'pushbutton', 'String', 'Reset',...
            'Position', [80 20 50 20],...
            'Callback', {@resetaxes});                                     % Pushbutton to reset the viewpoint
        
        elapsedTime = 0;                                                   % Reset to 0
        
        tic;                                                               % Start timer
        
        while (elapsedTime < captureDuration)
            
            pause(DELAY_PERIOD);                                           % Pause for this period of time on each iteration to allow data to arrive in the buffer
            
            data = obj.obj.receiveData();                                  % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            
            newData = data(1);
            shimmer.checkDeviceConnection(newData);
            
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
                chIndex(1) = find(ismember(signalNameCellArray, 'Timestamp')); % Get signal indices
                chIndex(2) = find(ismember(signalNameCellArray, 'Accel_LN_X'));
                chIndex(3) = find(ismember(signalNameCellArray, 'Accel_LN_Y'));
                chIndex(4) = find(ismember(signalNameCellArray, 'Accel_LN_Z'));
                chIndex(5) = find(ismember(signalNameCellArray, 'Gyro_X'));
                chIndex(6) = find(ismember(signalNameCellArray, 'Gyro_Y'));
                chIndex(7) = find(ismember(signalNameCellArray, 'Gyro_Z'));
                chIndex(8) = find(ismember(signalNameCellArray, 'Mag_X'));
                chIndex(9) = find(ismember(signalNameCellArray, 'Mag_Y'));
                chIndex(10) = find(ismember(signalNameCellArray, 'Mag_Z'));
            end
            
            if (firsttime==true && isempty(newData)~=1)
               
                % Adding new quaternion header to new file
                newChIndex = chIndex;
                
                signalNameCellArray = [signalNameCellArray; newSignalName(:)];
                signalFormatCellArray = [signalFormatCellArray; newSignalFormat(:)];
                signalUnitCellArray = [signalUnitCellArray; newSignalUnit(:)];
                
                newChIndex(11) = find(ismember(signalNameCellArray, 'Quat_Madge_9DOF_W'));
                newChIndex(12) = find(ismember(signalNameCellArray, 'Quat_Madge_9DOF_X'));
                newChIndex(13) = find(ismember(signalNameCellArray, 'Quat_Madge_9DOF_Y'));
                newChIndex(14) = find(ismember(signalNameCellArray, 'Quat_Madge_9DOF_Z'));
                
                firsttime = newWriteHeadersToFile(fileName,signalNameCellArray(newChIndex),signalFormatCellArray(newChIndex),signalUnitCellArray(newChIndex));
            end
            
            if ~isempty(newData)                                                                          % TRUE if new data has arrived
                
                filtredData = newData(:, chIndex);
                quaternionData = shimmer.orientationModule(filtredData,'9dof');
                
                updatedData = [filtredData quaternionData];
                
                dlmwrite(fileName, double(updatedData), '-append', 'delimiter', '\t', 'precision', 16);
                
                quaternionChannels(1) = 1;                                 % Find Quaternion signal indices.
                quaternionChannels(2) = 2;
                quaternionChannels(3) = 3;
                quaternionChannels(4) = 4;
                
                quaternion = quaternionData(end, quaternionChannels);      % Only use the most recent quaternion sample for the graphic
                                
                shimmer3dRotated.p1 = quatrotate(quaternion, [0 shimmer3d.p1]);                           % Rotate the vertices
                shimmer3dRotated.p2 = quatrotate(quaternion, [0 shimmer3d.p2]);
                shimmer3dRotated.p3 = quatrotate(quaternion, [0 shimmer3d.p3]);
                shimmer3dRotated.p4 = quatrotate(quaternion, [0 shimmer3d.p4]);
                shimmer3dRotated.p5 = quatrotate(quaternion, [0 shimmer3d.p5]);
                shimmer3dRotated.p6 = quatrotate(quaternion, [0 shimmer3d.p6]);
                shimmer3dRotated.p7 = quatrotate(quaternion, [0 shimmer3d.p7]);
                shimmer3dRotated.p8 = quatrotate(quaternion, [0 shimmer3d.p8]);
                shimmer3dRotated.p9 = quatrotate(quaternion, [0 shimmer3d.p9]);
                shimmer3dRotated.p10 = quatrotate(quaternion, [0 shimmer3d.p10]);
                shimmer3dRotated.p11 = quatrotate(quaternion, [0 shimmer3d.p11]);
                shimmer3dRotated.p12 = quatrotate(quaternion, [0 shimmer3d.p12]);
                shimmer3dRotated.p13 = quatrotate(quaternion, [0 shimmer3d.p13]);
                shimmer3dRotated.p14 = quatrotate(quaternion, [0 shimmer3d.p14]);
                shimmer3dRotated.p15 = quatrotate(quaternion, [0 shimmer3d.p15]);
                shimmer3dRotated.p16 = quatrotate(quaternion, [0 shimmer3d.p16]);

                x = [shimmer3dRotated.p1(2),shimmer3dRotated.p2(2),shimmer3dRotated.p3(2),shimmer3dRotated.p4(2),...      % Calculate the convex hull for the graphic
                     shimmer3dRotated.p5(2),shimmer3dRotated.p6(2),shimmer3dRotated.p7(2),shimmer3dRotated.p8(2),...      
                     shimmer3dRotated.p9(2),shimmer3dRotated.p10(2),shimmer3dRotated.p11(2),shimmer3dRotated.p12(2)]';
                y = [shimmer3dRotated.p1(3),shimmer3dRotated.p2(3),shimmer3dRotated.p3(3),shimmer3dRotated.p4(3),...
                     shimmer3dRotated.p5(3),shimmer3dRotated.p6(3),shimmer3dRotated.p7(3),shimmer3dRotated.p8(3),...      
                     shimmer3dRotated.p9(3),shimmer3dRotated.p10(3),shimmer3dRotated.p11(3),shimmer3dRotated.p12(3)]';
                z = [shimmer3dRotated.p1(4),shimmer3dRotated.p2(4),shimmer3dRotated.p3(4),shimmer3dRotated.p4(4),...
                     shimmer3dRotated.p5(4),shimmer3dRotated.p6(4),shimmer3dRotated.p7(4),shimmer3dRotated.p8(4),...      
                     shimmer3dRotated.p9(4),shimmer3dRotated.p10(4),shimmer3dRotated.p11(4),shimmer3dRotated.p12(4)]';
                 
                X = [x,y,z];
                K = convhulln(X);      

                set(0,'CurrentFigure',h.figure1);
                hold off;
                % Plot object surface
                trisurf(K,X(:,1),X(:,2),X(:,3),'EdgeColor','None','FaceColor','w');
                hold on;
                % Plot object outlines
                plot3([shimmer3dRotated.p1(2), shimmer3dRotated.p2(2)],[shimmer3dRotated.p1(3), shimmer3dRotated.p2(3)],[shimmer3dRotated.p1(4), shimmer3dRotated.p2(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p2(2), shimmer3dRotated.p3(2)],[shimmer3dRotated.p2(3), shimmer3dRotated.p3(3)],[shimmer3dRotated.p2(4), shimmer3dRotated.p3(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p3(2), shimmer3dRotated.p4(2)],[shimmer3dRotated.p3(3), shimmer3dRotated.p4(3)],[shimmer3dRotated.p3(4), shimmer3dRotated.p4(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p4(2), shimmer3dRotated.p1(2)],[shimmer3dRotated.p4(3), shimmer3dRotated.p1(3)],[shimmer3dRotated.p4(4), shimmer3dRotated.p1(4)],'-k','LineWidth',2)

                plot3([shimmer3dRotated.p5(2), shimmer3dRotated.p6(2)],[shimmer3dRotated.p5(3), shimmer3dRotated.p6(3)],[shimmer3dRotated.p5(4), shimmer3dRotated.p6(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p6(2), shimmer3dRotated.p7(2)],[shimmer3dRotated.p6(3), shimmer3dRotated.p7(3)],[shimmer3dRotated.p6(4), shimmer3dRotated.p7(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p7(2), shimmer3dRotated.p8(2)],[shimmer3dRotated.p7(3), shimmer3dRotated.p8(3)],[shimmer3dRotated.p7(4), shimmer3dRotated.p8(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p8(2), shimmer3dRotated.p5(2)],[shimmer3dRotated.p8(3), shimmer3dRotated.p5(3)],[shimmer3dRotated.p8(4), shimmer3dRotated.p5(4)],'-k','LineWidth',2)

                plot3([shimmer3dRotated.p9(2), shimmer3dRotated.p10(2)],[shimmer3dRotated.p9(3), shimmer3dRotated.p10(3)],[shimmer3dRotated.p9(4), shimmer3dRotated.p10(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p10(2), shimmer3dRotated.p11(2)],[shimmer3dRotated.p10(3), shimmer3dRotated.p11(3)],[shimmer3dRotated.p10(4), shimmer3dRotated.p11(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p11(2), shimmer3dRotated.p12(2)],[shimmer3dRotated.p11(3), shimmer3dRotated.p12(3)],[shimmer3dRotated.p11(4), shimmer3dRotated.p12(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p12(2), shimmer3dRotated.p9(2)],[shimmer3dRotated.p12(3), shimmer3dRotated.p9(3)],[shimmer3dRotated.p12(4), shimmer3dRotated.p9(4)],'-k','LineWidth',2)

                plot3([shimmer3dRotated.p1(2), shimmer3dRotated.p5(2)],[shimmer3dRotated.p1(3), shimmer3dRotated.p5(3)],[shimmer3dRotated.p1(4), shimmer3dRotated.p5(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p2(2), shimmer3dRotated.p6(2)],[shimmer3dRotated.p2(3), shimmer3dRotated.p6(3)],[shimmer3dRotated.p2(4), shimmer3dRotated.p6(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p3(2), shimmer3dRotated.p7(2)],[shimmer3dRotated.p3(3), shimmer3dRotated.p7(3)],[shimmer3dRotated.p3(4), shimmer3dRotated.p7(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p4(2), shimmer3dRotated.p8(2)],[shimmer3dRotated.p4(3), shimmer3dRotated.p8(3)],[shimmer3dRotated.p4(4), shimmer3dRotated.p8(4)],'-k','LineWidth',2)

                plot3([shimmer3dRotated.p1(2), shimmer3dRotated.p9(2)],[shimmer3dRotated.p1(3), shimmer3dRotated.p9(3)],[shimmer3dRotated.p1(4), shimmer3dRotated.p9(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p2(2), shimmer3dRotated.p10(2)],[shimmer3dRotated.p2(3), shimmer3dRotated.p10(3)],[shimmer3dRotated.p2(4), shimmer3dRotated.p10(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p3(2), shimmer3dRotated.p11(2)],[shimmer3dRotated.p3(3), shimmer3dRotated.p11(3)],[shimmer3dRotated.p3(4), shimmer3dRotated.p11(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p4(2), shimmer3dRotated.p12(2)],[shimmer3dRotated.p4(3), shimmer3dRotated.p12(3)],[shimmer3dRotated.p4(4), shimmer3dRotated.p12(4)],'-k','LineWidth',2)

                % Plot outline of dock connector
                plot3([shimmer3dRotated.p13(2), shimmer3dRotated.p14(2)],[shimmer3dRotated.p13(3), shimmer3dRotated.p14(3)],[shimmer3dRotated.p13(4), shimmer3dRotated.p14(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p14(2), shimmer3dRotated.p15(2)],[shimmer3dRotated.p14(3), shimmer3dRotated.p15(3)],[shimmer3dRotated.p14(4), shimmer3dRotated.p15(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p15(2), shimmer3dRotated.p16(2)],[shimmer3dRotated.p15(3), shimmer3dRotated.p16(3)],[shimmer3dRotated.p15(4), shimmer3dRotated.p16(4)],'-k','LineWidth',2)
                plot3([shimmer3dRotated.p16(2), shimmer3dRotated.p13(2)],[shimmer3dRotated.p16(3), shimmer3dRotated.p13(3)],[shimmer3dRotated.p16(4), shimmer3dRotated.p13(4)],'-k','LineWidth',2)

                xlim([-2,2])
                ylim([-2,2])
                zlim([-2,2])
                grid on
                view(cameraPosition(2:4))
                set(gca,'CameraUpVector',cameraUpVector(2:4));
            end
            
            elapsedTime = elapsedTime + toc;                                                              % Stop timer and add to elapsed time
            tic;                                                                                          % Start timer
            
        end
        
        elapsedTime = elapsedTime + toc;                                                                  % Stop timer
        fprintf('The percentage of received packets: %d \n',obj.shimmer.getPacketReceptionRateCurrent()); % Detect lost packets
        obj.shimmer.stopStreaming();                                                                                     % Stop data streaming
        
    end
    obj.shimmer.disconnect;                                                                                   % Disconnect from shimmer
    
end

    function setaxes(hObj,event) 
        % Called when user presses "Set" button  

        % Calculate camera position and angle for front view
        cameraPosition = quatrotate(quaternion,[0,0,0,1]);
        if (obj.shimmer.getHardwareVersion()~=3)
            cameraUpVector = quatrotate(quaternion,[0,1,0,0]);  % orientation for Shimmer2/2r 
        else
            cameraUpVector = quatrotate(quaternion,[0,-1,0,0]); % orientation for Shimmer3
        end
    end

    function resetaxes(hObj,event) 
        % Called when user presses "reset" button  

        % Reset camera position and angle to original view
        cameraPosition = [0,0,0,1];
        cameraUpVector = [0,1,0,0];

    end

end

