function orientation3Dexample(comPort, captureDuration, fileName)
%ORIENTATION3DEXAMPLE - Demonstrate 3D orientation visualation and write to file
%
%  ORIENTATION3DEXAMPLE(COMPORT, CAPTUREDURATION, FILENAME) streams 3
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
%  NOTE: This example uses the method 'getdata' which is a more advanced
%  alternative to the 'getuncalibrateddata' method in the beta release.
%  The user is advised to use the updated method 'getdata'.
%
%  SYNOPSIS: orientation3Dexample(comPort, captureDuration, fileName)
%
%  INPUT: comPort - String value defining the COM port number for Shimmer
%  INPUT: captureDuration - Numerical value defining the period of time
%                           (in seconds) for which the function will stream
%                           data from  the Shimmers.
%  INPUT : fileName - String value defining the name of the file that data
%                     is written to in a comma delimited format.
%  OUTPUT: none
%
%  EXAMPLE: orientation3Dexample('7', 30, 'testdata.dat')
%
%  See also plotandwriteexample twoshimmerexample ShimmerHandleClass

addpath('./quaternion/')                                                   % directory containing quaternion functions
addpath('./Resources/')                                                    % directory containing supporting functions

SensorMacros = SetEnabledSensorsMacrosClass;                               % assign user friendly macros for setenabledsensors

% Note: these constants are only relevant to this examplescript and are not used
% by the ShimmerHandle Class
DELAY_PERIOD = 0.2;                                                        % A delay period of time in seconds between data read operations

shimmer = ShimmerHandleClass(comPort);                                     % Define shimmer as a ShimmerHandle Class instance with comPort1
firsttime = true;

if (shimmer.connect)                                                       % TRUE if the shimmer connects
    
    % Define settings for shimmer
    shimmer.setsamplingrate(51.2);                                         % Set the shimmer sampling rate to 51.2Hz
    shimmer.setinternalboard('9DOF');                                      % Set the shimmer internal daughter board to '9DOF'
    shimmer.disableallsensors;                                             % disable all sensors
    shimmer.setenabledsensors(SensorMacros.GYRO,1,SensorMacros.MAG,1,...   % Enable the gyroscope, magnetometer and accelerometer.
    SensorMacros.ACCEL,1);                                                  
    shimmer.setaccelrange(0);                                              % Set the accelerometer range to 0 (+/- 1.5g for Shimmer2/2r, +/- 2.0g for Shimmer3)
    shimmer.setorientation3D(1);                                           % Enable orientation3D
    shimmer.setgyroinusecalibration(1);                                    % Enable gyro in-use calibration
    
    if (shimmer.start)                                                     % TRUE if the shimmer starts streaming
        
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
            'Callback', {@resetaxes});                                       % Pushbutton to reset the viewpoint
        
        elapsedTime = 0;                                                   % Reset to 0
        
        tic;                                                               % Start timer
        
        while (elapsedTime < captureDuration)
            
            pause(DELAY_PERIOD);                                           % Pause for this period of time on each iteration to allow data to arrive in the buffer
            
            [newData,signalNameArray,signalFormatArray,signalUnitArray] = shimmer.getdata('c');   % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit
            
            if (firsttime==true && isempty(newData)~=1)
                firsttime = writeHeadersToFile(fileName,signalNameArray,signalFormatArray,signalUnitArray);
            end
            
            if ~isempty(newData)                                                                          % TRUE if new data has arrived
                
                allData = [allData; newData];
                
                dlmwrite(fileName, newData, '-append', 'delimiter', '\t','precision',16);                                % Append the new data to the file in a tab delimited format
                
                quaternionChannels(1) = find(ismember(signalNameArray, 'Quaternion 0'));                  % Find Quaternion signal indices.
                quaternionChannels(2) = find(ismember(signalNameArray, 'Quaternion 1'));
                quaternionChannels(3) = find(ismember(signalNameArray, 'Quaternion 2'));
                quaternionChannels(4) = find(ismember(signalNameArray, 'Quaternion 3'));
                
                quaternion = newData(end, quaternionChannels);                                            % Only use the most recent quaternion sample for the graphic
                                
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
        fprintf('The percentage of received packets: %d \n',shimmer.getpercentageofpacketsreceived(allData(:,1))); % Detect lost packets
        shimmer.stop;                                                                                     % Stop data streaming
        
    end
    shimmer.disconnect;                                                                                   % Disconnect from shimmer
    
end

    function setaxes(hObj,event) 
        % Called when user presses "Set" button  

        % Calculate camera position and angle for front view
        cameraPosition = quatrotate(quaternion,[0,0,0,1]);
        if (shimmer.getshimmerversion~=3)
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


