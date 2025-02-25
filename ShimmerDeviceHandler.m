classdef ShimmerDeviceHandler
    properties
        obj
        shimmer
        comPort
        sensorClass
        sampleRate
        orientationObj
    end
    
    methods
        function this = ShimmerDeviceHandler(comPort)
            javaaddpath('ShimmerBiophysicalProcessingLibrary_Rev_0_10.jar');
            javaaddpath('libs/ShimmerJavaClass.jar');
            javaaddpath('libs/jssc-2.9.6.jar');
            javaaddpath('libs/vecmath-1.3.1.jar');
            javaaddpath('libs/commons-lang3-3.8.1.jar');
            
            % this.sampleRate = this.shimmer.getSamplingRateShimmer();
            % relies on previous input / takes time to calibrated
            this.orientationObj = javaObjectEDT('com.shimmerresearch.algorithms.orientation.GradDes3DOrientation', 1/51.2);

            this.sensorClass = javaObjectEDT('com.shimmerresearch.driver.Configuration$Shimmer3$SENSOR_ID');
            this.comPort = comPort;
            this.obj = com.shimmerresearch.tools.matlab.ShimmerJavaClass();
        end
        
        function [success, this] = connect(this)
            this.obj.mBluetoothManager.connectShimmerThroughCommPort(this.comPort);

            timeout = 15;
            startTime = tic;
            this.shimmer = [];

            while isempty(this.shimmer) && toc(startTime) < timeout
                pause(1);
                this.shimmer = this.obj.mBluetoothManager.getShimmerDeviceBtConnected(this.comPort);

                if ~isempty(this.shimmer)
                    disp(['Device connected... Elapsed time: ', num2str(toc(startTime)), ' seconds']);
                    success = true;
                    pause(10);
                    return;
                end
            end
            success = false;
        end
        
        function success = start(this)
            this.shimmer = this.obj.mBluetoothManager.getShimmerDeviceBtConnected(this.comPort);
            if isempty(this.shimmer)
                error('Shimmer device is not connected.');
            end
            this.shimmer.startStreaming();
            success = true;
        end
        
        function quaternions = orientationModule(this, receivedData, dofMode)
            this.shimmer = this.obj.mBluetoothManager.getShimmerDeviceBtConnected(this.comPort);
            if iscell(receivedData)
                receivedData = receivedData{1};
            end

            receivedData = single(receivedData);
            
            if strcmp(dofMode, '6dof')
                [N, M] = size(receivedData);
            
                acc = receivedData(:, 2:4); % Accelerometer (ax, ay, az)
                gyr = receivedData(:, 5:7); % Gyroscope (gx, gy, gz)

                quaternions = zeros(N, 4, 'single');

                for i = 1:N
                    this.orientationObj.update(acc(i, 1), acc(i, 2), acc(i, 3), ...
                                                             gyr(i, 1) * (pi/180.0), gyr(i, 2) * (pi/180.0), gyr(i, 3) * (pi/180.0));
                    qw = this.orientationObj.getQuaternionW();
                    qx = this.orientationObj.getQuaternionX();
                    qy = this.orientationObj.getQuaternionY();
                    qz = this.orientationObj.getQuaternionZ();

                    quaternions(i, :) = [qw, qx, qy, qz];
                end
            end
            if strcmp(dofMode, '9dof')
                [N, M] = size(receivedData);
            
                acc = receivedData(:, 2:4); % Accelerometer (ax, ay, az)
                gyr = receivedData(:, 5:7); % Gyroscope (gx, gy, gz)
                mag = receivedData(:, 8:10); % Magnetometer (mx, my, mz)
                
                quaternions = zeros(N, 4);
                
                for i = 1:N
             
                    this.orientationObj.update(acc(i, 1), acc(i, 2), acc(i, 3), ...
                                               gyr(i, 1) * (pi/180.0), gyr(i, 2) * (pi/180.0), gyr(i, 3) * (pi/180.0), ...
                                               mag(i, 1), mag(i, 2), mag(i, 3));

                    qw = this.orientationObj.getQ0();
                    qx = this.orientationObj.getQ1();
                    qy = this.orientationObj.getQ2();
                    qz = this.orientationObj.getQ3();

                    quaternions(i, :) = [qw, qx, qy, qz];
                end
            end
        end
        
        function checkDeviceConnection(this, newData)
            persistent dataPreviouslyReceived warningDisplayed

            if isempty(dataPreviouslyReceived)
                dataPreviouslyReceived = false;
            end
            if isempty(warningDisplayed)
                warningDisplayed = false;
            end

            if isempty(newData)
                if dataPreviouslyReceived && ~warningDisplayed
                    uiwait(msgbox('Device disconnected.', 'Warning', 'warn'));
                    warningDisplayed = true;
                end
            else
                dataPreviouslyReceived = true;
                warningDisplayed = false;
            end
        end
    end
end
        
