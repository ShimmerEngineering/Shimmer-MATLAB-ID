classdef ShimmerDeviceHandler < handle
    events
        DeviceConnected
        DeviceDisconnected
        DeviceConnectionLost
    end
    properties
        obj
        bluetoothManager
        sensorClass
        orientationObj
        javaHandle
        samplingRate
    end

    methods
        function this = ShimmerDeviceHandler(samplingRate)
            % samplingRate (Hz) is optional; it sets the orientation
            % algorithm update period. Defaults to 51.2 Hz.
            if nargin < 1 || isempty(samplingRate)
                samplingRate = 51.2;
            end
            this.samplingRate = samplingRate;

            % List of JARs you want to add
            jarsToAdd = {
                'ShimmerBiophysicalProcessingLibrary_Rev_0_10.jar'
                'libs/ShimmerJavaClass.jar'
                'libs/jssc-2.9.6.jar'
                'libs/vecmath-1.3.1.jar'
                'libs/commons-lang3-3.8.1.jar'
            };
            
            % Get current dynamic classpath
            currentClasspath = javaclasspath('-dynamic');

            % Resolve JARs relative to this class file, not the caller's pwd
            baseDir = fileparts(which('ShimmerDeviceHandler'));

            for i = 1:numel(jarsToAdd)
                jarPath = jarsToAdd{i};
                fullJarPath = fullfile(baseDir, jarPath);
            
                % Check if the jar is already in classpath (case-insensitive)
                isInClasspath = any(strcmpi(currentClasspath, fullJarPath));
            
                if ~isInClasspath
                    try
                        javaaddpath(fullJarPath);
                        fprintf('Added to classpath: %s\n', fullJarPath);
                    catch ME
                        fprintf('Failed to add %s: %s\n', fullJarPath, ME.message);
                    end
                else
                    fprintf('Already in classpath: %s\n', fullJarPath);
                end
            end

            % Update period is derived from the configured sampling rate.
            this.orientationObj = javaObjectEDT('com.shimmerresearch.algorithms.orientation.GradDes3DOrientation', 1/this.samplingRate);

            this.sensorClass = javaObjectEDT('com.shimmerresearch.driver.Configuration$Shimmer3$SENSOR_ID');
            this.obj = com.shimmerresearch.tools.matlab.ShimmerJavaClass();
            this.obj.setDebugMode(false);
            this.bluetoothManager = this.obj.mBluetoothManager;
            this.javaHandle = handle(this.obj, 'callbackproperties');
            this.javaHandle.PropertyChangeCallback = @(src,evt)this.handleJavaEvent(evt);


        end

        function delete(this)
            % Detach the MATLAB callback from the Java object so the
            % handle can be garbage-collected.
            try
                if ~isempty(this.javaHandle)
                    this.javaHandle.PropertyChangeCallback = [];
                end
            catch
            end
        end
        
        function quaternions = orientationModule(this, receivedData, dofMode)
            if iscell(receivedData)
                receivedData = receivedData{1};
            end

            receivedData = single(receivedData);

            if strcmp(dofMode, '6dof')
                if size(receivedData, 2) < 7
                    error('ShimmerDeviceHandler:orientationModule', '6dof mode requires at least 7 columns of received data.');
                end
                N = size(receivedData, 1);

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
            elseif strcmp(dofMode, '9dof')
                if size(receivedData, 2) < 10
                    error('ShimmerDeviceHandler:orientationModule', '9dof mode requires at least 10 columns of received data.');
                end
                N = size(receivedData, 1);

                acc = receivedData(:, 2:4); % Accelerometer (ax, ay, az)
                gyr = receivedData(:, 5:7); % Gyroscope (gx, gy, gz)
                mag = receivedData(:, 8:10); % Magnetometer (mx, my, mz)

                quaternions = zeros(N, 4, 'single');

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
            else
                error('ShimmerDeviceHandler:orientationModule', 'dofMode must be ''6dof'' or ''9dof''.');
            end
        end

        function handleJavaEvent(this, evt)
            try
                eventObj = evt.getNewValue();   % this is MatlabConnectionEvent
                state   = char(eventObj.state);
                comPort = char(eventObj.comPort);
            catch ME
                disp("Error extracting event data:");
                disp(ME.message);
                return;
            end
        
            switch state
                case 'CONNECTED'
                    fprintf("MATLAB: Device connected on %s\n", comPort);
                    notify(this, 'DeviceConnected', ComPortEventData(comPort));
            
                case 'DISCONNECTED'
                    fprintf("MATLAB: Device disconnected on %s\n", comPort);
                    notify(this, 'DeviceDisconnected', ComPortEventData(comPort));
            
                case 'CONNECTION_LOST'
                    fprintf("MATLAB: Connection lost on %s\n", comPort);
                    notify(this, 'DeviceConnectionLost', ComPortEventData(comPort));

                otherwise
                    % Intermediate states (e.g. connecting) are not surfaced.
            end
        end
    end
end
