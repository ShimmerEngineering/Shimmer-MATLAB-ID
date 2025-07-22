classdef ShimmerDeviceHandler
    properties
        obj
        bluetoothManager
        sensorClass
        orientationObj
    end
    
    methods
        function this = ShimmerDeviceHandler()
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
            
            for i = 1:numel(jarsToAdd)
                jarPath = jarsToAdd{i};
                fullJarPath = fullfile(pwd, jarPath);  % Resolve relative to current directory
            
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

            % this.sampleRate = this.shimmer.getSamplingRateShimmer();
            % relies on previous input / takes time to calibrated
            this.orientationObj = javaObjectEDT('com.shimmerresearch.algorithms.orientation.GradDes3DOrientation', 1/51.2);

            this.sensorClass = javaObjectEDT('com.shimmerresearch.driver.Configuration$Shimmer3$SENSOR_ID');
            this.obj = com.shimmerresearch.tools.matlab.ShimmerJavaClass();
            this.bluetoothManager = this.obj.mBluetoothManager;
        end
        
        function quaternions = orientationModule(this, receivedData, dofMode)
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
    end
end
        
