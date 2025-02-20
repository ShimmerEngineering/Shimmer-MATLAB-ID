classdef ShimmerDeviceHandler
    properties
        obj
        shimmer
        comPort
        sensorClass
    end
    
    methods
        function this = ShimmerDeviceHandler(comPort)
            javaaddpath('ShimmerBiophysicalProcessingLibrary_Rev_0_10.jar');
            javaaddpath('libs/ShimmerJavaClass.jar');
            javaaddpath('libs/jssc-2.9.6.jar');
            javaaddpath('libs/vecmath-1.3.1.jar');
            javaaddpath('libs/commons-lang3-3.8.1.jar');
            
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

                sampleRate = this.shimmer.getSamplingRateShimmer();
                orientationObj = javaObjectEDT('com.shimmerresearch.algorithms.orientation.GradDes3DOrientation', sampleRate);

                quaternions = zeros(N, 4, 'single');

                for i = 1:N
                    orientation3DObj = orientationObj.update(acc(i, 1), acc(i, 2), acc(i, 3), ...
                                                             gyr(i, 1), gyr(i, 2), gyr(i, 3));
                    qw = orientation3DObj.getQuaternionW();
                    qx = orientation3DObj.getQuaternionX();
                    qy = orientation3DObj.getQuaternionY();
                    qz = orientation3DObj.getQuaternionZ();

                    quaternions(i, :) = [qw, qx, qy, qz];
                end
            end
            if strcmp(dofMode, '9dof')
                [N, M] = size(receivedData);
            
                acc = receivedData(:, 2:4); % Accelerometer (ax, ay, az)
                gyr = receivedData(:, 5:7); % Gyroscope (gx, gy, gz)
                mag = receivedData(:, 8:10); % Magnetometer (mx, my, mz)

                sampleRate = this.shimmer.getSamplingRateShimmer();
                orientationObj = javaObjectEDT('com.shimmerresearch.algorithms.orientation.GradDes3DOrientation', sampleRate);

                quaternions = zeros(N, 4);

                for i = 1:N
                    orientation3DObj = orientationObj.update(acc(i, 1), acc(i, 2), acc(i, 3), ...
                                                             gyr(i, 1), gyr(i, 2), gyr(i, 3), ...
                                                             mag(i, 1), mag(i, 2), mag(i, 3));
                    qw = orientation3DObj.getQuaternionW();
                    qx = orientation3DObj.getQuaternionX();
                    qy = orientation3DObj.getQuaternionY();
                    qz = orientation3DObj.getQuaternionZ();

                    quaternions(i, :) = [qw, qx, qy, qz];
                end
            end
        end
        
        function quaternions = orientationModule2(this, receivedData)
            [N, M] = size(receivedData);

            % Extract sensor data
            acc = receivedData(:, 2:4); % Accelerometer (ax, ay, az)
            gyr = receivedData(:, 5:7); % Gyroscope (gx, gy, gz)
            mag = receivedData(:, 8:10); % Magnetometer (mx, my, mz)

            % Initialize quaternion storage
            quaternions = zeros(N, 4, 'single');
            q = single([1 0 0 0]); % Initial quaternion

            for i = 1:N
                % Update quaternion using Madgwick filter
                q = this.madgwickFilter(q, acc(i, :), gyr(i, :), mag(i, :));
                quaternions(i, :) = q; % Store result
            end
        end

        function q = madgwickFilter(this, q, acc, gyr, mag)
            mBeta = 0.5;
            mSamplingPeriod = 1/51.2;

            %if all(mag == 0)
            %    q = this.update(q, gyr, acc);
            %    return;
            %end

            accNorm = norm(acc);
            if isfinite(accNorm) && accNorm > 0
                acc = acc / accNorm;
            end

            magNorm = norm(mag);
            if isfinite(magNorm) && magNorm > 0
                mag = mag / magNorm;
            end

            q0 = q(1);
            q1 = q(2);
            q2 = q(3);
            q3 = q(4);

            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;
            
            X2q0 = 2 * q0;
            X2q1 = 2 * q1;
            X2q2 = 2 * q2;
            X2q3 = 2 * q3;
            
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q3 = q2 * q3;
            
            % Normalize accelerometer measurement
            recipNorm = sqrt(acc(1)^2 + acc(2)^2 + acc(3)^2);
            % Check for NaN or Infinity
            if isfinite(recipNorm) && recipNorm > 0.0
                recipNorm = 1.0 / recipNorm;
                acc(1) = acc(1) * recipNorm;
                acc(2) = acc(2) * recipNorm;
                acc(3) = acc(3) * recipNorm;
            else
                % TODO: Handle the case where recipNorm is NaN or Infinity
            end
            
            % Normalize magnetometer measurement
            recipNorm = sqrt(mag(1)^2 + mag(2)^2 + mag(3)^2);
            % Check for NaN or Infinity
            if isfinite(recipNorm) && recipNorm > 0.0
                recipNorm = 1.0 / recipNorm;
                mag(1) = mag(1) * recipNorm;
                mag(2) = mag(2) * recipNorm;
                mag(3) = mag(3) * recipNorm;
            else
                % TODO: Handle the case where recipNorm is NaN or Infinity
            end
            
            % Reference direction of Earth's magnetic field
            hx = mag(1) * (q0q0 + q1q1 - q2q2 - q3q3) + 2 * mag(2) * (q1q2 - q0q3) + 2 * mag(3) * (q1q3 + q0q2);
            hy = 2 * mag(1) * (q0q3 + q1q2) + mag(2) * (q0q0 - q1q1 + q2q2 - q3q3) * 2 * mag(3) * (q2q3 - q0q1);
            X2bz = 2 * mag(1) * (q1q3 - q0q2) + 2 * mag(2) * (q0q1 + q2q3) + mag(3) * (q0q0 - q1q1 - q2q2 + q3q3);
            X2bx = sqrt(hx^2 + hy^2);
            
            % Gradient descent algorithm corrective step
            s0 = -X2q2 * (2 * (q1q3 - q0q2) - acc(1)) + ...
                 X2q1 * (2 * (q0q1 + q2q3) - acc(2)) - ...
                 X2bz * q2 * (X2bx * (0.5 - q2q2 - q3q3) + X2bz * (q1q3 - q0q2) - mag(1)) + ...
                 (-X2bx * q3 + X2bz * q1) * (X2bx * (q1q2 - q0q3) + X2bz * (q0q1 + q2q3) - mag(2)) + ...
                 X2bx * q2 * (X2bx * (q0q2 + q1q3) + X2bz * (0.5 - q1q1 - q2q2) - mag(3));

            s1 = X2q3 * (2 * (q1q3 - q0q2) - acc(1)) + ...
                 X2q0 * (2 * (q0q1 + q2q3) - acc(2)) - ...
                 4 * q1 * (1 - 2 * (q1q1 + q2q2) - acc(3)) + ...
                 X2bz * q3 * (X2bx * (0.5 - q2q2 - q3q3) + X2bz * (q1q3 - q0q2) - mag(1)) + ...
                 (X2bx * q2 + X2bz * q0) * (X2bx * (q1q2 - q0q3) + X2bz * (q0q1 + q2q3) - mag(2)) + ...
                 (X2bx * q3 - X2bz * X2q1) * (X2bx * (q0q2 + q1q3) + X2bz * (0.5 - q1q1 - q2q2) - mag(3));

            s2 = -X2q0 * (2 * (q1q3 - q0q2) - acc(1)) + ...
                 X2q3 * (2 * (q0q1 + q2q3) - acc(2)) - ...
                 4 * q2 * (1 - 2 * (q1q1 + q2q2) - acc(3)) + ...
                 (-X2bx * X2q2 - X2bz * q0) * (X2bx * (0.5 - q2q2 - q3q3) + X2bz * (q1q3 - q0q2) - mag(1)) + ...
                 (X2bx * q1 + X2bz * q3) * (X2bx * (q1q2 - q0q3) + X2bz * (q0q1 + q2q3) - mag(2)) + ...
                 (X2bx * q0 - X2bz * X2q2) * (X2bx * (q0q2 + q1q3) + X2bz * (0.5 - q1q1 - q2q2) - mag(3));

            s3 = X2q1 * (2 * (q1q3 - q0q2) - acc(1)) + ...
                 X2q2 * (2 * (q0q1 + q2q3) - acc(2)) + ...
                 (-X2bx * X2q3 + X2bz * q1) * (X2bx * (0.5 - q2q2 - q3q3) + X2bz * (q1q3 - q0q2) - mag(1)) + ...
                 (-X2bx * q0 + X2bz * q2) * (X2bx * (q1q2 - q0q3) + X2bz * (q0q1 + q2q3) - mag(2)) + ...
                 X2bx * q1 * (X2bx * (q0q2 + q1q3) + X2bz * (0.5 - q1q1 - q2q2) - mag(3));

            % Normalize step magnitude
            recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

            % Check for NaN/Infinity before normalizing
            if isfinite(recipNorm) && recipNorm > 0.0
                s0 = s0 * recipNorm;
                s1 = s1 * recipNorm;
                s2 = s2 * recipNorm;
                s3 = s3 * recipNorm;
            else
                % TODO: Handle NaN/Infinity case
            end

            % Compute rate of change of quaternion
            qDot1 = 0.5 * (-q1 * gyr(1) - q2 * gyr(2) - q3 * gyr(3)) - mBeta * s0;
            qDot2 = 0.5 * ( q0 * gyr(1) - q3 * gyr(2) + q2 * gyr(3)) - mBeta * s1;
            qDot3 = 0.5 * ( q3 * gyr(1) + q0 * gyr(2) - q1 * gyr(3)) - mBeta * s2;
            qDot4 = 0.5 * (-q2 * gyr(1) + q1 * gyr(2) + q0 * gyr(3)) - mBeta * s3;
            
            % Integrate rate of change of quaternion to yield quaternion
            q0 = q0 + qDot1 * mSamplingPeriod;
            q1 = q1 + qDot2 * mSamplingPeriod;
            q2 = q2 + qDot3 * mSamplingPeriod;
            q3 = q3 + qDot4 * mSamplingPeriod;

            % Normalize quaternion
            recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

            % Check if recipNorm is finite and >0 before normalizing
            if isfinite(recipNorm) && recipNorm > 0.0
                q0 = q0 * recipNorm;
                q1 = q1 * recipNorm;
                q2 = q2 * recipNorm;
                q3 = q3 * recipNorm;
            else
                % TODO: Handle NaN/Infinity case
            end

            q = [q0, q1, q2, q3];
        end

    end
end
        
