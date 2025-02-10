classdef ShimmerDeviceHandler
    properties
        obj
        shimmer
        comPort
    end
    
    methods
        function this = ShimmerDeviceHandler(comPort)
            javaaddpath('ShimmerBiophysicalProcessingLibrary_Rev_0_10.jar');
            javaaddpath('libs/ShimmerJavaClass.jar');
            javaaddpath('libs/jssc-2.9.6.jar');
            javaaddpath('libs/vecmath-1.3.1.jar');
            javaaddpath('libs/commons-lang3-3.8.1.jar');
            
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
        
        function enabledsensors(this, samplingRate, sensorsArray)
            
            %   Valid values for SENSORNAME are:
            %       'LowNoiseAccel', 'WideRangeAccel', 'AlternativeAccel', 'Gyro',
            %       'Mag', 'AlternativeMag', 'ECG', 'EMG', 'EXG', 'GSR',
            %       'EXT A7', 'EXT A9', 'EXT A6', 'EXT A11', 'EXT A15', 'EXT A2',
            %       'Bridge Amplifier', 'BattVolt', 'INT A1', 'INT A7', 'INT A12',
            %       'INT A10', 'INT A13', 'INT A15', 'INT A14', 'INT A16', 'Pressure'
            
            this.shimmer = this.obj.mBluetoothManager.getShimmerDeviceBtConnected(this.comPort);

            shimmerClone = this.shimmer.deepClone();
            shimmerClone.setSamplingRateShimmer(samplingRate);
            shimmerClone.disableAllSensors();
            shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);

            for iSensor = 1:length(sensorsArray)
                
                sensorName = sensorsArray{iSensor};
                enabledSensor = this.obj.getSensorId(sensorName);
                shimmerClone.setSensorEnabledState(enabledSensor, true);
            end

            commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
            com.shimmerresearch.driverUtilities.AssembleShimmerConfig.generateSingleShimmerConfig(shimmerClone, commType);
            this.shimmer.configureFromClone(shimmerClone);

            btState = javaMethod('valueOf', 'com.shimmerresearch.bluetooth.ShimmerBluetooth$BT_STATE', 'CONFIGURING');
            this.shimmer.operationStart(btState);

            pause(20);
        end

    end
end
