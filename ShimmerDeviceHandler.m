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
            
            sensorClass = javaObjectEDT('com.shimmerresearch.driver.Configuration$Shimmer3$SENSOR_ID');
            hwVersion = javaObjectEDT('com.shimmerresearch.driverUtilities.ShimmerVerDetails$HW_ID');
            
            shimmerClone = this.shimmer.deepClone();
            shimmerClone.setSamplingRateShimmer(samplingRate);
            shimmerClone.disableAllSensors();
            shimmerClone.setEnabledAndDerivedSensorsAndUpdateMaps(0, 0);

            for iSensor = 1:length(sensorsArray)

                sensorName = sensorsArray{iSensor};

                switch sensorName
                    case 'LowNoiseAccel'
                        if this.shimmer.getHardwareVersion() == hwVersion.SHIMMER_3R
                            enabledSensor = sensorClass.SHIMMER_LSM6DSV_ACCEL_LN;
                        else
                            enabledSensor = sensorClass.SHIMMER_ANALOG_ACCEL;
                        end
                    case 'WideRangeAccel'
                        if this.shimmer.getHardwareVersion() == hwVersion.SHIMMER_3R
                            enabledSensor = sensorClass.SHIMMER_LIS2DW12_ACCEL_WR;
                        else
                            enabledSensor = sensorClass.SHIMMER_LSM303_ACCEL;
                        end
                    case 'AlternativeAccel'
                        if this.shimmer.getHardwareVersion() == hwVersion.SHIMMER_3R
                            enabledSensor = sensorClass.SHIMMER_ADXL371_ACCEL_HIGHG;
                        else
                            enabledSensor = sensorClass.SHIMMER_MPU9X50_ACCEL;
                        end
                    case 'Gyro'
                        if this.shimmer.getHardwareVersion() == hwVersion.SHIMMER_3R
                            enabledSensor = sensorClass.SHIMMER_LSM6DSV_GYRO;
                        else
                            enabledSensor = sensorClass.SHIMMER_MPU9X50_GYRO;
                        end
                    case 'Mag'
                        if this.shimmer.getHardwareVersion() == hwVersion.SHIMMER_3R
                            enabledSensor = sensorClass.SHIMMER_LIS3MDL_MAG;
                        else
                            enabledSensor = sensorClass.SHIMMER_LSM303_MAG;
                        end
                    case 'AlternativeMag'
                        if this.shimmer.getHardwareVersion() == hwVersion.SHIMMER_3R
                            enabledSensor = sensorClass.SHIMMER_LIS2MDL_MAG_WR;
                        else
                            enabledSensor = sensorClass.SHIMMER_MPU9X50_MAG;
                        end
                    case 'ECG'
                        enabledSensor = sensorClass.HOST_ECG;
                    case 'EMG'
                        enabledSensor = sensorClass.HOST_EMG;
                    case 'EXG'
                        enabledSensor = sensorClass.HOST_EXG_TEST;
                    case 'GSR'
                        enabledSensor = sensorClass.SHIMMER_GSR;
                    case {'EXT A7', 'EXT A9'}
                        enabledSensor = sensorClass.SHIMMER_EXT_EXP_ADC_A7;
                    case {'EXT A6', 'EXT A11'}
                        enabledSensor = sensorClass.SHIMMER_EXT_EXP_ADC_A6;
                    case {'EXT A15', 'EXT A2'}
                        enabledSensor = sensorClass.SHIMMER_EXT_EXP_ADC_A15;
                    case 'Bridge Amplifier'
                        enabledSensor = sensorClass.SHIMMER_BRIDGE_AMP;
                    case 'BattVolt'
                        enabledSensor = sensorClass.SHIMMER_VBATT;
                    case {'INT A1', 'INT A7'}
                        enabledSensor = sensorClass.SHIMMER_INT_EXP_ADC_A1;
                    case {'INT A12', 'INT A10'}
                        enabledSensor = sensorClass.SHIMMER_INT_EXP_ADC_A12;
                    case {'INT A13', 'INT A15'}
                        enabledSensor = sensorClass.SHIMMER_INT_EXP_ADC_A13;
                    case {'INT A14', 'INT A16'}
                        enabledSensor = sensorClass.SHIMMER_INT_EXP_ADC_A14;
                    case 'Pressure'
                        if this.shimmer.getHardwareVersion() == hwVersion.SHIMMER_3R
                            enabledSensor = sensorClass.SHIMMER_BMP390_PRESSURE;
                        else
                            enabledSensor = sensorClass.SHIMMER_BMPX80_PRESSURE;
                        end
                    otherwise
                        error('Invalid Sensor Name');
                end

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
