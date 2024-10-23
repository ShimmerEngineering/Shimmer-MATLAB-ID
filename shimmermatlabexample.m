clear java;
javaaddpath('libs/ShimmerJavaClass.jar');
javaaddpath('libs/jssc-2.9.6.jar');
javaaddpath('libs/vecmath-1.3.1.jar');
javaaddpath('libs/commons-lang3-3.8.1.jar');

import com.shimmerresearch.driverUtilities.AssembleShimmerConfig;
import com.shimmerresearch.driver.ShimmerDevice;
import com.shimmerresearch.tools.bluetooth.BasicShimmerBluetoothManagerPc;

btManager = BasicShimmerBluetoothManagerPc();
comPort = 'COM5';
SAMPLE_RATE = 51.2;

%%

btManager.connectShimmerThroughCommPort(comPort);
pause(5);
disp('Device Connected');

%%

shimmerDevice = btManager.getShimmerDeviceBtConnected(comPort);
cloneDevice = shimmerDevice.deepClone();
cloneDevice.setEnabledAndDerivedSensorsAndUpdateMaps(0,0);
cloneDevice.setSensorEnabledState(2, true);
cloneDevice.setSamplingRateShimmer(SAMPLE_RATE);

commType = javaMethod('valueOf', 'com.shimmerresearch.driver.Configuration$COMMUNICATION_TYPE', 'BLUETOOTH');
AssembleShimmerConfig.generateSingleShimmerConfig(cloneDevice, commType);
shimmerDevice.configureFromClone(cloneDevice);

btState = javaMethod('valueOf', 'com.shimmerresearch.bluetooth.ShimmerBluetooth$BT_STATE', 'CONFIGURING');
shimmerDevice.operationStart(btState);
disp('Device Configured');
%%

shimmerDevice.startStreaming();
disp('Device Streamed')

%%

shimmerDevice.stopStreaming();

