comPort = 'COM3';
captureDuration = 20;
fileName = 'DefaultTrial_Session1_test_Calibrated_SD';

newplotandwriteexample(comPort, captureDuration, fileName);

%%
comPort = 'COM5';
captureDuration = 20;
fileName = 'SyncTrial_3_Session1_Shimmer_6749_Calibrated_PC_ECG';

newecgtoheartrateexample(comPort, captureDuration, fileName);