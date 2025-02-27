%%
comPort = 'COM3';
captureDuration = 20;
fileName = 'DefaultTrial_Session1_test_Calibrated_SD.dat';

newplotandwriteexample(comPort, captureDuration, fileName);

%%
comPort = 'COM3';
captureDuration = 20;
fileName = 'DefaultTrial_Session1_test_Calibrated_PC_PPG.dat';

newppgtoheartrateexample(comPort, captureDuration, fileName);