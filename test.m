%%
comPort = 'COM3';
captureDuration = 20;
fileName = 'DefaultTrial_Session1_test_Calibrated_SD';

newplotandwriteexample(comPort, captureDuration, fileName);

%%
comPort = 'COM3';
PPGChannelNum = 13;
captureDuration = 20;
fileName = 'DefaultTrial_Session1_test_Calibrated_PC_PPG';

newppgtoheartrateexample(comPort, PPGChannelNum, captureDuration, fileName);