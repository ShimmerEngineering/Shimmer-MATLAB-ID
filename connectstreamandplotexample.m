clear java
javaaddpath('libs/ShimmerJavaClass.jar');
javaaddpath('libs/jssc-2.9.6.jar');
javaaddpath('libs/commons-math-2.2.jar');
javaaddpath('libs/vecmath-1.3.1.jar');
javaaddpath('libs/commons-lang3-3.8.1.jar');

obj = ShimmerJavaClass();

obj.connectDevice('COM8');

%% Display list of Object Cluster

obj.configureDevice();

%% 

obj.startStreaming();
disp('Streaming started.');

%%
while true
    data = obj.receiveData("Accel_LN_X");
    disp(data);
    pause(0.01);
end

%%

figure;
hPlot = plot(nan, nan);
xlabel('Index');
ylabel('Data');
title('Real-Time Data Plot');

indexVec = [];

dataVec = [];

while true
    data = obj.receiveData("Accel_LN_X");
    
    indexVec = [indexVec, length(indexVec) + 1];
    dataVec = [dataVec, data];
    
    set(hPlot, 'XData', indexVec, 'YData', dataVec);
    
    if length(indexVec) > 1
        xlim([1, length(indexVec)]);
    end
    
    if length(dataVec) > 1
        ylim([min(dataVec), max(dataVec)]);
    end
    
    drawnow;
    pause(0.01);
end

%%

obj.configureDevice();

%%

obj.stopStreaming();

%%

obj.disconnectDevice();