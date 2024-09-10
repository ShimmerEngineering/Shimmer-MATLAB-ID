clear java;
javaaddpath('libs/ShimmerJavaClass.jar');
javaaddpath('libs/jssc-2.9.6.jar');
javaaddpath('libs/vecmath-1.3.1.jar');
javaaddpath('libs/commons-lang3-3.8.1.jar');
 
%%
obj = ShimmerJavaClass();
 
obj.connectDevice('COM3');
 
disp('File created.');
sensorName = 'Accel-LN-X';
csvFileName = 'sensor_data.csv';
csvFileID = fopen(csvFileName, 'w');
fprintf(csvFileID, 'Index,Timestamp,%s\n', sensorName);
 
figure;
hPlot = plot(nan, nan);
xlabel('Index');
ylabel('Data');
title([sensorName ' Plot']);
 
indexVec = [];
dataVec = [];
 
%%
 
disp('Streaming started.');
obj.startStreaming();
 
try
    while true
        data = obj.receiveData();
 
        if ~isempty(data) && numel(data) >= 5
            disp(['Data: ', num2str(data(1)), ', Timestamp: ', num2str(data(5))]);
 
            indexVec = [indexVec, length(indexVec) + 1];
            dataVec = [dataVec, data(1)];
 
            set(hPlot, 'XData', indexVec, 'YData', dataVec);
 
            fprintf(csvFileID, '%d,%.6f,%.6f\n', length(indexVec), data(5), data(1));
        end
        drawnow;
    end
catch e
    disp(e.message);
    disp('Streaming interrupted.');
end
 
%%
fclose(csvFileID);

obj.stopStreaming();
obj.disconnectDevice();
 
disp('Streaming stopped. Data saved to CSV file.');