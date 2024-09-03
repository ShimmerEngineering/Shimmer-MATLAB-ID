clear java;
javaaddpath('libs/ShimmerJavaClass.jar');
javaaddpath('libs/jssc-2.9.6.jar');
javaaddpath('libs/commons-math-2.2.jar');
javaaddpath('libs/vecmath-1.3.1.jar');
javaaddpath('libs/commons-lang3-3.8.1.jar');

obj = ShimmerJavaClass();

obj.connectDevice('COM8');

%%
% obj.configureDevice();
disp('File created.');
sensorName = {"Accel_LN_X", "Timestamp"};
csvFileName = 'sensor_data.csv';
csvFileID = fopen(csvFileName, 'w');
fprintf(csvFileID, 'Index,Timestamp,%s\n', sensorName{1});

%%
disp('Streaming started.');
obj.startStreaming();

figure;
hPlot = plot(nan, nan);
xlabel('Index');
ylabel('Data');
title(sensorName{1} + " Plot");

indexVec = [];
dataVec = [];

try
    while true
        data = obj.receiveData();

        if ~isempty(data) && numel(data) >= 5
            disp(['Data: ', num2str(data(1)), ', Timestamp: ', num2str(data(5))]);

            indexVec = [indexVec, length(indexVec) + 1];
            dataVec = [dataVec, data(1)];

            set(hPlot, 'XData', indexVec, 'YData', dataVec);

            if length(indexVec) > 1
                xlim([1, length(indexVec)]);
            end

            if length(dataVec) > 1
                ylim([min(dataVec), max(dataVec)]);
            end

            fprintf(csvFileID, '%d,%.6f,%.6f\n', length(indexVec), data(5), data(1));
        else
            disp('Received invalid or incomplete data.');
        end

        drawnow;
    end
catch e
    disp('Streaming interrupted.');
    disp(e.message);
end

%%
fclose(csvFileID);

obj.stopStreaming();
obj.disconnectDevice();

disp('Streaming stopped. Data saved to CSV file.');
