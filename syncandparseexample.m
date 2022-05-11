function void = syncandparseexample( uuid, binFilePath, trialName, participantID)

exe_path = 'VerisenseTools\VerisenseConfigureAndSyncConsoleApp\VerisenseConfigureAndSyncConsole.exe';

system([exe_path ' ' uuid ' DATA_SYNC ' binFilePath ' ' trialName ' ' participantID])

system(['java -jar VerisenseFileParserPC.jar ' binFilePath '\' trialName '\' participantID])

participantIDPath = [strrep(binFilePath,'\\','\') '\' trialName '\' participantID];
participantIDDirectory = dir([participantIDPath '\*']);

% remove currently directory and up one directory
participantIDDirectory(ismember( {participantIDDirectory.name}, {'.', '..'})) = [];

parsedFilesPath = [participantIDPath '\' participantIDDirectory(1).name '\ParsedFiles'];
parsedFilesDirectory = dir([parsedFilesPath '\*.csv']);
filepath = [parsedFilesPath '\' parsedFilesDirectory(1).name];

for k = 1 : length(parsedFilesDirectory)
if isempty(strfind(parsedFilesDirectory(k).name, 'Metadata'))
    filepath = [parsedFilesPath '\' parsedFilesDirectory(k).name];
    break
end
end

data = readtable(filepath, 'ReadVariableNames',false, 'HeaderLines',10);
plot(data.Var1)
title('Accel X')
end

