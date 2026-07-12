function void = syncandparseexample( uuid, binFilePath, trialName, participantID)
%SYNCANDPARSEEXAMPLE - Demonstrate how to use the
%VerisenseConfigureAndSyncConsole app and the file parser jar
%
%  SYNCANDPARSEEXAMPLE(UUID, BINFILEPATH, TRIALNAME, PARTICIPANTID) connect and sync
%  the Verisense device using the uuid provided and store the binary files at the path provided. 
%  The function will also parse the data and plot some of the parsed data. Note that you should
%  pair the Verisense device prior of using this function. For Windows, you can pair
%  the Verisense device at Bluetooth & devices > Add a device >
%  And enter the pin number for the Verisense device.
%
%  SYNOPSIS: syncandparseexample( uuid, binFilePath, trialName, participantID)
%
%  INPUT: uuid - UUID of the Verisense device. For Windows, you can get the mac address
%  at Control Panel > Devices and Printers > Properties e.g. d02b463da2bb.
%  The UUID will be the 00000000-0000-0000-0000- followed by the mac
%  address.
%
%  INPUT: binFilePath - The path where the binary files are stored.
%  INPUT : trialName - Trial name.
%  INPUT : participantID - Participant ID.
%  OUTPUT: none
%
%  EXAMPLE: syncandparseexample('00000000-0000-0000-0000-d02b463da2bb', 'C:\Users\Username\Desktop',
%  'TrialA', 'ParticipantB')


if ~ispc
    error('syncandparseexample:unsupportedPlatform', ...
        'VerisenseConfigureAndSyncConsole.exe is only supported on Windows.');
end

toolsDir = fileparts(mfilename('fullpath'));
exe_path = fullfile(toolsDir, 'VerisenseConfigureAndSyncConsoleApp', 'VerisenseConfigureAndSyncConsole.exe');
jar_path = fullfile(toolsDir, 'FileParser', 'VerisenseFileParserPC.jar');

binFilePath = strrep(strrep(binFilePath, '\', filesep), '/', filesep);

[status, cmdout] = system(['"' exe_path '" "' uuid '" DATA_SYNC "' binFilePath '" "' trialName '" "' participantID '"']);
if status ~= 0
    error('syncandparseexample:dataSyncFailed', 'Failed to sync data: %s', cmdout);
end

participantIDPath = fullfile(binFilePath, trialName, participantID);

[status, cmdout] = system(['java -jar "' jar_path '" "' participantIDPath '"']);
if status ~= 0
    error('syncandparseexample:parseFailed', 'Failed to parse data: %s', cmdout);
end

participantIDDirectory = dir(participantIDPath);
participantIDDirectory = participantIDDirectory([participantIDDirectory.isdir]);

% remove currently directory and up one directory
participantIDDirectory(ismember( {participantIDDirectory.name}, {'.', '..'})) = [];

if isempty(participantIDDirectory)
    error('syncandparseexample:noSyncFolder', 'No sync folder found in %s', participantIDPath);
end

[~, sortOrder] = sort([participantIDDirectory.datenum], 'descend');
participantIDDirectory = participantIDDirectory(sortOrder);

parsedFilesPath = fullfile(participantIDPath, participantIDDirectory(1).name, 'ParsedFiles');
parsedFilesDirectory = dir(fullfile(parsedFilesPath, '*.csv'));
dirSize = size(parsedFilesDirectory);
if dirSize(1) == 0
    disp('Parsed file not found');
else
    plotCount = 0;
    for k = 1 : length(parsedFilesDirectory)
        % ignore metadata files
        if isempty(strfind(parsedFilesDirectory(k).name, 'Metadata'))
            filepath = fullfile(parsedFilesPath, parsedFilesDirectory(k).name);
            plotCount = plotCount + 1;
            figure(plotCount)
            plotfile(filepath)
        end
    end


end
end

