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
%  EXAMPLE: syncandparseexample('00000000-0000-0000-0000-d02b463da2bb', 'C:\\Users\\Username\\Desktop', 
%  'TrialA', 'ParticipantB')


exe_path = 'VerisenseConfigureAndSyncConsoleApp\VerisenseConfigureAndSyncConsole.exe';

system([exe_path ' ' uuid ' DATA_SYNC ' binFilePath ' ' trialName ' ' participantID])

system(['java -jar FileParser\VerisenseFileParserPC.jar ' binFilePath '\' trialName '\' participantID])

participantIDPath = [strrep(binFilePath,'\\','\') '\' trialName '\' participantID];
participantIDDirectory = dir([participantIDPath '\*']);

% remove currently directory and up one directory
participantIDDirectory(ismember( {participantIDDirectory.name}, {'.', '..'})) = [];

parsedFilesPath = [participantIDPath '\' participantIDDirectory(1).name '\ParsedFiles'];
parsedFilesDirectory = dir([parsedFilesPath '\*.csv']);
dirSize = size(parsedFilesDirectory);
if dirSize(1) == 0
    disp('Parsed file not found');
else
    for k = 1 : length(parsedFilesDirectory)
        if isempty(strfind(parsedFilesDirectory(k).name, 'Metadata'))
            filepath = [parsedFilesPath '\' parsedFilesDirectory(k).name];
            figure(k)
            plotfile(filepath)
        end
    end


end
end

