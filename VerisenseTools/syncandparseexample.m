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
    filepath = [parsedFilesPath '\' parsedFilesDirectory(1).name];

    for k = 1 : length(parsedFilesDirectory)
        if isempty(strfind(parsedFilesDirectory(k).name, 'Metadata'))
            filepath = [parsedFilesPath '\' parsedFilesDirectory(k).name];
            if ~isempty(strfind(parsedFilesDirectory(k).name, 'Accel'))
                plot_type = 'Accel';
            elseif ~isempty(strfind(parsedFilesDirectory(k).name, 'GSR'))
                plot_type = 'GSR';
            elseif ~isempty(strfind(parsedFilesDirectory(k).name, 'PPG'))
                plot_type ='PPG';
            end
            break
        end
    end

    switch plot_type
        case 'Accel'
            data = xlsread(filepath, '10:2000');
            plot(data(:,1),'color','blue')
            hold on
            plot(data(:,2),'color','red')
            plot(data(:,3),'color','green')
            hold off
            legend('Accel X', 'Accel Y', 'Accel Z')
            title('Accel')
        case 'GSR'
            data = xlsread(filepath, '10:2000');
            plot(data(:,1),'color','blue')
            legend('GSR')
            title('GSR')
        case 'PPG'
            data = xlsread(filepath, '10:2000');
            plot(data(:,1),'color','red')
            hold on
            plot(data(:,2),'color','m')
            plot(data(:,3),'color','green')
            plot(data(:,4),'color','blue')
            hold off
            legend('PPG RED', 'PPG IR', 'PPG GREEN', 'PPG BLUE')
            title('PPG')
        otherwise
            disp('plot does not exist')
    end

end
end

