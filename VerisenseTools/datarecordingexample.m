function void = datarecordingexample( uuid, durationInMinutes, defaultconfig, binFilePath, trialName, participantID)
%DATARECORDINGEXAMPLE - Demonstrate how to do a data recording using
%VerisenseConfigureAndSyncConsole app and the file parser jar
%
%  DATARECORDINGEXAMPLE(UUID, DURATIONINMINUTES, DEFAULTCONFIG, BINFILEPATH, TRIALNAME, PARTICIPANTID) 
%  erase data on the device and set default operational config. Collect data throughout the duration specified 
%  and sync the Verisense device and store the binary files at the path provided. Parse the data and plot some 
%  of the parsed data. Disable logging after the data collection is completed. Note that you should pair the 
%  Verisense device prior of using this function. For Windows, you can pair the Verisense device at 
%  Bluetooth & devices > Add a device > Enter the pin number for the Verisense device.
%
%  SYNOPSIS: datarecordingexample( uuid, durationinminutes, defaultconfig, binFilePath, trialName, participantID)
%
%  INPUT: uuid - UUID of the Verisense device. For Windows, you can get the mac address
%  at Control Panel > Devices and Printers > Properties e.g. d02b463da2bb.
%  The UUID will be the 00000000-0000-0000-0000- followed by the mac
%  address.
%
%  INPUT: durationinminutes - The duration of the data collection.
%  INPUT: defaultconfig - The default operational configuration (ACCEL1/ACCEL2_GYRO/GSR_BATT_ACCEL1/GSR_BATT/PPG).
%  INPUT: binFilePath - The path where the binary files are stored.
%  INPUT : trialName - Trial name.
%  INPUT : participantID - Participant ID.
%  OUTPUT: none
%
%  EXAMPLE: datarecordingexample('00000000-0000-0000-0000-d02b463da2bb', 1, 'ACCEL1',
%  'C:\Users\WeiWentan\Desktop', 'Trial', 'Participant')

allowedConfigs = {'ACCEL1','ACCEL2_GYRO','GSR_BATT_ACCEL1','GSR_BATT','PPG'};
if ~ismember(defaultconfig, allowedConfigs)
    error('datarecordingexample:invalidDefaultConfig', ...
        'defaultconfig must be one of: %s', strjoin(allowedConfigs, ', '));
end

if ~isnumeric(durationInMinutes) || ~isscalar(durationInMinutes) || durationInMinutes <= 0
    error('datarecordingexample:invalidDuration', ...
        'durationInMinutes must be a positive numeric scalar.');
end

if ~ispc
    error('datarecordingexample:unsupportedPlatform', ...
        'VerisenseConfigureAndSyncConsole.exe is only supported on Windows.');
end

toolsDir = fileparts(mfilename('fullpath'));
exe_path = fullfile(toolsDir, 'VerisenseConfigureAndSyncConsoleApp', 'VerisenseConfigureAndSyncConsole.exe');

disp('Erasing data')
[status, cmdout] = system(['"' exe_path '" "' uuid '" ERASE_DATA']);
if status ~= 0
    error('datarecordingexample:eraseDataFailed', 'Failed to erase data: %s', cmdout);
end

disp('Writing default operational config')
[status, cmdout] = system(['"' exe_path '" "' uuid '" WRITE_DEFAULT_OPCONFIG "' defaultconfig '"']);
if status ~= 0
    error('datarecordingexample:writeDefaultOpConfigFailed', ...
        'Failed to write default operational configuration: %s', cmdout);
end

disp('Data Collecton start')
pause(durationInMinutes * 60)
disp('Data Collecton end')

syncandparseexample( uuid, binFilePath, trialName, participantID)

[status, cmdout] = system(['"' exe_path '" "' uuid '" DISABLE_LOGGING']);
if status ~= 0
    error('datarecordingexample:disableLoggingFailed', 'Failed to disable logging: %s', cmdout);
end

end

