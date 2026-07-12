function void = configureexample( uuid, defaultconfig)
%CONFIGUREEXAMPLE - Demonstrate how to use the
%VerisenseConfigureAndSyncConsole app to write default operational configuration
%
%  CONFIGUREEXAMPLE(UUID, DEFAULTCONFIG) connect to the Verisense device using the uuid provided and update 
%  the operational configuration bytes to the default operational configuration. Note that you should pair 
%  the Verisense device prior of using  this function. For Windows, you can pair the Verisense device at 
%  Bluetooth & devices > Add a device > And enter the pin number for the Verisense device.
%
%  SYNOPSIS: configureexample( uuid, defaultconfig)
%
%  INPUT: uuid - UUID of the Verisense device. For Windows, you can get the mac address
%  at Control Panel > Devices and Printers > Properties e.g. d02b463da2bb.
%  The UUID will be the 00000000-0000-0000-0000- followed by the mac
%  address.
%
%  INPUT: defaultconfig - The default operational configuration (ACCEL1/ACCEL2_GYRO/GSR_BATT_ACCEL1/GSR_BATT/PPG).
%  OUTPUT: none
%
%  EXAMPLE: configureexample('00000000-0000-0000-0000-d02b463da2bb', 'ACCEL1')

allowedConfigs = {'ACCEL1','ACCEL2_GYRO','GSR_BATT_ACCEL1','GSR_BATT','PPG'};
if ~ismember(defaultconfig, allowedConfigs)
    error('configureexample:invalidDefaultConfig', ...
        'defaultconfig must be one of: %s', strjoin(allowedConfigs, ', '));
end

if ~ispc
    error('configureexample:unsupportedPlatform', ...
        'VerisenseConfigureAndSyncConsole.exe is only supported on Windows.');
end

toolsDir = fileparts(mfilename('fullpath'));
exe_path = fullfile(toolsDir, 'VerisenseConfigureAndSyncConsoleApp', 'VerisenseConfigureAndSyncConsole.exe');

[status, cmdout] = system(['"' exe_path '" "' uuid '" WRITE_DEFAULT_OPCONFIG "' defaultconfig '"']);
if status ~= 0
    error('configureexample:writeDefaultOpConfigFailed', ...
        'Failed to write default operational configuration: %s', cmdout);
end

end

