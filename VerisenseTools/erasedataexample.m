function void = erasedataexample( uuid)
%ERASEDATAEXAMPLE - Demonstrate how to use the
%VerisenseConfigureAndSyncConsole app to erase the sensor data
%
%  ERASEDATAEXAMPLE(UUID) connect to the Verisense device using the uuid provided and erase the sensor data. 
%  Note that you should pair the Verisense device prior of using  this function. For Windows, you can pair 
%  the Verisense device at Bluetooth & devices > Add a device > And enter the pin number for the Verisense device.
%
%  SYNOPSIS: erasedataexample( uuid)
%
%  INPUT: uuid - UUID of the Verisense device. For Windows, you can get the mac address
%  at Control Panel > Devices and Printers > Properties e.g. d02b463da2bb.
%  The UUID will be the 00000000-0000-0000-0000- followed by the mac
%  address.
%
%  EXAMPLE: erasedataexample('00000000-0000-0000-0000-d02b463da2bb')

exe_path = 'VerisenseConfigureAndSyncConsoleApp\VerisenseConfigureAndSyncConsole.exe';

system([exe_path ' ' uuid ' ERASE_DATA'])

end

