function firstTime = writeHeadersToFile(fileName,signalNameArray,signalFormatArray,signalUnitArray)
%WRITEHEADERSTOFILE - writes signal information headers to file

signalNamesString = char(signalNameArray(1,1));                                           % Create a single string, signalNamesString, which lists the names of the enabled sensors
signalFormatsString = char(signalFormatArray(1,1));                                       % Create a single string, signalFormatArray, which lists the signal format
signalUnitsString = char(signalUnitArray(1,1));                                           % Create a single string, signalUnitArray, which lists the signal units
for i= 2:length(signalNameArray)
    tabbedNextSignalName = [char(9), char(signalNameArray(1,i))];                         % Add tab delimiter before signal name
    signalNamesString = strcat(signalNamesString,tabbedNextSignalName);                   % Concatenate signal names delimited by a tab.
    tabbedNextSignalFormat = [char(9), char(signalFormatArray(1,i))];                     % Add tab delimiter before signal format
    signalFormatsString = strcat(signalFormatsString,tabbedNextSignalFormat);             % Concatenate signal formats delimited by a tab.
    tabbedNextSignalUnit = [char(9), char(signalUnitArray(1,i))];                         % Add tab delimiter before signal unit
    signalUnitsString = strcat(signalUnitsString,tabbedNextSignalUnit);                   % Concatenate signal units delimited by a tab.
    firstTime=false;
end
% write headers to file
headerLines = {signalNamesString; signalFormatsString; signalUnitsString};
fid = fopen(fileName, 'wt');
for l = 1:numel(headerLines)
    fprintf(fid, '%s\n',headerLines{l});
end
fclose(fid);

end
