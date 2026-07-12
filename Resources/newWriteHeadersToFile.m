function firstTime = newWriteHeadersToFile(fileName, signalNameArray, signalFormatArray, signalUnitArray)

%NEWWRITEHEADERSTOFILE - writes signal header rows to a data file
%
%  Opens FILENAME for writing (overwriting any existing content) and writes
%  three tab-delimited header rows: signal names, signal formats and signal
%  units.
%
%  INPUT: fileName - name of the file to write the headers to
%  INPUT: signalNameArray - cell array (or char row) of signal names
%  INPUT: signalFormatArray - cell array (or char row) of signal formats
%  INPUT: signalUnitArray - cell array (or char row) of signal units
%  OUTPUT: firstTime - always false; signals to the caller that the headers
%          have been written and should not be written again

    signalNames = validateHeaderInput(signalNameArray, 'signalNameArray');
    signalFormats = validateHeaderInput(signalFormatArray, 'signalFormatArray');
    signalUnits = validateHeaderInput(signalUnitArray, 'signalUnitArray');

    signalNamesString = strjoin(signalNames, '\t');
    signalFormatsString = strjoin(signalFormats, '\t');
    signalUnitsString = strjoin(signalUnits, '\t');

    fid = fopen(fileName, 'wt');
    if fid == -1
        error('newWriteHeadersToFile:cannotOpenFile', 'Cannot open file: %s', fileName);
    end
    closeFile = onCleanup(@() fclose(fid));

    fprintf(fid, '%s\n%s\n%s\n', signalNamesString, signalFormatsString, signalUnitsString);

    firstTime = false;
end

function values = validateHeaderInput(input, name)
    if ischar(input) && (isrow(input) || isempty(input))
        values = cellstr(input);
    elseif iscell(input)
        values = cellfun(@char, input, 'UniformOutput', false);
    else
        error('newWriteHeadersToFile:invalidInput', '%s must be a cell array or a char row.', name);
    end
end
