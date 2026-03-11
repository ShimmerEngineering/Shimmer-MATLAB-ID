function firstTime = newWriteHeadersToFile(fileName, signalNameArray, signalFormatArray, signalUnitArray)

    if iscell(signalNameArray)
        signalNames = cellfun(@char, signalNameArray, 'UniformOutput', false);
    end
    
    if iscell(signalFormatArray)
        signalFormats = cellfun(@char, signalFormatArray, 'UniformOutput', false);
    end
    
    if iscell(signalUnitArray)
        signalUnits = cellfun(@char, signalUnitArray, 'UniformOutput', false);
    end

    signalNamesString = strjoin(signalNames, '\t');
    signalFormatsString = strjoin(signalFormats, '\t');
    signalUnitsString = strjoin(signalUnits, '\t');

    fid = fopen(fileName, 'wt');
    if fid == -1
        error('Cannot open file: %s', fileName);
    end
    fprintf(fid, '%s\n%s\n%s\n', signalNamesString, signalFormatsString, signalUnitsString);
    fclose(fid);

    firstTime = false;
end
