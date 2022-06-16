function void = plotfile( filepath)
%PLOTFILE - Plot the data based on the file name
%
%  PLOTFILE(FILEPATH) Plot the data based on the file name provided
%
%  SYNOPSIS: plotfile( filepath)
%
%  INPUT: filepath - the full path where the csv file is located
%
%  OUTPUT: none
%
%  EXAMPLE: plotfile('C:\Users\username\TrialA\ParticipantB\210827013745\ParsedFiles\220614_121244_Accel_DEFAULT_CAL_00000')

filename = regexp(filepath,'\','split');
filename = char(filename(end));

plot_type = '';
if ~isempty(strfind(filename, 'Accel'))
    plot_type = 'Accel';
elseif ~isempty(strfind(filename, 'Gyro'))
    plot_type ='Gyro';
elseif ~isempty(strfind(filename, 'GSR'))
    plot_type = 'GSR';
elseif ~isempty(strfind(filename, 'PPG'))
    plot_type ='PPG';
end

switch plot_type
    case 'Accel'
        data = xlsread(filepath);
        plot(data(:,1),'color','blue')
        hold on
        plot(data(:,2),'color','red')
        plot(data(:,3),'color','green')
        hold off
        legend('Accel X', 'Accel Y', 'Accel Z')
    case 'Gyro'
        data = xlsread(filepath);
        plot(data(:,1),'color','blue')
        hold on
        plot(data(:,2),'color','red')
        plot(data(:,3),'color','green')
        hold off
        legend('Gyro X', 'Gyro Y', 'Gyro Z')
    case 'GSR'
        data = xlsread(filepath);
        plot(data(:,1),'color','blue')
        legend('GSR')
    case 'PPG'
        data = xlsread(filepath);
        plot(data(:,1),'color','red')
        hold on
        plot(data(:,2),'color','m')
        plot(data(:,3),'color','green')
        plot(data(:,4),'color','blue')
        hold off
        legend('PPG RED', 'PPG IR', 'PPG GREEN', 'PPG BLUE')
    otherwise
        disp('plot does not exist')
end

title(strrep(filename,'_','\_'))
end

