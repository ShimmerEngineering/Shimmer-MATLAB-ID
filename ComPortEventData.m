classdef ComPortEventData < event.EventData
    properties
        ComPort
    end
    methods
        function data = ComPortEventData(cp)
            data.ComPort = cp;
        end
    end
end
