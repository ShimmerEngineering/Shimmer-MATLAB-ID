classdef FilterClass < handle
   
    
    properties (SetAccess='protected', GetAccess='public')
        
    maxNPoles = 20;

    % filter parameters    
    samplingRate = nan;
    cornerFrequency = nan;
    nPoles = nan;
    pbRipple = nan;
    
    LoHi = 0; % low-pass (0), high-pass (1), band-pass (2), band-stop (3): default low-pass
    
    % buffered data (for filtering streamed data)
    bufferedX = [];
    
    % filter coefficients {a,b}
    num = [];           
    denom = [];
    
    % input parameters are invalid
    validparameters = false; 
    
    end % properties
    
    properties (GetAccess = 'public', Constant=true)
        LPF = 0;
        HPF = 1;
        BPF = 2;
        BSF = 3;        
    end % properties (macros)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Constructor Method
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        
        function thisFilter = FilterClass(varargin)
            
            % FILTERCLASS - Constructor for the class
            %
            %   FILTERCLASS creates a handle to an
            %   instance of the FilterClass. 
            %
            %   SYNOPSIS: thisFilter = FilterClass
            %
            %   OUTPUT: thisFilter - handle to the instance of the
            %                                FilterClass
            %
            %   EXAMPLE: thisFilter = FilterClass;
            %
            defaultNPoles = 2;
            defaultPBRipple = 0.5;
            switch nargin
                case 0
                    error('Error: at least one input parameter to choose low-pass or high-pass must be specified.');
                case 3
                    thisFilter.setFilterParameters(varargin{1}, varargin{2}, varargin{3}, defaultNPoles, defaultPBRipple);
                    disp('Warning: sampling rate and cornerFrequency specified; default values will be used for all other parameters.');
                case 4
                    thisFilter.setFilterParameters(varargin{1}, varargin{2}, varargin{3}, varargin{4}, defaultPBRipple);
                    disp('Warning: sampling rate, cornerFrequency and number of poles specified; default value will be used for passband ripple.');
                case 5
                    thisFilter.setFilterParameters(varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
                case 2
                    error('Error: invalid number of input parameters specified.');
                otherwise
                    thisFilter.validparameters = false;
                    error('Error: too many input parameters specified.');
            end
        end % function HighPassFilterClass  
        
    end % methods (Constructor)
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Public Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
    methods (Access = 'public')
             
        function resetFilterBuffer(thisFilter)
        % RESETFILTER - Resets the data buffer. This function should be
        % called if the filter is to be reused on different dataset.
        %   RESETFILTER() clears the contents of the bufferedX array.
        %   
        %   SYNOPSIS: thisFilter.resetFilterBuffer()
        %   
        %   EXAMPLE: thisFilter.resetFilterBuffer();
        
            thisFilter.bufferedX = [];
        end % function resetFilterBuffer()
        
        function setFilterParameters(thisFilter, LoHi, samplingRate, cornerFrequency, nPoles, pbRipple)
            % SETFILTERPARAMETERS - Sets filter parameters.
            % This is a Chebyshev low- or high-pass filter. Algorithm for
            % calculating filter coefficients from "The Scientist and 
            % Engineer's Guide to Digital Signal Processing", copyright 
            % ©1997-1998 by Steven W. Smith. For more information visit the
            % book's website at: www.DSPguide.com.
            %   SETFILTERPARAMETERS(SAMPLINGRATE, CORNERFREQUENCY,
            %   NPOLES, PBRIPPLE) provides filter coefficients for
            %   the specified parameters.
            %
            %   SYNOPSIS: thisFilter.setFilterParameters(samplingRate,
            %   cornerFrequency, PBripple, nPoles)
            %
            %   INPUT: samplingRate - sampling rate of the sampled data in
            %   units of Hz.
            %                                
            %   INPUT: cornerFrequency - corner frequency of the filter in
            %   units of Hz.
            %
            %   INPUT: nPoles - number of poles in the filter. Must be an
            %   even-valued integer (e.g. 2, 4, 6, ...).
            %
            %   INPUT: pbRipple - desired passband ripple of the filter
            %   measured as a percentage (e.g. pbRipple = 0.5 means 0.5%
            %   ripple).
            %
            %   EXAMPLE: thisFilter.setFilterParameters(0, 512, 5, 2, 0.5);
            %   EXAMPLE: thisFilter.setFilterParameters(2, 512, [5, 150], 2, 0.5);
            %
            thisFilter.resetFilterBuffer();
            if max(cornerFrequency) >= samplingRate/2
                thisFilter.num = nan;
                thisFilter.denom = nan;
                thisFilter.validparameters = false;
                error('Error: cornerFrequency is greater than or equal to Nyquist frequency. Please choose valid parameters.');
            else
                if nPoles > 20
                    nPoles = 20;
                    disp('Warning: nPoles will be decreased to 20.');
                elseif floor(nPoles/2) ~= nPoles/2
                    nPoles = 2*floor(nPoles/2);
                    fprintf(strcat(['Warning: nPoles is not an even number. nPoles will be rounded to ','',num2str(2*floor(nPoles/2)),'.\n']));
                end
                
                if ( LoHi == thisFilter.HPF || LoHi == thisFilter.LPF ) % low-pass or high-pass filter
                    thisFilter.samplingRate = samplingRate;
                    thisFilter.cornerFrequency = cornerFrequency;
                    thisFilter.nPoles = nPoles;
                    thisFilter.pbRipple = pbRipple;

                    % filter parameters
                    fc = cornerFrequency(1) / samplingRate;

                    % calculate filter coefficients
                    [numerator, denominator] = thisFilter.calculateCoefficients(fc, LoHi, pbRipple, nPoles);
                    thisFilter.num = numerator(1:nPoles+1);
                    thisFilter.denom = denominator(1:nPoles+1);

                    thisFilter.validparameters = true;
                elseif ( LoHi == thisFilter.BPF || LoHi == thisFilter.BSF ) % band-pass or band-stop filter
                    if ~( ( size(cornerFrequency,1) == 2 && size(cornerFrequency, 2) == 1 ) || ...
                            ( size(cornerFrequency,2) == 2 && size(cornerFrequency, 1) == 1 ) )
                        error('Error: Bandpass or bandstop filter requires two corner frequencies to be specified.');
                    end
                    thisFilter.samplingRate = samplingRate;
                    thisFilter.nPoles = nPoles;
                    thisFilter.pbRipple = pbRipple;

                    % filter parameters: corners at +/- 1Hz from notch
                    % center frequency
                    if LoHi == 2 % bandpass (High Pass Filter at lower corner frequency; Low Pass Filter at upper corner frequency
                        fcHigh = min(cornerFrequency) / samplingRate;
                        fcLow = max(cornerFrequency) / samplingRate;
                    else         % bandstop (High Pass Filter at upper corner frequency; Low Pass Filter at lower corner frequency
                        fcHigh = max(cornerFrequency) / samplingRate;
                        fcLow = min(cornerFrequency) / samplingRate;
                    end

                    % calculate filter coefficients
                    [numHighPass, denomHighPass] = thisFilter.calculateCoefficients(fcHigh, thisFilter.HPF, pbRipple, nPoles);
                    [numLowPass, denomLowPass] = thisFilter.calculateCoefficients(fcLow, thisFilter.LPF, pbRipple, nPoles);
                    numHighPass = [numHighPass(1:nPoles+1) zeros(1,nPoles)];
                    denomHighPass = [denomHighPass(1:nPoles+1) zeros(1,nPoles)];
                    numLowPass = [numLowPass(1:nPoles+1) zeros(1,nPoles)];
                    denomLowPass = [denomLowPass(1:nPoles+1) zeros(1,nPoles)];
                    
                    numerator = zeros(1, 2*nPoles + 1);
                    denominator = zeros(1, 2*nPoles + 1);
                    for i = 1:2*nPoles + 1
                        if LoHi == 2    % bandpass
                            for j = 1:i
                                numerator(i) = numerator(i) + numHighPass(j)*numLowPass(i-j+1);
                                denominator(i) = denominator(i) + denomHighPass(j)*denomLowPass(i-j+1);
                            end
                        else            % bandstop
                            for j = 1:i
                                numerator(i) = numerator(i) + numHighPass(j)*denomLowPass(i-j+1) + numLowPass(j)*denomHighPass(i-j+1);
                                denominator(i) = denominator(i) + denomHighPass(j)*denomLowPass(i-j+1);
                            end
                        end                                                   
                    end
                    
                    thisFilter.num = numerator(1:2*nPoles+1);
                    thisFilter.denom = denominator(1:2*nPoles+1);

                    thisFilter.validparameters = true;
                else
                    error('Error: Undefined filter type: use 0 - lowpass, 1 - highpass, 2 - bandpass, or 3 - bandstop.');
                end
            end
            
        end % function setFilterParameters

        function filteredData = filterData(thisFilter, data)
        % FILTERDATA - Filters the data according to the parameters
            if size(data,2) ~= 1
                error('Error: data input argument must be a single column; i.e. dimensions Nx1.');
            elseif ~thisFilter.validparameters
               filteredData = nan;
               disp('Error: filter parameters are invalid. Please set filter parameters before filtering data.');
            else
                nSamples = size(data,1);
                bufferSize = thisFilter.samplingRate*5;
                if isempty(thisFilter.bufferedX)
                    thisFilter.bufferedX = [data(1)*ones(bufferSize,1); data];
                else
                    thisFilter.bufferedX = [thisFilter.bufferedX(nSamples+1:end); data];
                end
                A = thisFilter.denom;
                B = thisFilter.num;
                
                Y = filter(B,A,thisFilter.bufferedX);
                filteredData = Y(end-nSamples+1:end);
            end
        end
        
    end %     methods (Access = 'public')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Private Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Access = 'private')
        
        function [numerator, denominator] = calculateCoefficients(thisFilter, fc, LoHi, pbRipple, nPoles)
            if ~( LoHi == thisFilter.HPF || LoHi == thisFilter.LPF )
                error('Error: the function calculateCoefficients() can only be called for LPF or HPF.');
            end
            % initialisation
            sizeAB = thisFilter.maxNPoles + 3;
            numerator = zeros(1,sizeAB);
            denominator = zeros(1,sizeAB);

            numerator(3) = 1;
            denominator(3) = 1;

            % calculate coefficients for each pole pair
            for p = 1:nPoles/2
                [A0, A1, A2, B1, B2] = thisFilter.calculateAB(fc, LoHi, pbRipple, nPoles, p);

                tempNum = numerator;
                tempDenom = denominator;

                for i = 3:sizeAB
                    numerator(i) = A0*tempNum(i) + A1 * tempNum(i-1) + A2*tempNum(i-2);
                    denominator(i) = tempDenom(i) - B1*tempDenom(i-1) - B2*tempDenom(i-2);
                end
            end

            % delete 'dummy' coefficients
            numerator = numerator(3:end);
            denominator = denominator(3:end);
            
            gain = thisFilter.calculateGain(numerator, denominator, LoHi);         
            numerator = numerator/gain;

        end

        function gain = calculateGain(thisFilter, numerator, denominator, LoHi)
            % Normalise the gain
            denominator(1) = 0;
            sNum = 0;
            sDenom = 0;
            for i = 1:length(numerator)
                if LoHi == 0
                   sNum = sNum + numerator(i);
                   sDenom = sDenom - denominator(i);
                elseif LoHi == 1
                   sNum = sNum + numerator(i)*(-1)^(i-1);
                   sDenom = sDenom + denominator(i)*(-1)^(i);
                else
                   sNum = sNum + sqrt(numerator(i)^2);
                   sDenom = sDenom + sqrt(denominator(i)^2);
                end                    

            end
            gain = sNum / (1 - sDenom);
        end % function calculateGain
        
        function [A0, A1, A2, B1, B2] = calculateAB(thisFilter, fc, LoHi, ripple, nPoles, pole)
            % Calculate pole location on unit circle
            realP = -cos(pi/(2*nPoles) + (pole-1)*pi/nPoles);
            imagP = sin(pi/(2*nPoles) + (pole-1)*pi/nPoles);
            
            % warp from circle to ellipse
            if ripple ~= 0
                eS = sqrt((100/(100-ripple))^2-1);
                vX = 1/nPoles*log(1/eS + sqrt(1/eS^2 + 1));
                kX = 1/nPoles*log(1/eS + sqrt(1/eS^2 - 1));
                kX = (exp(kX) + exp(-kX))/ 2;
                realP = realP * ((exp(vX) - exp(-vX))/2)/kX;
                imagP = imagP * ((exp(vX) + exp(-vX))/2)/kX;
            end
            
            % transform from s-domain to z-domain
            t = 2*tan(0.5);
            tSq = t^2;
            omega = 2*pi*fc;
            m = realP^2 + imagP^2;
            d = 4 - 4*realP*t + m*tSq;
            x0 = tSq/d;
            x1 = 2*x0;
            x2 = x0;
            y1 = (8 - 2*m*tSq)/d;
            y2 = (-4 -4*realP*t - m*tSq)/d;
            
            % LP to LP or LP to HP transform
            if LoHi == 0
                k = sin(0.5 - omega/2)/sin(0.5 + omega/2);
            else
                k = -cos(0.5 + omega/2)/cos(0.5 - omega/2);
            end
            kSq = k^2;
            d = 1 + y1*k - y2*kSq;
            
            A0 = (x0 - x1*k + x2*kSq)/d;
            A1 = (-2*x0*k + x1 + x1*kSq - 2*x2*k)/d;
            A2 = (x0*kSq - x1*k + x2)/d;
            
            B1 = (2*k + y1 + y1*kSq - 2*y2*k)/d;
            B2 = (-kSq - y1*k + y2)/d;
            
            if LoHi ~= 0
                A1 = -A1;
                B1 = -B1;
            end
            
        end % function calculateAB
        
    end %     methods (Access = 'private')        
end