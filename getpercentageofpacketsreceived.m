function percentageofPacketsReceived = getpercentageofpacketsreceived(SamplingRate,continuousTimeStamp)

    numberofLostPackets=0;                      
            jitterMargin = 0.1;   % accommodate for minor possible variations in internal Shimmer clock
            for i=1:length(continuousTimeStamp)-1
                
                if (continuousTimeStamp(i+1)-continuousTimeStamp(i)) > ((1+jitterMargin)/double(SamplingRate)*1000) %To detect a discontinuity in the time stamp
                    numberofLostPackets=numberofLostPackets+round((continuousTimeStamp(i+1)-continuousTimeStamp(i))/(1/double(thisShimmer.SamplingRate)*1000))-1; %if there is a dropped packet, detect how many additional packets have been dropped
                end
                if (continuousTimeStamp(i+1)-continuousTimeStamp(i) <= 0)
                    disp('Warning: getpercentageofpacketsreceived - The percentage of packets received is unknown,')
                    disp('please check TimeStamp signal.')
                end
               
            end
            percentageofPacketsReceived=((length(continuousTimeStamp)-numberofLostPackets)/length(continuousTimeStamp))*100;
        end