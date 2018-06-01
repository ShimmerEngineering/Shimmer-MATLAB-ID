Shimmer Biophysical Processing Library (Close source code) 
REV 0.10 (ShimmerBiophysicalProcessingLibrary_Rev_0_10.jar)

Changes since REV0.9
- added ECGtpHRAdaptive, note this is a more accurate and efficient vs ECGtoHRAlgorithm. 
- ECGtoHRAlgorithm can be deprecated

Changes since REV0.8
- repackaged not to include data folder

INCLUDES THE FOLLOWING CLASSES  
- PPGtoHRAlgorithm 
- ECGtoHRAlgorithm (Deprecated)
- ECGtoHRAdaptive


**********************************************
ECGtoHRAdaptive
Rev0.1
11/09/2015
***********************************************

This algorithm calculates heart rate from calibrated ECG data.

It is recommended that the input ECG signal is filtered prior to calling the method ecgToHrConversion

*Constructors

1. 	ECGtoHRAdaptive(double samplingRate)

2.	ECGtoHRAdaptive(double samplingRate, int HRWindowSize)

Where,
samplingRate: 			sampling rate in Hz

HRVWindowSize: 	size of window (in milliseconds) for calculating HRV metric

*Function calls

double heartRate = ecgToHrConversion(double ecgData, double ecgTimeStamp)
	computes heart rate
	ecgData is ECG data
	ecgTimeStamp is the time stamp for the ecg data in milliSeconds
	data fed in sample by sample
	heartRate is in BPM

double[] heartRate = ecgToHrConversion(double[] ecgData, double[] ecgTimeStamp)
	same functionality as above but takes in array of ecgData and ecgTimeStamp and returns array of heart rates

reset()
	Clears the algorithm buffers and resets variables to initial values.

setParameters(double rate)
	will reset all variables stored in the class


**********************************************
PPGtoHRAlgorithm
Rev0.10
08/05/2015
***********************************************

This algorithm calculates heart rate from calibrated PPG data.

It is recommended that the input PPG signal is filtered prior to calling the method ppgToHrConversion

-	Has been tested with second order Chebyshev LPF with corner freq 5Hz and second order Chebyshev HPF with corner freq 0.5Hz

*Constructors

1. 	PPGtoHRAlgorithm(double samplingRate)

2.	PPGtoHRAlgorithm(double samplingRate, int numberOfBeatsToAve)

3.	PPGtoHRAlgorithm(double samplingRate, int numberOfBeatsToAve,int trainingInterval) //training interval is legacy support

4.  PPGtoHRAlgorithm(double samplingRate, int numberOfBeatsToAve, boolean useLastEstimate)

Where,
samplingRate: 			sampling rate in Hz

numberOfBeatsToAve: 	the number of consecutive heart beats that are averaged to calculate the heart rate.
		    	(instantaneous heart rate is calculated after each detected pulse. So the last X instantaneous heart rates are averaged to give the output (where X is numberOfBeatsToAve))
		    	must be >= 1.
		    	typical value would be ~ 5.
		    	
trainingInterval: Legacy support (no effect on algorithm performance). 
	
useLastEstimate: True for repeating last valid estimate when invalid data is detected.

default values:	numberOfBeatsToAve = 5					

*FUNCTION CALLS

double heartRate = ppgToHrConversion(double ppgData, double ppgTimeStamp)
	computes heart rate
	ppgData is calibrated PPG data
	ppgTimeStamp is the time stamp for the ppg data in milliSeconds
	data fed in sample by sample
	returns -1 until training period is complete (i.e this has been called (trainingInterval*rate) times)
	also returns -1 if heartRate<30 or heartRate>215 or standard deviation of last X interbeat intervals > 100 (X = numberOfBeatsToAve and 100 is in ms)
	heartRate is in BPM


double[] heartRate = ppgToHrConversion(double[] ppgData, double[] ppgTimeStamp)
	same functionality as above but takes in array of ppgData and ppgTimeStamp and returns array of heart rates

resetParameters()
	resets all parameters stored in the class

setParameters(double rate, int numberOfBeatsToAve,int trainingInterval)
	same inputs as constructor
	will reset all variables stored in the class
	
retrain	()
	forces the ppg to hr algorithm to retrain, during which the heart rate is set to -1
-----------------------------------------------------------------------------------------------------------------------------------------------------------

DEPRECATED PLEASE USE ECGtoHRAdaptive
**********************************************
ECGtoHRAlgorithm
Rev0.11
08/05/2015
***********************************************

This algorithm calculates heart rate from calibrated ECG data.

It is recommended that the input ECG signal is filtered prior to calling the method ecgToHrConversion

*Constructors

1. 	ECGtoHRAlgorithm(double samplingRate)

2.	ECGtoHRAlgorithm(double samplingRate, int trainingInterval, int statisticalLimitTestBufferSize)

3.	ECGtoHRAlgorithm(double samplingRate, int trainingInterval, int statisticalLimitTestBufferSize, boolean useLastEstimate)

Where,
samplingRate: 			sampling rate in Hz

trainingInterval: 	of which to determine characteristics of R peak which is used to calculate heart rate. Training interval is in seconds

statisticalLimitTestBufferSize: number of interbeat intervals used in statistical test
            see function setStaticticalLimitsTest below for further details 
            
default values:		    trainingInterval = 10 secs	
						see function setStaticticalLimitsTest below for further details on default values for statistical test

useLastEstimate: True for repeating last valid estimate when invalid data is detected.

*Function calls

double heartRate = ecgToHrConversion(double ecgData, double ecgTimeStamp)
	computes heart rate
	ecgData is ECG data
	ecgTimeStamp is the time stamp for the ecg data in milliSeconds
	data fed in sample by sample
	returns -1 until training period is complete  
	also returns -1 if statistical test fails. See function setStaticticalLimitsTest below for further details
	heartRate is in BPM

double[] heartRate = ecgToHrConversion(double[] ecgData, double[] ecgTimeStamp)
	same functionality as above but takes in array of ecgData and ecgTimeStamp and returns array of heart rates

resetParameters()
	resets all parameters stored in the class

setParameters(double rate, int trainingInterval)
	same inputs as constructor
	will reset all variables stored in the class
	
setStaticticalLimitsTest(int testLimit, int statisticalLimitTestBufferSize, int heartRateUpperLimitTest, int heartRateLowerLimitTest){
	Used to set the statistical test for calculated heart rates. 
	Calculated heart rate is set to -1 if heartRate<heartRateLowerLimitTest or heartRate>heartRateUpperLimitTest or standard deviation of last X interbeat intervals > testLimit (X = statisticalLimitTestBufferSize; and testLimit is in ms) 
	By default testlimit is set to 100ms, upperHearRateLimit is 215, LowerHeartRate is 30 and statisticalLimitTestBufferSize is 1
     
retrain	()
	forces the ecg to hr algorithm to retrain, during which the heart rate value is set to -1
