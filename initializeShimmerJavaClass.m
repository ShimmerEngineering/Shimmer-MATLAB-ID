function ShimmerPC = InitializeShimmerJavaClass()
%INITIALIZESHIMMERJAVACLASS Initialize Shimmer Java Class and return ShimmerPC instance
%   This function sets up the required Java classpaths and initializes the
%   BasicShimmerBluetoothManagerPc instance for use in Shimmer applications.

    clear java;

    javaaddpath('libs/ShimmerJavaClass.jar');
    javaaddpath('libs/jssc-2.9.6.jar');
    javaaddpath('libs/vecmath-1.3.1.jar');
    javaaddpath('libs/commons-lang3-3.8.1.jar');
    
    ShimmerPC = com.shimmerresearch.tools.bluetooth.BasicShimmerBluetoothManagerPc();
   
end
