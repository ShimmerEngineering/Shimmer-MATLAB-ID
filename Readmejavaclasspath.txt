Configuring Java Class Path for Older MATLAB Versions

This project requires the use of specific versions of the Apache Commons Math library:

- libs/commons-math-2.2.jar

- libs/commons-math3-3.6.jar

Older versions of MATLAB (such as MATLAB 2012 and MATLAB 2014) include outdated versions of these libraries. To ensure compatibility and correct functionality, it is necessary to override the built-in versions by modifying MATLAB’s Java class path using the javaclasspath.txt file.