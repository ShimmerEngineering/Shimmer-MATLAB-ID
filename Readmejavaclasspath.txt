Configuring Java Class Path for Older MATLAB Versions

This project requires specific versions of the Apache Commons Math library:

- libs/commons-math-2.2.jar
- libs/commons-math3-3.6.jar

Older MATLAB versions (such as MATLAB 2012 and MATLAB 2014) include outdated versions of these libraries. To ensure compatibility and correct functionality, you must override MATLAB’s built-in versions by modifying its Java class path.

To do this, create or update a javaclasspath.txt file in the same directory as MATLAB script and add the paths to the required JAR files. This ensures MATLAB loads the correct versions when running the script.