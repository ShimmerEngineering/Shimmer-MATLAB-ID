function t = quatconjugate(q)

%QUATCONJUGATE - calculates the conjugate of a quaternion
%
%  QUATCONJUGATE(Q) calculates the conjugate of the  
%  quaternion input argument.  
%
%  SYNOPSIS: orientation3Dexample(comPort, captureDuration, fileName)
%
%  INPUT: q - input quaternion
%  OUTPUT: t - the conjugate of the input quaternion
%
%  EXAMPLE: t = quatconjugate([0.5,0.5,0.5,0.5])


if size(q,2)~=4
    disp('Error: input array must be of dimension mx4.');
else
    numSamples = size(q,1);
    t = zeros(numSamples,4);
    for n = 1:numSamples
        q0 = q(n,1);
        q1 = q(n,2);
        q2 = q(n,3);
        q3 = q(n,4);

        t(n,1) = q0;
        t(n,2) = -q1;
        t(n,3) = -q2;
        t(n,4) = -q3;
    end
end

