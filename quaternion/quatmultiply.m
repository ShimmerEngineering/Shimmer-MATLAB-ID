function t = quatmultiply(q, r)

%QUATMULTIPLY - calculates the product of two quaternions
%
%  QUATMULTIPLY(Q, R) calculates the quaternion product, Q x R, of the two 
%  quaternion input arguments.  
%
%  SYNOPSIS: quatmultiply(q, r)
%
%  INPUT: q - first input quaternion
%  INPUT: r - second input quaternion
%  OUTPUT: t - the quaternion product of the input quaternions
%
%  EXAMPLE: t = quatmultiply([1,0,0,0], [0.5,0.5,0.5,0.5])


if size(q,2)~=4 || size(r,2)~=4 || size(q,2)~=size(r,2)
    disp('Error: input arrays must both be of dimension mx4.');
else
    numSamples = size(q,1);
    t = zeros(numSamples,4);
    for n = 1:numSamples
        q0 = q(n,1);
        q1 = q(n,2);
        q2 = q(n,3);
        q3 = q(n,4);

        r0 = r(n,1);
        r1 = r(n,2);
        r2 = r(n,3);
        r3 = r(n,4);

        t(n,1) = r0*q0 - r1*q1 - r2*q2 - r3*q3;
        t(n,2) = r0*q1 + r1*q0 - r2*q3 + r3*q2;
        t(n,3) = r0*q2 + r1*q3 + r2*q0 - r3*q1;
        t(n,4) = r0*q3 - r1*q2 + r2*q1 + r3*q0;
    end
end

