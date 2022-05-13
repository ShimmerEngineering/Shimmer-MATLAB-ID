function t = quatrotate(q, v)

%QUATROTATE - rotates the vector, v, by the quaternion, q.
%
%  QUATROTATE(Q, V) rotates the vector V, by the quaternion, Q, according 
%  to the rules of quaternion algebra.
%
%  SYNOPSIS: quatrotate(q, v)
%
%  INPUT: q - input quaternion
%  INPUT: v - input vector
%  OUTPUT: t - rotated vector
%
%  EXAMPLE: t = quatrotate([1,0,0,0], [0.5,0.5,0.5,0.5])


if size(q,2)~=4 || size(v,2)~=4 || size(q,2)~=size(v,2)
    disp('Error: input arrays must both be of dimension mx4.');
else
    numSamples = size(q,1);
    t = zeros(numSamples,4);
    for n = 1:numSamples
        t(n,:) =  quatmultiply(quatmultiply(q,v),quatconjugate(q));
    end
end

