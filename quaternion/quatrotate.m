function t = quatrotate(q, v)

%QUATROTATE - rotates the vector, v, by the quaternion, q.
%
%  QUATROTATE(Q, V) rotates the vector V, by the quaternion, Q, according 
%  to the rules of quaternion algebra.
%
%  SYNOPSIS: quatrotate(q, v)
%
%  INPUT: q - input quaternion
%  INPUT: v - input vector supplied as an mx4 pure quaternion [0 vx vy vz]
%  OUTPUT: t - rotated vector
%
%  EXAMPLE: t = quatrotate([1,0,0,0], [0,0.5,0.5,0.5])


if size(q,2)~=4 || size(v,2)~=4
    error('quatrotate:invalidInput', 'input arrays must be of dimension mx4.');
end

numSamples = size(q,1);
t = zeros(numSamples,4);
for n = 1:numSamples
    t(n,:) =  quatmultiply(quatmultiply(q(n,:),v(n,:)),quatconjugate(q(n,:)));
end

