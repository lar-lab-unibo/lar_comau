function [ R ] = roty( a )
%ANG2ROT Summary of this function goes here
%   Detailed explanation goes here

R=[cosd(a) 0 sind(a) 0
  0 1 0 0
  -sind(a) 0  cosd(a) 0
  0 0 0 1];

end

