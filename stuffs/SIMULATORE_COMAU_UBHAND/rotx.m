function [ R ] = rotx( a )
%ANG2ROT Summary of this function goes here
%   Detailed explanation goes here

R=[1 0 0 0
   0 cosd(a) -sind(a) 0
   0 sind(a) cosd(a) 0
   0 0 0 1] ;

end

