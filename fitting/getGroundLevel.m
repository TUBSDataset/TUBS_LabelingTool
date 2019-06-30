function [fGroundLevel] = getGroundLevel(vfPoint, mfPoints)
% ---------------------------------------------------------------------------------------------
% Function getGroundLevel(...) computes the ground level by averaging the ground level information
% of n closest points from a given set (e.g. points in an object's bounding box).
%
% INPUT:
%   vfPoint:        The point whose ground level is desired.
%   mfPoints:       Set of given points as a matrix.
%
% OUTPUT:
%   fGroundLevel:   Computed ground level
% ---------------------------------------------------------------------------------------------

bGetFromGrid = 0;
if nargin == 1
   bGetFromGrid = 1;
end

if ~bGetFromGrid 
    fGroundLevel = -1.8; % tbd
else
    fGroundLevel = -1.8;
end

% tbd...

end

