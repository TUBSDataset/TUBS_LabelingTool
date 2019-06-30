function [nObjIdx] = determineClosestBox(voPCMovableLabel, vfPoint)
% ---------------------------------------------------------------------------------------------
% Function determineClosestBox(...) determines the closest object to a specified point. 
%
% INPUT:
%   voPCMovableLabel:   Object vector of class cPCMovableLabel, e.g. current point cloud objects. 
%   vfPoint:            Point specified by the user.
%
% OUTPUT:
%   nObjIdx:            Index of closest object.   
% ---------------------------------------------------------------------------------------------

% Extract middle position
index       = (1:1:(size(voPCMovableLabel,1)))';
bbMiddle_x  = [voPCMovableLabel.m_fBBMiddle_x]';
bbMiddle_y  = [voPCMovableLabel.m_fBBMiddle_y]';
sort = [index, bbMiddle_x, bbMiddle_y];

cells       = num2cell(sort, 2);
distFnc     = @(v) sqrt((vfPoint(1,1)-v(1,2))^2 + (vfPoint(2,1)-v(1,3))^2);
distance    = cellfun(distFnc, cells);
sort(:,4)   = distance;
sort        = sortrows(sort,4);

nObjIdx = sort(1,1);

end

