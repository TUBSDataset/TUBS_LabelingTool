function [Vvec_hg] = drawVelocityVector(oPCMovableLabel, vnColor, BB_hg)
% ---------------------------------------------------------------------------------------------
% Function drawVelocityVector draws an object's velocity.
%
% INPUT:
%   oPCMovableLabel:    Object of class cPCMovableLabel containing velocity data
%   vnColor:            Bounding box color
%   BB_hg:              Bounding box graphic object (hg group)
%
% OUTPUT:
%   BB_hgs:             Velocity vector graphic object
% ---------------------------------------------------------------------------------------------

Vvec_hg = hggroup;
set(Vvec_hg, 'Parent', BB_hg, 'Tag', 'velocity vector');

fBBYaw = oPCMovableLabel.m_fBBYaw * pi()/180;

% middle point
bbMiddlePlane(1,1) = oPCMovableLabel.m_fBBMiddle_x;
bbMiddlePlane(2,1) = oPCMovableLabel.m_fBBMiddle_y;
lvec(1,1) = oPCMovableLabel.m_fBBLength/2*cos(fBBYaw);
lvec(2,1) = oPCMovableLabel.m_fBBLength/2*sin(fBBYaw);

startPoint(1,1) = bbMiddlePlane(1,1) + lvec(1,1);
startPoint(2,1) = bbMiddlePlane(2,1) + lvec(2,1);
startPoint(3,1) = oPCMovableLabel.m_fBBMiddle_z;

velPhi      = atan2(oPCMovableLabel.m_fVyAbs, oPCMovableLabel.m_fVxAbs);
lengthNorm  = sqrt(oPCMovableLabel.m_fVyAbs^2 + oPCMovableLabel.m_fVxAbs^2) / 13.8*10;

endPoint(1,1) = startPoint(1) + lengthNorm * cos(velPhi);
endPoint(2,1) = startPoint(2) + lengthNorm * sin(velPhi);
endPoint(3,1) = oPCMovableLabel.m_fBBMiddle_z;

plot3(startPoint(1,:), startPoint(2,:), startPoint(3,:),  '.', 'Color', 'black', 'MarkerSize', 10, 'Parent', Vvec_hg, 'Tag', 'front point');

%% Draw vector
p  = endPoint - startPoint;
x1 = endPoint(1);
y1 = endPoint(2);
alpha   = 0.3;    % Size of arrow head relative to vector's length
beta    = 0.3;     % Width of the base of the arrow head (length relative)
hu = [x1-alpha*(p(1)+beta*(p(2)+eps)), x1, x1-alpha*(p(1)-beta*(p(2)+eps))];
hv = [y1-alpha*(p(2)-beta*(p(1)+eps)), y1, y1-alpha*(p(2)+beta*(p(1)+eps))];

pts(:,1) = startPoint;
pts(:,2) = endPoint;
plot3(pts(1,:),pts(2,:), pts(3,:), 'Linestyle','-','Color', vnColor, 'Parent', Vvec_hg, 'Tag', 'velocity arrow base');
plot3(hu(:), hv(:), repmat(endPoint(3,1), [length(hu), 1]), 'Color', vnColor', 'Parent', Vvec_hg, 'Tag', 'velocity arrow head');  % Plot arrow head

end

