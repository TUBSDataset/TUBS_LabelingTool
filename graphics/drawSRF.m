function [] = drawSRF(oPCAxes_h)
% ---------------------------------------------------------------------------------------------
% Function drawSRF(...) draw the velodyne's sensor reference frame into given axes.
%
% INPUT:
%   oPCAxes_h:  Parent axes object in which to draw 
% ---------------------------------------------------------------------------------------------

fSizeArrow       = 0.3;
fLength          = 3;
fLengthFactor    = 3;

colorX = 'r';
colorY = 'g';
colorZ = 'b';

origin = [0 0 0];
xEnd = [fLength 0 0];
yEnd = [0 fLength 0];
zEnd = [0 0 fLength];

% x arrow
points(1,:) = transpose(origin);
points(2,:) = transpose(xEnd);
plot3(points(:,1),points(:,2),points(:,3), 'Linestyle','-','Color', colorX, 'Parent', oPCAxes_h);
hu = [xEnd(1,1)-fLengthFactor*fSizeArrow; xEnd(1,1); xEnd(1,1)-fLengthFactor*fSizeArrow];
hv = [fSizeArrow; 0; -fSizeArrow];
hw = [0;0;0];
plot3(hu(:),hv(:),hw(:),'Color',colorX','Parent',oPCAxes_h);  % Plot arrow head

% y arrow
points(1,:) = transpose(origin);
points(2,:) = transpose(yEnd);
plot3(points(:,1),points(:,2),points(:,3), 'Linestyle','-','Color', colorY, 'Parent', oPCAxes_h);
hu = [-fSizeArrow; 0; fSizeArrow];
hv = [yEnd(1,2)-fLengthFactor*fSizeArrow; yEnd(1,2); yEnd(1,2)-fLengthFactor*fSizeArrow];
hw = [0;0;0];
plot3(hu(:),hv(:),hw(:),'Color',colorY','Parent',oPCAxes_h);  % Plot arrow head

% z arrow
points(1,:) = transpose(origin);
points(2,:) = transpose(zEnd);
plot3(points(:,1),points(:,2),points(:,3), 'Linestyle','-','Color', colorZ, 'Parent', oPCAxes_h);

hu = [-fSizeArrow; 0; fSizeArrow];
hv = [0; 0; 0;];
hw = [zEnd(1,3)-fLengthFactor*fSizeArrow; zEnd(1,3); zEnd(1,3)-fLengthFactor*fSizeArrow];
plot3(hu(:),hv(:),hw(:),'Color',colorZ','Parent',oPCAxes_h);  % Plot arrow head

end

