function BB_hg = drawBB(fBBMiddle_x, fBBMiddle_y, fBBMiddle_z, fBBYaw, fBBLength, fBBWidth, fBBHeight, vnColor, oAxes_h, sLine)
% ---------------------------------------------------------------------------------------------
% Function drawBB(...) draws a bounding box as specified by its coordinates, orientation and dimensions.
%
% INPUT:
%   vnColor:        Color vector in which to draw the bounding boxs, e.g. [1 0 0]
%   oAxes_h:        Handle to parent axes object
%   sLine:          String that specifies a dotted or solid line, e.g. '-'
%   
%
% OUTPUT:
%   BB_hg:           hg-group of created boundig box graphics object
% ---------------------------------------------------------------------------------------------
BB_hg = hggroup;
set(BB_hg, 'Parent', oAxes_h, 'Tag', 'BB_hg');


fBBYaw = fBBYaw * pi()/180;
plot3(fBBMiddle_x, fBBMiddle_y, fBBMiddle_z ,'x','MarkerEdgeColor', vnColor,'MarkerFaceColor', vnColor, 'MarkerSize', 9, 'Parent', BB_hg, 'Visible', 'off', 'Tag', 'mid')

%% Create edge points
bbMiddlePlane(1,1) = fBBMiddle_x;
bbMiddlePlane(2,1) = fBBMiddle_y;

lvec(1,1) = fBBLength/2*cos(fBBYaw);
lvec(2,1) = fBBLength/2*sin(fBBYaw);
wvec(1,1) = fBBWidth/2*cos(fBBYaw + pi()/2);
wvec(2,1) = fBBWidth/2*sin(fBBYaw + pi()/2);

frontLeftD  =  lvec + wvec + bbMiddlePlane;
backLeftD   = -lvec + wvec + bbMiddlePlane;
backRightD  = -lvec - wvec + bbMiddlePlane;
frontRightD =  lvec - wvec + bbMiddlePlane;

% Downside
frontLeftD(3,1)     = fBBMiddle_z - fBBHeight/2;
backLeftD(3,1)      = fBBMiddle_z - fBBHeight/2;
backRightD(3,1)     = fBBMiddle_z - fBBHeight/2;
frontRightD(3,1)    = fBBMiddle_z - fBBHeight/2;

%% Draw bounding box
points(:,1) = frontLeftD; 
points(:,2) = backLeftD;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine, 'Color', vnColor, 'Parent', BB_hg, 'Tag', 'left');
points(:,1) = backLeftD;
points(:,2) = backRightD;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'back');
points(:,1) = frontRightD;
points(:,2) = backRightD;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'right');
points(:,1) = frontLeftD;
points(:,2) = frontRightD;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'front');

% Upside
frontLeftU  = frontLeftD    +   [0;0;fBBHeight];
backLeftU   = backLeftD     +   [0;0;fBBHeight];
backRightU  = backRightD    +   [0;0;fBBHeight];
frontRightU = frontRightD   +   [0;0;fBBHeight];

points(:,1) = frontLeftU; 
points(:,2) = backLeftU;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'left');
points(:,1) = backLeftU;
points(:,2) = backRightU;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'back');
points(:,1) = frontRightU;
points(:,2) = backRightU;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'right');
points(:,1) = frontLeftU;
points(:,2) = frontRightU;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'front');

% Up strokes
points(:,1) = frontLeftD;
points(:,2) = frontLeftU;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'up front left');
points(:,1) = backLeftD;
points(:,2) = backLeftU;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'up back left');
points(:,1) = backRightD;
points(:,2) = backRightU;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'up back right');
points(:,1) = frontRightD;
points(:,2) = frontRightU;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine,'Color', vnColor, 'Parent', BB_hg, 'Tag', 'up front right');

% Draw markers
offset = 0.4; scale = 0.3; sLine = '-';
lvec(1,1) = (fBBLength+offset)/2*cos(fBBYaw);
lvec(2,1) = (fBBLength+offset)/2*sin(fBBYaw);
wvec(1,1) = (fBBWidth+offset)/2*cos(fBBYaw + pi()/2);
wvec(2,1) = (fBBWidth+offset)/2*sin(fBBYaw + pi()/2);

frontLeftM  =  lvec + wvec + bbMiddlePlane;
backLeftM   = -lvec + wvec + bbMiddlePlane;
backRightM  = -lvec - wvec + bbMiddlePlane;
frontRightM =  lvec - wvec + bbMiddlePlane;

frontLeftM(3,1)     = fBBMiddle_z;
backLeftM(3,1)      = fBBMiddle_z;
backRightM(3,1)     = fBBMiddle_z;
frontRightM(3,1)    = fBBMiddle_z;

% Upper left corner
vec    = (frontRightM-frontLeftM);
dest   = vec .* scale ./ norm(vec) + frontLeftM;
points(:,1) = frontLeftM;
points(:,2) = dest;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine, 'Color', [0 0 0], 'Parent', BB_hg, 'Tag', 'marker', 'Visible', 'off');
vec    = (backLeftM-frontLeftM);
dest   = vec .* scale ./ norm(vec) + frontLeftM;
points(:,2) = dest;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine, 'Color', [0 0 0], 'Parent', BB_hg, 'Tag', 'marker', 'Visible', 'off');

% Upper right 
vec    = (frontLeftM-frontRightM);
dest   = vec .* scale ./ norm(vec) + frontRightM;
points(:,1) = frontRightM;
points(:,2) = dest;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine, 'Color', [0 0 0], 'Parent', BB_hg, 'Tag', 'marker', 'Visible', 'off');
vec    = (backRightM-frontRightM);
dest   = vec .* scale ./ norm(vec) + frontRightM;
points(:,2) = dest;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine, 'Color', [0 0 0], 'Parent', BB_hg, 'Tag', 'marker', 'Visible', 'off');

% Back right
vec    = (backLeftM-backRightM);
dest   = vec .* scale ./ norm(vec) + backRightM;
points(:,1) = backRightM;
points(:,2) = dest;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine, 'Color', [0 0 0], 'Parent', BB_hg, 'Tag', 'marker', 'Visible', 'off');
vec    = (frontRightM-backRightM);
dest   = vec .* scale ./ norm(vec) + backRightM;
points(:,2) = dest;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine, 'Color', [0 0 0], 'Parent', BB_hg, 'Tag', 'marker', 'Visible', 'off');

% Back left
vec    = (frontLeftM-backLeftM);
dest   = vec .* scale ./ norm(vec) + backLeftM;
points(:,1) = backLeftM;
points(:,2) = dest;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine, 'Color', [0 0 0], 'Parent', BB_hg, 'Tag', 'marker', 'Visible', 'off');
vec    = (backRightM-backLeftM);
dest   = vec .* scale ./ norm(vec) + backLeftM;
points(:,2) = dest;
plot3(points(1,:),points(2,:),points(3,:), 'Linestyle', sLine, 'Color', [0 0 0], 'Parent', BB_hg, 'Tag', 'marker', 'Visible', 'off')
end

