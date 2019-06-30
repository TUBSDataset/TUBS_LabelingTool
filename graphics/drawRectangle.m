function [oRectangle_h] = drawRectangle(varargin)
% ---------------------------------------------------------------------------------------------
% Function drawRectangle(...) draws an point cloud object as an rectangle to the specifed axes. 
%
% INPUT:
%   oGUIData:       GUIData object.
%   oGUIObjects:    Object of class cGUIObjects.
%   oPCAxes_h:      PC axes object.
%   x,y:            Current point as defined by the user.
%   lw:             Line width, if specified
%
% OUTPUT:
%   oGUIData:       Changed GUIData object.
%   oGUIObjects:    Changed object of class cGUIObjects.

% Examples on how to call drawRectangle(...):
% drawRectangle(oAxes_h, vfColor, mfEdgePoints, voPCMovableLabels)  -> plots rectangle using edge points instead of using length, width and mid information
% drawRectangle(oAxes_h, vfColor, voPCMovableLabels, fGroundLevelZ) 
% drawRectangle(oAxes_h, vfColor, voPCMovableLabels, fGroundLevelZ, lw) 
% drawRectangle(oAxes_h, vfColor, mfEdgePoints, voPCMovableLabels, lw)
% ---------------------------------------------------------------------------------------------

oAxes_h = varargin{1};
vfColor = varargin{2};

bUseEdgePoints = 0;
if(~isa(varargin{3}, 'cPCMovableLabel'))
    mfEdgePoints        = varargin{3};
    voPCMovableLabels   = varargin{4};
    bUseEdgePoints = 1;
else
    voPCMovableLabels   = varargin{3};
    fGroundLevelZ        = varargin{4};
end
if(nargin == 5)
    lw = varargin{5};
else
   lw = 1; 
end

oRectangle_h = hggroup;
set(oRectangle_h, 'Parent', oAxes_h, 'Tag', 'BB_hg');

m_fBBYaw = voPCMovableLabels.m_fBBYaw * pi()/180;
lvec(1,1) = voPCMovableLabels.m_fBBLength/2*cos(m_fBBYaw);
lvec(2,1) = voPCMovableLabels.m_fBBLength/2*sin(m_fBBYaw);

if(bUseEdgePoints)
    mfEdgePoints(5,:) = mfEdgePoints(1,:);
    plot3(mfEdgePoints(:,1), mfEdgePoints(:,2), mfEdgePoints(:,3), 'Color', vfColor, 'LineStyle', '--', 'Parent', oRectangle_h);
else
    % Calculate edge points
    bbMiddlePlane(1,1) = voPCMovableLabels.m_fBBMiddle_x;
    bbMiddlePlane(2,1) = voPCMovableLabels.m_fBBMiddle_y;
    lvec(1,1) = voPCMovableLabels.m_fBBLength/2*cos(m_fBBYaw);
    lvec(2,1) = voPCMovableLabels.m_fBBLength/2*sin(m_fBBYaw);
    wvec(1,1) = voPCMovableLabels.m_fBBWidth/2*cos(m_fBBYaw + pi()/2);
    wvec(2,1) = voPCMovableLabels.m_fBBWidth/2*sin(m_fBBYaw + pi()/2);
    frontLeft = lvec + wvec + bbMiddlePlane; backLeft = -lvec + wvec + bbMiddlePlane;
    backRight = -lvec - wvec + bbMiddlePlane; frontRight = lvec - wvec + bbMiddlePlane;
    
    points(:,1) = frontLeft; 
    points(:,2) = backLeft;
    plot3(points(1,:), points(2,:), fGroundLevelZ*ones(1, length(points)), 'Linestyle','--','Color', vfColor, 'Parent', oRectangle_h, 'Tag', 'left', 'LineWidth', lw);
    points(:,1) = backLeft;
    points(:,2) = backRight;
    plot3(points(1,:), points(2,:), fGroundLevelZ*ones(1, length(points)), 'Linestyle','--','Color', vfColor, 'Parent', oRectangle_h, 'Tag', 'back', 'LineWidth', lw);
    points(:,1) = frontRight;
    points(:,2) = backRight;
    plot3(points(1,:), points(2,:), fGroundLevelZ*ones(1, length(points)), 'Linestyle','--','Color', vfColor, 'Parent', oRectangle_h, 'Tag', 'right', 'LineWidth', lw);
    points(:,1) = frontLeft;
    points(:,2) = frontRight;
    plot3(points(1,:), points(2,:), fGroundLevelZ*ones(1, length(points)), 'Linestyle','--','Color', vfColor, 'Parent', oRectangle_h, 'Tag', 'front', 'LineWidth', lw);
end

%% Draw length vector as indicator for front edge of BB
origin = [voPCMovableLabels.m_fBBMiddle_x, voPCMovableLabels.m_fBBMiddle_y, fGroundLevelZ];
lEnd = [voPCMovableLabels.m_fBBMiddle_x+lvec(1,1), voPCMovableLabels.m_fBBMiddle_y+lvec(2,1), fGroundLevelZ];

p = lEnd - origin;
x1 = lEnd(1);
y1 = lEnd(2);
alpha = 0.3;    % Size of arrow head relative to the length of the vector
beta = 0.3;     % Width of the base of the arrow head relative to the length
hu = [x1-alpha*(p(1)+beta*(p(2)+eps)), x1, x1-alpha*(p(1)-beta*(p(2)+eps))];
hv = [y1-alpha*(p(2)-beta*(p(1)+eps)), y1, y1-alpha*(p(2)+beta*(p(1)+eps))];

pts(:,1) = origin;
pts(:,2) = lEnd;
plot3(pts(1,:),pts(2,:), pts(3,:), 'Linestyle','-','Color', vfColor, 'Parent', oRectangle_h, 'Tag', 'arrow base', 'LineWidth', lw);
plot3(hu(:), hv(:), repmat(fGroundLevelZ, [length(hu), 1]), 'Color', vfColor', 'Parent', oRectangle_h, 'Tag', 'arrow head', 'LineWidth', lw);  % Plot arrow head
drawnow;

end

