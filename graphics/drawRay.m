function [oRay_h, oPix_h] = drawRay(x, y, mfMap_x, mfMap_y, oImageAxes_h, oPCAxes_h, oCalibration)
% ---------------------------------------------------------------------------------------------
% Function drawRay(...) draws a ray of sight specified by its x and y position within the image plane
% into the point cloud axes.

% INPUT:
%   x               x-position within image plane (pixel)
%   y               y-position within image plane (pixel)
%   mfMap_x         Matrix containing distorted x-values for given undistorted pixels (row, column)
%   mfMap_y         Matrix containing distorted <-values for given undistorted pixels (row, column)
%   oImageAxes_h    Handle to current image axes
%   oPCAxes_h       Handle to point cloud axes
%   oCalibration    Object of class cCalibration containing camera calibration matrices
%
% OUTPUT:
%   oRay_h          Generated Ray graphic handle
%   oPix_h          Pixel graphic handle (x-y-marking)
% ---------------------------------------------------------------------------------------------

%% Draw pixel
col = [rand(1,1) rand(1,1) rand(1,1)];
oPix_h = scatter(x, y, 250, col, '.', 'Parent', oImageAxes_h);

%% Map distorted to undistorted pixel

% Get closest point within map to distorted, user defined pixel (x, y)
r1 = mfMap_x-x;
r2 = mfMap_y-y;
r  = r1.^2 + r2.^2;
[val,   idx_col] = min(r);
[~,     pix_x]   = min(val);

pix_y = idx_col(pix_x);

%% Compute ray
K = oCalibration.m_mfK;
Zc = 120;                           % arbitrary distance
pixh = [pix_x*Zc; pix_y*Zc; Zc]; 	% pixel (homogenous coordinates)
vfTarget = K\pixh;                  % target of ray (camera reference frame)

% Rotate into point cloud reference frame
R = oCalibration.m_mfR;
c = oCalibration.m_vfc;
vfTarget = R'*vfTarget + c;

% Set origin
vfOrigin = oCalibration.m_vfc;

% Draw ray
vfPoints = [vfTarget'; vfOrigin'];
oRay_h = plot3(vfPoints(:,1), vfPoints(:,2), vfPoints(:,3), 'LineWidth', 1, 'Parent', oPCAxes_h, 'Color', col); 

end

