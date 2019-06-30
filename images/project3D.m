function [vfPoint_2D] = project3D(vfPoint_3D, oCalibration)
% ---------------------------------------------------------------------------------------------
% Function project3D(...) projects a 3D point into an 2D point according to the calibration data.
%
% INPUT:
%   vfPoint_3D:     (3x1) 3D point
%   oCalibration:   Object of class cCalibration containing intrinsics, distortion coefficients and extrinsics
%
% OUTPUT:
%   vfPoint_2D:     (2x1) 2D point
% ---------------------------------------------------------------------------------------------

K   = oCalibration.m_mfK;
R   = oCalibration.m_mfR;
cs  = oCalibration.m_vfc;
d   = oCalibration.m_vfd;

% Transform into camera reference frame
w = R*(vfPoint_3D - cs);

% Distort
w = w./w(3,1);  % x/z, y/z
x = w(1,1);
y = w(2,1);
k1 = d(1,1);
k2 = d(1,2);

bUseThreeCoeff = 0;
if size(d,2) > 2
   k3 = d(1,3); 
   bUseThreeCoeff = 1;
end

r  = x^2 + y^2;
if ~bUseThreeCoeff
%     x = x * (1 + k1*r^2 + k2*r^4);
%     y = y * (1 + k1*r^2 + k2*r^4);
    x = x * (1 + k1*r + k2*r^2);
    y = y * (1 + k1*r + k2*r^2);
else
    x = x * (1 + k1*r^2 + k2*r^4 + k3*r^6);
    y = y * (1 + k1*r^2 + k2*r^4 + k3*r^6);
end

% Project
vfPoint_2D = K * [x;y;1];
vfPoint_2D = vfPoint_2D(1:2,1);

end

