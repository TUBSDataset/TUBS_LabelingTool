function [mfImage, u, v] = undistortImage(mfImage, oCalibration)
% ---------------------------------------------------------------------------------------------
% Function undistortImage(...) undistort an image using given calibration data.
%
% INPUT:
%   mfImage:        Image to undistort
%   oCalibration:  	Object of class cCalibration containing intrinsics and distortion coefficients
%
% OUTPUT:
%   mfImage:        Undistorted image
%   u:              Mapping x -> x_distorted (used to compensate distorting for ray projection)
%   v:              Mapping y -> y_distorted 
% ---------------------------------------------------------------------------------------------

bPlot = 0;
if bPlot
    I_distorted = mfImage;
end

mfImage = double(mfImage);

K = oCalibration.m_mfK;
d = oCalibration.m_vfd;
k1 = d(1,1);
k2 = d(1,2);

bUseThreeCoeff = 0;
if size(d,2) > 2
   k3 = d(1,3); 
   bUseThreeCoeff = 1;
end

I = zeros(size(mfImage, 1), size(mfImage, 2));
[iy_u, ix_u] = find(~isnan(I));

% Get 3D points in camera coordinate system (z = 1) for given homogenous (undistorted) pixel coordinates
Xp = inv(K)*[ix_u iy_u ones(size(iy_u,1),1)]';

% Forward 3D points through the distortion
r   = Xp(1,:).^2 + Xp(2,:).^2;
x_c = Xp(1,:);
y_c = Xp(2,:);

if ~bUseThreeCoeff
    x_c = x_c.*(1 + k1.*r + k2.*r.^2);
    y_c = y_c.*(1 + k1.*r + k2.*r.^2);
else
    x_c = x_c.*(1 + k1*r.^2 + k2*r.^4 + k3*r.^6);
    y_c = y_c.*(1 + k1*r.^2 + k2*r.^4 + k3*r.^6);
end

% u and v are the distorted cooridnates
u = reshape(K(1,1)*x_c + K(1,3), size(I));
v = reshape(K(2,2)*y_c + K(2,3), size(I));

% Sample the distorted image at computed coordinates to undistort
sMethod = 'linear';
mfImage(:,:,1) = interp2(mfImage(:,:,1), u, v, sMethod);
mfImage(:,:,2) = interp2(mfImage(:,:,2), u, v, sMethod);
mfImage(:,:,3) = interp2(mfImage(:,:,3), u, v, sMethod);
mfImage = uint8(mfImage);

if bPlot
    figure; axes; imshow(I_distorted);
    figure; axes; imshow(mfImage);
end


end

