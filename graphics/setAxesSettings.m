function [] = setAxesSettings(oPCAxes_h, sViewTag, vfCameraPosition, vfCameraTarget)
% ---------------------------------------------------------------------------------------------
% Function setAxesSettings sets the necessary settings for displaying a point cloud properly.
%
% Example: setAxesSettings(oPCAxes_h, sViewTag)
%
% INPUT:
%   sViewTag:           String that specifies the axes' limits.
%   vCameraPosition:    (Optional) position vector of MATLAB's camera to observe and display axes. 
%   vCameraTarget:      (Optional) target vector specifying the camera's target within the axes.
% --------------------------------------------------------------------------------------------- 
rangeX = 120; rangeY = 120; offset = 40;

zoom(oPCAxes_h, 'out');
if(nargin < 4)
    % eval front or back
    switch sViewTag
        case 'front'
            vfCameraPosition = [0 offset   1030];
            vfCameraTarget   = [0 offset      1];
        case 'back'
            vfCameraPosition = [0 -offset   1030];
            vfCameraTarget   = [0 -offset      1];
        case 'global'
            vfCameraPosition = [0 0   1030]; 
            vfCameraTarget   = [0 0      1];
        case 'center'
            vfCameraPosition = [0 0   1030]; 
            vfCameraTarget   = [0 0      1];
    end
end

set(oPCAxes_h, 'units', 'normalized', 'Position', [0 0 1 1], 'XLim', [-rangeX, rangeX], 'YLim', [-rangeY, rangeY], 'ZLim', [-3, 5],...
    'cameraPosition', vfCameraPosition, 'cameraTarget', vfCameraTarget, 'CameraViewAngleMode', 'manual',...
    'Projection', 'perspective', 'DataAspectRatio', [1 1 1.1], 'NextPlot', 'add'); % 1 1.15 1.1

if (strcmp(sViewTag, 'global'))
    set(oPCAxes_h, 'CameraViewAngle', 14.4678)
end

drawnow;

end

