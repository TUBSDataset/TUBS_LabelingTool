function [oPC_h, oGround_h] = drawPC(oLaserscan, oEditorConfig, nNumPoints, oPCAxes_h, bShowGround)
% ---------------------------------------------------------------------------------------------
% Function drawPC draws a point cloud to the specified axis. Colors according to EditorConfig.xml
%
% INPUT:
%   oLaserScan:      Object of class cLaserScan containing the point data to be drawn
%   oEditorConfig:   Object of class cEditorConfig created on Startup according to EditorConfig.xml
%   nNumberOfPoints: Total number of points (64*2000)
%   oPCAxes_h:       Handle to PC axes
%   bShowGround:     Boolean whether or not to make ground points visible
%   
%
% OUTPUT:
%   oPC_h:           hg-Group of all created scatter objects
%   oGround_h:       Handle to ground points (scatter object)
% ---------------------------------------------------------------------------------------------

clClassPoints = extractLabels(oLaserscan, oEditorConfig, nNumPoints);

nSize_Background = 5;
nSize_Moving     = 20;

oPC_h = hggroup;
set(oPC_h, 'Parent', oPCAxes_h);

for i = 1 : size(clClassPoints,1)
    % Name | Points | Counter | ID | Color
    if clClassPoints{i,3} > 0
        vfPoints = clClassPoints{i,2};
        if isempty(vfPoints)
            continue
        end
        if strcmp(clClassPoints{i,1}, 'Ground')
            if ~bShowGround
                oGround_h = scatter3(vfPoints(:,1), vfPoints(:,2), vfPoints(:,3), nSize_Background, clClassPoints{i,5}, '.', 'Parent', oPC_h, 'Visible', 'off'); hold on;
            else
                oGround_h = scatter3(vfPoints(:,1), vfPoints(:,2), vfPoints(:,3), nSize_Background, clClassPoints{i,5}, '.', 'Parent', oPC_h, 'Visible', 'on'); hold on;
            end
        else
            scatter3(vfPoints(:,1), vfPoints(:,2), vfPoints(:,3), nSize_Moving, clClassPoints{i,5}, '.', 'Parent', oPC_h); hold on;
        end
    end
end

drawSRF(oPCAxes_h);
drawnow;
end

