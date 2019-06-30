function [fYawFitting, bYawFittingAvailable]  = ellipseYawConsistencyCheck(     oInfo_h, ...
                                                                                fYawFitting, ...
                                                                                fPreviousBBYaw_CSRF, ...
                                                                                fConsistencyYawThreshold, ...
                                                                                guiBoxes, ...
                                                                                i, ...
                                                                                enableDebugWarnings)
% ---------------------------------------------------------------------------------------------
% Function ellipseYawConsistencyCheck(...) checks the estimated yaw value by the ellipse fitting algorithm
% for consistency.
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
% ---------------------------------------------------------------------------------------------

bYawFittingAvailable = 1;

DYawEstimatedEllipse = fYawFitting - fPreviousBBYaw_CSRF;

% Before angluar consistency check: Try to establish a mapping by turining the ellipse by 180°. 
if (abs(DYawEstimatedEllipse) > 135)
    if fYawFitting > oPCMovableLabel.m_fBBYaw
        if(enableDebugWarnings)
            fprintf('Warning %s %d: Fitted ellipse measurement mapped by -180.\n', guiBoxes.m_MovableData(i).m_Classification, i);
        end
        fYawFitting = fYawFitting - 180;
    else
        if(enableDebugWarnings)
            fprintf('Warning %s %d: Fitted ellipse measurement mapped by +180.\n', guiBoxes.m_MovableData(i).m_Classification, i);
        end
        fYawFitting = fYawFitting + 180;
    end
end

DYawEstimatedEllipse = fYawFitting - fPreviousBBYaw_CSRF;
% ellipse might be roated by 90 deg (length of object point cloud is smaller than width and that case was not already handeled in fitEllipseToContour(...)
if((abs(DYawEstimatedEllipse) > 45))
    if(fYawFitting > oPCMovableLabel.m_fBBYaw)
        if(enableDebugWarnings)
            fprintf('Warning %s %d: Fitted ellipse turned -90 deg.\n', guiBoxes.m_MovableData(i).m_Classification, i);
        end
        fYawFitting = fYawFitting - 90; % yawFitting = 180, BBYaw = 90 || yawFitting = 270, BBYaw = 180 || yawFitting, = 359, BBYaw = 270
    else
        if(enableDebugWarnings)
            fprintf('Warning %s %d: Fitted ellipse turned +90 deg.\n', guiBoxes.m_MovableData(i).m_Classification, i);
        end
        fYawFitting = fYawFitting + 90; % yawFitting = 0,   BBYaw = 90 || yawFitting = 180, BBYaw = 270 || yawFitting, = 270, BBYaw = 359
    end
end

DYawEstimatedEllipse = fYawFitting - fPreviousBBYaw_CSRF;
% ellipse consistency check, treat middle and yaw separately
if((abs(DYawEstimatedEllipse) > fConsistencyYawThreshold))
    bYawFittingAvailable = 0;
    if(enableDebugWarnings)
        setInfoText(oInfo_h, sprintf('Warning %s %d: Ellipse yaw result inconsistent, ignoring.', guiBoxes.m_MovableData(i).m_Classification, i), 1);
    end
end

end

