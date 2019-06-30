function [oGUIData, oGUIObjects] = zoomToObject(oGUIData, oGUIObjects, oPCAxes_h, x, y)
% ---------------------------------------------------------------------------------------------
% Function zoomToPoint(...) zooms the point cloud axes to a desired points and changes the
% displayed text accoringly.
%
% INPUT:
%   oGUIData:       GUIData object.
%   oGUIObjects:    Object of class cGUIObjects.
%   oPCAxes_h:      PC axes object.
%   x,y:            Current point as defined by the user.
%   
%
% OUTPUT:
%   oGUIData:       Changed GUIData object.
%   oGUIObjects:    Changed object of class cGUIObjects.
% ---------------------------------------------------------------------------------------------

% Zoom only once
if ~oGUIData.m_bGUIZoomed
    camzoom(oPCAxes_h, oGUIData.m_fZoomFactor);
    oGUIData.m_bGUIZoomed = 1;
end

target2Point = [x y 0] - oPCAxes_h.CameraTarget;
camdolly(oPCAxes_h, target2Point(1,1), target2Point(1,2), 0, 'movetarget', 'data');
set(oPCAxes_h, 'Position', [0 0.06 1 1]);

oGUIData.m_vfCameraTarget(1,1) = x;
oGUIData.m_vfCameraTarget(1,2) = y;

% Change text display of current object
nIndexObj   = oGUIData.m_nIndexCurrentObject;
voPCObjects = oGUIObjects.m_voPCMovableLabel;
try
    if(nIndexObj > 0)
        text_h = findobj(voPCObjects(nIndexObj,1).m_Box_h, 'Tag', 'text');
        mid_h  = findobj(voPCObjects(nIndexObj,1).m_Box_h, 'Tag', 'mid');
        set(mid_h, 'Visible', 'on');
        
        marker_h  = findobj(voPCObjects(nIndexObj,1).m_Box_h, 'Tag', 'marker');
        set(marker_h, 'Visible', 'on');
        
        ref_h  = findobj(voPCObjects(nIndexObj,1).m_Box_h, 'Tag', 'reference');
        set(ref_h, 'Visible', 'on');
        
        if oGUIData.m_bDisplayObjectData
            obj_h  = findobj(voPCObjects(nIndexObj,1).m_Box_h, 'Tag', 'object_data');
            set(obj_h, 'Visible', 'on');
        end
        
        oAccu_h = findobj(voPCObjects(nIndexObj,1).m_Box_h, 'Tag', 'accu');
        if ~isempty(oAccu_h)
            sState = 'on';
            if ~oGUIData.m_bShowAccu
               sState = 'off'; 
            end
            set(oAccu_h, 'Visible', sState);
        end
        
        sClass = sprintf('%d %s', nIndexObj, voPCObjects(nIndexObj,1).m_sClassification);
        if ~isa(text_h, 'matlab.graphics.GraphicsPlaceholder')
            str = text_h.String;
            if(strcmp(sClass, str))
                if(voPCObjects(nIndexObj,1).m_bIsNew)
                    str = sprintf('%s  | new!', str);
                end
                if(~voPCObjects(nIndexObj,1).m_bIsPredicted)
                    str = sprintf('%s  | uncertain!', str);
                end
                if(~voPCObjects(nIndexObj,1).m_bIsCorrected)
                    str = sprintf('%s  | uncorrected!', str);
                end
                text_h.String = str;
                drawnow;
            end
        end
        
        oClass_h = findobj(oGUIData.m_oClassesButtonGroup_h, 'String', voPCObjects(nIndexObj,1).m_sClassification); 
        oClass_h.Value = 1;
    end
catch
    oGUIData.m_nIndexCurrentObject = 1;
end

end

