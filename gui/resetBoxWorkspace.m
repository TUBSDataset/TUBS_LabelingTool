function [oGUIData, oGUIObjects] = resetBoxWorkspace(oGUIData, oGUIObjects)
% ---------------------------------------------------------------------------------------------
% Function resetBoxWorkspace(...) resets point cloud axes, state machines and display of objects.
%
% INPUT:
%   oGUIData:       Current GUI data.
%   oGUIObjects:    Current GUI objects. 

% OUTPUT:
%   oGUIData:       Changed GUIData object.
%   oGUIObjects:    Changed object of class cGUIObjects.         
% ---------------------------------------------------------------------------------------------

oGUIData.m_sDefinitionMode  = 'none';
oGUIData.m_sDefinitionState = 'none';

delete(oGUIData.m_voEdgePoints_h);
oGUIData.m_mfEdgePoints     = zeros(4,3);
oGUIData.m_nNumEdgePoints   = 0;

% change all text displays to standard
voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;

for i = 1 : oGUIData.m_nNumPCObjects;
    text_h = findobj(voPCMovableLabel(i,1).m_Box_h, 'Tag', 'text');
    if(~isempty(text_h))
        text_h.String = sprintf('%d %s', i, voPCMovableLabel(i,1).m_sClassification);
    end
    mid_h       = findobj(voPCMovableLabel(i,1).m_Box_h, 'Tag', 'mid');
    set(mid_h, 'Visible', 'off');
    marker_h    = findobj(voPCMovableLabel(i,1).m_Box_h, 'Tag', 'marker');
    set(marker_h, 'Visible', 'off');
    data_h    = findobj(voPCMovableLabel(i,1).m_Box_h, 'Tag', 'object_data');
    set(data_h, 'Visible', 'off');
end
drawnow;

% Unzoom
if oGUIData.m_bGUIZoomed
    oGUIData.m_bGUIZoomed = 0;
    camzoom(oGUIData.m_oPCAxes_h, 1/(oGUIData.m_fZoomFactor));
    setAxesSettings(oGUIData.m_oPCAxes_h, oGUIData.m_sView);
end

end

