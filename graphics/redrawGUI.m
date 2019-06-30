function [oGUIData, oGUIObjects] = redrawGUI(oGUIData, oGUIObjects, bRedrawImageLabels_in)
% ---------------------------------------------------------------------------------------------
% Function redrawGUI(...) redraws the laserscan and updated objects within the GUI.
%
% INPUT:
%   oGUIData:               Object of class cGUIData containing metadata information
%   oGUIObjects:            Object of class cGUIObjects containing all object labels to be redrawn
%   bRedrawImageLabels_in: (Optional) False: image labels will not be redrawn
%
% OUTPUT:
%   oGUIData:               Updated oGUIData object
%   oGUIObjects:            Updated oGUIObjects object
% ---------------------------------------------------------------------------------------------

bRedrawImageLabels = 1;
if nargin > 2
    bRedrawImageLabels = bRedrawImageLabels_in;
end

voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;

% Redraw PC and point cloud objects
delete(oGUIData.m_oPC_h); drawnow;

[oGUIData.m_oPC_h, oGUIData.m_oGround_h] = drawPC(oGUIData.m_oLaserScan, oGUIData.m_oEditorConfig, ...
    oGUIData.m_nNumPoints, oGUIData.m_oPCAxes_h, oGUIData.m_bShowGround);

for i = 1 : size(voPCMovableLabel, 1)
    delete(voPCMovableLabel(i).m_Box_h)
end
[voPCMovableLabel, ~]  = drawPCObjects(voPCMovableLabel, oGUIData.m_oPCAxes_h, oGUIData.m_oEditorConfig);
oGUIObjects.m_voPCMovableLabel = voPCMovableLabel;

% Update tables (object might have been deleted during relabeling)
fillObjectListTable(oGUIData, oGUIObjects);
oGUIData.m_oMetadataTable_h.Data = fillMetadataTable(oGUIData.m_oPCMetadata); 
nObj = oGUIData.m_nIndexCurrentObject;
if (nObj > 0) && (nObj <= oGUIData.m_nNumPCObjects)
    oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable(oGUIObjects.m_voPCMovableLabel(nObj));
end

% Image labels
if oGUIData.m_bEnableImageLabeling && bRedrawImageLabels
    [oGUIObjects] = drawImageLabels(oGUIData, oGUIObjects);
end

% Set accu visible
if oGUIData.m_bShowAccu
    voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
    for i = 1 : size(voPCMovableLabel,1)
        oAccu_h = findobj(voPCMovableLabel(i,1).m_Box_h, 'Tag', 'accu');
        if ~isempty(oAccu_h)
            set(oAccu_h, 'Visible', 'on');
        end
    end
end

drawnow;

end

