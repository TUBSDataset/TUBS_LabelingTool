function [] = markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h, bRedraw_In)
% ---------------------------------------------------------------------------------------------
% Function markCurrentBoxAsCorrected(...) marks a point cloud object as corrected by changing its appearance.
%
% INPUT:
%   oGUIData:           Current GUI data.
%   oPCAxes_h:          Handle to PC axes object.
%   voPCMovableLabel:   Vector of objects of class cPCMovableLabel.
%   bRedraw_In:         (Optional) Boolean, true if to redraw graphic object.
% OUTPUT:
%   -
% EXAMPLES:
%    markCurrentBoxAsCorrected(oGUIData, voPCMovableLabel, oPCAxes_h)
%    markCurrentBoxAsCorrected(oGUIData, voPCMovableLabel, oPCAxes_h, bRedraw_In)
% ---------------------------------------------------------------------------------------------
bRedraw = 1;
if nargin > 3
    bRedraw = bRedraw_In;
end

voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
nObj = oGUIData.m_nIndexCurrentObject;

if (nObj > oGUIData.m_nNumPCObjects) || (nObj == 0)
    return;
end

% Mark object for measurement upate by correction
voPCMovableLabel(nObj).m_bNewMeasurement = 1;

% Update reference point
voPCMovableLabel(nObj).updateReferencePoint();

% Do not mark if already corrected
if voPCMovableLabel(nObj).m_bIsCorrected && ~bRedraw
    return;
end
delete(voPCMovableLabel(nObj).m_Box_h); drawnow nocallbacks;

% Redraw box
voPCMovableLabel(nObj).m_bIsCorrected = 1;
voPCMovableLabel(nObj) = drawPCObjects(voPCMovableLabel(nObj), oPCAxes_h, oGUIData.m_oEditorConfig, nObj);
oMid_h  = findobj(voPCMovableLabel(nObj).m_Box_h, 'Tag', 'mid');
set(oMid_h, 'Visible', 'on');
oRef_h  = findobj(voPCMovableLabel(nObj).m_Box_h, 'Tag', 'reference');
set(oRef_h, 'Visible', 'on');

if oGUIData.m_bDisplayObjectData
    obj_h  = findobj(voPCMovableLabel(nObj,1).m_Box_h, 'Tag', 'object_data');
    set(obj_h, 'Visible', 'on');
end

oAccu_h = findobj(voPCMovableLabel(nObj,1).m_Box_h, 'Tag', 'accu');
if ~isempty(oAccu_h)
    sState = 'on';
    if ~oGUIData.m_bShowAccu
        sState = 'off';
    end
    set(oAccu_h, 'Visible', sState);
end
drawnow nocallbacks;

% Update GUI elements
fillObjectListTable(oGUIData, oGUIObjects);
setSavedPanel(oGUIData.m_oSavedText_h, oGUIData.m_oSavedPanel_h, 'edited', 'y')

drawnow;
end

