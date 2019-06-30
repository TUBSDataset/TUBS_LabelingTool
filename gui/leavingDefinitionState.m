function [oGUIData] = leavingDefinitionState(oDefinitionButtonGroup_h, oClassesButtonGroup_h, oViewButtonGroup_h, oGUIData, oGUIObjects)
% ---------------------------------------------------------------------------------------------
% Function leavingDefinitionState(...) resets according GUI elements when leaving object definition state. 
%
% INPUT:
%   oDefinitionButtonGroup_h:   Handle to definition button group.
%   oClassesButtonGroup_h:      Handle to class definition button group.
%   oViewButtonGroup_h:         Handle to view button group.
%   oGUIData:                   Current GUI data.
%
% OUTPUT:
%   oGUIData:                   Changed GUI data object.   
% ---------------------------------------------------------------------------------------------

defsButtons_h = allchild(oDefinitionButtonGroup_h);
for i = 1 : length(defsButtons_h)
    defsButtons_h(i).Enable = 'off';
end
oDefinitionButtonGroup_h.SelectedObject = [];
classesButtons_h = allchild(oClassesButtonGroup_h);
for i = 1 : length(classesButtons_h)
    classesButtons_h(i).Enable = 'off';
end
oClassesButtonGroup_h.SelectedObject = [];

viewButtons_h = allchild(oViewButtonGroup_h);
for i = 1 : length(viewButtons_h)
    viewButtons_h(i).Enable = 'on';
end

% Reset view and zoom
switch oGUIData.m_sView
    case 'front'
        viewButtons_h(4).Value = 1;
    case 'back'
        viewButtons_h(3).Value = 1;
    case 'global'
        viewButtons_h(2).Value = 1;
    case 'center'
        viewButtons_h(1).Value = 1;
end
oGUIData.m_oPCAxes_h.CameraViewAngle = oGUIData.m_fCameraViewAngleUnzoomed; % reset zoom
oGUIData.m_vfCameraTarget = [0 0 1];

oGUIData.m_sDefinitionState     = 'none';
oGUIData.m_bInDefinitionState   = 0; 
oGUIData.m_nIndexCurrentObject  = 0;
oGUIData.m_bGUIZoomed           = 0;
fillObjectListTable(oGUIData, oGUIObjects);

end

