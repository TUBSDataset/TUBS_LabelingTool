function [oGUIData] = enteringDefinitionState(oDefinitionButtonGroup_h, oClassesButtonGroup_h, oViewButtonGroup_h, oGUIData, oGUIObjects)
% ---------------------------------------------------------------------------------------------
% Function enteringDefinitionState(...) sets GUI elements in object definition state.
%
% INPUT:
%   oInfo_h:                    Handle to the info box.
%   oDefinitionButtonGroup_h:   Handle to definition button group.
%   oClassesButtonGroup_h:      Handle to class definition button group.
%   oViewButtonGroup_h:         Handle to view button group.
%   oGUIData:                   Current GUI data.

% OUTPUT:
%   oGUIData:                   Changed GUI data object.                 
% ---------------------------------------------------------------------------------------------

oInfo_h = oGUIData.m_oInfo_h;

setInfoText(oInfo_h, 'Press [q] or [e] to change definition state!',                 0);
setInfoText(oInfo_h, 'Move box with [w] [a] [s] [d]!',                               1);
setInfoText(oInfo_h, 'Press [esc] to exit!',                                         1);
setInfoText(oInfo_h, 'Double press [entf/del] to delete box!',                       1);
setInfoText(oInfo_h, 'Press [f] to toggle "is active" flag!',                        1);
setInfoText(oInfo_h, 'Press [u] to reinit Kalman filter!',                           1);
setInfoText(oInfo_h, 'Press [h] to recalculate height!',                             1);
setInfoText(oInfo_h, 'Press [x] to mark box as corrected!',                          1);
setInfoText(oInfo_h, 'Press [space] to restore the view point!',                     1);

% Set definition state
oGUIData.m_sDefinitionState = 'yaw';
oGUIData.m_bInDefinitionState = 1;
% Unset view buttons
oGUIData.m_oShowFrontButton_h.Value = 0;
oGUIData.m_oShowBackButton_h.Value  = 0;
% Enable definition group
oDefsButtons_h = allchild(oDefinitionButtonGroup_h);
for i = 1 : length(oDefsButtons_h)
    if strcmp(oDefsButtons_h(i).String, 'edges')
        continue
    end
    oDefsButtons_h(i).Enable = 'on';
end
oYawButton_h = findobj(oDefsButtons_h, 'String', 'yaw');
oYawButton_h.Value = 1;

oClassesButtons_h = allchild(oClassesButtonGroup_h);
for i = 1 : length(oClassesButtons_h)
    oClassesButtons_h(i).Enable = 'on';
end

oViewButtons_h = allchild(oViewButtonGroup_h);
for i = 1 : length(oViewButtons_h)
    oViewButtons_h(i).Enable = 'off';
end

fillObjectListTable(oGUIData, oGUIObjects);
drawnow;
end



