function [oGUIData] = addImagePanelContextMenu(oGUIData)
% ---------------------------------------------------------------------------------------------
% Function addImagePanelContext(...) adds a label context menu to the image axes.
%
% INPUT:
%   oGUIData:   Object of class cGUIData containing axes and figure handles.
%
% OUTPUT:
%   oGUIData:   Updated object of class cGUIData.
% ---------------------------------------------------------------------------------------------

% Context menu for image labeling
oContextMenu_h = uicontextmenu(oGUIData.m_oImages_h);
uimenu(oContextMenu_h, 'Label', 'Add license plate',  'Callback', @oGUIData.m_hImageMenuCallback);
uimenu(oContextMenu_h, 'Label', 'Add face',           'Callback', @oGUIData.m_hImageMenuCallback);

% Add to menu to image axes
if oGUIData.m_bInit
    for i = 1 : 4
        if isgraphics(oGUIData.m_voImageAxes_h(i,1))
            oGUIData.m_voImageAxes_h(i,1).UIContextMenu = oContextMenu_h;
        end
    end
end

end

