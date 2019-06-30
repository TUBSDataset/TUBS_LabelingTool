function [oGUIData, oGUIObjects] = deleteBox(oGUIData, oGUIObjects, nIdxToDelete)
% ---------------------------------------------------------------------------------------------
% Function deleteBox(...) takes care of all necessary tasks if a box needs to be deleted from the GUI.
%
% INPUT:
%   oGUI_h:             Graphic handle to GUI frame
%   nIndexBoxToDelete:  Object index in cPCMovableLabel vector to delete
%
% OUTPUT:
%   voPCMovableLabel:   Updated cPCMovableLabel vector
% ---------------------------------------------------------------------------------------------

voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
nTrackID = voPCMovableLabel(nIdxToDelete,1).m_nTrackID;

% Delete graphic objects
delete(voPCMovableLabel(nIdxToDelete,1).m_Box_h);
delete(voPCMovableLabel(nIdxToDelete,1).m_Accu_h)

% Delte object from vector
index = true(1, size(voPCMovableLabel, 1));
index(nIdxToDelete)     = false; % mark index of box to delete
voPCMovableLabel        = voPCMovableLabel(index,:);

% Renumber box graphics
for i = nIdxToDelete : size(voPCMovableLabel, 1)
    text_h = findobj(voPCMovableLabel(i,1).m_Box_h, 'Tag', 'text');
    if (~isa(text_h,'matlab.graphics.GraphicsPlaceholder'))
        str = text_h.String;
        index = str2double(str(1,1:2)); % there is useless space at (1,1)...
        index = index - 1;
        if(index == 9)
            str(1,1:2) = sprintf('%d ', index);
        elseif(index < 10)
            str(1,1) = sprintf('%d', index);
        else
            str(1,1:2) = sprintf('%d', index);
        end      
        text_h.String = str;
    end
end

% Update label correspondences
for i = 1 : 4
   voImageLabels = oGUIObjects.m_voImageData(i,1).m_voImageLabels;
   index = true(size(voImageLabels,1), 1);
   for j = 1 : size(voImageLabels,1)
        if      (voImageLabels(j,1).m_nIdxPCObject == nIdxToDelete)
            delete(voImageLabels(j,1).m_oPoly_h);
            delete(voImageLabels(j,1));
            index(j,1) = false;
        elseif  (voImageLabels(j,1).m_nIdxPCObject > nIdxToDelete)
            voImageLabels(j,1).m_nIdxPCObject = voImageLabels(j,1).m_nIdxPCObject - 1;
            oText_h = findobj(voImageLabels(j,1).m_oPoly_h, 'tag', 'correspondence');
            if ~isempty(oText_h)
                oText_h.String = num2str(voImageLabels(j,1).m_nIdxPCObject);
            end
        end
   end
   voImageLabels = voImageLabels(index);
   oGUIObjects.m_voImageData(i,1).m_voImageLabels = voImageLabels;
end

% Update GUIData and GUIObjects
oGUIData.m_nNumPCObjects        = oGUIData.m_nNumPCObjects -1;
oGUIData.m_nIndexCurrentObject  = nIdxToDelete-1;
oGUIData.m_vnDeletedTrackIDs    = [oGUIData.m_vnDeletedTrackIDs; nTrackID];
oGUIObjects.m_voPCMovableLabel  = voPCMovableLabel;

drawnow;

end

