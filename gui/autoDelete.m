function [oGUIData, oGUIObjects] = autoDelete(oGUIData, oGUIObjects)
% ---------------------------------------------------------------------------------------------
% Function autoDelete(...) checks point cloud objects for deletion conditions.

% INPUT:
%   oGUIObjects:        Object of class cGUIObjects containing point cloud objects to check
%   fRangeThreshold:    Maximum scanner range. Exceeding this range will erase the object
%   oGUIData:           Object of class cGUIData containing e.g. threshold information
%
% OUTPUT:
%   oGUIObjects:        Modified oGUIObjects object
%   oGUIData:           Modified oGUIData object
% ---------------------------------------------------------------------------------------------

voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
oInfo_h = oGUIData.m_oInfo_h;

nNumDelete = 0;
clDelete = cell(100,1);

for i = 1 : size(voPCMovableLabel, 1)
    bDelete = 0;
    if size(voPCMovableLabel(i,1).m_mfPointMatrix, 1) < oGUIData.m_nPointsDeleteThreshold
        if ~voPCMovableLabel(i,1).m_bCounterIncreased
            voPCMovableLabel(i,1).m_nEmptyCounter = voPCMovableLabel(i,1).m_nEmptyCounter + 1;
            voPCMovableLabel(i,1).m_bCounterIncreased = 1;
        end
        
        setInfoText(oInfo_h, sprintf('Warning %s %d: Sparsely populated box found for %d/%d scans.', ...
            voPCMovableLabel(i,1).m_sClassification, i, voPCMovableLabel(i,1).m_nEmptyCounter, oGUIData.m_nDeleteDelay), 1);
    else
        voPCMovableLabel(i,1).m_nEmptyCounter       = 0;
        voPCMovableLabel(i,1).m_bCounterIncreased   = 0;
    end
    
    if voPCMovableLabel(i).m_nEmptyCounter >= oGUIData.m_nDeleteDelay
        bDelete = 1;
        setInfoText(oInfo_h, sprintf('Warning %s %d: Empty box. Deleting.', voPCMovableLabel(i).m_sClassification, i), 1);
    end
    
    fRange = sqrt(voPCMovableLabel(i).m_fBBMiddle_x^2 + voPCMovableLabel(i).m_fBBMiddle_y^2);
    if fRange >= oGUIData.m_fMaxRange
        bDelete = 1;
        setInfoText(oInfo_h, sprintf('Warning %s %d: Object exceeds range. Deleting.', voPCMovableLabel(i).m_sClassification, i), 1);
    end
    
    if bDelete
        nNumDelete = nNumDelete + 1;
        clDelete{nNumDelete,1} = voPCMovableLabel(i);
    end
end

clDelete = clDelete(1:nNumDelete,:);
for i = 1 : nNumDelete
    for j = 1 : size(oGUIObjects.m_voPCMovableLabel, 1)
        if oGUIObjects.m_voPCMovableLabel(j,1) == clDelete{i,1}
            [oGUIData, oGUIObjects] = deleteBox(oGUIData, oGUIObjects, j);
            break;
        end
    end
end

end

