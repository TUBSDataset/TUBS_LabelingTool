function [nCode, oGUIData, oGUIObjects, oGUIHistory] = saveObjectLists(oGUIData, oGUIObjects, oGUIHistory)
% ---------------------------------------------------------------------------------------------
% Function saveObjectLists saves edited point cloud and image lists to XML and binary files, respectively.
%
% INPUT:
%   oGUIData:           Object of class cGUIData containing metadata information
%   oGUIObjects:        Object of class cGUIObjects containing all object labels
%
% OUTPUT:
%   nCode:              Error code. 0 = success, 1 = error while writing files occured
%   oGUIData:           Updated cGUIData object
%   oGUIObjects:        Updated cGUIObjects object
% ---------------------------------------------------------------------------------------------
nCode               = 0;
oInfo_h             = oGUIData.m_oInfo_h;
voPCMovableLabel    = oGUIObjects.m_voPCMovableLabel;
%% Relabel
%  In order to enable height calculation for completeObjectData(...)
fMargin         = oGUIData.m_fRelabelMargin;
bUseHeight      = 0;    % get all points in newly defined rectangles
bSelective      = 0;

posHeader                     = setInfoText(oInfo_h, 'Relabeling:', 0);
[oGUIData, voPCMovableLabel]  = relabelPC(voPCMovableLabel, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h);
setInfoText(oInfo_h, 'Relabeling: ... done!', 1, posHeader);

% Auto delete
if oGUIData.m_bAutoDelete
    oGUIObjects.m_voPCMovableLabel = voPCMovableLabel;
    [oGUIData, oGUIObjects] = autoDelete(oGUIData, oGUIObjects);
    voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
end

%% Complete object data
posHeader           = setInfoText(oInfo_h, 'Completing object data: ... ', 1);
voPCMovableLabel    = completeObjectData(oGUIData, voPCMovableLabel);
setInfoText(oInfo_h, 'Completing box data: done!', 1, posHeader);

%% Project objects
if oGUIData.m_bEnableImageLabeling
    sShapeType  = oGUIData.m_sProjectionMode;
    oGUIObjects = projectPCObjects(oGUIData, oGUIObjects, sShapeType, oGUIData.m_bRandomColor);
end

%% Relabel
%  In order to apply new heights to PC labeling
bUseHeight = 1;    % get all points in newly defined rectangles

posHeader                      = setInfoText(oInfo_h, 'Relabeling:', 1);
[oGUIData, voPCMovableLabel]   = relabelPC(voPCMovableLabel, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h);
setInfoText(oInfo_h, 'Relabeling: ... done!', 1, posHeader);
oGUIObjects.m_voPCMovableLabel = voPCMovableLabel;

%% History
nPos = oGUIData.m_nPosInHistory;
if oGUIHistory(nPos,1).m_bIsSaved
    sName = buttondlg(sprintf('Object lists have been save previously.\nConfirm rewirte operation.'), ...
        'Confirm', 'Rewrite', 'Abort', struct('Default', 'Rewrite', 'IconString', 'warn'));
    if strcmp(sName, 'Abort')
        return
    end
end
oGUIHistory(nPos,1).m_bIsSaved = 1;

%% Correction measurement update
if oGUIData.m_bEnableCorrections
    posHeader           = setInfoText(oInfo_h, 'Correction measurement update: ...' , 1);
    voPCMovableLabel    = updateKalmanByCorrections(voPCMovableLabel, oGUIData);
    setInfoText(oInfo_h, 'Correction measurement update: ... done!' , 1, posHeader);
end

%% Save object lists

nCode1 = writePCMovableLabels(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, oGUIData.m_nPCID, voPCMovableLabel);
if nCode1 == 0
    setInfoText(oInfo_h, 'Saving point cloud labels to XML: ... done!', 1);
else
    setInfoText(oInfo_h, 'Saving point cloud labels to XML: ... error.', 1);
end

nCode2 = writePCMovableMatrices(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, oGUIData.m_nPCID, oGUIData.m_oLaserScan);
if nCode2 == 0
    setInfoText(oInfo_h, 'Saving object label matrices to binary: ... done!', 1);
else
    setInfoText(oInfo_h, 'Saving object label matrices to binary: ... error.', 1);
end

nCode3 = 0;
if oGUIData.m_bEnableImageLabeling
    for i = 1 : 4
        if oGUIObjects.m_voImageData(i,1).m_bImageAvailable
            nCode3 = nCode3 | writeImageData(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, oGUIData.m_nPCID, oGUIObjects.m_voImageData(i,1));
        end
    end
    
    if nCode3 == 0
        setInfoText(oInfo_h, 'Saving image labels to XML: ... done!', 1);
    else
        setInfoText(oInfo_h, 'Saving image labels to XML: ... error.', 1);
    end
end

%% Update last written sequence and point cloud

if ~(nCode1 || nCode2 || nCode3)
    oGUIData.m_oEditorConfig.m_LastEditedSequenceID = oGUIData.m_nSequenceID;
    oGUIData.m_oEditorConfig.m_LastEditedPCID       = oGUIData.m_nPCID;
    oGUIData.m_oEditorConfig.save(oGUIData.m_sRootDir, oInfo_h);
else
    setInfoText(oInfo_h, 'Error while saving occured.', 1);
    setInfoText(oInfo_h, 'Editor config file was not updated.', 1);
    nCode = 1;
end

end

