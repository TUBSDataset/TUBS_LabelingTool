function [oGUIData, oGUIHistory] = initHistory(oGUIData)
% ---------------------------------------------------------------------------------------------
% Function initHistory(...) initializes an object of class cHistory for cGUIData.
% INPUT:
%   oGUIData:           Object of class cGUIData, contains the cHistory object
%   bSetSavedFlagInHis  True

% OUTPUT:
%   oGUIData:           Updated object of class cGUIData.
% ---------------------------------------------------------------------------------------------
%% Get number of edited point clouds

nMaxNum = 3;   % maximum number of previous object lists in history
sDir_PCMovableLabel = buildPath(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, oGUIData.m_nPCID, 15);
vsPCIDs = dir(sDir_PCMovableLabel);
i = 1;

while i < size(vsPCIDs,1)
    i = i + 1;
    if ~strcmp(vsPCIDs(i,1).name(1,1), '.')
        break;
    end
end

nFirstPC            = i;
nNumberEditedPCs    = 0;
if ~isempty(vsPCIDs)
    if strcmp(vsPCIDs(i,1).name(1,1), '.')
        nFirstPC            = 0;
    else
        nNumberEditedPCs = size(vsPCIDs,1) - nFirstPC + 1;
    end
end

%% No history to be initialized
if ~oGUIData.m_bInitFromEditedPC || nNumberEditedPCs == 0
    oEntry = cHistory();
    oEntry.m_bIsSaved = 0;
    
    oGUIData.m_nPosInHistory    = 1;
    oGUIData.m_nNumInHistory    = 1;
    oGUIHistory = oEntry;
    return;
end

%% Init history from prelabeled directory
setInfoText(oGUIData.m_oInfo_h, sprintf('Info: Initializing with max. %d edited PCs.', nMaxNum), 1);
setInfoText(oGUIData.m_oInfo_h, sprintf('Info: %d edited PCs found in sequence.', nNumberEditedPCs), 1);
%  Get PCIDs
vfPCIDs = zeros(nNumberEditedPCs,1);
for i = nFirstPC : size(vsPCIDs,1)
    vfPCIDs(i-nFirstPC+1,1) = str2double(vsPCIDs(i,1).name(1,1:10));
end

bHistoryMatch = 0;
nDesiredPC    = 0;
for i = 1 : size(vfPCIDs,1)
    if vfPCIDs(i,1) == oGUIData.m_nPCID
        bHistoryMatch   = 1;
        nDesiredPC      = i;
        break;
    end
end
 
if ~bHistoryMatch
    setInfoText(oGUIData.m_oInfo_h, 'Warning: Cannot find desired PCID in history.', 1);
    setInfoText(oGUIData.m_oInfo_h, 'Aborting to initialize editing history.', 1);
    
    oEntry = cHistory();
    oEntry.m_bIsSaved = 0;
    
    oGUIData.m_nPosInHistory    = 1;
    oGUIData.m_nNumInHistory    = 1;
    oGUIHistory = oEntry;
    
    return;
end

if nDesiredPC < size(vfPCIDs,1)
    setInfoText(oGUIData.m_oInfo_h, 'Warning: More PCs edited after desired PC.', 1);
    setInfoText(oGUIData.m_oInfo_h, 'Warning: Samples will be overwritten without further warning.', 1);
end

nReadEnd = nDesiredPC;

if nNumberEditedPCs > nMaxNum
    nNumberEditedPCs = nMaxNum;
end

nReadStart = nReadEnd - nNumberEditedPCs + 1;
if nReadStart < 1
    nReadStart = 1;
end

% Create history vector
nNumberEditedPCs = nReadEnd - nReadStart + 1;
voHistory(nNumberEditedPCs,1)   = cHistory();
oGUIData.m_nPosInHistory        = nNumberEditedPCs;
                 
% Read object lists into history
posHeader   = setInfoText(oGUIData.m_oInfo_h, 'Initializing history:', 1);
nCtr        = 0;
for i = nReadStart : nReadEnd
    setInfoText(oGUIData.m_oInfo_h, 'Initializing history:', 1, 'end', i);
    
    % Read label
    oGUIObjects = cGUIObjects();
    bLoadEdited = 1;
    
    [oPCMetadata,       nCode1] = readPCMetadata(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, vfPCIDs(i,1));
    [voPCMovableLabel,  nCode2] = readPCMovableLabels(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, vfPCIDs(i,1), bLoadEdited);
    
    % Init movable label
    for j = 1 : size(voPCMovableLabel,1)
        voPCMovableLabel(j,1).init();
    end

    oGUIObjects.m_voPCMovableLabel  = voPCMovableLabel;
    bReadImages = 0;
    oGUIObjects.m_voImageData(1,1)  = readImageData(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, vfPCIDs(i,1), 'Front', bReadImages);
    oGUIObjects.m_voImageData(2,1)  = readImageData(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, vfPCIDs(i,1), 'Right', bReadImages);
    oGUIObjects.m_voImageData(3,1)  = readImageData(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, vfPCIDs(i,1), 'Rear', bReadImages);
    oGUIObjects.m_voImageData(4,1)  = readImageData(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, vfPCIDs(i,1), 'Left', bReadImages);
    
    if ~oGUIData.m_bLoadImageLabels
        for j = 1 : 4
            oGUIObjects.m_voImageData(j,1).m_voImageLabels = [];
        end
    end
    
    % Update image labels to PC correspondences
    oGUIObjects.updateImageLabelsCorrespondences(oGUIData);

    nCtr = nCtr + 1;
    voHistory(nCtr,1).m_oPCMetadata = oPCMetadata;
    voHistory(nCtr,1).m_oGUIObjects = oGUIObjects;
    voHistory(nCtr,1).m_bIsSaved    = 1;
    
    if nCode1 || nCode2
        setInfoText(oGUIData.m_oInfo_h, 'Error occured while reading point cloud labels.', 1);
        break;
    end
end
oGUIData.m_nNumInHistory    = nNumberEditedPCs;
oGUIHistory = voHistory;

% Consistency check
if oGUIData.m_oEditorConfig.m_LastEditedPCID ~= vfPCIDs(nReadEnd,1)
    setInfoText(oGUIData.m_oInfo_h, 'Warning: Last PC in history differs from editor config.', 1);
end

setInfoText(oGUIData.m_oInfo_h, 'Initializing history: ... done!', 1, posHeader);
end

