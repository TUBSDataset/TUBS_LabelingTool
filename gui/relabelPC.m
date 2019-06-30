function [oGUIData, voPCMovableLabel] = relabelPC(voPCMovableLabel, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h)
% ---------------------------------------------------------------------------------------------
% Function relabelPC(...) relabels each point according its position within a bounding box. Additionally,
% this function initializes a oPCMovabelLabel's point matrix.
%
% Example: relabelPC(voPCMovableLabel, oGUIData, ...)
%
% INPUT:
%   voPCMovableLabel:   Object vector of class cPCMovableLabel 
%   oGUIData:           Object of class cGUIData storing GUI settings
%   bEnableDelete:      (Optional) Enable automatic deletion
%   bSelective:         (Optional) Suppresses relabeling to stationary, if only the initialization of a point matrix is desired
%   fMargin:            (Optional) Margin around bounding box
%   bUseHeight:         (Optional) Include z-coordinate in bounding box matching
%   oInfo_h:            (Optional) Handle to info field if progress monitor is desired
% OUTPUT:
%   voPCMovableLabel:   Modified object vector 
% ---------------------------------------------------------------------------------------------

if nargin < 3
    fMargin         = .5;
    bUseHeight      = 1;
    bSelective      = 0;
    bDoInfo         = 0;
end

if nargin > 5
    bDoInfo  = 1;
    nCtr     = 0;
end

oLaserScan       = oGUIData.m_oLaserScan;
mapClassToID     = oGUIData.m_MapClassToID;
nNumLayers       = size(oLaserScan.m_mfRange, 1);
nNumChannels     = size(oLaserScan.m_mfRange, 2);
nNumPoints       = nNumLayers * nNumChannels;
nNumObjects      = size(voPCMovableLabel, 1);
mfBBTestMatrix   = determineBBTestMatrix(voPCMovableLabel, fMargin, fMargin, fMargin, fMargin);
fZMax            = zeros(nNumObjects, 1);
nLabelGround     = mapClassToID('Ground');
nLabelStationary = mapClassToID('Stationary');

% Preallocate point matrices
pointData  = zeros(nNumObjects, nNumPoints);
indexVecX  = ones(nNumObjects, 1);
indexVecZ  = ones(nNumObjects, 1)*3;
groundData = zeros(nNumObjects, nNumPoints);
indexVecG  = ones(nNumObjects, 1);

for i = 1 : size(fZMax,1)
    if(bUseHeight)
        fZMax(i,1) = voPCMovableLabel(i,1).m_fBBMiddle_z + voPCMovableLabel(i,1).m_fBBHeight/2 + fMargin;
        % New box: Take all points. Height will be recalculated during completeBoxData(...).
        if(voPCMovableLabel(i,1).m_fBBHeight == 0)
            fZMax(i,1) = 1000;
        end
    else
        fZMax(i,1) = 1000;
    end
end

%% Relabeling logic

oLaserScan.m_mnListIndex = zeros(nNumLayers, nNumChannels);
for nRow = 1 : nNumLayers
    if bDoInfo
        setInfoText(oInfo_h, 'Relabeling:', 1, 'end', nCtr);
        nCtr = nCtr + 1;
    end
    
    for nColumn = 1 : nNumChannels
        if ~oLaserScan.m_mbIsValid(nRow, nColumn)
            continue
        end
        if oLaserScan.m_mnLabelID(nRow, nColumn) == nLabelGround;
            continue
        end
        % Test point
        bMatch = 0;
        for i = 1 : size(mfBBTestMatrix,1)
            D12 = mfBBTestMatrix(i,1) * oLaserScan.m_mfX(nRow,nColumn)+mfBBTestMatrix(i,2) * oLaserScan.m_mfY(nRow,nColumn)+mfBBTestMatrix(i,3);
            D23 = mfBBTestMatrix(i,4) * oLaserScan.m_mfX(nRow,nColumn)+mfBBTestMatrix(i,5) * oLaserScan.m_mfY(nRow,nColumn)+mfBBTestMatrix(i,6);
            D34 = mfBBTestMatrix(i,7) * oLaserScan.m_mfX(nRow,nColumn)+mfBBTestMatrix(i,8) * oLaserScan.m_mfY(nRow,nColumn)+mfBBTestMatrix(i,9);
            D41 = mfBBTestMatrix(i,10)* oLaserScan.m_mfX(nRow,nColumn)+mfBBTestMatrix(i,11)* oLaserScan.m_mfY(nRow,nColumn)+mfBBTestMatrix(i,12);
            if (D12 > 0) && (D23 > 0) && (D34 > 0) && (D41 > 0) && (oLaserScan.m_mfZ(nRow,nColumn) <= fZMax(i,1))
                bMatch = 1;
                oLaserScan.m_mnListIndex(nRow,nColumn)  = i;
                oLaserScan.m_mnLabelID(nRow,nColumn)    = mapClassToID(voPCMovableLabel(i,1).m_sClassification);
                
                pointData(i,indexVecX(i):indexVecZ(i)) = [   oLaserScan.m_mfX(nRow,nColumn), ...
                    oLaserScan.m_mfY(nRow,nColumn), ...
                    oLaserScan.m_mfZ(nRow,nColumn)];
                groundData(i, indexVecG(i)) = oLaserScan.m_mfGroundLevelZ(nRow,nColumn);
                
                indexVecX(i) = indexVecX(i) + 3;
                indexVecZ(i) = indexVecZ(i) + 3;
                indexVecG(i) = indexVecG(i) + 1;
                break;
            end
        end
        if(~bMatch && ~bSelective)
            oLaserScan.m_mnLabelID(nRow,nColumn) = nLabelStationary;
        end
    end
end

% Fill point matrices 
for i = 1 : size(voPCMovableLabel, 1)
    voPCMovableLabel(i,1).m_mfPointMatrix   = reshape(pointData( i, 1:indexVecZ(i)-3), 3, [])';
    voPCMovableLabel(i,1).m_vfGroundLevels  = reshape(groundData(i, 1:indexVecG(i)-1), 1, [])';
    % Rough downsampling
    mfPoints     = voPCMovableLabel(i,1).m_mfPointMatrix;
    maxPoints    = oGUIData.m_nSizeICPFifo;
    numPoints    = size(voPCMovableLabel(i,1).m_mfPointMatrix, 1);
    overhead     = numPoints - maxPoints;
    deletionList = true(1, numPoints);
    ctr = 0;
    while (ctr < overhead)
        idxToDelete = round((numPoints-1) * rand(1,1) + 1);
        if (deletionList(1, idxToDelete) == true)
            deletionList(1, idxToDelete) = false;
            ctr = ctr + 1;
        end
    end
    mfPoints = mfPoints(deletionList,:);
    voPCMovableLabel(i,1).m_mfPointMatrix = mfPoints;
end
end

