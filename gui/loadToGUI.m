function [oGUIData, oGUIObjects, oGUIHistory, sStatus] = loadToGUI(oGUIData, oGUIObjects, oGUIHistory, bForward)
% ---------------------------------------------------------------------------------------------
% Function loadToGUI(...) loads metadata, images, the scan and labels to the GUI. It either initializes
% the GUI, moves within the editing history and controls object prediction.
%
% INPUT:
%   oGUIData:     Object of class cGUIData
%   oGUIObjects:  Object of class cGUIObjects
%   oGUIHistory:  Vector of class cHistory
%   bForward:     Boolean specifying the load operation's direction
%
% OUTPUT:
%   oGUIData:     Updated object of class cGUIData
%   oGUIObjects:  Updated object of class cGUIObjects
%   sStatus:      Status string (saved, unsaved, delta)
% ---------------------------------------------------------------------------------------------
oInfo_h       = oGUIData.m_oInfo_h;
sStatus       = 'unsaved';

% Save previous metadata
oPreviousMetadata = [];
if ~isempty(oGUIData.m_oPCMetadata)
    oPreviousMetadata = copy(oGUIData.m_oPCMetadata);
end

% Delete all PC graphics
for i = 1 : size(oGUIObjects.m_voPCMovableLabel, 1)
    delete(oGUIObjects.m_voPCMovableLabel(i,1).m_Box_h);
    oGUIObjects.m_voPCMovableLabel(i,1).m_Box_h = [];
end
cla(oGUIData.m_oPCAxes_h);

% Init: Delete all image graphics
if ~oGUIData.m_bInit
    for i = 1 : 4
        if isgraphics(oGUIData.m_voImageAxes_h(i,1))
            cla(oGUIData.m_voImageAxes_h(i,1));
        end
    end
end

%% Check history
bRestored = 0;      % indicates if object lists were restored from history
bObjectDelta = 0;   % indicates if delta between history object lists is found
if oGUIData.m_bInit
    % 1. Correction update
    if oGUIData.m_bEnableCorrections
        setInfoText(oInfo_h, '------------------Correction------------------', 0);
        posHeader           = setInfoText(oInfo_h, 'Correction measurement update: ...' , 1);
        voPCMovableLabel    = updateKalmanByCorrections(oGUIObjects.m_voPCMovableLabel, oGUIData);
        setInfoText(oInfo_h, 'Correction measurement update: ... done!' , 1, posHeader);
    end
    
    % Append to history
    nPos = oGUIData.m_nPosInHistory;
    nNum = oGUIData.m_nNumInHistory;
    oGUIHistory(nPos,1).m_oPCMetadata = oPreviousMetadata;
    oGUIHistory(nPos,1).m_oGUIObjects = copy(oGUIObjects);
    
    % Restore
    if bForward
        nPos = nPos + 1;
    else
        if nPos > 1
            nPos = nPos - 1;
        else
            oGUIData.m_nPCID = oGUIData.m_nPCID + 1;    % undo former decrease
            sStatus = 'end';
            setInfoText(oInfo_h, 'Begin of history reached.', 0);
            return;
        end
    end

    % Delete image label graphics
    for i = 1 : 4
        voLabels = oGUIObjects.m_voImageData(i,1).m_voImageLabels;
        for j = 1 : size(voLabels)
            if ~isempty(voLabels(j,1).m_oPoly_h)
                delete(voLabels(j,1).m_oPoly_h.Children);
            end
            voLabels(j,1).m_oPoly_h = [];
        end
        
        % Delete images
        delete(oGUIObjects.m_voImageData(i,1).m_oImage_h);
        oGUIObjects.m_voImageData(i,1).m_oImage_h = [];
        
        for j = 1 : size(oGUIObjects.m_voImageData(i,1).m_clRays,1)
            delete(oGUIObjects.m_voImageData(i,1).m_clRays{j}{1,1})
            delete(oGUIObjects.m_voImageData(i,1).m_clRays{j}{1,2})
        end
        oGUIObjects.m_voImageData(i,1).m_clRays = [];
        oGUIObjects.m_voImageData(i,1).m_nRaysCtr = 0;
        
    end
    
    % Restore from history
    if (nPos <= nNum) && (nPos > 0)
        setInfoText(oInfo_h, sprintf('Restoring from history. %d/%d', nPos, nNum), 0);
        bRestored = 1;
        oGUIData.m_nPosInHistory = nPos;
        
        oPCMetadata = oGUIHistory(nPos,1).m_oPCMetadata;
        oGUIObjects = oGUIHistory(nPos,1).m_oGUIObjects;
        
        oGUIData.m_oPCMetadata = oPCMetadata;
        voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
        oGUIData.m_nNumPCObjects = size(voPCMovableLabel,1);
        
        if oGUIHistory(nPos,1).m_bIsSaved
            sStatus = 'saved';
        end
    end
    
    % Check for deltas (objects that were defined in history missing in currently selected frame)
    if bRestored && bForward && (nPos >= 2)
        % Compare object lists in history
        nCtr = 0;
        voDeltaObjects(100,1) = cPCMovableLabel();
        voPCMovableLabel_prev = oGUIHistory(nPos-1,1).m_oGUIObjects.m_voPCMovableLabel;
        for i = 1 : size(voPCMovableLabel_prev,1)
            bMissingObject = 1;
            for j = 1 : size(voPCMovableLabel,1)
                if (voPCMovableLabel_prev(i,1).m_nTrackID == voPCMovableLabel(j,1).m_nTrackID)
                    bMissingObject = 0;
                    break;
                end
            end
            
            if bMissingObject
                nCtr = nCtr + 1;
                voDeltaObjects(nCtr,1) = copy(voPCMovableLabel_prev(i,1));
            end
        end
        
        if nCtr > 0
            voDeltaObjects = voDeltaObjects(1:nCtr,1);
            bObjectDelta = 1;
            sStatus = 'delta';
        end
    end
end

%% Load laser scan and labels
setInfoText(oInfo_h, '------------------Loading------------------', 1);
sBaseDir    = oGUIData.m_sDatasetDir;
nSequenceID = oGUIData.m_nSequenceID;
nPCID       = oGUIData.m_nPCID;

bLoadEdited = 0;
if bRestored && strcmp(sStatus, 'saved')
    bLoadEdited = 1;
end

posHeader = setInfoText(oInfo_h, 'Loading scan and PC labels:', 1);
oLaserScan = readPCDataMatrices(sBaseDir, nSequenceID, nPCID, bLoadEdited);
if ~bRestored
    bLoadEdited = 0;
    oPCMetadata             = readPCMetadata(sBaseDir, nSequenceID, nPCID);
    voPCMovableLabel_Loaded = readPCMovableLabels(sBaseDir, nSequenceID, nPCID, bLoadEdited);
end
setInfoText(oInfo_h, 'Loading scan and PC labels: ... done!', 1, posHeader);

% Consistency check when proceeding to consecutive sequence
if ~isempty(oPreviousMetadata) && bForward && oGUIData.m_bInit
    fDeltaT_ms = (oPCMetadata.m_nTimestamp_us - oPreviousMetadata.m_nTimestamp_us)/1000;
    if abs(fDeltaT_ms - 100) > 25
        setInfoText(oInfo_h, 'Timegap between sequences. Reinitializing.', 1);
        oGUIData.m_bInit = 0;
        pause(0.5);
    end
end

oGUIData.m_nNumPoints       = oPCMetadata.m_nNumberOfLayers * oPCMetadata.m_nNumberOfChannels;
oGUIData.m_oLaserScan       = oLaserScan;
oGUIData.m_nPCID            = nPCID;
oGUIData.m_oPCMetadata      = oPCMetadata;

%% Load images and labels
posHeader = setInfoText(oInfo_h, 'Loading images:', 1);

% Load images
if bRestored
    nReadMode = 3;  % image only
    if oPCMetadata.m_bImagesAvailable_Front
        oData = readImageData(sBaseDir, nSequenceID, nPCID, 'Front', nReadMode);
        oGUIObjects.m_voImageData(1,1).m_oImage_h = oData.m_oImage_h;
    end
    if oPCMetadata.m_bImagesAvailable_Right
        oData = readImageData(sBaseDir, nSequenceID, nPCID, 'Right', nReadMode);
        oGUIObjects.m_voImageData(2,1).m_oImage_h = oData.m_oImage_h;
    end
    if oPCMetadata.m_bImagesAvailable_Rear
        oData = readImageData(sBaseDir, nSequenceID, nPCID, 'Rear', nReadMode);
        oGUIObjects.m_voImageData(3,1).m_oImage_h = oData.m_oImage_h;
    end
    if oPCMetadata.m_bImagesAvailable_Left
        oData = readImageData(sBaseDir, nSequenceID, nPCID, 'Left', nReadMode);
        oGUIObjects.m_voImageData(4,1).m_oImage_h = oData.m_oImage_h;
    end
else
    % Load images and labels
    % Save previous labels
    clLabels = cell(4,1);
    for i = 1 : 4
        voLabels = oGUIObjects.m_voImageData(i,1).m_voImageLabels;
        nNumLabels = size(voLabels,1);
        vbKeep = true(nNumLabels,1);
        for j = 1 : nNumLabels
            if ~(voLabels(j,1).m_nIdxPCObject > 0)
                vbKeep(j,1) = false;
            end
        end
        clLabels{i,1} = voLabels(vbKeep);
    end
    
    % Load images and labels
    if oPCMetadata.m_bImagesAvailable_Front
        oGUIObjects.m_voImageData(1,1) = readImageData(sBaseDir, nSequenceID, nPCID, 'Front');
    end
    if oPCMetadata.m_bImagesAvailable_Right
        oGUIObjects.m_voImageData(2,1) = readImageData(sBaseDir, nSequenceID, nPCID, 'Right');
    end
    if oPCMetadata.m_bImagesAvailable_Rear
        oGUIObjects.m_voImageData(3,1) = readImageData(sBaseDir, nSequenceID, nPCID, 'Rear');
    end
    if oPCMetadata.m_bImagesAvailable_Left
        oGUIObjects.m_voImageData(4,1) = readImageData(sBaseDir, nSequenceID, nPCID, 'Left');
    end
    
    % Join or adopt image labels
    for i = 1 : 4
        % Init case: Load image labels
        if ~oGUIData.m_bInit
            if oGUIData.m_bLoadImageLabels && oGUIData.m_bInitFromEditedPC
                oGUIObjects.m_voImageData(i,1).m_voImageLabels = oGUIObjects.m_voImageData(i,1).m_voImageLabels;
            else
                oGUIObjects.m_voImageData(i,1).m_voImageLabels = [];
            end
            % Proceed / go back: Join or adopt
        else
            % Join labels
            if oGUIData.m_bLoadImageLabels
                oGUIObjects.m_voImageData(i,1).m_voImageLabels = [clLabels{i,1}; oGUIObjects.m_voImageData(i,1).m_voImageLabels];
                % Adopt previous labels
            else
                oGUIObjects.m_voImageData(i,1).m_voImageLabels = clLabels{i,1};
            end
        end
    end
end
setInfoText(oInfo_h, 'Loading images: ... done!', 1, posHeader);

%% Intitalize image axes according to settings
oImagePanel_h  = oGUIData.m_oImagePanel_h;

% Display images
for i = 1 : 4
    if ~oGUIObjects.m_voImageData(i,1).m_bImageAvailable
        continue
    end
    % Create axes if not initialized
    if ~oGUIData.m_bInit
        oAxes_h = axes('Visible', 'off', 'Parent', oImagePanel_h);
        oGUIData.m_voImageAxes_h(i,1) = oAxes_h;
    else
        oAxes_h = oGUIData.m_voImageAxes_h(i,1);
    end
    
    % Undistort images
    if oGUIObjects.m_voImageData(i,1).m_bImageAvailable
        oCalibration = oGUIData.m_voCalibration(i,1);
        [mfImg_undist, u, v] = undistortImage(oGUIObjects.m_voImageData(i,1).m_oImage, oCalibration);
        oGUIObjects.m_voImageData(i,1).m_mfMap_x = u;
        oGUIObjects.m_voImageData(i,1).m_mfMap_y = v;
    end
    
    if oGUIData.m_bUndistortImages
        oGUIObjects.m_voImageData(i,1).m_oImage = mfImg_undist;
    end
    
    nHeight  = oGUIObjects.m_voImageData(i,1).m_nHeight;
    nWidth   = oGUIObjects.m_voImageData(i,1).m_nWidth;
    
    fWidth = .4; fHeight = 0.3;
    switch oGUIObjects.m_voImageData(i,1).m_sImageType
        case 'Front'
            vfPosition = [(1-fWidth)/2   (1-fHeight)     fWidth fHeight];
        case 'Right'
            vfPosition = [(1-fWidth)     (1-fHeight)/2   fWidth fHeight];
        case 'Rear'
            vfPosition = [(1-fWidth)/2   0               fWidth fHeight];
        case 'Left'
            vfPosition = [0              (1-fHeight)/2   fWidth fHeight];
    end
    
    oImage_h = imshow(oGUIObjects.m_voImageData(i,1).m_oImage, 'Parent', oAxes_h);
    
    set(oAxes_h, 'Visible', 'on', 'XTick', [], 'YTick', [], 'units','normalized', 'Position', vfPosition, ...
        'XLim', [0 nWidth], 'YLim', [0 nHeight], 'NextPlot', 'add', 'XDir', 'normal', 'YDir', 'reverse',  ...
        'YAxisLocation', 'left', 'XAxisLocation', 'top', 'ButtonDownFcn', oGUIData.m_hImageCallback, ...
        'UserData', oGUIObjects.m_voImageData(i,1).m_sImageType);
    
    set(oImage_h, 'HitTest', 'off');
    oGUIObjects.m_voImageData(i,1).m_oImage_h = oImage_h;
end

drawnow;

%% Initialize PC and image objects
bInitPerformed = 0;
if ~oGUIData.m_bInit
    % Init history
    [oGUIData, oGUIHistory] = initHistory(oGUIData);
    
    if oGUIHistory(end,1).m_bIsSaved
        voPCMovableLabel  = oGUIHistory(end,1).m_oGUIObjects.m_voPCMovableLabel;
    else
        voPCMovableLabel  = voPCMovableLabel_Loaded;     % init with loaded objects
    end
    
    % Init point matrices by relabeling
    fMargin = 0; bUseHeight = 1; bSelective = 0;
    posHeader                       = setInfoText(oInfo_h, 'Relabeling:', 1);
    [oGUIData, voPCMovableLabel]    = relabelPC(voPCMovableLabel, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h);
    setInfoText(oInfo_h, 'Relabeling: ... done!', 1, posHeader);
    
    % Init movable label
    for i = 1 : size(voPCMovableLabel,1)
        voPCMovableLabel(i,1).init();
    end
    
    oGUIData.m_bInit = 1;
    oGUIData.m_nConsecutiveFrames = 0;
    oGUIData.m_nNumPCObjects        = size(voPCMovableLabel, 1);
    oGUIObjects.m_voPCMovableLabel  = voPCMovableLabel;
    
    % Add object definition context menu if ray projection is not active
    if strcmp(oGUIData.m_sImageContext, 'Object definition')
        oGUIData = addImagePanelContextMenu(oGUIData);
    end
    
    % Image labels
    if oGUIData.m_bEnableImageLabeling
        % Update projection flag according to image labels correspondences (suppress doubled projections)
        oGUIObjects.updateProjectionFlag();
        
        % Update image labels to PC correspondences (reference points etc.)
        oGUIObjects.updateImageLabelsCorrespondences(oGUIData);
        
        % Projection is done at the end of this function
    end
    bInitPerformed = 1;
end

%% Predict objects

if oGUIData.m_bPredict && ~bRestored && ~bInitPerformed
    setInfoText(oInfo_h, '------------------Prediction------------------', 1);
    
    % Create new history entry
    if nNum < oGUIData.m_nMaxNumInHistory
        oGUIData.m_nNumInHistory = nNum + 1;
    else
        oGUIHistory(1:oGUIData.m_nMaxNumInHistory-1,1) = oGUIHistory(2:oGUIData.m_nMaxNumInHistory,1);
        nPos = nNum;
    end
    oGUIData.m_nPosInHistory = nPos;
    oGUIHistory(nPos,1) = cHistory();
    
    % 1. Perform measurement update by corrections (done above)
    % 2. Predict
    % 3. Perform measurement update by prelabeling
    % 4. Perform measurement update by optimization
    % 5. Import new objects from prelabeling
    % 6. Predict image labels
    
    % Update image labels: Set PC correspondences and projection reference points (before prediction)
    if oGUIData.m_bEnableImageLabeling
        oGUIObjects.updateImageLabelsCorrespondences(oGUIData);
    end
    
    voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
    
    % Reset the prediction flag assuming a accurate prediction
    for i = 1 : size(voPCMovableLabel,1)
        voPCMovableLabel(i,1).m_bIsPredicted = 1;
    end
    
    % 2. Predict
    posHeader           = setInfoText(oInfo_h, 'Predicting: ...' , 1);
    voPCMovableLabel    = predictKalman(voPCMovableLabel, oGUIData, oGUIHistory);
    setInfoText(oInfo_h, 'Predicting: ... done!' , 1, posHeader);
    
    % 3. Prelabeling update
    if oGUIData.m_bEnablePrelabeling
        posHeader        = setInfoText(oInfo_h, 'Prelabeling update: ...' , 1);
        voPCMovableLabel = updateKalmanByPrelabeling(voPCMovableLabel, voPCMovableLabel_Loaded, oGUIData);
        setInfoText(oInfo_h, 'Prelabeling update: ... done!' , 1, posHeader);
    end
    
    % 4. Optimization update
    if oGUIData.m_bEnableOptimization
        setInfoText(oInfo_h, '------------------Optimization------------------', 1);
        % Relabel to get current points within box
        fMargin = 0.3; bUseHeight = 1; bSelective = 0;
        posHeader                       = setInfoText(oInfo_h, 'Relabeling:', 1);
        [oGUIData, voPCMovableLabel]    = relabelPC(voPCMovableLabel, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h);
        setInfoText(oInfo_h, 'Relabeling: ... done!', 1, posHeader);
        % Perform optimization
        posHeader           = setInfoText(oInfo_h, 'Optimizing:', 1);
        voPCMovableLabel    = updateKalmanByOptimization(voPCMovableLabel, oGUIData, oGUIHistory);
        setInfoText(oInfo_h, 'Optimizing: ... done!', 1, posHeader);
    end
    
    % Reset first frame status and corrected flag
    for i = 1 : size(voPCMovableLabel,1)
        voPCMovableLabel(i,1).m_bFirstFrame       = 0;
        voPCMovableLabel(i,1).m_bIsCorrected      = 0;
        voPCMovableLabel(i,1).m_bIsNew            = 0;
        voPCMovableLabel(i,1).m_bCounterIncreased = 0;
    end
    
    % 5. Import new objects from prelabeling
    bNewObjects = 0;
    if oGUIData.m_bAutoImport
        fAssociationThreshold = 3;    % m
        voNewObjects(100,1) = cPCMovableLabel;
        nCtr = 0;
        for i = 1 : size(voPCMovableLabel_Loaded,1)
            x_l = voPCMovableLabel_Loaded(i,1).m_fBBMiddle_x;
            y_l = voPCMovableLabel_Loaded(i,1).m_fBBMiddle_y;
            
            % Check close objects
            bAssociationFound = 0;
            for j = 1 : size(voPCMovableLabel,1)
                x = voPCMovableLabel(j,1).m_fBBMiddle_x;
                y = voPCMovableLabel(j,1).m_fBBMiddle_y;
                fDist = sqrt((x - x_l)^2 + (y - y_l)^2);
                
                % Check association list
                if fDist < fAssociationThreshold
                    bAssociationFound = 1;
                    break;
                end
            end
            
            % Check blacklist of deleted objects
            vnBlacklist = oGUIData.m_vnDeletedTrackIDs;
            for b = 1 : size(oGUIData.m_vnDeletedTrackIDs,1)
                if voPCMovableLabel_Loaded(i,1).m_nTrackID == vnBlacklist(b,1)
                    bAssociationFound = 1;
                    break;
                end
            end
            
            if ~bAssociationFound
                bNewObjects = 1;
                nCtr = nCtr + 1;
                voNewObjects(nCtr,1) = voPCMovableLabel_Loaded(i,1);
            end
        end
        
        if bNewObjects
            voNewObjects = voNewObjects(1:nCtr,1);
            setInfoText(oInfo_h, 'New objects imported.', 1);
            voPCMovableLabel = [voPCMovableLabel; voNewObjects];
            oGUIData.m_nNumPCObjects = size(voPCMovableLabel,1);
        end
    end
    
    % Final relabeling
    fMargin = 0; bUseHeight = 1; bSelective = 0;
    posHeader                       = setInfoText(oInfo_h, 'Relabeling:', 1);
    [oGUIData, voPCMovableLabel]    = relabelPC(voPCMovableLabel, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h);
    setInfoText(oInfo_h, 'Relabeling: ... done!', 1, posHeader);
    
    % Init new objects (after relabeling)
    if bNewObjects
        for i = 1 : size(voNewObjects,1)
            voNewObjects(i,1).init();
        end
    end
    
    % 6. Predict image labels
    if oGUIData.m_bEnableImageLabeling
        oGUIObjects.updateLabelVectorToPrediction(oGUIData);
    end
    
    % Increment the consecutive frames counter
    oGUIData.m_nConsecutiveFrames = oGUIData.m_nConsecutiveFrames + 1;
end

%% Predict object deltas
% Deltas between object lists occur when new objects in history frames were defined
if oGUIData.m_bPredict && bObjectDelta
    setInfoText(oInfo_h, '------------------Delta------------------', 1);
    setInfoText(oInfo_h, sprintf('Predicting %d missing objects into current frame.', size(voDeltaObjects,1)), 1);
    
    % Correction update
    if oGUIData.m_bEnableCorrections
        posHeader           = setInfoText(oInfo_h, 'Correction measurement update: ...' , 1);
        voDeltaObjects      = updateKalmanByCorrections(voDeltaObjects, oGUIData);
        setInfoText(oInfo_h, 'Correction measurement update: ... done!' , 1, posHeader);
    end
    
    % Prediction
    posHeader       = setInfoText(oInfo_h, 'Predicting: ...' , 1);
    voDeltaObjects  = predictKalman(voDeltaObjects, oGUIData, oGUIHistory);
    setInfoText(oInfo_h, 'Predicting: ... done!' , 1, posHeader);
    
    % Optimization update
    if oGUIData.m_bEnableOptimization
        % Relabel to get current points within box
        fMargin = 0.3; bUseHeight = 1; bSelective = 0;
        posHeader                       = setInfoText(oInfo_h, 'Relabeling:', 1);
        [oGUIData, voDeltaObjects]      = relabelPC(voDeltaObjects, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h);
        setInfoText(oInfo_h, 'Relabeling: ... done!', 1, posHeader);
        % Perform optimization
        posHeader      = setInfoText(oInfo_h, 'Optimizing:', 1);
        voDeltaObjects = updateKalmanByOptimization(voDeltaObjects, oGUIData, oGUIHistory);
        setInfoText(oInfo_h, 'Optimizing: ... done!', 1, posHeader);
    end
    
    % Reset first frame status and corrected flag
    for i = 1 : size(voDeltaObjects,1)
        voDeltaObjects(i,1).m_bFirstFrame     = 0;
        voDeltaObjects(i,1).m_bIsCorrected    = 0;
        voDeltaObjects(i,1).m_bIsNew          = 1;
        voDeltaObjects(i,1).m_bIsProjected    = 0;
    end
    
    % Append to object list
    voPCMovableLabel = [voPCMovableLabel; voDeltaObjects];
    oGUIData.m_nNumPCObjects = size(voPCMovableLabel,1);
    
    % Final relabeling
    fMargin = 0; bUseHeight = 1; bSelective = 0;
    posHeader                     = setInfoText(oInfo_h, 'Relabeling:', 1);
    [oGUIData, voPCMovableLabel]  = relabelPC(voPCMovableLabel, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h);
    setInfoText(oInfo_h, 'Relabeling: ... done!', 1, posHeader);
end

%% Prediction disabled
%  Set loaded objects if not predicted
if ~oGUIData.m_bPredict && ~bRestored && ~bInitPerformed
    voPCMovableLabel = voPCMovableLabel_Loaded;
end

%% Assign object lists
oGUIObjects.m_voPCMovableLabel = voPCMovableLabel;

%% Project objects
if oGUIData.m_bEnableImageLabeling && ~bRestored
    sShapeType  = oGUIData.m_sProjectionMode;
    oGUIObjects = projectPCObjects(oGUIData, oGUIObjects, sShapeType, oGUIData.m_bRandomColor);
end

%% Draw
[oGUIData, oGUIObjects] = redrawGUI(oGUIData, oGUIObjects);


end

