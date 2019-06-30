function [voPCMovableLabel] = completeObjectData(oGUIData, voPCMovableLabel)
% ---------------------------------------------------------------------------------------------
% Function completeObjectData(...) completes object data for newly defined objects.
% bCalcHeightByGrowing activates a growing algorithm for height calcuation that avoids mistakes
% due to overhanging trees etc.
%
% INPUT:
%   oGUIData:               Object of class cGUIData containing additional information such as oPCMetadata.
%   voPCMovableLabel(i):    Object of class cPCMovableLabel to be completed.
%
% OUTPUT:
%   voPCMovableLabel(i):    Updated cPCMovableLabel object.
% ---------------------------------------------------------------------------------------------
bCalcHeightByGrowing = 1;

fSigXY      = oGUIData.m_fDeltaXY;
fSigYaw     = oGUIData.m_fDeltaYaw;
oEditorConf = oGUIData.m_oEditorConfig;

for i = 1 : size(voPCMovableLabel, 1)
    % Skip already complete objects
    if voPCMovableLabel(i).m_bCompleteObjectData
        voPCMovableLabel(i).m_bCompleteObjectData = 0;  % Object considered complete for repeated calls of completeObjectData(...)
    
        %% State related entries
        % Note that Kalman filters are initialized indepedently. Filter values are applied after first prediction step.
        
        % Inactive object (e.g. parking cars or stationary objects): Assign apriori knowledge to dynamics
        if ~voPCMovableLabel(i).m_bIsActive
            
            voPCMovableLabel(i).m_fVarBBMiddle_x       = fSigXY^2;
            voPCMovableLabel(i).m_fVarBBMiddle_y       = fSigXY^2;
            voPCMovableLabel(i).m_fVarBBYaw            = fSigYaw^2;
            
            voPCMovableLabel(i).m_fVxAbs               = 0;
            voPCMovableLabel(i).m_fVyAbs               = 0;
            voPCMovableLabel(i).m_fAxAbs               = 0;
            voPCMovableLabel(i).m_fAyAbs               = 0;
            voPCMovableLabel(i).m_fBBYawRatePerDist    = 0;
            
            voPCMovableLabel(i).m_fVarVxAbs            = 0;
            voPCMovableLabel(i).m_fVarVyAbs            = 0;
            voPCMovableLabel(i).m_fVarAxAbs            = 0;
            voPCMovableLabel(i).m_fVarAyAbs            = 0;
            voPCMovableLabel(i).m_fVarBBYawRatePerDist = 0;
            
            % Active new track that is defined by the user (not a newly imported track from prelabeled object list)
        elseif voPCMovableLabel(i).m_bIsNew && voPCMovableLabel(i).m_bIsUserDefined
            
            yaw = voPCMovableLabel(i).m_fBBYaw * pi()/180;
            
            voPCMovableLabel(i).m_fVarBBMiddle_x       = voPCMovableLabel(i).m_fVarPos_init;
            voPCMovableLabel(i).m_fVarBBMiddle_y       = voPCMovableLabel(i).m_fVarPos_init;
            voPCMovableLabel(i).m_fVarBBYaw            = voPCMovableLabel(i).m_fVarPos_init;
            
            voPCMovableLabel(i).m_fVxAbs               = voPCMovableLabel(i).m_fV_init * cos(yaw);
            voPCMovableLabel(i).m_fVyAbs               = voPCMovableLabel(i).m_fV_init * sin(yaw);
            voPCMovableLabel(i).m_fAxAbs               = voPCMovableLabel(i).m_fA_init * cos(yaw);
            voPCMovableLabel(i).m_fAyAbs               = voPCMovableLabel(i).m_fA_init * sin(yaw);
            voPCMovableLabel(i).m_fBBYawRatePerDist    = 0;
            
            voPCMovableLabel(i).m_fVarVxAbs            = voPCMovableLabel(i).m_fVarV_init;
            voPCMovableLabel(i).m_fVarVyAbs            = voPCMovableLabel(i).m_fVarV_init;
            voPCMovableLabel(i).m_fVarAxAbs            = voPCMovableLabel(i).m_fVarA_init;
            voPCMovableLabel(i).m_fVarAyAbs            = voPCMovableLabel(i).m_fVarA_init;
            voPCMovableLabel(i).m_fVarBBYawRatePerDist = voPCMovableLabel(i).m_fVarK_init;
        end
        
        % (Re)Init Kalman Filter from data in PCMovableLabel
        bDefault = 0;
        setInfoText(oGUIData.m_oInfo_h, sprintf('Info %s %d: (Re)initializing Kalman Filter.', voPCMovableLabel(i).m_sClassification, i), 1);
        voPCMovableLabel(i).m_oKalman.initModels(voPCMovableLabel(i), bDefault);
    end
    
    %% Height calculation
    %  If bCalcHeightByGrowing = 1: The object's bounding box grows in steps to tackle overhanging issues
    mfPoints        = voPCMovableLabel(i).m_mfPointMatrix;
    fGroundLevelZ   = voPCMovableLabel(i).getGroundLevel();
    
    % Height calculation
    bIsDefault = 0;
    if ~isempty(mfPoints)
        if bCalcHeightByGrowing
            % Calculate height by discretization, grow until empty subcontainer was found
            % Settings
            step            = 0.3; 
            numLookahead    = 1;
            threshold       = 0.1;
            
            % Inits
            stop            = 0;
            minZ            = fGroundLevelZ;
            maxZ            = minZ + step;
            ctrInLast       = 0;
            ctrLookedAhead  = 0;
            maxSubZ         = max(mfPoints(:,3));   % as fallback
            
            % Find first filled subcontainer
            ctr = 0;
            for p = 1 : 5  % max 5 attempts to find filled subcontainer
                for k = 1 : size(mfPoints,1)
                    if((mfPoints(k,3) >= minZ) && (mfPoints(k,3) <= maxZ))
                        ctr = ctr + 1;
                    end
                end
                if ctr == 0
                    minZ  = maxZ;
                    maxZ = maxZ + step;
                else
                    % Start growing
                    break;
                end
            end
            
            % Growing
            while (~stop)
                ctr = 0;
                subcontainer = zeros(size(mfPoints,1),3);
                for k = 1 : size(mfPoints,1)
                    if((mfPoints(k,3) >= minZ) && (mfPoints(k,3) <= maxZ))
                        ctr = ctr + 1;
                        subcontainer(ctr,:) = mfPoints(k,:);
                    end
                end
                if ctr > (threshold*ctrInLast)
                    subcontainer = subcontainer(1:ctr,:);
                    maxSubZ      = max(subcontainer(:,3));
                    minZ  = maxZ;
                    maxZ = maxZ + step;
                    ctrInLast       = ctr;
                    ctrLookedAhead  = 0;
                else
                    ctrLookedAhead = ctrLookedAhead + 1;
                    minZ  = maxZ;
                    maxZ = maxZ + step;
                end
                if (ctrLookedAhead > numLookahead)
                    stop = 1;
                end
            end
            fHeight = abs(maxSubZ - fGroundLevelZ);
        else
            maxZ = max(mfPoints(:,3));
            fHeight = abs(maxSubZ - fGroundLevelZ);
        end
    else
        fHeight = 2; bIsDefault = 1;
    end
    
    % Assign computed height
    if bIsDefault && voPCMovableLabel(i).m_fBBHeight == 0   % use default height only for new objects (fBBHeight = 0)
        setInfoText(oGUIData.m_oInfo_h, sprintf('Warning %s %d: Empty box found! Using default height for new box!', ...
            voPCMovableLabel(i).m_sClassification, i), 1);
        voPCMovableLabel(i).m_fBBHeight = fHeight;
    else
        % Size can only grow. Allow reinit of height if it's a new box or [h] pressed to enforce
        if (fHeight > voPCMovableLabel(i).m_fBBHeight) || voPCMovableLabel(i).m_bIsNew || voPCMovableLabel(i).m_bRecalcHeight
            voPCMovableLabel(i).m_fBBHeight = fHeight;
            voPCMovableLabel(i).m_bRecalcHeight = 0;
        end
    end
    
    % Bounding box middle
    voPCMovableLabel(i).m_fBBMiddle_z = fGroundLevelZ + voPCMovableLabel(i).m_fBBHeight/2;
    
    % Init registered point matrices by current points
    if isempty(voPCMovableLabel(i).m_mfRegisteredPointMatrix)
        voPCMovableLabel(i).m_mfRegisteredPointMatrix = voPCMovableLabel(i).m_mfPointMatrix;
    end
    
    %% Height pausibility check
    
    fMaxCarHeight           = 2.5;
    fMaxVanHeight           = 3;
    maxTruckHeight          = 5;
    fMaxPedestrianHeight    = 2.2;
    fMaxBicycleHeight       = 2.2;
    switch voPCMovableLabel(i).m_sClassification
        case 'Car'
            if  voPCMovableLabel(i).m_fBBHeight > fMaxCarHeight
                voPCMovableLabel(i).m_fBBHeight = fMaxCarHeight;
            end
        case 'Truck'
            if  voPCMovableLabel(i).m_fBBHeight > maxTruckHeight
                voPCMovableLabel(i).m_fBBHeight = maxTruckHeight;
            end
        case 'Van'
            if  voPCMovableLabel(i).m_fBBHeight > fMaxVanHeight
                voPCMovableLabel(i).m_fBBHeight = fMaxVanHeight;
            end
        case 'Pedestrian'
            if  voPCMovableLabel(i).m_fBBHeight > fMaxPedestrianHeight
                voPCMovableLabel(i).m_fBBHeight = fMaxPedestrianHeight;
            end
        case 'Bicycle'
            if  voPCMovableLabel(i).m_fBBHeight > fMaxBicycleHeight
                voPCMovableLabel(i).m_fBBHeight = fMaxBicycleHeight;
            end
        case 'Motorbike'
            if  voPCMovableLabel(i).m_fBBHeight > fMaxBicycleHeight
                voPCMovableLabel(i).m_fBBHeight = fMaxBicycleHeight;
            end
        case 'Movable'
            if  voPCMovableLabel(i).m_fBBHeight > maxTruckHeight
                voPCMovableLabel(i).m_fBBHeight = maxTruckHeight;
            end
    end
    
    %% Class probabilities
    voPCMovableLabel(i).m_fExistenceLikelihood = 1;
    
    % Assign class probabiliy vector using editor config file
    voAssingableClasses     = oEditorConf.getAssignableClasses();
    nNumClasses             = size(voAssingableClasses, 1);
    
    % Create class vector
    voClassesVector(nNumClasses, 1) = struct('Name', '', 'Probability', 0);
    for j = 1 : nNumClasses
        voClassesVector(j,1).Name = voAssingableClasses(j,1).Name;
        voClassesVector(j,1).Probability = 0;
        if strcmp(voPCMovableLabel(i).m_sClassification, voClassesVector(j,1).Name)
            voClassesVector(j,1).Probability = 1;
        end
    end
    
    if strcmp(voPCMovableLabel(i).m_sClassification,'Undefined')
        setInfoText(oGUIData.m_oInfo_h, sprintf('Warning %s %d: Undefined classification found. Correcting to movable!', ...
            voPCMovableLabel(i).m_sClassification, i), 1);
        voPCMovableLabel(i).m_sClassification = 'Movable';
    end
    
    voPCMovableLabel(i,1).m_voProbabilityVector.Class = voClassesVector;
end
end
