function [oGUIObjects] = projectPCObjects(oGUIData, oGUIObjects, sShapeType, bRandomColor)
% ---------------------------------------------------------------------------------------------
% Function projectPCObjects(...) projects point cloud objects into the image planes.

% INPUT:
%   oGUIData        Object of class cGUIData containing axes and calibration information
%   oGUIObjects     Object of class cGUIData containing objects to be projected
%   sShapeType      Currently selected shape type / projection mode
%   bRandomColor    (Optional) True if random color is desired. Elsewise the configured object color is taken.
%
% OUTPUT:
%   oGUIObjects     Modified oGUIObjects object
% ---------------------------------------------------------------------------------------------

voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
nNumObj = size(voPCMovableLabel,1);
if nNumObj == 0
    return
end
moImageLabels(nNumObj, 4) = cImageLabel();     % row: label, column: image index (front, right, rear, left)

vnCtr = zeros(1,4);     % four image specifiy object counters
nNumPCObjects = size(voPCMovableLabel,1);

oConfig = oGUIData.m_oEditorConfig;

%% Clear existing objects (if not edited). Edited objects need to be deleted for a new projection
nCtr_candidates = 0;
nCtr_edited = 0;
vnUnsetProjectionFlagIndices = zeros(100,1);
vnEditedIndices = zeros(100,1);

for i = 1 : 4
    voImageLabels = oGUIObjects.m_voImageData(i,1).m_voImageLabels;
    vbKeep = true(size(voImageLabels,1) ,1);
    for j = 1 : size(voImageLabels, 1)
        % Keep user defined, predicted and objects without correspondences
        if (voImageLabels(j,1).m_bIsUserDefined || voImageLabels(j,1).m_bIsPredicted ...
                || (voImageLabels(j,1).m_bIsLoaded && ~voImageLabels(j,1).m_nIdxPCObject > 0))  % faces and LPs
            continue
        end
        
        nObj = voImageLabels(j,1).m_nIdxPCObject;
        
        if voImageLabels(j,1).m_bIsEdited && nObj > 0
            nCtr_edited = nCtr_edited + 1;
            vnEditedIndices(nCtr_edited,1) = nObj;
            continue
        end
        
        % Unset projection flag: candidates. One PC label can occur as two image labels. If one is edited, the
        % projection flag is not to be unset
        if nObj > 0
            nCtr_candidates = nCtr_candidates + 1;
            vnUnsetProjectionFlagIndices(nCtr_candidates,1) = nObj;
        end
        
        oPoly_h = voImageLabels(j,1).m_oPoly_h;
        if ~isempty(oPoly_h)
            delete(oPoly_h.Children)
        end
        vbKeep(j,1) = false;
    end
    oGUIObjects.m_voImageData(i,1).m_voImageLabels = voImageLabels(vbKeep);
end

% Unset projection flags
vnUnsetProjectionFlagIndices = vnUnsetProjectionFlagIndices(1:nCtr_candidates,1);
vnEditedIndices = vnEditedIndices(1:nCtr_edited,1);

for i = 1 : size(vnUnsetProjectionFlagIndices,1)
    nObj = vnUnsetProjectionFlagIndices(i,1);
    bUnset = 1;
    for j = 1 : size(vnEditedIndices,1)
        if nObj == vnEditedIndices(j,1)
            bUnset = 0;
            break;
        end
    end
    
    if bUnset
        oGUIObjects.m_voPCMovableLabel(nObj,1).m_bIsProjected = 0;
    end
end

%% Projection 

for i = 1 : nNumPCObjects
    if voPCMovableLabel(i,1).m_bIsProjected || voPCMovableLabel(i,1).m_bShapeDeleted
        continue
    end
    
    % Determine image
    x = voPCMovableLabel(i,1).m_fBBMiddle_x;
    y = voPCMovableLabel(i,1).m_fBBMiddle_y;
    
    % Choose two candidates for projection according to the object's quadrant. (Object may be in both images.)
    if      (y >= 0) && (x >= 0)
        vsCandiates{1,1} = 'Front';
        vsCandiates{2,1} = 'Right';
    elseif  (y < 0) && (x >= 0)
        vsCandiates{1,1} = 'Right';
        vsCandiates{2,1} = 'Rear';
    elseif  (y <= 0) && (x < 0)
        vsCandiates{1,1} = 'Rear';
        vsCandiates{2,1} = 'Left';
    elseif  (y > 0) && (x < 0)
        vsCandiates{1,1} = 'Left';
        vsCandiates{2,1} = 'Front';
    end
    
    mfPosVel = voPCMovableLabel(i,1).get3DPoints();
    mfPosImg = zeros(9,2);
    % Project into image planes
    for nImg = 1 : 2
        sImageType = vsCandiates{nImg,1};
        nIdx = oGUIObjects.getImageIndex(sImageType);
        
        % If image not available: proceed
        if nIdx == 0
            continue
        end
        oCalibration = oGUIData.getCalibration(sImageType);
        oImageData = oGUIObjects.m_voImageData(nIdx,1);
        
        % Project into image plane
        % Order of points: [downside] front left, front right, back right, back left [upside] ..., [mid]
        for j = 1 : size(mfPosImg,1)
            w = mfPosVel(j,:)';
            
            pix = project3D(w, oCalibration);
            
            mfPosImg(j,:) = pix(1:2,:)';
        end
        
        % Check for position within the image plane. Two edge points are sufficient.
        nCtr = 0;
        for j = 1 : size(mfPosImg,1)
            if (mfPosImg(j,1) > 0) && (mfPosImg(j,2) > 0) ...
                    && (mfPosImg(j,1) <= oImageData.m_nWidth) && (mfPosImg(j,2) <= oImageData.m_nHeight)
                nCtr = nCtr + 1;
            end
        end
        
        if ~(nCtr >= 2)
            continue;
        end
        
        % Check if any lines (bottom or top plane) intersect. This indicates a wrong projection caused by an object that
        % is only partly visible within the image plane. 
        
        mfLinesBottom = [mfPosImg(1,1) mfPosImg(1,2) mfPosImg(2,1) mfPosImg(2,2);
            mfPosImg(2,1) mfPosImg(2,2) mfPosImg(3,1) mfPosImg(3,2);
            mfPosImg(3,1) mfPosImg(3,2) mfPosImg(4,1) mfPosImg(4,2);
            mfPosImg(4,1) mfPosImg(4,2) mfPosImg(1,1) mfPosImg(1,2);];
        
        mfLinesTop = [mfPosImg(5,1) mfPosImg(5,2) mfPosImg(6,1) mfPosImg(6,2);
            mfPosImg(6,1) mfPosImg(6,2) mfPosImg(7,1) mfPosImg(7,2);
            mfPosImg(7,1) mfPosImg(7,2) mfPosImg(8,1) mfPosImg(8,2);
            mfPosImg(8,1) mfPosImg(8,2) mfPosImg(5,1) mfPosImg(5,2);];

        if testIntersection(mfLinesBottom) && testIntersection(mfLinesTop)
            continue;
        end
        
        % Get color  
        if bRandomColor
            col = [rand(1,1) rand(1,1) rand(1,1)];
        else
            col = [1 0 0];
            for c = 1 : size(oConfig.m_voMovableClasses, 1)
                if strcmp(voPCMovableLabel(i,1).m_sClassification, oConfig.m_voMovableClasses(c,1).Name)
                    col =  oConfig.m_voMovableClasses(c,1).Color_RGB;
                end
            end
        end
        
        % Set projected flag
        voPCMovableLabel(i,1).m_bIsProjected = 1;

        % Project
        vnCtr(1, nIdx)  = vnCtr(1, nIdx) + 1;
        nObj            = vnCtr(1, nIdx);
        
        oAxes_h         = oGUIData.m_voImageAxes_h(nIdx,1);
        oImageData      = oGUIObjects.m_voImageData(nIdx,1);
        sClass          = voPCMovableLabel(i,1).m_sClassification;
        nCorrespondence = i;
        
        % Create Image Labels
        if  strcmp(sShapeType, '3D')
            oImageLabel = createImageLabel([], oAxes_h, sShapeType, sClass, mfPosImg, oImageData, col, nCorrespondence, ...
                voPCMovableLabel(i,1));
            
        elseif  strcmp(sShapeType, 'Polygon')
            % Get outmost extreme edge points
            mfVerticesPoly = zeros(4,1);
            vfTargets = [0 0; oImageData.m_nWidth 0; oImageData.m_nWidth oImageData.m_nHeight; 0 oImageData.m_nHeight];
            for t = 1 : 4
                mfSort         = mfPosImg;
                cells          = num2cell(mfSort, 2);
                distFnc        = @(v) sqrt((vfTargets(t,1)-v(1,1))^2 + (vfTargets(t,2)-v(1,2))^2);
                distance       = cellfun(distFnc, cells);
                mfSort(:,3)    = distance;
                mfSort         = sortrows(mfSort, 3);
                mfVerticesPoly(t,1:2) = mfSort(1,1:2);
            end
            
            oImageLabel = createImageLabel([], oAxes_h, sShapeType, sClass, mfVerticesPoly, oImageData, col, nCorrespondence, ...
                voPCMovableLabel(i,1));
        end
        
        % Set reference point
        oImageLabel.m_vfReferencePoint_PC = mfPosImg(9,:);
        oImageLabel.m_bIsProjected = 1;
        
        % Store in matrix
        moImageLabels(nObj, nIdx) = oImageLabel;
    end
end
oGUIObjects.m_voPCMovableLabel = voPCMovableLabel;

% Sort projections into object list
for i = 1 : 4
    nObj = vnCtr(1,i);
    voImageLabels = moImageLabels(1:nObj,i);
    if isempty(oGUIObjects.m_voImageData(i,1).m_voImageLabels)
        % Init objects
        oGUIObjects.m_voImageData(i,1).m_voImageLabels = voImageLabels;
    else
        % Append objects
        oGUIObjects.m_voImageData(i,1).m_voImageLabels(end+1:end+nObj,1) = voImageLabels;
    end
end

%% Nested functions
    function bIntersection = testIntersection(mfLines)
        oResult = lineSegmentIntersect(mfLines, mfLines);
        bIntersection = 0;
        if sum(sum(oResult.intAdjacencyMatrix)) > 8
            bIntersection = 1;
        end
    end
end

