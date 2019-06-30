classdef cGUIObjects < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cGUIObjects aims to contain all object information such as the scan's object labels
    % and image labels, as well as the scan's metadata. This class represents the former "guiBoxes" data structure.
    % ---------------------------------------------------------------------------------------------
    properties
        m_voPCMovableLabel  = cPCMovableLabel.empty;     % vector of cMovableLabel objects
        m_voImageData       = cImageData.empty;          % vector of class cImageData containing up to 4 images (front, right, rear, left)
    end
    
    methods
        function obj = cGUIObjects()
            obj.m_voImageData(4,1) = cImageData();     % create object vector for four images
        end
        
        % This function returns the index within the image object vector of cGUIObjects.
        function [nIdx] = getImageIndex(obj, sImageType)
            nIdx = 0;
            for i = 1 : 4
                if strcmp(sImageType, obj.m_voImageData(i,1).m_sImageType)
                    nIdx = i;
                end
            end
        end
        
        % This function updates the PC objects' projection flags (e.g. when image labels are read from file)
        function updateProjectionFlag(obj)
            for i = 1 : 4
                oImageData = obj.m_voImageData(i,1);
                voImageLabels = oImageData.m_voImageLabels;
                for j = 1 : size(voImageLabels,1)
                    nObj = voImageLabels(j,1).m_nIdxPCObject;
                    if nObj > 0
                        obj.m_voPCMovableLabel(nObj).m_bIsProjected = 1;
                    end
                end
            end
        end
        
        % This function updates the point cloud correspondences and reference points for image labels
        function obj = updateImageLabelsCorrespondences(obj, oGUIData)
            voPCMovableLabel = obj.m_voPCMovableLabel;
            for i = 1 : 4
                oImageData = obj.m_voImageData(i,1);
                oCalib = oGUIData.getCalibration(oImageData.m_sImageType);
                voImageLabels = oImageData.m_voImageLabels;
                
                if isempty(oCalib)  % image not available
                    continue
                end
                
                for j = 1 : size(voImageLabels, 1)
                    if voImageLabels(j,1).m_nIdxPCObject > 0
                        % Set handle to PC object
                        oPCLabel = voPCMovableLabel(voImageLabels(j,1).m_nIdxPCObject,1);
                        voImageLabels(j,1).m_oPCMovableLabel = oPCLabel;
                        
                        % Set projected reference point or corresponding PC object (used for tracking in image)
                        vfMid = [oPCLabel.m_fBBMiddle_x; oPCLabel.m_fBBMiddle_y; oPCLabel.m_fBBMiddle_z];
                        vfPoint_2D = project3D(vfMid, oCalib);
                        voImageLabels(j,1).m_vfReferencePoint_PC = vfPoint_2D';
                    end
                end
            end
        end
        
        % This function updates the label vector according to predicted point cloud objects using the point cloud
        % correspondences.
        function obj = updateLabelVectorToPrediction(obj, oGUIData)
            for i = 1 : 4
                oImageData = obj.m_voImageData(i,1);
                oCalib = oGUIData.getCalibration(oImageData.m_sImageType);
                voImageLabels = oImageData.m_voImageLabels;
                voPCMovableLabel = obj.m_voPCMovableLabel;
                
                % Compute position delta in 2D and change position
                bIndex = true(size(voImageLabels,1), 1);
                for j = 1 : size(voImageLabels, 1)
                    if voImageLabels(j,1).m_nIdxPCObject > 0
                        % Get predicted corresponding PC object
                        oPCLabel = voPCMovableLabel(voImageLabels(j,1).m_nIdxPCObject,1);
                            
                        % Polygons/rectangles: Use reference point of corresponding projected PC object
                        if ~strcmp(voImageLabels(j,1).m_sShapeType, '3D')
                            % Get new projected mid
                            vfMid = [oPCLabel.m_fBBMiddle_x; oPCLabel.m_fBBMiddle_y; oPCLabel.m_fBBMiddle_z];
                            vfPoint_2D = project3D(vfMid, oCalib);
                            
                            % Compute delta to predecessor position
                            vfDelta = vfPoint_2D' - voImageLabels(j,1).m_vfReferencePoint_PC;
                            
                            % Update shape
                            voImageLabels(j,1).updateShape(vfDelta);
                            
                            % Set predicted flag
                            voImageLabels(j,1).m_bIsPredicted = 1;
                            
                            % Set new reference point
                            voImageLabels(j,1).m_vfReferencePoint_PC = vfPoint_2D';
                            
                        % 3D-shapes: Reproject 3D bounding box
                        else
                            mfPos = oPCLabel.get3DPoints();
                            mfPosImg = zeros(size(mfPos,1), 2);
                            for p = 1 : size(mfPos,1)
                                pix = project3D(mfPos(p,:)', oCalib);
                                mfPosImg(p,:) = pix';
                            end
                            % Set new reference point
                            voImageLabels(j,1).m_vfReferencePoint_PC = mfPosImg(9,:);
                            
                            % Apply correction delta
                            if ~isempty(voImageLabels(j,1).m_vfCorrectionDelta)
                                mfPosImg = mfPosImg + repmat(voImageLabels(j,1).m_vfCorrectionDelta, size(mfPosImg,1), 1);
                            end
                            
                            % Update shape
                            voImageLabels(j,1).updateVertexMatrix(mfPosImg);
                        end
                    end
                    
                    % Check if object is out of bounds
                    xLim = [0 oImageData.m_nWidth];
                    yLim = [0 oImageData.m_nHeight];
                    if voImageLabels(j,1).outOfBounds(xLim, yLim)
                        bIndex(j,1) = false;
                    end
                end
                
                obj.m_voImageData(i,1).m_voImageLabels = voImageLabels(bIndex);
            end
        end
        
        % This function returns the next object with the prediction flag unset
        function [nObj_next] = getNextUncertainObject(obj, nObj_cur)
            nObj_next = 0;

            % Proceed to the end of the vector
            for i = (nObj_cur+1) : size(obj.m_voPCMovableLabel,1)
                if ~obj.m_voPCMovableLabel(i,1).m_bIsPredicted && ~obj.m_voPCMovableLabel(i,1).m_bIsCorrected
                    nObj_next = i;
                    return
                end
            end
            
            % Search from the beginning
            for i = 1 : (nObj_cur-1) 
                if ~obj.m_voPCMovableLabel(i,1).m_bIsPredicted && ~obj.m_voPCMovableLabel(i,1).m_bIsCorrected
                    nObj_next = i;
                    return
                end
            end
        end
        
        function [bAllCorrected] = checkForUncorrectedObjects(obj)
            bAllCorrected = 1;
            for i = 1 : size(obj.m_voPCMovableLabel, 1);
                if ~obj.m_voPCMovableLabel(i,1).m_bIsCorrected
                    bAllCorrected = 0;
                    return
                end
            end
        end
    end

    methods (Access = protected)
        function oCopy = copyElement(obj)
            oCopy = copyElement@matlab.mixin.Copyable(obj);
            oCopy.m_voPCMovableLabel = copy(obj.m_voPCMovableLabel);
            oCopy.m_voImageData = copy(obj.m_voImageData);
        end
    end
end

