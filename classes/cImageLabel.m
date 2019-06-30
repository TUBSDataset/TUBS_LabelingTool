classdef cImageLabel < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cImageLabel stores object label information, such as class, shape and position information.
    % Objects of this class are part of cImageData as a container class.
    % ---------------------------------------------------------------------------------------------
    properties      
        m_sClass                        = '';
        m_sShapeType                    = '';   % either rectangle, polygon or 3D-Box
        m_nIdxPCObject                  = 0;   % index of corresponding point cloud object. 0 for non existing correspondence 
        m_mfVertexVector                = [];   % (n x 2)-matrix containing vertices as pixels
        
        % Helper variables go here
        m_bIsEdited = 0;             % true if object is edited (e.g. moved) 
        m_bIsUserDefined = 0;        % true if label is not created by projection but by user definition (e.g. face)
        m_bIsPredicted = 0;          % true if label is predicted into current frame 
        m_bIsLoaded = 0;             % true if label was loaded from file
        m_bIsProjected = 0;          % true if label is created by projection
        
        m_oPCMovableLabel;           % handle to corresponding point cloud object
        m_vfColor;                   % object color
        m_oPoly_h;                   % graphics object
        m_mfPosition;                % position matrix for graphics, shape type specific. See createImageLabel(...) for more information.
        m_vfReferencePoint;          % reference point (mid for rectangles, 3D-shapes and polygons)
        m_vfReferencePoint_PC;       % projected reference point for PC object 
                                     % (used to track user defined image labels and apply corrections to 3D-shapes)
        
        m_bClicked          = false; % object clicked by mouse
        m_vfClickPosistion  = [];    % coorinates at mouse click
        
        m_vfReferencePoint_prev;     % Dragging 3D-shapes: Reference point before correction
        m_vfCorrectionDelta;         % Delta to original projected reference point (m_vfReferencePoint_PC)
    end
    
    methods
        % Callback function that is called if an object changed position
        function obj = updateVertexMatrix(obj, mfPos)
            if      strcmp(obj.m_sShapeType, 'Rectangle')
                % vfPos = [left, bottom, width, height]
                fWidth      = mfPos(1,3); 
                fHeight     = mfPos(1,4);
                vfBotLeft    = [mfPos(1,1), mfPos(1,2)];
                vfBotRight   = vfBotLeft + [fWidth, 0];
                vfTopLeft    = vfBotLeft + [0, fHeight];
                vfTopRight   = vfBotLeft + [fWidth, fHeight];
            
                mfVertices = [vfBotLeft; vfBotRight; vfTopLeft; vfTopRight];
                obj.m_mfVertexVector    = mfVertices;
                obj.m_mfPosition        = mfPos;
                obj.m_vfReferencePoint  = [mfPos(1,1) + fWidth/2, mfPos(1,2) + fHeight/2];
            
            elseif  strcmp(obj.m_sShapeType, '3D')
                % vfPos = [[(downside) front left, front right, back right, back left (upside) ..., (mid)]
                obj.m_mfVertexVector   = mfPos;
                obj.m_mfPosition       = mfPos;
                obj.m_vfReferencePoint = mfPos(9,:);
                
            elseif  strcmp(obj.m_sShapeType, 'Polygon')
                % vfPos = vertices
                obj.m_mfVertexVector   = mfPos;
                obj.m_mfPosition       = mfPos;
                obj.m_vfReferencePoint = mean(mfPos);
            end
        end
        
        % This function transforms the shape according to a given 2D delta
        function obj = updateShape(obj, vfDelta_2D)
            % Compute position (mfPos) and update
            mfPos = obj.m_mfPosition;
            fDelta_x = vfDelta_2D(1,1);
            fDelta_y = vfDelta_2D(1,2);
            switch obj.m_sShapeType
                case 'Rectangle'
                    mfPos = mfPos + [fDelta_x, fDelta_y, 0, 0];
                case '3D'
                    mfCorr = repmat(vfDelta_2D, 9, 1);
                    mfPos = mfPos + mfCorr;
                case 'Polygon'
                    mfCorr = repmat(vfDelta_2D, size(mfPos,1), 1);
                    mfPos = mfPos + mfCorr;
            end
            obj.updateVertexMatrix(mfPos);
        end
        
        function bLimitReached = outOfBounds(obj, xLim, yLim)
            bLimitReached = 1;
            for i = 1 : size(obj.m_mfVertexVector, 1)
                xData = obj.m_mfVertexVector(i,1);
                yData = obj.m_mfVertexVector(i,2);
                
                % Test vectors for limits, one point within the image is enough to keep an object
                if xData <= xLim(1,2) && xData >= xLim(1,1) && yData <= yLim(1,2) && yData >= yLim(1,1)
                    bLimitReached = 0;
                    break;
                end
            end
        end
        
        % Following functions implement dragging 3D-shapes to new positions
        function mouseClick(obj)
            obj.m_bClicked = true;
            coordinates = get(gca, 'CurrentPoint');
            obj.m_vfClickPosistion = coordinates(1,1:2);
        end
        
        function mouseRelease(obj)
            obj.m_bClicked = false;
            obj.m_vfClickPosistion = [];
        end
        
        function b = isClicked(obj)
            b = obj.m_bClicked;
        end
        
        function move(obj)
            obj.m_bIsEdited = 1;
            coordinates = get(gca, 'CurrentPoint');
            x = coordinates(1,1);
            y = coordinates(1,2);
            
            bAxesLimitReached = 0;
            
            for i = 1:size(obj.m_oPoly_h.Children, 1)
                fDelta_x = x - obj.m_vfClickPosistion(1,1);
                fDelta_y = y - obj.m_vfClickPosistion(1,2);
                
                % New position vector
                if strcmp(obj.m_oPoly_h.Children(i,1).Type, 'text')
                    continue; 
                end
            end
            
            if ~bAxesLimitReached
                for i = 1:size(obj.m_oPoly_h.Children, 1)
                    fDelta_x = x - obj.m_vfClickPosistion(1,1);
                    fDelta_y = y - obj.m_vfClickPosistion(1,2);
                    
                    if strcmp(obj.m_oPoly_h.Children(i,1).Type, 'text')
                        continue;
                    end
                    
                    obj.m_oPoly_h.Children(i,1).XData = obj.m_oPoly_h.Children(i,1).XData + fDelta_x;
                    obj.m_oPoly_h.Children(i,1).YData = obj.m_oPoly_h.Children(i,1).YData + fDelta_y;
                end
                
                % Transform correspondence text
                oText_h = findobj(obj.m_oPoly_h, 'Tag', 'correspondence');
                oText_h.Position(1,1) = oText_h.Position(1,1) + fDelta_x;
                oText_h.Position(1,2) = oText_h.Position(1,2) + fDelta_y;
            end
            
            obj.m_vfClickPosistion(1,1) = x;
            obj.m_vfClickPosistion(1,2) = y;
            
            % Update vertex matrix
            mfPos = zeros(8,2);
            for i = 1:size(obj.m_oPoly_h.Children, 1)
                if strcmp(obj.m_oPoly_h.Children(i,1).Tag, 'bottom')
                    mfPos(1:4,1) = obj.m_oPoly_h.Children(i,1).XData(1,1:4)';   % bottom
                    mfPos(1:4,2) = obj.m_oPoly_h.Children(i,1).YData(1,1:4)';
                end
                if strcmp(obj.m_oPoly_h.Children(i,1).Tag, 'top')
                    mfPos(5:8,1) = obj.m_oPoly_h.Children(i,1).XData(1,1:4)';   % top
                    mfPos(5:8,2) = obj.m_oPoly_h.Children(i,1).YData(1,1:4)';
                end
                if strcmp(obj.m_oPoly_h.Children(i,1).Tag, 'mid')
                    mfPos(9,1)   = obj.m_oPoly_h.Children(i,1).XData;           % mid
                    mfPos(9,2)   = obj.m_oPoly_h.Children(i,1).YData;
                end
            end
            
            obj.updateVertexMatrix(mfPos);
            
            % Compute correction delta
            obj.m_vfCorrectionDelta = mfPos(9,:) - obj.m_vfReferencePoint_PC;
        end
    end
end

