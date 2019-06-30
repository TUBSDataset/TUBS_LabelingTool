function [oImageLabel] = createImageLabel(oImageLabel, oAxes_h, sShapeType, sClass, mfPosition, oImageData, vnColor, ...
    nPC_correspondence, oPCMovableLabel)
% ---------------------------------------------------------------------------------------------
% Function createRectangle(...) creates an object of class cImageLabel and a corresponding graphics object.
% Note that for polygons and rectangles the image processing toolbox is necessary.

% mfPosition for rectangles:    [left, bottom, width, height]
% mfPosition for 3D-Shapes:     Matrix of edge points: [[(downside) front left, front right, back right, back left (upside) ..., (mid)]
% mfPosition for polyongs:      Matrix of vertices
%
% INPUT:
%   oImageLabel:        Handle to existing label object that is be updated, if empty a new object will be created
%   oAxes_h:            Handle to axes were shape is created
%   sShapeType:         Shape type string (Rectangle, Polygon, 3D)
%   sClass:             The object's class
%   mfPosition:         Matrix of position data, see above for more information
%   oImageData:         Corresponding object of class cImageData
%   vnColor:            Color of shape. Currently predefined for classes licence plate and face).
%   nPCCorres:          (Optional) Index of corresponding point cloud object
%   oPCMovableLabel:    (Optional) Handle to corresponding point cloud object
%
% OUTPUT:
%   oImageLabel:        Created object of class cImageLabel
%
% DETAILS:
%   Requirements:       image processing toolbox for rectangle and polygon shapes.
% ---------------------------------------------------------------------------------------------

hDeletionFnc    = @oImageData.validateImageLabelVector;
oPoly_h         = hggroup;

% Create new object if none is passed
if isempty(oImageLabel)
    oImageLabel = cImageLabel();
end

set(oPoly_h, 'Parent', oAxes_h, 'Tag', sClass, 'DeleteFcn', {@deleteCallback, oPoly_h, oImageLabel});

oImageLabel.m_sClass     = sClass;
oImageLabel.m_sShapeType = sShapeType;
oImageLabel.m_vfColor    = vnColor;

if nargin > 7
    oImageLabel.m_nIdxPCObject      = nPC_correspondence;
    oImageLabel.m_oPCMovableLabel   = oPCMovableLabel;
end

% Create rectangle for faces and licence plates
if      strcmp(sShapeType, 'Rectangle')
    oShape_h = imrect(oPoly_h, mfPosition);
    
    if strcmp(sClass, 'Face')
        setColor(oShape_h, 'red');
        oImageLabel.m_vfColor = [1 0 0];
    else
        setColor(oShape_h, 'blue');
        oImageLabel.m_vfColor = [0 0 1];
    end
    
    hFcn = makeConstrainToRectFcn('imrect', get(oAxes_h,'XLim'), get(oAxes_h,'YLim'));
    setPositionConstraintFcn(oShape_h, hFcn);
    
    % Delete unnecessary menu options
    voChildren_h    = get(oShape_h, 'Children');
    oMenu_h         = get(voChildren_h(1), 'UIContextMenu');
    voEntrys_h      = get(oMenu_h, 'Children');
    delete(voEntrys_h(4));
    delete(voEntrys_h(3));
    delete(voEntrys_h(2));
    delete(voEntrys_h(1));
    
    % Add vertex update callback
    addNewPositionCallback(oShape_h, @newPositionCallback);
    
    % Add menu options
    uimenu(oMenu_h, 'Label', 'Delete rectangle', 'Callback', {@deleteCallback, oPoly_h, oImageLabel});
    uimenu(oMenu_h, 'Label', 'Set LiDAR correspondence', 'Callback', {@setCorrespondence, oPoly_h, oImageLabel});
    
    if oImageLabel.m_nIdxPCObject > 0
        sCorrespondence = sprintf('%d', oImageLabel.m_nIdxPCObject);
        text(mfPosition(1,1)+mfPosition(1,3)/2, mfPosition(1,2)+mfPosition(1,4)/2, ...
            sCorrespondence, 'Parent', oPoly_h, 'Color', oImageLabel.m_vfColor, 'Tag', 'correspondence', 'FontWeight', 'bold');
    end   
    
    % Update position vector
    oImageLabel.updateVertexMatrix(mfPosition);
    
elseif  strcmp(sShapeType, '3D') % Create 3D Boxes
    
    try
        oMenu_h = uicontextmenu(oAxes_h.Parent.Parent);     % Set image panel figure as parent
        uimenu(oMenu_h, 'Label', 'Delete 3D', 'Callback', {@deleteCallback, oPoly_h, oImageLabel});
    catch
        oMenu_h = [];
    end
    
    % Draw 3D bounding box
    sLine = '-'; fLineWidth = 1.5;
    
    % Bottom
    x = [mfPosition(1:4,1); mfPosition(1,1)];     % close rectangle
    y = [mfPosition(1:4,2); mfPosition(1,2)];
    plot(x, y, 'Linestyle', sLine, 'Color', vnColor, 'Parent', oPoly_h, 'LineWidth', fLineWidth, ...
        'Tag', 'bottom', 'ButtonDownFcn', @mouseClick_Callback, 'UIContextMenu', oMenu_h);
    
    % Upside
    x = [mfPosition(5:8,1); mfPosition(5,1)];
    y = [mfPosition(5:8,2); mfPosition(5,2)];
    plot(x, y, 'Linestyle', sLine, 'Color', vnColor, 'Parent', oPoly_h, 'LineWidth', fLineWidth, ...
        'Tag', 'top', 'ButtonDownFcn', @mouseClick_Callback, 'UIContextMenu', oMenu_h);
    
    % Upstrokes
    for i = 1 : 4
        x = [mfPosition(i,1); mfPosition(i+4,1)];
        y = [mfPosition(i,2); mfPosition(i+4,2)];
        plot(x, y, 'Linestyle', sLine, 'Color', vnColor, 'Parent', oPoly_h, 'Tag', 'up', ...
            'ButtonDownFcn', @mouseClick_Callback, 'LineWidth', fLineWidth, 'UIContextMenu', oMenu_h);
    end
    
    % Mid
    scatter(mfPosition(9,1), mfPosition(9,2), 50, vnColor, 'x', 'Parent', oPoly_h, 'LineWidth', fLineWidth,...
        'Tag', 'mid', 'ButtonDownFcn', @mouseClick_Callback, 'UIContextMenu', oMenu_h, 'Visible', 'off');
    oImageLabel.updateVertexMatrix(mfPosition);
    
    % Text
    sCorrespondence = sprintf('%d', nPC_correspondence);
    text(mfPosition(9,1), mfPosition(9,2), sCorrespondence, 'Parent', oPoly_h, 'Color', vnColor, ...
        'Tag', 'correspondence', 'FontWeight', 'bold');
    
    % Update vertices
    oImageLabel.updateVertexMatrix(mfPosition);
    
elseif strcmp(sShapeType, 'Polygon') % Create Polygons
    
    oShape_h = impoly(oPoly_h, mfPosition);
    setColor(oShape_h, vnColor);
    
    hFcn = makeConstrainToRectFcn('impoly', get(oAxes_h,'XLim'), get(oAxes_h,'YLim'));
    setPositionConstraintFcn(oShape_h, hFcn);
    
    % Delete unnecessary menu options
    voChildren_h    = get(oShape_h, 'Children');
    
    oMenu_h         = get(voChildren_h(1), 'UIContextMenu');
    voEntrys_h      = get(oMenu_h, 'Children');
    
    delete(voEntrys_h(3));
    delete(voEntrys_h(2));
    
    oMenu_h         = get(voChildren_h(size(voChildren_h, 1)), 'UIContextMenu');
    voEntrys_h      = get(oMenu_h, 'Children');
    
    delete(voEntrys_h(3));
    delete(voEntrys_h(2));
    delete(voEntrys_h(1))
    
    % Add menu options
    uimenu(oMenu_h, 'Label', 'Delete polygon', 'Callback', {@deleteCallback, oPoly_h, oImageLabel});
    
    % Add vertex update callback
    addNewPositionCallback(oShape_h, @newPositionCallback);
    
    % Text
    sCorrespondence = sprintf('%d', nPC_correspondence);
    text(mean(mfPosition(:, 1)), mean(mfPosition(:, 2)), sCorrespondence, 'Parent', oPoly_h, 'Color', vnColor, ...
        'Tag', 'correspondence', 'FontWeight', 'bold');
    
    % Update position vector
    oImageLabel.updateVertexMatrix(mfPosition);
    
else
    error('Undefined shape type.');
end

% Set graphics
oImageLabel.m_oPoly_h = oPoly_h;

%% Nested functions
    function newPositionCallback(vfPos)
        oImageLabel.updateVertexMatrix(vfPos);
        oText_h = findobj(oImageLabel.m_oPoly_h, 'Tag', 'correspondence');
        
        % Update text position
        if ~isempty(oText_h)
            if  strcmp(oImageLabel.m_sShapeType, 'Rectangle')
                oText_h.Position(1,1:2) = [vfPos(1,1) + vfPos(1,3)/2, vfPos(1,2) + vfPos(1,4)/2];
            elseif  strcmp(oImageLabel.m_sShapeType, 'Polygon')
                oText_h.Position(1,1:2) = mean(vfPos);
            end
        end
        
        oImageLabel.m_bIsEdited = 1;
    end

    function deleteCallback(~, ~, oPoly_h, oImageLabel)
        if ~isvalid(oImageLabel)
            return
        end
        % Unset projection flag
        oImageLabel.m_oPCMovableLabel.m_bIsProjected = 0;
        oImageLabel.m_oPCMovableLabel.m_bShapeDeleted = 1;
        
        % Delete graphic and label
        delete(oPoly_h.Children);
        delete(oImageLabel);
        
        % Remove deleted items from label vector of class cImageData
        hDeletionFnc();
        drawnow;
    end

    function mouseClick_Callback(~,~)
        oImageLabel.mouseClick();
    end

    function setCorrespondence(~, ~, oPoly_h, oImageLabel)
        % Create dialog
        sCorres = inputdlg('Set number of corresponding LiDAR object');
        if isempty(sCorres)
            return
        end
        nCorres = str2double(sCorres);
        
        while isnan(nCorres)
            sCorres = inputdlg('Please type in a valid number.');
            if isempty(sCorres)
                return
            end
            nCorres = str2double(sCorres);
        end
        oImageLabel.m_nIdxPCObject = nCorres;
        
        % Get middle / reference point from shape specific position information
        vfMid = oImageLabel.m_vfReferencePoint;
        
        % Display correspondence
        text(vfMid(1,1), vfMid(1,2), num2str(nCorres), 'Color', oImageLabel.m_vfColor, 'Parent', oPoly_h, ...
            'Tag', 'correspondence', 'FontWeight', 'bold');
    end
end

