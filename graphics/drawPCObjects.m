function [voMovableLabel, voBB_hg] = drawPCObjects(varargin)
% ---------------------------------------------------------------------------------------------
% Function drawPCObjects(...) draws objects of class cMovableLabel to the specified axes.

% Examples how to call drawPCObjects(...):
% drawPCObjects(voMovableLabel, oAxes_h)                  -> vnColor = default
% drawPCObjects(voMovableLabel, oAxes_h, vnColor)         -> overwrites color by argument
% drawPCObjects(voMovableLabel, oAxes_h, oEditorConfig)   -> Colors according to EditorConfig.xml
% drawPCObjects(voMovableLabel, oAxes_h, oEditorConfig, nMovableDataVectorIndex)
%
% INPUT:
%   voMovableLabel:             Object vector of class cMovableLabel that needs to be drawn
%   oAxes_h:                    Handle to axes where to draw the objects
%   oEditorConfig:              Object of class cEditorConfig containing color information
%   vbIsNewVec:                 Vector of booleans. True if object is new.
%   vbIsPredictedVec:           Vector of booleans. True if object is well predicted.
%   vbIsCorrectedVec:           Vector of booleans. True if object has been corrected.
%   nMovableDataVectorIndex:    If specified, only handed index will be drawn.
%
% OUTPUT:
%   BB_hgs:                     Vector of graphic objects (hg-group)
% ---------------------------------------------------------------------------------------------

% Parse input
voMovableLabel  = varargin{1}; 
oAxes_h         = varargin{2};
vnColor         = [1 0 0];
if isempty(voMovableLabel)
    voBB_hg = gobjects(0,1);
    return;
end

bColorOverwritten = 0;
bUseEditorConfig  = 0;
if(nargin > 2)
    if ~isa(varargin{3}, 'cEditorConfig')
        vnColor = varargin{3};
        bColorOverwritten = 1;
    else
        oEditorConfig = varargin{3};
        bUseEditorConfig = 1;
    end
end

bIndexOverwritten = 0;
if(nargin > 3)
    nMovableDataVectorIndex = varargin{4};
    bIndexOverwritten       = 1;
end

% Create color map
if bUseEditorConfig
    nNumMovableClasses = size(oEditorConfig.m_voMovableClasses,1);
    vsNameSet   = cell(1, nNumMovableClasses);
    vsColorSet  = cell(1, nNumMovableClasses);
    for i = 1 : nNumMovableClasses
       vsNameSet{1,i}  = oEditorConfig.m_voMovableClasses(i,1).Name;
       vsColorSet{1,i} = oEditorConfig.m_voMovableClasses(i,1).Color_RGB;
    end
    mapNameToColor = containers.Map(vsNameSet, vsColorSet);
end

% Create graphic objects
voBB_hg = gobjects(size(voMovableLabel,1),1);

for i = 1 : size(voMovableLabel,1)
    if ~bColorOverwritten
        try
            vnColor = mapNameToColor(voMovableLabel(i,1).m_sClassification);
        catch
            vnColor = [1 0 0]; % default color for classes not contained in EditorConfig.xml 
        end
    end

    sLine = '--';
    if(voMovableLabel(i,1).m_bIsNew && ~voMovableLabel(i,1).m_bIsCorrected)
        sLine = '-';
    end
    if(~voMovableLabel(i,1).m_bIsCorrected && ~voMovableLabel(i,1).m_bIsPredicted)
         sLine = '-';
    end
    
    BB_hg = drawBB(voMovableLabel(i,1).m_fBBMiddle_x, voMovableLabel(i,1).m_fBBMiddle_y,...
                   voMovableLabel(i,1).m_fBBMiddle_z, voMovableLabel(i,1).m_fBBYaw, ... 
                   voMovableLabel(i,1).m_fBBLength, voMovableLabel(i,1).m_fBBWidth, voMovableLabel(i,1).m_fBBHeight, vnColor, oAxes_h, sLine);
               
    % Draw reference point
    refPoint = zeros(3,1);
    refPoint(1:2,1) = voMovableLabel(i,1).m_vfReferencePoint(1:2);
    refPoint(3,1)   = voMovableLabel(i,1).m_fBBMiddle_z;
    scatter3(refPoint(1,1), refPoint(2,1), refPoint(3,1), 40, vnColor, 'x', 'Parent', BB_hg, 'Tag', 'reference', 'Visible', 'off');
    
    if(bIndexOverwritten)
        idx = nMovableDataVectorIndex;
    else
        idx = i;
    end
    
    % Display classification
    sClass = sprintf('%d %s', idx, voMovableLabel(i,1).m_sClassification);
    
    fBBYaw = voMovableLabel(i,1).m_fBBYaw * pi()/180;
    bbMiddlePlane(1,1) = voMovableLabel(i,1).m_fBBMiddle_x;
    bbMiddlePlane(2,1) = voMovableLabel(i,1).m_fBBMiddle_y;
    lvec(1,1) = voMovableLabel(i,1).m_fBBLength/2*cos(fBBYaw);
    lvec(2,1) = voMovableLabel(i,1).m_fBBLength/2*sin(fBBYaw);
    wvec(1,1) = voMovableLabel(i,1).m_fBBWidth/2*cos(fBBYaw + pi()/2);
    wvec(2,1) = voMovableLabel(i,1).m_fBBWidth/2*sin(fBBYaw + pi()/2);
    
    fScal = dot(lvec, [0;1]);
    if(fScal >= 0)
        vfPos = bbMiddlePlane + lvec - wvec;
    else
        vfPos = bbMiddlePlane - lvec + wvec;
    end

    text(vfPos(1,1)+.2, vfPos(2,1)+.2, voMovableLabel(i,1).m_fBBMiddle_z, sClass, 'Parent', BB_hg, 'Color', vnColor, ...
        'Tag', 'text', 'Visible', 'on', 'FontWeight', 'bold');
    
    % Draw velocity vector
    drawVelocityVector(voMovableLabel(i,1), vnColor, BB_hg);
    
    % Display object data
    sObjectData = getObjectDataText(voMovableLabel(i,1));
    text(vfPos(1,1)+1, vfPos(2,1)-2, voMovableLabel(i,1).m_fBBMiddle_z, sObjectData, 'Parent', BB_hg, 'Color', vnColor, ...
        'Tag', 'object_data', 'Visible', 'off');
    
    % Set object vector
    voBB_hg(i,1)                = BB_hg;
    voMovableLabel(i,1).m_Box_h = BB_hg;
    
    % Draw accumulator
    mfAccu = voMovableLabel(i,1).m_mfRegisteredPointMatrix;
    if ~isempty(mfAccu)
        scatter3(mfAccu(:,1), mfAccu(:,2), mfAccu(:,3), 20, vnColor, '.', 'Visible', 'off', 'Parent', BB_hg, ...
            'Tag', 'accu');
    end
end

end

