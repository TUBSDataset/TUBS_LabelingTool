function [oImageData, nCode] = readImageData(sBaseDir, nSequenceID, nPCID, sImageType, nReadMode_in)
% ---------------------------------------------------------------------------------------------
% Function readImageData(...) reads XML files and .jpg images and constructs an cImageData object containing
% image metadata and labels.
%
% INPUT:
%   sBaseDir:           String containing the base directory as specified within PCEditorToolGUI.m
%   nSequuenceID:       Current sequence ID
%   nPCID:              Current PCID to be read
%   bReadMode_in:       (Optional) 0: Both, 1: Images, 2: Labels
%
% OUTPUT:
%   oImageData:         Object of class cImageData
%   nCode:              Error code. 0 = success, 1 = error while reading XML file, 2 = no files available
% ---------------------------------------------------------------------------------------------

oImageData  = cImageData();
nCode = 0;

nReadMode = 0;
if nargin > 4
    nReadMode = nReadMode_in;
end

bReadImages = 0;
bImageOnly  = 0;
bReadLabels = 0;

switch nReadMode
    case 0
        bReadImages = 1;
        bReadLabels = 1;
    case 1
        bReadImages = 1;
    case 2
        bReadLabels = 1;
    case 3
        bImageOnly = 1;
end
   
switch sImageType
    case 'Front'
        nIdentifier_XML = 5;
        nIdenfifier_JPG = 1;
    case 'Left'
        nIdentifier_XML = 6;
        nIdenfifier_JPG = 2;
    case 'Rear'
        nIdentifier_XML = 7;
        nIdenfifier_JPG = 3;
    case 'Right'
        nIdentifier_XML = 8;
        nIdenfifier_JPG = 4;
end

sFilePath_XML = buildPath(sBaseDir, nSequenceID, nPCID, nIdentifier_XML);
sFilePath_JPG = buildPath(sBaseDir, nSequenceID, nPCID, nIdenfifier_JPG);

if checkForEmptyDirectory(sFilePath_XML) || checkForEmptyDirectory(sFilePath_JPG)
    nCode = 2;
    return;
end
try
    % Read images
    if bReadImages
        oImageData.m_oImage = imread(sFilePath_JPG); 
    end
    
    if bImageOnly
        return 
    end
    
    % Parse XML file using the external xml_read function. See xml_read.m for more information.
    oTree = xml_read(sFilePath_XML);
    
    oImageData.m_bImageAvailable = 1;
catch
    nCode = 1;
    return;
end

% Parse tree
oImageData.m_fFormatVersion = oTree.FormatVersion;
oImageData.m_sImageType     = oTree.ImageType;
oImageData.m_nHeight        = oTree.Height;
oImageData.m_nWidth         = oTree.Width;
oImageData.m_nTimestamp     = oTree.Timestamp;
oImageData.m_fDeltaT_ms     = oTree.PCDeltaT_ms;

if ~bReadLabels
    return
end

% Create image label vector
if ~isempty(oTree.Labels)
    oImageData.m_voImageLabels(size(oTree.Labels.Object,1),1) = cImageLabel();
end

for i = 1 : size(oImageData.m_voImageLabels,1)
    oImageData.m_voImageLabels(i,1).m_sClass        = oTree.Labels.Object(i,1).Class;
    oImageData.m_voImageLabels(i,1).m_sShapeType    = oTree.Labels.Object(i,1).ShapeType;
    oImageData.m_voImageLabels(i,1).m_nIdxPCObject  = oTree.Labels.Object(i,1).CorrespondingPCObject;
    
    % Set flags
    oImageData.m_voImageLabels(i,1).m_bIsLoaded = 1;
    
    % Parse vertex matrix
    nNumVertices = size(oTree.Labels.Object(i,1).VertexVector.Vertex,1);
    oImageData.m_voImageLabels(i,1).m_mfVertexVector  = zeros(nNumVertices, 2);
    for j = 1 : nNumVertices
        oImageData.m_voImageLabels(i,1).m_mfVertexVector(j,1) = oTree.Labels.Object(i,1).VertexVector.Vertex(j,1).x;
        oImageData.m_voImageLabels(i,1).m_mfVertexVector(j,2) = oTree.Labels.Object(i,1).VertexVector.Vertex(j,1).y;
    end
    mfVertexVector = oImageData.m_voImageLabels(i,1).m_mfVertexVector;
    
    % Parse position for rectangle shapes
    if      strcmp(oImageData.m_voImageLabels(i,1).m_sShapeType, 'Rectangle')
       fXmin = 10^6; 
       fYmin = 10^6;
       fXmax = 0;
       fYmax = 0;
       for j = 1 : size(mfVertexVector, 1)
           if mfVertexVector(j,1) < fXmin
               fXmin = mfVertexVector(j,1);
           end
           if mfVertexVector(j,1) > fXmax
               fXmax = mfVertexVector(j,1);
           end
           if mfVertexVector(j,2) < fYmin
               fYmin = mfVertexVector(j,2);
           end
           if mfVertexVector(j,2) > fYmax
               fYmax = mfVertexVector(j,2);
           end
       end
       oImageData.m_voImageLabels(i,1).m_mfPosition = [fXmin, fYmin, fXmax-fXmin, fYmax-fYmin];
    
    % Parse position for 3D shapes
    elseif  strcmp(oImageData.m_voImageLabels(i,1).m_sShapeType, '3D')
        oImageData.m_voImageLabels(i,1).m_mfPosition = mfVertexVector;
     
    % Parse position for polygons
    elseif  strcmp(oImageData.m_voImageLabels(i,1).m_sShapeType, 'Polygon')
        oImageData.m_voImageLabels(i,1).m_mfPosition = mfVertexVector;
    else
        error('Unimplemented shape.'); 
    end
end

end

