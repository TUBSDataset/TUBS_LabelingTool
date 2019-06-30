function [nCode] = writeImageData(sBaseDir, nSequenceID, nPCID, oImageData)
% ---------------------------------------------------------------------------------------------
% Function writeImageData writes XML files containing image object labels.
%
% INPUT:
%   sBaseDir:           String containing the base directory as specified within PCEditorToolGUI.m
%   nSequenceID:        Current sequence ID
%   nPCID               Current PCID to be read
%   oImageData:         Object of class cImageData to write as XML
%
% OUTPUT:
%   nCode:              Error code. 0 = success, 1 = error while writing XML file
% ---------------------------------------------------------------------------------------------
nCode       = 0;

switch oImageData.m_sImageType
    case 'Front'
        nIdentifier = 5;
    case 'Left'
        nIdentifier = 6;
    case 'Rear'
        nIdentifier = 7;
    case 'Right'
        nIdentifier = 8;
end

sFileDir = buildPath(sBaseDir, nSequenceID, nPCID, nIdentifier);

% Build tree
oTree.FormatVersion = oImageData.m_fFormatVersion;
oTree.ImageType     = oImageData.m_sImageType;
oTree.Height        = oImageData.m_nHeight;
oTree.Width         = oImageData.m_nWidth;
oTree.Timestamp     = oImageData.m_nTimestamp;
oTree.PCDeltaT_ms   = oImageData.m_fDeltaT_ms;
oTree.Labels        = [];

for i = 1 : size(oImageData.m_voImageLabels,1) 
    oTree.Labels.Object(i,1).Class      = oImageData.m_voImageLabels(i,1).m_sClass;
    oTree.Labels.Object(i,1).ShapeType  = oImageData.m_voImageLabels(i,1).m_sShapeType;
    oTree.Labels.Object(i,1).CorrespondingPCObject = oImageData.m_voImageLabels(i,1).m_nIdxPCObject;
    
    nNumVertices = size(oImageData.m_voImageLabels(i,1).m_mfVertexVector,1);
    for j = 1 : nNumVertices
        oTree.Labels.Object(i,1).VertexVector.Vertex(j,1).x = round(oImageData.m_voImageLabels(i,1).m_mfVertexVector(j,1)*100)/100;
        oTree.Labels.Object(i,1).VertexVector.Vertex(j,1).y = round(oImageData.m_voImageLabels(i,1).m_mfVertexVector(j,2)*100)/100;
    end
end

if isempty(oImageData.m_voImageLabels)
    oTree.Labels = '';
end

% Write tree using the external xml_write function. See xml_write.m for more information.
try
    xml_write(sFileDir, oTree, strcat('ImageLabels_', oImageData.m_sImageType));
catch
    nCode = 1;
end

end

