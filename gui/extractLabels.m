function [clClassPoints] = extractLabels(oLaserscan, oEditorConfig, nNumPoints)
% ---------------------------------------------------------------------------------------------
% Function extractLabels sorts and extracts all points according to their label ID.
%
% INPUT:
%   oLaserScan:      Object of class cLaserScan containing the point data.
%   oEditorConfig:   Object of class cEditorConfig created on Startup according to EditorConfig.xml
%   nNumberOfPoints: Total number of points (64*2000)
%
% OUTPUT:
%   clClassPoints:   Cell array storing class information: Name | Points | Counter | ID | Color
% ---------------------------------------------------------------------------------------------

nNumStationaryClasses = size(oEditorConfig.m_voStationaryClasses,1);
nNumMovableClasses    = size(oEditorConfig.m_voMovableClasses,1);
nNumClasses           = nNumStationaryClasses + nNumMovableClasses;

% Read classes config from EditorConfig.xml
clClassPoints      = cell(nNumClasses, 5);      % cell array containing name of class, points, number of points, ID, color
vnLabelIDSet       = zeros(1, nNumClasses);
vnIndicesSet       = zeros(1, nNumClasses);
for i = 1 : nNumStationaryClasses
    clClassPoints{i, 1} = oEditorConfig.m_voStationaryClasses(i,1).Name;
    clClassPoints{i, 2} = zeros(nNumPoints,3);
    clClassPoints{i, 3} = 0;
    clClassPoints{i, 4} = oEditorConfig.m_voStationaryClasses(i,1).LabelID;
    clClassPoints{i, 5} = oEditorConfig.m_voStationaryClasses(i,1).Color_RGB;
    vnLabelIDSet(1,i)   = oEditorConfig.m_voStationaryClasses(i,1).LabelID;
    vnIndicesSet(1,i)   = i;
end
nOff = i;
for i = 1 : nNumMovableClasses
    clClassPoints{i+nOff, 1} = oEditorConfig.m_voMovableClasses(i,1).Name;
    clClassPoints{i+nOff, 2} = zeros(nNumPoints,3);
    clClassPoints{i+nOff, 3} = 0;
    clClassPoints{i+nOff, 4} = oEditorConfig.m_voMovableClasses(i,1).LabelID;
    clClassPoints{i+nOff, 5} = oEditorConfig.m_voMovableClasses(i,1).Color_RGB;
    vnLabelIDSet(1,i+nOff)   = oEditorConfig.m_voMovableClasses(i,1).LabelID;
    vnIndicesSet(1,i+nOff)   = i+nOff;
end

vnLabelVector = reshape(oLaserscan.m_mnLabelID, [nNumPoints,1]);   % Reshaped LabelID matrix
x = reshape(oLaserscan.m_mfX, [nNumPoints,1]);
y = reshape(oLaserscan.m_mfY, [nNumPoints,1]);
z = reshape(oLaserscan.m_mfZ, [nNumPoints,1]);

% vnCounter = zeros(nNumClasses,1);
% vmPoints  = zeros(nNumClasses, nNumPoints, 3);

vnCounter = zeros(256, 1);              % runtime optimization: Assume not more than 256 classes (LabelID coded by 8 bit). LabelID corresponds to index+1
vmPoints  = zeros(256, nNumPoints, 3);  % E.g. LabelID = 0 (NoLabel) will be sorted at position 1.
vbInit    = zeros(256, 1);              % 1 if class is present in scan

% Create map: LabelID to index in cell array
mapLabelToIndex = containers.Map(vnLabelIDSet, vnIndicesSet);

% Sort points
for i = 1 : nNumPoints
    % nIdx     = mapLabelToIndex(vnLabelVector(i,1));
    nIdx = vnLabelVector(i,1) + 1;  % Shift to labelID to corresponding index
    vnCounter(nIdx,1)  = vnCounter(nIdx,1) + 1;
    vbInit(nIdx,1)     = 1;
    vmPoints(nIdx, vnCounter(nIdx,1), 1) = x(i,1);
    vmPoints(nIdx, vnCounter(nIdx,1), 2) = y(i,1);
    vmPoints(nIdx, vnCounter(nIdx,1), 3) = z(i,1);
end

% Iterate through all possible labelIDs and sort out surplus points
for i = 1 : 256
    nCtr = vnCounter(i,1);          % Get number of points
    if ~vbInit(i)
        continue
    end
    nIdx = mapLabelToIndex(i-1);    % Map to index in cell array. (i-1) is LabelID
    if nCtr > 0
        clClassPoints{nIdx,3} = nCtr;
        clClassPoints{nIdx,2} = [vmPoints(i,1:nCtr,1)', vmPoints(i,1:nCtr,2)', vmPoints(i,1:nCtr,3)'];
    else 
        clClassPoints{nIdx,2} = [];
    end
end

end

