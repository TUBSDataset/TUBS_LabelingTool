function [oLaserScan, nCode] = readPCDataMatrices(sBaseDir, nSequenceID, nPCID, bLoadEdited_in)
% ---------------------------------------------------------------------------------------------
% Function readPCDataMatrices reads binary files containing the scanner's measurement data in the directory PCDataMatrices.
% Additionally, this function reads binary files in the directory PCMovableMatrices_Prelabeled, which contain a matrix with label IDs
% and references to the cPCMovableLabel object list.
%
% INPUT:
%   sBaseDir:               String containing the base directory as specified within PCEditorToolGUI.m
%   nSequenceID:            Current sequence ID 
%   nPCID                   Current PCID to be read
%   oPCMetadata:            Metadata object, containing additional information how to read the matrices
%   bLoadEdited (optional): True if label ID matrix must to be loaded from edited directory
% OUTPUT:
%   nCode:                  Error code. 0: Success, 1: Could not open file
%   oLaserScan:             Object of class cLaserScan containing all point matrices
% ---------------------------------------------------------------------------------------------
bLoadEdited = 0;
if nargin > 3
    bLoadEdited = bLoadEdited_in;
end
oLaserScan = cLaserScan();
nCode      = 0;

% Read point matrices
sFileDir = buildPath(sBaseDir, nSequenceID, nPCID, 9);
nFileID  = fopen(sFileDir);
if nFileID == -1
   nCode = 1; 
   return
end

nFactor             = 100; % Used to decrease dataset size
nNumberOfLayers     = 64;
nNumberOfChannels   = 2000;

oLaserScan.m_mbIsValid   = fread(nFileID, [nNumberOfLayers, nNumberOfChannels], 'ubit8');
oLaserScan.m_mfRange     = fread(nFileID, [nNumberOfLayers, nNumberOfChannels], 'int16');
oLaserScan.m_mfRange     = double(oLaserScan.m_mfRange) ./ nFactor;
oLaserScan.m_mfIntensity = fread(nFileID, [nNumberOfLayers, nNumberOfChannels], 'int16');
oLaserScan.m_mfIntensity = double(oLaserScan.m_mfIntensity) ./ nFactor;
oLaserScan.m_mfX         = fread(nFileID, [nNumberOfLayers, nNumberOfChannels], 'int16');
oLaserScan.m_mfX         = double(oLaserScan.m_mfX) ./ nFactor;
oLaserScan.m_mfY         = fread(nFileID, [nNumberOfLayers, nNumberOfChannels], 'int16');
oLaserScan.m_mfY         = double(oLaserScan.m_mfY) ./ nFactor;
oLaserScan.m_mfZ         = fread(nFileID, [nNumberOfLayers, nNumberOfChannels], 'int16');
oLaserScan.m_mfZ         = double(oLaserScan.m_mfZ) ./ nFactor;
oLaserScan.m_mfGroundLevelZ         = fread(nFileID, [nNumberOfLayers, nNumberOfChannels], 'int16');
oLaserScan.m_mfGroundLevelZ         = double(oLaserScan.m_mfGroundLevelZ) ./ nFactor;

fclose(nFileID);

% Read movable matrices
if bLoadEdited
    sFileDir = buildPath(sBaseDir, nSequenceID, nPCID, 13);
else
    sFileDir = buildPath(sBaseDir, nSequenceID, nPCID, 14);
end

nFileID  = fopen(sFileDir);
if nFileID == -1
   nCode = 1; 
   return
end

oLaserScan.m_mnLabelID     = fread(nFileID, [nNumberOfLayers, nNumberOfChannels], 'ubit8');
oLaserScan.m_mnListIndex   = fread(nFileID, [nNumberOfLayers, nNumberOfChannels], 'ubit8');
fclose(nFileID);

end

