function [nCode] = writePCMovableMatrices(sBaseDir, nSequenceID, nPCID, oLaserScan)
% ---------------------------------------------------------------------------------------------
% Function writePCMovableMatrices writes data matrices related to point cloud object labels 
% (m_mLabelID and m_mListIndex in cLaserScan) to a binary file.
%
% INPUT:
%   sBaseDir:               String containing the base directory as specified within PCEditorToolGUI.m
%   nSequenceID:            Current sequence ID 
%   nPCID:                  Current PCID to be read
%   oLaserScan:             Object of cLaserScan whose movable matrices needs to be written
% OUTPUT:
%   nCode:                  Error code. 0: Success, 1: Could not write file
% ---------------------------------------------------------------------------------------------

nCode    = 0;
sFileDir = buildPath(sBaseDir, nSequenceID, nPCID, 13);
nFileID  = fopen(sFileDir, 'w+'); 

if nFileID == -1
    nCode = 1;
    return;
end

fwrite(nFileID, oLaserScan.m_mnLabelID,      'ubit8');
fwrite(nFileID, oLaserScan.m_mnListIndex,   'ubit8');
fclose(nFileID);

end

