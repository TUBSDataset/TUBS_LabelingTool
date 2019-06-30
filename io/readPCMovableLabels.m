function [voPCMovableLabel, nCode] = readPCMovableLabels(sBaseDir, nSequenceID, nPCID, bLoadEdited)
% ---------------------------------------------------------------------------------------------
% Function readPCMovableLabels reads XML files containing object labels such as their dynamics.
%
% INPUT:
%   sBaseDir:           String containing the base directory as specified within PCEditorToolGUI.m
%   nSequuenceID:       Current sequence ID
%   nPCID               Current PCID to be read
%
% OUTPUT:
%   oPCMovableLabel:    Object of class cPCMetadata containing read data
%   nCode:              Error code. 0 = success, 1 = error while reading XML file, 2 = no files available
% --------------------------------------------------------------------------------------------

nCode = 0;

if bLoadEdited
    nIdentifier = 11;
else
    nIdentifier = 12;
end

sFilePath = buildPath(sBaseDir, nSequenceID, nPCID, nIdentifier);
if checkForEmptyDirectory(sFilePath)
    nCode = 2;
    return;
end
try
    % Parse XML file using the external xml_read function. See xml_read.m for more information.
    voTree = xml_read(sFilePath);
catch
    nCode = 1;
    return;
end

try
    % No objects to parse
    nNumberOfTracks = size(voTree.Object,1);
catch
    nNumberOfTracks = 0;
end
voPCMovableLabel = cPCMovableLabel.empty;
if nNumberOfTracks > 0
    for i = 1 : nNumberOfTracks
        voPCMovableLabel(i,1) = cPCMovableLabel();
    end
else
    voPCMovableLabel = cPCMovableLabel.empty;
    return
end

% Parse trees
for i = 1 : nNumberOfTracks
    voPCMovableLabel(i,1).m_nTrackID              = voTree.Object(i,1).TrackID;
    voPCMovableLabel(i,1).m_bIsActive             = voTree.Object(i,1).isActive;
    voPCMovableLabel(i,1).m_fExistenceLikelihood  = voTree.Object(i,1).ExistenceLikelihood;
    voPCMovableLabel(i,1).m_sClassification       = voTree.Object(i,1).Classification;
    voPCMovableLabel(i,1).m_nTimestamp            = voTree.Object(i,1).Timestamp;
    
    % Probability vector (struct array)
    voPCMovableLabel(i,1).m_voProbabilityVector   = voTree.Object(i,1).ProbabilityVector;
    
    % Bounding box variables
    voPCMovableLabel(i,1).m_fBBMiddle_x           = voTree.Object(i,1).BBMiddle_x;
    voPCMovableLabel(i,1).m_fBBMiddle_y           = voTree.Object(i,1).BBMiddle_y;
    voPCMovableLabel(i,1).m_fBBMiddle_z           = voTree.Object(i,1).BBMiddle_z;
    voPCMovableLabel(i,1).m_fBBHeight             = voTree.Object(i,1).BBHeight;
    voPCMovableLabel(i,1).m_fBBWidth              = voTree.Object(i,1).BBWidth;
    voPCMovableLabel(i,1).m_fBBLength             = voTree.Object(i,1).BBLength;
    voPCMovableLabel(i,1).m_fBBYaw                = voTree.Object(i,1).BBYaw;
    
    % Dynamics
    voPCMovableLabel(i,1).m_fVxAbs                  = voTree.Object(i,1).VxAbs;
    voPCMovableLabel(i,1).m_fVyAbs                  = voTree.Object(i,1).VyAbs;
    voPCMovableLabel(i,1).m_fAxAbs                  = voTree.Object(i,1).AxAbs;
    voPCMovableLabel(i,1).m_fAyAbs                  = voTree.Object(i,1).AyAbs;
    voPCMovableLabel(i,1).m_fBBYawRatePerDist       = voTree.Object(i,1).YawRatePerDist;
    
    % Variances
    voPCMovableLabel(i,1).m_fVarBBMiddle_x              = voTree.Object(i,1).VarBBMiddle_x;
    voPCMovableLabel(i,1).m_fVarBBMiddle_y              = voTree.Object(i,1).VarBBMiddle_y;
    voPCMovableLabel(i,1).m_fVarVxAbs                   = voTree.Object(i,1).VarVxAbs;
    voPCMovableLabel(i,1).m_fVarVyAbs                   = voTree.Object(i,1).VarVyAbs;
    voPCMovableLabel(i,1).m_fVarAxAbs                   = voTree.Object(i,1).VarAxAbs;
    voPCMovableLabel(i,1).m_fVarAyAbs                   = voTree.Object(i,1).VarAyAbs;
    voPCMovableLabel(i,1).m_fVarBBYaw                   = voTree.Object(i,1).VarBBYaw;
    voPCMovableLabel(i,1).m_fVarBBYawRatePerDist        = voTree.Object(i,1).VarBBYawRatePerDist;
end

end

