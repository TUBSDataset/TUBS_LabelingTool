function [nCode] = writePCMovableLabels(sBaseDir, nSequenceID, nPCID, voPCMovableLabel)
% ---------------------------------------------------------------------------------------------
% Function writePCMovableLabels writes XML files containing point cloud object labels.
%
% INPUT:
%   sBaseDir:           String containing the base directory as specified within PCEditorToolGUI.m
%   nSequenceID:        Current sequence ID
%   nPCID               Current PCID to be read
%   voPCMovableLabel:   Movable label vector be be written as XML
%
% OUTPUT:
%   nCode:              Error code. 0 = success, 1 = error while writing XML file
% ---------------------------------------------------------------------------------------------
nCode       = 0;
sFileDir    = buildPath(sBaseDir, nSequenceID, nPCID, 11);

% Build tree
voTree.Object = [];
for i = 1 : size(voPCMovableLabel,1)
    % General information
    voTree.Object(i,1).TrackID              = voPCMovableLabel(i,1).m_nTrackID;
    voTree.Object(i,1).isActive             = voPCMovableLabel(i,1).m_bIsActive;
    voTree.Object(i,1).ExistenceLikelihood  = voPCMovableLabel(i,1).m_fExistenceLikelihood;
    voTree.Object(i,1).Classification       = voPCMovableLabel(i,1).m_sClassification;
    voTree.Object(i,1).Timestamp            = voPCMovableLabel(i,1).m_nTimestamp;
    
    % Probability vector (struct array)
    voTree.Object(i,1).ProbabilityVector    = voPCMovableLabel(i,1).m_voProbabilityVector;
    
    % Bounding box variables
    voTree.Object(i,1).BBMiddle_x    = voPCMovableLabel(i,1).m_fBBMiddle_x;
    voTree.Object(i,1).BBMiddle_y    = voPCMovableLabel(i,1).m_fBBMiddle_y;
    voTree.Object(i,1).BBMiddle_z    = voPCMovableLabel(i,1).m_fBBMiddle_z;
    voTree.Object(i,1).BBHeight      = voPCMovableLabel(i,1).m_fBBHeight;
    voTree.Object(i,1).BBWidth       = voPCMovableLabel(i,1).m_fBBWidth;
    voTree.Object(i,1).BBLength      = voPCMovableLabel(i,1).m_fBBLength;
    voTree.Object(i,1).BBYaw         = voPCMovableLabel(i,1).m_fBBYaw;
    
    % Dynamics
    voTree.Object(i,1).VxAbs            = voPCMovableLabel(i,1).m_fVxAbs;
    voTree.Object(i,1).VyAbs            = voPCMovableLabel(i,1).m_fVyAbs;
    voTree.Object(i,1).AxAbs            = voPCMovableLabel(i,1).m_fAxAbs;
    voTree.Object(i,1).AyAbs            = voPCMovableLabel(i,1).m_fAyAbs;
    voTree.Object(i,1).YawRatePerDist   = voPCMovableLabel(i,1).m_fBBYawRatePerDist;
    
    % Variances
    voTree.Object(i,1).VarBBMiddle_x        = voPCMovableLabel(i,1).m_fVarBBMiddle_x;
    voTree.Object(i,1).VarBBMiddle_y        = voPCMovableLabel(i,1).m_fVarBBMiddle_y;
    voTree.Object(i,1).VarVxAbs             = voPCMovableLabel(i,1).m_fVarVxAbs;
    voTree.Object(i,1).VarVyAbs             = voPCMovableLabel(i,1).m_fVarVyAbs;
    voTree.Object(i,1).VarAxAbs             = voPCMovableLabel(i,1).m_fVarAxAbs;
    voTree.Object(i,1).VarAyAbs             = voPCMovableLabel(i,1).m_fVarAyAbs;
    voTree.Object(i,1).VarBBYaw             = voPCMovableLabel(i,1).m_fVarBBYaw;
    voTree.Object(i,1).VarBBYawRatePerDist  = voPCMovableLabel(i,1).m_fVarBBYawRatePerDist;
end

% Write tree using the external xml_write function. See xml_write.m for more information.
try
    xml_write(sFileDir, voTree, 'MovableLabels');
catch
    nCode = 1;
end

end

