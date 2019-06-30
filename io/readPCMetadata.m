function [oPCMetadata, nCode] = readPCMetadata(sBaseDir, nSequenceID, nPCID)
% ---------------------------------------------------------------------------------------------
% Function readPCMetadata reads XML files containing metadata used e.g. used for prediction.
%
% INPUT:
%   sBaseDir:       String containing the base directory as specified within PCEditorToolGUI.m
%   nSequenceID:    Current sequence ID 
%   nPCID           Current PCID to be read
% OUTPUT:
%   oPCMetadata:    Object of class cPCMetadata containing read data
%   nCode:          Error code. 0 = success, 1 = error while reading XML file  
% --------------------------------------------------------------------------------------------- 

oPCMetadata = cPCMetadata();
nCode       = 0;

% Parse XML file using the external xml_read function. See xml_read.m for more information.
try
    sFilePath = buildPath(sBaseDir, nSequenceID, nPCID, 10);
    oTree = xml_read(sFilePath);
    
    % General information
    oPCMetadata.m_fFormatVersion            = oTree.FormatVersion;
    oPCMetadata.m_nPCID                     = oTree.PCID;
    oPCMetadata.m_nSequenceID               = oTree.SequenceID;
    oPCMetadata.m_sRecordingName            = oTree.RecordingName;
    oPCMetadata.m_bIsFirstOfSequence        = oTree.isFirstOfSequence;
    oPCMetadata.m_bIsLastOfSequence         = oTree.isLastOfSequence;
    oPCMetadata.m_bSegmentsAvailable        = oTree.SegmentsAvailable;
    oPCMetadata.m_nNumberOfLayers           = oTree.NumberOfLayers;
    oPCMetadata.m_nNumberOfChannels         = oTree.NumberOfChannels;
    
    oPCMetadata.m_bImagesAvailable_Front    = oTree.ImagesAvailable_Front;
    oPCMetadata.m_bImagesAvailable_Right    = oTree.ImagesAvailable_Right;
    oPCMetadata.m_bImagesAvailable_Rear     = oTree.ImagesAvailable_Rear;
    oPCMetadata.m_bImagesAvailable_Left     = oTree.ImagesAvailable_Left;
    
    % Ego data
    oPCMetadata.m_fEgoVx        = oTree.EgoVx;
    oPCMetadata.m_fEgoVy        = oTree.EgoVy;
    oPCMetadata.m_fEgoVarVx     = oTree.EgoVarVx;
    oPCMetadata.m_fEgoVarVy     = oTree.EgoVarVy;
    
    oPCMetadata.m_fEgoAx        = oTree.EgoAx;
    oPCMetadata.m_fEgoAy        = oTree.EgoAy;
    oPCMetadata.m_fEgoVarAx     = oTree.EgoVarAx;
    oPCMetadata.m_fEgoVarAy     = oTree.EgoVarAy;
    
    oPCMetadata.m_fEgoYawRate    = oTree.EgoYawRate;
    oPCMetadata.m_fEgoVarYawRate = oTree.EgoVarYawRate;
    
    % Timestamps
    oPCMetadata.m_nTimestamp_us         = oTree.Timestamp_us;
    oPCMetadata.m_nFirstTimestamp_us    = oTree.FirstTimestamp_us;
    oPCMetadata.m_nLastTimestamp_us     = oTree.LastTimestamp_us;
    
    oPCMetadata.m_nSuccessor_PCID               = oTree.Successor_PCID;
    oPCMetadata.m_nSuccessor_Timestamp_us       = oTree.Successor_Timestamp_us;
    oPCMetadata.m_nSuccessor_FirstTimestamp_us  = oTree.Successor_FirstTimestamp_us;
    oPCMetadata.m_nSuccessor_LastTimestamp_us   = oTree.Successor_LastTimestamp_us;
    
    oPCMetadata.m_nPredecessor_PCID               = oTree.Predecessor_PCID;
    oPCMetadata.m_nPredecessor_Timestamp_us       = oTree.Predecessor_Timestamp_us;
    oPCMetadata.m_nPredecessor_FirstTimestamp_us  = oTree.Predecessor_FirstTimestamp_us;
    oPCMetadata.m_nPredecessor_LastTimestamp_us   = oTree.Predecessor_LastTimestamp_us;
catch
    nCode = 1;
end
end

