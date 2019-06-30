classdef cPCMetadata < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cPCMetadata contains meta data information, such as ego movement and timestamps.
    % The class inherits from class Copyable in order to avoid ambiguous object handles. 
    %
    % ---------------------------------------------------------------------------------------------
    properties
        % General information
        m_fFormatVersion     = 0;
        m_nPCID              = 0;
        m_nSequenceID        = 0;
        m_sRecordingName     = '';
        m_bIsFirstOfSequence = 0;
        m_bIsLastOfSequence  = 0;
        m_bSegmentsAvailable = 0;
        
        m_bImagesAvailable_Front    = 0;
        m_bImagesAvailable_Right    = 0;
        m_bImagesAvailable_Rear     = 0;
        m_bImagesAvailable_Left     = 0;
        
        m_nNumberOfLayers           = 0;
        m_nNumberOfChannels         = 0;
        
        % Ego data
        m_fEgoVx    = 0;
        m_fEgoVy    = 0;
        m_fEgoVarVx = 0;
        m_fEgoVarVy = 0;
        
        m_fEgoAx    = 0;
        m_fEgoAy    = 0;
        m_fEgoVarAx = 0;
        m_fEgoVarAy = 0;
        
        m_fEgoYawRate    = 0;
        m_fEgoVarYawRate = 0;
        
        % Timestamp information
        m_nTimestamp_us         = 0;
        m_nFirstTimestamp_us    = 0;
        m_nLastTimestamp_us     = 0;
        
        m_nSuccessor_PCID               = 0;
        m_nSuccessor_Timestamp_us       = 0;
        m_nSuccessor_FirstTimestamp_us  = 0;
        m_nSuccessor_LastTimestamp_us   = 0;
        
        m_nPredecessor_PCID               = 0;
        m_nPredecessor_Timestamp_us       = 0;
        m_nPredecessor_FirstTimestamp_us  = 0;
        m_nPredecessor_LastTimestamp_us   = 0;
    end
    
    methods
        % See i/o functions to read or write the dataformat.
    end
end

