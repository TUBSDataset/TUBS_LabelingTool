classdef cLaserScan < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cLaserScan contains point information as (64 x 2000) data matrices, corresponding to the 
    % scanner's resolution. The member m_mSegmentID is only available if m_bSegmentsAvailable in cPCMetadata is true.
    % ---------------------------------------------------------------------------------------------
    
    properties
        % Point matrices
        m_mbIsValid      = []; % valid matrix containing information about invalid measurements or disfunctional layers.
        m_mfRange        = []; 
        m_mfIntensity    = []; % intensity of the reflected beam
        m_mfX            = []; % cartesian point coordinates  
        m_mfY            = [];
        m_mfZ            = [];
        m_mfGroundLevelZ = []; % matrices containing ground information (z Coordinate) estimated by our tracking system.
        m_mnSegmentID    = []; % contains segmentID given by the algorithm from our tracking system.         
        % Movable Matrices 
        m_mnLabelID      = []; % matrix containing label as specified within EditorConfig.xml
        m_mnListIndex    = []; % matrix referencing each point to an entry in the corresponding cMovableLabel vector
    end
    
    methods
    end
    
end

