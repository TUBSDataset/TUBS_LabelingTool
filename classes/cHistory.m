classdef cHistory
    % ---------------------------------------------------------------------------------------------
    % Class cHistory stores point cloud and object information within the editing process.
    % ---------------------------------------------------------------------------------------------
    
    properties
        m_oGUIObjects;  % Object of class cGUIObjects containing point cloud and image labels
        m_oPCMetadata;  % Object of class cPCMetadata 
        m_bIsSaved = 0; % Saved flag
    end
    
    methods 
    end
end

