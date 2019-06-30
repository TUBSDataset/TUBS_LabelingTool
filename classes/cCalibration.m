classdef cCalibration
    % ---------------------------------------------------------------------------------------------
    % Class cHistory stores camera calibration data.
    % ---------------------------------------------------------------------------------------------
    
    properties
        m_sType   % image type (front, right, rear, left)
        
        m_mfK;    % intrinsic parameters
        m_mfR;    % rotation matrix from velodyne into camera reference frame
        m_vfc;    % camera position in velodyne reference frame
        m_vfd;    % distortion coefficients
    end
    
    methods
    end
    
end

