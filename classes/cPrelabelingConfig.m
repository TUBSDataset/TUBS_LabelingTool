classdef cPrelabelingConfig < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cPrelabelingConfig access to PrelabelingConfig.xml file containing information 
    % about last contained trackIDs, number of labeled point clouds etc. 
    % ---------------------------------------------------------------------------------------------
    properties
        m_voRecordings; % vector containing recording information. See PrelabelingConfig.xml for more information.
    end
    methods
        % Function to load and parse PrelabelingConfig.xml using external xml_read function by Jarek Tuszynski. 
        function [obj, nCode] = load(obj, sConfigDir, info_h)
            try
                oTree = xml_read(strcat(sConfigDir, '\PrelabelingConfig.xml'));
                obj.m_voRecordings  = oTree.RecordingInfo.Recording;
                nCode = 0; % success
            catch
                setInfoText(info_h, 'Error: Could not load prelabeling config file!', 1);
                nCode = 1; % error
            end
        end
    end
end

