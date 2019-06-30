classdef cEditorConfig < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cEditorConfig access to EditorConfig.xml file containing information to resume editing 
    % or to provide unique track IDs within a sensor recording. 
    % ---------------------------------------------------------------------------------------------
    properties
        m_LastWrittenTrackID    = 0;
        m_LastEditedSequenceID  = -1; % unset state
        m_LastEditedPCID        = -1;
        m_sRecording            = '';
        
        m_voMovableClasses;         % vector of movable classes containing name, ID and color information
        m_voStationaryClasses;      % vector of stationary classes containing name, ID and color information
        m_voRecordings;             % vector of recording information (lastWrittenTrackID, lastEditedSequenceID, lastEditedPCID)
        m_oTree;                    % tree of structs used for XML i/o
    end
    methods
        % Function to load and parse XML file using external xml_read function by Jarek Tuszynski.
        function [obj, nCode] = load(obj, sConfigDir, oInfo_h)
            bDisplayInfo = 1;
            if (nargin < 3)     % No handle to info box handed
                bDisplayInfo = 0;
            end
            try
                oTree = xml_read(strcat(sConfigDir, '\EditorConfig.xml'));
                obj.m_oTree = oTree;
                obj.m_voStationaryClasses  = oTree.ClassesVectors.StationaryClasses.Class;
                obj.m_voMovableClasses     = oTree.ClassesVectors.MovableClasses.Class;
                % Normalize color
                for i = 1 : size(obj.m_voStationaryClasses,1)
                    obj.m_voStationaryClasses(i,1).Color_RGB = obj.m_voStationaryClasses(i,1).Color_RGB ./ 255;
                end
                for i = 1 : size(obj.m_voMovableClasses,1)
                    obj.m_voMovableClasses(i,1).Color_RGB = obj.m_voMovableClasses(i,1).Color_RGB ./ 255;
                end
                if ~isempty(oTree.RecordingInfo)
                    obj.m_voRecordings  = oTree.RecordingInfo.Recording;
                end
                if bDisplayInfo
                    setInfoText(oInfo_h, 'Editor config: Successfully loaded.', 1);
                end
                nCode = 0; % success
            catch
                if bDisplayInfo
                    setInfoText(oInfo_h, 'Error: Could not load editor config file!', 1);
                end
                nCode = 1; % error
            end
        end
        
        % Function to save XML tree
        function obj = save(obj, sConfigDir, oInfo_h)
            try
                % Update tree
                for i = 1 : size(obj.m_oTree.RecordingInfo.Recording,1)
                    if strcmp(obj.m_oTree.RecordingInfo.Recording(i,1).Name, obj.m_sRecording)
                        obj.m_oTree.RecordingInfo.Recording(i,1).LastWrittenTrackID   = obj.m_LastWrittenTrackID;
                        obj.m_oTree.RecordingInfo.Recording(i,1).LastEditedSequenceID = obj.m_LastEditedSequenceID;
                        obj.m_oTree.RecordingInfo.Recording(i,1).LastEditedPCID       = obj.m_LastEditedPCID;
                        break;
                    end
                end
                xml_write(strcat(sConfigDir, '\EditorConfig.xml'), obj.m_oTree, 'Config');
            catch
                setInfoText(oInfo_h, 'Error: Could not write editor config file!', 1);
            end
        end
        
        % Function to check if an entry exists in EditorConfig.xml. 
        function obj = compare(obj, oPrelabelingConf, sRecordingName, info_h)
            % Search for recording name in editor config
            bCreateEntry = 0;
            bFound = 0;
            if ~isempty(obj.m_voRecordings)
                for i = 1 : size(obj.m_voRecordings, 1)
                    if strcmp(sRecordingName, obj.m_voRecordings(i,1).Name)
                        obj.m_LastWrittenTrackID    = obj.m_voRecordings(i,1).LastWrittenTrackID;
                        obj.m_LastEditedSequenceID  = obj.m_voRecordings(i,1).LastEditedSequenceID;
                        obj.m_LastEditedPCID        = obj.m_voRecordings(i,1).LastEditedPCID;
                        bFound = 1;
                        break;
                    end
                end
            end
            
            if ~bFound
                setInfoText(info_h, 'Editor config: Recording entry does not exist.', 1);
                setInfoText(info_h, 'Editor config: Creating from prelabeling config file.', 1);
                bCreateEntry = 1;
            end
            
            % Take entry from prelabelingConfig if not existing
            bFound = 0;
            if bCreateEntry
                for i = 1 : size(oPrelabelingConf.m_voRecordings, 1)
                    if strcmp(sRecordingName, oPrelabelingConf.m_voRecordings(i,1).Name)
                        obj.m_LastWrittenTrackID = oPrelabelingConf.m_voRecordings(i,1).LastTrackID;
                        obj.m_LastEditedSequenceID = -1;
                        obj.m_LastEditedPCID = -1;
                        % Create tree entry
                        oEntry = struct('Name', '', 'LastWrittenTrackID', 0, 'LastEditedSequenceID', 0, 'LastEditedPCID', 0);
                        oEntry.Name = sRecordingName;
                        oEntry.LastWrittenTrackID = oPrelabelingConf.m_voRecordings(i,1).LastTrackID;
                        if ~isempty(obj.m_oTree.RecordingInfo)
                            obj.m_oTree.RecordingInfo.Recording(end+1,1) = oEntry;
                        else
                            obj.m_oTree.RecordingInfo.Recording = oEntry;
                        end
                        
                        bFound = 1;
                        break;
                    end
                end
                if ~bFound
                    setInfoText(info_h, 'Warning: No corresponding entry found. TrackIDs will not be unique.', 1);
                end
            end
        end
        
        % Generate class to ID mapping
        function mapClassToID = getLabelMap(obj)
            nNumMovableClasses      = size(obj.m_voMovableClasses, 1);
            nNumStationaryClasses   = size(obj.m_voStationaryClasses, 1);
            clClasses   = cell(1, (nNumMovableClasses + nNumStationaryClasses));
            vnLabelIDs  = zeros(1, (nNumMovableClasses + nNumStationaryClasses));
            nCtr        = 0;
            for i = 1 : nNumMovableClasses
                nCtr            = nCtr + 1;
                clClasses{1,nCtr}  = obj.m_voMovableClasses(i,1).Name;
                vnLabelIDs(1,nCtr) = obj.m_voMovableClasses(i,1).LabelID;
            end
            for i = 1 : nNumStationaryClasses
                nCtr            = nCtr + 1;
                clClasses{1,nCtr}  = obj.m_voStationaryClasses(i,1).Name;
                vnLabelIDs(1,nCtr) = obj.m_voStationaryClasses(i,1).LabelID;
            end
            mapClassToID = containers.Map(clClasses, vnLabelIDs);
        end
        
        % Get all assignable classes
        function voClasses = getAssignableClasses(obj)
            % Prase stationary classes
            nCtr = 0;
            voClasses(256,1) = struct('Name', '', 'LabelID', 0,  'Color_RGB', [0 0 0]);    % create empty class struct vector
            for i = 1 : size(obj.m_voStationaryClasses, 1)
                % Skip nonassignable classes
                if          strcmp(obj.m_voStationaryClasses(i,1).Name, 'NoLabel') ...
                        ||  strcmp(obj.m_voStationaryClasses(i,1).Name, 'Stationary') ...
                        ||  strcmp(obj.m_voStationaryClasses(i,1).Name, 'Ground') ...
                        ||  strcmp(obj.m_voStationaryClasses(i,1).Name, 'Undefined')
                    continue
                else
                    nCtr = nCtr + 1;
                    voClasses(nCtr, 1) = obj.m_voStationaryClasses(i,1);
                end
            end
            voClasses = voClasses(1:nCtr, :);
            voClasses = [voClasses; obj.m_voMovableClasses];
        end
        
        function oClass = getClassUndefined(obj)
            oClass = [];
            for i = 1 : size(obj.m_voStationaryClasses, 1)
                if strcmp(obj.m_voStationaryClasses(i,1).Name, 'Undefined')
                    oClass = obj.m_voStationaryClasses(i,1);
                    return
                end
            end
        end
    end
end

