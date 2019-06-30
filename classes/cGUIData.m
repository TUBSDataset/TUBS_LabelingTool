classdef cGUIData < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cGUIData contain sall settings, variables and other global data.
    % Most functions that manipulate oGUIData will return the changed object although changes will affect the actual
    % object immediately. If you choose to manipulate oGUIData with additional functions, please follow this scheme
    % for a more lucid code. 
    % ---------------------------------------------------------------------------------------------
    properties
        % Config files. See cEditorConfig and cPrelabelingConfig for more information.
        m_oPrelabelingConfig;
        m_oEditorConfig;
        m_voCalibration;            % Vector containing the camera calibration parameters (front, right, rear, left)

        % GUI variables
        m_oLaserScan;               % Object of class cLaserScan containing the current scan
        m_oPCMetadata;              % Object of class cPCMetadata
        m_bInit;                    % True if GUI is initialized with objects (see loadToGui(...))
        m_sRootDir;                 % String containing the root directory (config files and recordings)
        m_sRecording;               % String containing the recording name
        m_sDatasetDir;              % String specifying the current dataset's (recording's) directory
        m_nSequenceID;              % Current sequenceID
        m_nPCID;                    % Current PCID
        m_sView;                    % view (front, back...)
        m_bShowGround;              % Show ground points
        m_bGUIZoomed;               % GUI is zoomed (definition state or using mouse wheel)
        m_nNumPoints;               % Number of points contained in a laser scan
        m_bConfigFilesLoaded;       % True if config files were successfully loaded
        m_MapClassToID;             % Maps a class name to according ID as defined in editor config file
        m_nConsecutiveFrames;       % Counter for consecutive edited frames (for global correction intervall)
        m_nGlobalCorrIntervall;     % Correct all objects despite the editor's self assessement every n frames
        
        % Definition state machines
        m_sDefinitionMode;          % 'new': new box mode, 'correction': correction mode, 'pick': pick box mode, 'none'
        m_sDefinitionState;         % 'pos', 'edge', 'length', 'width', 'yaw', cam', 'none', 'zoomToPoint'
        m_bInDefinitionState;       % True if editor is in box definition state (all but edge or zoomToPoint)
        
        % GUI object definiton
        m_voEdgePoints_h;           % Vector of edge point graphics (for box definition)
        m_mfEdgePoints;             % Vector of according edge coordinates
        m_nNumEdgePoints;           % Number of edge points set
        m_bConfirmBoxDeletion;      % Bit used to wait for confirmatin of a box deletion
        
        m_nNumPCObjects;            % Number of objects in current PC
        m_nIndexCurrentObject;      % Index of selected, current object
        m_vnDeletedTrackIDs;        % Vector of deleted trackIDs (blacklist to suppress those IDs during the import)
        
        % Image settings
        m_sProjectionMode;          % 3D or polygon projection of point cloud objects
        m_sProjectionMode_prev;     % Previous projection mode
        m_sImageContext;            % Choose between ray projection or object definition
        m_bUndistortImages;         % True if images ought to be undistorted
        m_bRandomColor;             % False: Color of shape corresponds to defined class color (EditorConfig.xml)
        m_bLoadImageLabels;         % True: Load image labels from prelabling (such as licence plates or faces)
        m_bDisplayPD;               % Display personal data labels from prelabeling (image processing toolbox required)

        % Modes
        m_bEnableImageLabeling;     % True if image labeing is active (load and save image projection, enable image segmentation etc.)
        m_bCameraCalibrationMode;   % True if camera calibration mode is active (record and save point a point container)
        
        % Camera calibration
        m_clPointContainer = {};    % Container holding 3D-2D point correspondences for camera calibration
        m_nObj = 0;                 % Counter of defined objects
        m_nNumTargets = 0;          % Number of calibration targets
       
        % Gereral settings
        m_fZoomFactor;              % Factor that the point cloud is zoomed (e.g. definition state)
        m_fDeltaLW;                 % Length discretization
        m_fDeltaYaw;                % Yaw discretization
        m_fDeltaXY;                 % Position discretization
        
        m_nNumICPIterations;        % Maximum number of ICP iterations 
        m_nSizeICPFifo;             % Size of point accumulator (ICP and optimization algorithms)
        m_bShowAccu;                % Display point accumulator
        m_fRelabelMargin;           % Margin around bounding boxes within to include points to box
        m_nDeleteDelay;             % Number of scans that a box has to be empty to be deleted
        m_nPointsDeleteThreshold;   % Threshold below which boxes will be assessed as empty
        m_fMaxRange;                % Range threshold for auto deletion
        m_nMaxNumInHistory;         % Maximum number of history entries
        
        % Kalman settings
        m_bEnablePrelabeling;       % Use prelabeling object data for measurement update
        m_bEnableCorrections;       % Use manual corrections for measurement update
        m_bEnableOptimization;      % Use ICP and optimization algorithms for measurement update
        
        % Optimization settings
        m_bEnableRegistration;          % Enable ICP algorithm
        m_bEnableFitting;               % Enable fitting algorithms (rectangle, ellipse, edges)
        m_bEnableEdgesEstimation;       % Enable edges estimation (part of fitting algorithms)
        
        % Menu settings
        m_bEnableDebugInformation;      % Display debuf information in info panel
        m_bInitFromEditedPC;            % Try to initialize the GUI and editing history from an edited point clouds
        m_bAutoImport;                  % Automatically import missing objects from prelabeling into GUI
        m_bAutoDelete;                  % Enable automatic deletion of objects (exceeding range or lacking points)
        m_bEnableOptimizationPlots;     % Display debug plots for optimization algorithms
        m_nPosInHistory;                % Current position is history
        m_nNumInHistory;                % Number of edited PCs in history
        m_bPredict;                     % Enables the prediction function. Elsewise prelabeled objects will be loaded
        m_bDisplayObjectData;           % Display an object's dynamics and variances
        m_bLogWarnings;                 % Output warnings (in addition to more general debug info)

        % Camera handling
        m_vfCameraTarget;
        m_bEnableCameraMov;
        m_bEnableCameraRot;
        m_bCameraTargetSet;
        m_vfPrevPoint;
        m_bLeftMouseDown;
        m_bRightMouseDown;
        m_fCameraViewAngleUnzoomed;
        m_fCameraViewAngleZoomed;

        % Graphic handles to various GUI graphic objects
        m_oPCAxes_h;                % Handle to main point cloud axes
        m_oPC_h;                    % Handle to the point cloud graphic object
        m_oInfo_h;                  % Handle to info box
        m_oObjectListTable_h;       % Handle to the object list
        m_oSavedPanel_h;            % Handle to the saved panel within the main PC axes
        m_oSavedText_h;             % Displayed text on the saved panel
        m_oImagePanel_h;            % Handle to the separate image figure
        m_oShowFrontButton_h;       % Handle to front view button
        m_oShowBackButton_h;        % Handle to rear view button
        m_oGround_h;                % Handle to ground graphic object
        m_oGUI_h;                   % Handle to main GUI figure
        m_oMetadataTable_h;         % Handle to the PC metadata table
        m_hImageCallback;           % function handle to image callback
        m_voImageAxes_h;            % Vector of image axes handles for image panel. Order: [front, right, rear, left]
        m_hImageMenuCallback        % function handle to image context menu callback
        m_oImages_h;                % Handle to image figure
        m_oClassesButtonGroup_h;    % Handle to classes button group
    end
    
    methods
       % This function returns the calibration object of the desired image type
       function [oCalib] = getCalibration(obj, sImageType)
           oCalib = [];
           for i = 1 : 4
               if strcmp(sImageType, obj.m_voCalibration(i,1).m_sType)
                   oCalib = obj.m_voCalibration(i,1);
                   break;
               end
           end
       end
    end
end

