function PCEditorToolGUI()
clc; close all;
addpath('io');          addpath('classes');         addpath('graphics');  addpath('fitting');
addpath('ext');         addpath('images');          addpath('gui');       addpath('kalman');
addpath('calibration');
warning('off', 'images:initSize:adjustingMag');
%% Default settings

sStdRootDir      = 'C:\Datasets\TUBS';          % directory for TUBS recordings
sStdRecording    = 'City Ring - Mid Day';       % name of sensor data recording

sStdDatasetDir   = strcat(sStdRootDir, '\', sStdRecording);
nStdSequenceID   = 1;    % will be set automatically
nStdPCID         = 1;

% Kalman Filter measurement update settings
bEnablePrelabelingUpdates   = 0;    % enable updates from prelabeled objects
bEnableOptimizationUpdates  = 1;    % enable updates from optimization algorithms
bEnableCorrectionUpdates    = 1;    % enable updates from manual corrections

% General settings
bPredictionActive   = 'on';     % activate prediction core. Elsewise: Only prelabeled object lists will be loaded
bAutoImport         = 'on';     % import new objects from the prelabeling stage automatically
bAutoDelete         = 'on';     % delete objects automatically according to configured deletion policy (see below)
bInitFromEdited     = 'on';     % init GUI from previously edited frames
bDebugInfo          = 'off';    % display debug information
bDisplayObjectData  = 'off';    % display the object's state vectors and label beside a box in focus
bLogWarnings        = 1;        % enforce the display of warnings
nGlobalCorrInterv   = 3;        % correct all objects despite the editor's self-assessement every n frames
bShowGround         = 0;        % default ground display setting
nDeleteDelay        = 3;        % number of frames to wait before deletion of an empty box (auto deletion active)
fMaxRange           = 100;      % exceeding this range will automatically delete an object (auto deletion active)

% Image labeling settings
bUndistortImages    = 0;        % true if images ought to be displayed undistorted. 
                                % Note that projection is computed for distorted images.
bRandomColor        = 1;        % false: Color of image shapes corresponds to defined class color (EditorConfig.xml)
bLoadImageLabels    = 1;        % true: Load saved image labels
bDisplayPD          = 0;        % true: Display personal data image labels from prelabling (faces, licence plates)
                                % requires the Image Processing Toolbox
                                
% Optimization settings
bEnableICP          = 'on';     % enable point cloud registration after each prediction step
bEnableFitting      = 'on';     % enable fitting algorithms (rectangle, ellipse)
bEnableEdges        = 'on';     % enable an additional edge estimation algorithm as part of the optimization procedure
bEnableDebugPlots   = 'off';    % enable debug plots for optimization algorithms

% Mode options
bEnableImageLabeling   = 'on';   % enable image labeling, e.g. projection of objects into image planes
bCameraCalibrationMode = 'off';  % use image planes to define and save point correspondences for the calibration script

% Calibration settings
nNumTargets = 4;                 % number of calibrations targets (see \documentation for more information)

%% GUI

fHeightGui = 0.027; fWidthGui = 0.003;
oGUI_h = figure('Visible', 'off', 'units', 'normalized', 'outerposition', [0-fWidthGui fHeightGui 1+2*fWidthGui 1-fHeightGui], ...
    'name', 'PCEditorTool', 'WindowScrollWheelFcn', @doScroll, 'WindowKeyPressFcn', @keyboard_Callback, 'WindowButtonDownFcn', ...
    @windowButtonDown_Callback, 'WindowButtonUpFcn', @windowButtonUp_Callback, 'WindowButtonMotionFcn', @windowButtonMotion_Callback);

oEditorPanel_h = uipanel(oGUI_h, 'units', 'normalized', 'Position', [0 0 .2 1]);
oPCPanel_h     = uipanel(oGUI_h, 'units', 'normalized', 'Position', [.2 0 .8 1]);
oSavedPanel_h  = uipanel(oGUI_h, 'units', 'normalized', 'Position', [0.2 0.978 .03 0.02], 'BackgroundColor', [.7 .7 .7]);
oSavedText_h   = uicontrol(oSavedPanel_h, 'Style', 'text', 'String', 'init', 'HorizontalAlignment', 'center', 'units', ...
    'normalized', 'Position', [0 0 1 .9], 'BackgroundColor', [.7 .7 .7], 'FontWeight', 'bold');
oPCAxes_h      = axes('Parent', oPCPanel_h, 'ButtonDownFcn', @pcAxes_Callback); setAxesSettings(oPCAxes_h, 'center'), ...
    grid on; zoom(oPCAxes_h, 'reset');

fHeigthDataPanel = .2; fHeigthBoxPanel = .6; fHeightInfoPanel = (1-fHeigthDataPanel-fHeigthBoxPanel);
oDataPanel_h     = uipanel(oEditorPanel_h, 'units', 'normalized', 'Position', [0 1-fHeigthDataPanel 1 fHeigthDataPanel]);
oObjectPanel_h      = uipanel(oEditorPanel_h, 'units', 'normalized', 'Position', [0 1-fHeigthDataPanel-fHeigthBoxPanel 1 fHeigthBoxPanel]);
oInfoPanel_h     = uipanel(oEditorPanel_h, 'units', 'normalized', ...
    'Position', [0 1-fHeigthDataPanel-fHeigthBoxPanel-fHeightInfoPanel 1 fHeightInfoPanel]);

oImages_h       = figure('Visible', 'off', 'units', 'normalized', 'outerposition', [0 0 1 1], ...
    'name', 'Image Panel', 'WindowButtonMotionFcn', @imagePanelMouseMove_Callback, 'WindowButtonUpFcn', @imagePanelMouseRelease_Callback);
oImagePanel_h   = uipanel(oImages_h, 'units', 'normalized', 'Position', [0 0 1 1], 'BackgroundColor', [1 1 1]);

oVehicleAxes_h = axes('Visible', 'on', 'Parent', oImagePanel_h, 'Visible', 'on', 'XTick', [], 'YTick', [], ...
    'units','normalized', 'Position', [.4 .4 .2 .2]);
mfVehicle = imread('Vehicle.png');
imshow(mfVehicle, 'Parent', oVehicleAxes_h);
%% Data panel: Directory selections

uicontrol(oDataPanel_h, 'Style', 'text', 'String', 'Prelabeled data directory:', 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [0 .9 1 .1]);
oRootDir_h =    uicontrol(oDataPanel_h, 'Style', 'edit', 'String', sStdRootDir, 'HorizontalAlignment', 'left', 'units', ...
    'normalized', 'Position', [0 .82 0.9 .1],     'Callback', @rootDir_Callback);
uicontrol(oDataPanel_h, 'Style', 'pushbutton', 'String', '...', 'HorizontalAlignment', 'left', 'units', ...
    'normalized', 'Position', [0.92 .82 0.08 .1], 'Callback', @rootDirSelect_Callback);

uicontrol(oDataPanel_h, 'Style', 'text', 'String', 'Recording:', 'FontWeight', 'bold', 'HorizontalAlignment', ...
    'left', 'units', 'normalized', 'Position', [0 .7 1 .1]);
oRecording_h =  uicontrol(oDataPanel_h, 'Style', 'edit', 'String', sStdRecording, 'HorizontalAlignment', ...
    'left', 'units', 'normalized', 'Position', [0 .62 0.9 .1], 'Callback', @recordingDir_Callback);
uicontrol(oDataPanel_h, 'Style', 'pushbutton', 'String', '...', 'HorizontalAlignment', ...
    'left', 'units', 'normalized', 'Position', [0.92 .62 0.08 .1], 'Callback', @recordingDirSelect_Callback);

uicontrol(oDataPanel_h, 'Style', 'text', 'String', 'Sequence ID:', 'HorizontalAlignment', ...
    'left', 'units', 'normalized',         'Position', [0 .46 .3 .1]);
oSequenceID_h = uicontrol(oDataPanel_h, 'Style', 'edit', 'String', num2str(nStdSequenceID), 'HorizontalAlignment', ...
    'left', 'units', 'normalized',  'Position', [.185 .48 .11 .1], 'Callback', @sequenceDir_Callback);
uicontrol(oDataPanel_h, 'Style', 'text', 'String', 'PC ID in Seq:', 'HorizontalAlignment', ...
    'left', 'units', 'normalized',        'Position', [.35 .46 .3 .1]);
oPCID_h       = uicontrol(oDataPanel_h, 'Style', 'edit', 'String', num2str(nStdPCID), 'HorizontalAlignment', ...
    'left', 'units', 'normalized',        'Position', [0.545 .48 .1 .1], 'Callback', @pcDir_Callback);
uicontrol(oDataPanel_h, 'Style', 'pushbutton', 'String', 'Load', 'units', 'normalized', ...
    'Position', [.7 .48 .3 .1], 'Callback', @loadButton_Callback);

%% Data panel: Load buttons

uicontrol(oDataPanel_h, 'Style', 'pushbutton', 'String', 'Load next PC',     'units', 'normalized', ...
    'Position', [0 .36 .3 .1],    'Callback', @loadNextButton_Callback);
uicontrol(oDataPanel_h, 'Style', 'pushbutton', 'String', 'Restore view',     'units', 'normalized', ...
    'Position', [.35 .36 .3 .1],  'Callback', @restoreAxesButton_Callback);
uicontrol(oDataPanel_h, 'Style', 'pushbutton', 'String', 'Load previous PC', 'units', 'normalized', ...
    'Position', [.7 .36 .3 .1],   'Callback', @loadPreviousButton_Callback);

uicontrol(oDataPanel_h, 'Style', 'text', 'String', 'Metadata:', 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [0 0.24 1 .1]);
oMetadataTable_h = uitable(oDataPanel_h, 'RowName',[], 'ColumnName',[], 'units', 'normalized', ...
    'Position',[0 0.0 .7 .52], 'ColumnEditable', false, 'Enable', 'inactive', ...
    'ColumnFormat', {'char', 'char', 'char', 'char'}, 'ColumnWidth', {60 65 60 55}, 'Data', fillMetadataTable());
oMetadataTable_h.Position(3) = oMetadataTable_h.Extent(3)-0.005; oMetadataTable_h.Position(4) = oMetadataTable_h.Extent(4)-0.005;

%% Data panel: View buttons
oViewButtonGroup_h   = uibuttongroup(oDataPanel_h, 'SelectionChangedFcn', @viewGroup_Callback, ...
    'Position', [0.67 oMetadataTable_h.Position(2) .4 .2], 'BorderType', 'none');
uicontrol(oDataPanel_h, 'Style', 'checkbox', 'String', 'Show ground', 'units', 'normalized', ...
    'Position', [.67 (oMetadataTable_h.Position(2) + oMetadataTable_h.Position(4)-.085) .3 .1], ...
    'Callback', @showGroundBox_Callback, 'Value', bShowGround);
oShowFrontButton_h   = uicontrol(oViewButtonGroup_h, 'Style', 'radiobutton', 'Enable', 'off', 'Tag', 'front',  ...
    'String', 'Front', 'units', 'normalized', 'Position', [0 .45 .5 .5]);
oShowBackButton_h    = uicontrol(oViewButtonGroup_h, 'Style', 'radiobutton', 'Enable', 'off', 'Tag', 'back',   ...
    'String', 'Back', 'units', 'normalized', 'Position', [0  0 .5  .5]);
uicontrol(oViewButtonGroup_h, 'Style', 'radiobutton', 'Enable', 'off', 'Tag', 'global', ...
    'String', 'Global',    'units', 'normalized', 'Position', (oShowFrontButton_h.Position + [0.45 0 0 0]));
uicontrol(oViewButtonGroup_h, 'Style', 'radiobutton', 'Enable', 'off', 'Tag', 'center', ...
    'String', 'Center',    'units', 'normalized', 'Position', (oShowBackButton_h.Position + [0.45 0 0 0]));
oViewButtonGroup_h.Children(1,1).Value = 1; % Activate center as default

%% Object panel: Mode buttons

uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'Label and Bounding Box workspace', ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [0 0.95 1 .05]);
oCorrectionButton_h  = uicontrol(oObjectPanel_h, 'Style', 'toggleButton', 'String', 'Correction mode', 'units', 'normalized', ...
    'Position', [0 0.94 .3 .03],    'Callback', @correctionMode_Callback);
oNewBoxButton_h      = uicontrol(oObjectPanel_h, 'Style', 'toggleButton', 'String', 'New box mode',    'units', 'normalized', ...
    'Position', [0.35 0.94 .3 .03], 'Callback', @newBoxMode_Callback);
oPickBoxButton_h     = uicontrol(oObjectPanel_h, 'Style', 'toggleButton', 'String', 'Pick box mode',   'units', 'normalized', ...
    'Position', [0.7 0.94 .3 .03],  'Callback', @pickBoxMode_Callback);
%% Box panel: Definition states

uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'Definition state: ', 'HorizontalAlignment', 'left', 'units', 'normalized', ...
    'Position', [0 0.87 .3 .05]);
oDefinitionButtonGroup_h = uibuttongroup(oObjectPanel_h, 'SelectionChangedFcn', @definitionGroup_Callback, ...
    'Position', [.22 0.895 1-.22 .03], 'BorderType', 'line');
oEdgeButton_h    = uicontrol(oDefinitionButtonGroup_h, 'Style', 'radiobutton', 'String', 'edges',  'Tag', 'boxEdgeDefinition',    ...
    'Enable', 'off', 'units', 'normalized', 'Position', [0      0 .2 1]);
oYawButton_h     = uicontrol(oDefinitionButtonGroup_h, 'Style', 'radiobutton', 'String', 'yaw',    'Tag', 'boxYawDefinition',     ...
    'Enable', 'off', 'units', 'normalized', 'Position', [0.18   0  1 1]);
oWidthButton_h   = uicontrol(oDefinitionButtonGroup_h, 'Style', 'radiobutton', 'String', 'width',  'Tag', 'boxWidthDefinition',   ...
    'Enable', 'off', 'units', 'normalized', 'Position', [0.33   0  1 1]);
oLengthButton_h  = uicontrol(oDefinitionButtonGroup_h, 'Style', 'radiobutton', 'String', 'length', 'Tag', 'boxLengthDefinition',  ...
    'Enable', 'off', 'units', 'normalized', 'Position', [0.51   0  1 1]);
oPosButton_h     = uicontrol(oDefinitionButtonGroup_h, 'Style', 'radiobutton', 'String', 'pos',    'Tag', 'boxPosDefinition',     ...
    'Enable', 'off', 'units', 'normalized', 'Position', [0.7    0  1 1]);
oCamButton_h     = uicontrol(oDefinitionButtonGroup_h, 'Style', 'radiobutton', 'String', 'cam',    'Tag', 'boxCamDefinition',     ...
    'Enable', 'off', 'units', 'normalized', 'Position', [.85    0  1 1]);
oDefinitionButtonGroup_h.SelectedObject = [];
uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'Current object information: ', 'HorizontalAlignment', ...
    'left', 'units', 'normalized', 'Position', [0 .835 1 .05]);
%% Box panel: Current object information table

oCurrentBoxInfoTable_h = uitable(oObjectPanel_h, 'RowName',[], 'ColumnName',[], 'units', 'normalized', 'Position',[0 .83 1 .05], ...
    'ColumnEditable',false, 'Enable', 'inactive', 'ColumnFormat', {'char', 'char', 'char', 'char', 'char', 'char', 'char', 'char'}, ...
    'ColumnWidth', {23 38 23 38 20 38 20 30 20 50 20 50}, 'Data', fillCurrentBoxInfoTable());
oCurrentBoxInfoTable_h.Position(3) = oCurrentBoxInfoTable_h.Extent(3)-0.005;
oCurrentBoxInfoTable_h.Position(4) = oCurrentBoxInfoTable_h.Extent(4)-0.002;
%% Box panel: Class selection
%  Class selection is generated after reading the config files

uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'Class selection: ', 'HorizontalAlignment', 'left', ...
    'units', 'normalized', 'Position', [0 .76 .25 .05]);

fClassesTopPos      = .785;
fClassesHeight      = .4;
fClassesBottom      = (fClassesTopPos - fClassesHeight);
%% Box panel: Object list table

fSizeMovTable = .16; fLeft = .28; fWidth = 0.71;
uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'Object list:', 'HorizontalAlignment', 'left', ...
    'units', 'normalized', 'Position', [fLeft .76 .65 .05]);
oObjectListTable_h = uitable(oObjectPanel_h, 'RowName',[], 'ColumnName', ...
    {'', '#   ', 'class          ', 'act', 'corr', 'new', 'pred'}, 'units', 'normalized', ...
    'Position',[fLeft fClassesBottom-fSizeMovTable fWidth fClassesHeight+fSizeMovTable], 'ColumnEditable', false, 'Enable', ...
    'inactive', 'ColumnFormat',{'char', 'char', 'char', 'logical', 'char', 'char', 'char'}, 'ColumnWidth', {20 20 65 35 35 35 35});

%% Settings: Discretization

uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'Discretization:', 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [0 .19 .26 .05]);
uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'position [m]', ...
    'HorizontalAlignment', 'left', 'units', 'normalized',  'Position', [0 .16 1 .05]);
oDisPos_h = uicontrol(oObjectPanel_h, 'Style', 'edit', 'String', '0.050', 'units', ...
    'normalized',  'Position', [.19 .183 .09 .03], 'Callback', @positioning_Callback);
uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'size l / w [m]', ...
    'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [.35 .16 1 .05]);
oDisLW_h  = uicontrol(oObjectPanel_h, 'Style', 'edit', 'String', '0.100', 'units', 'normalized', ...
    'Position', [.56 .183 .09 .03], 'Callback', @lengthWidth_Callback);
uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'yaw [deg]', ...
    'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [0.7 .16 1 .05]);
oDisYaw_h = uicontrol(oObjectPanel_h, 'Style', 'edit', 'String', '1.000', 'units', 'normalized', ...
    'Position', [0.87 .183 .09 .03], 'Callback', @yaw_Callback);

%% Settings: Optimization

uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'ICP settings:', 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [0 .125 1 .05]);
uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'Max. # iterations', ...
    'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [0 .1 1 .05]);
oIt_h   = uicontrol(oObjectPanel_h, 'Style', 'edit', 'String', '10', 'units', 'normalized', ...
    'Position', [.23 .123 .05 .03], 'Callback', @iterations_Callback);
uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'Max. size accu', ...
    'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [.35 .1 1 .05]);
oAccu_h = uicontrol(oObjectPanel_h, 'Style', 'edit', 'String', '1000', 'units', 'normalized', ...
    'Position', [.56 .123 .09 .03], 'Callback', @accu_Callback);
uicontrol(oObjectPanel_h, 'Style', 'checkbox', 'String', 'Show accumulator', 'units', 'normalized', ...
    'Position', [0.7 .123 .3 .03], 'Callback', @showAccu_Callback, 'Value',0);
%% Settings: Kalman Filters

uicontrol(oObjectPanel_h, 'Style', 'text', 'String', 'Kalman filter measurement updates:', ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'units', 'normalized', 'Position', [0 .065 1 .05]);
oPrelabeling    = uicontrol(oObjectPanel_h, 'Style', 'checkbox', 'String', 'Prelabeling', 'units', 'normalized', ...
    'Position', [0 0.06 .3 .03],     'Callback', @enablePrelabeling_Callback,  'Value', bEnablePrelabelingUpdates);
oCorrections_h  = uicontrol(oObjectPanel_h, 'Style', 'checkbox', 'String', 'Correction', 'units', 'normalized', ...
    'Position', [0.35 0.06 .3 .03],  'Callback', @enableCorrectionUpdates_Callback, 'Value', bEnableCorrectionUpdates);
oOptimization_h = uicontrol(oObjectPanel_h, 'Style', 'checkbox', 'String', 'Optimization', 'units', 'normalized', ...
    'Position', [0.7 0.06 .3 .03],   'Callback' , @enableOptimization_Callback, 'Value', bEnableOptimizationUpdates);

uicontrol(oObjectPanel_h, 'Style', 'pushbutton', 'String', 'Relabel & complete', 'units', 'normalized', ...
    'Position', [0 0.015 .3 .035],    'Callback', @relabelButton_Callback);
uicontrol(oObjectPanel_h, 'Style', 'pushbutton', 'String', 'Save',               'units', 'normalized', ...
    'Position', [0.35 0.015 .3 .035], 'Callback', @saveButton_Callback);
uicontrol(oObjectPanel_h, 'Style', 'pushbutton', 'String', 'Save & proceed',     'units', 'normalized', ...
    'Position', [.7 0.015 .3 .035],   'Callback', @saveAndProceed_Callback);

%% Info panel
oInfo_h = uicontrol(oInfoPanel_h, 'Style', 'edit', 'String', 'GUI initialized.', 'HorizontalAlignment', 'left', ...
    'units', 'normalized', 'Position', [0 0 1 1], 'Max',10, 'Min',0);

%% Image panel

fPanelLeft = 0.07; fPanelBottom = .15; fPanelWidth = .12; fPanelHeight = .15;
oImagePanelOptions_h = uipanel(oImagePanel_h, 'units', 'normalized', 'Position', ...
    [fPanelLeft, fPanelBottom, fPanelWidth, fPanelHeight]);

uicontrol(oImagePanelOptions_h, 'Style', 'text', 'String', 'Projection mode', 'FontWeight', 'bold', 'HorizontalAlignment', ...
    'left', 'units', 'normalized', 'Position', [0 .85 .5 .15]);

% Project button
uicontrol(oImagePanelOptions_h, 'Style', 'pushbutton', 'String', 'Project', 'units', 'normalized', ...
    'Position', [.6 .85 .4 .15], 'Callback', @projectObjectsIntoImages_Callback, 'Enable', bEnableImageLabeling);

% Segmentation button group
fHeight = .3; fOffset = 0.15;
oProjectionButtonGroup_h = uibuttongroup(oImagePanelOptions_h, 'SelectionChangedFcn', @projectionGroup_Callback, ...
    'Position', [0 1-fHeight-fOffset .5 fHeight], 'BorderType', 'line');

uicontrol(oProjectionButtonGroup_h, 'Style', 'radiobutton', 'String', '3D',        ...
    'units', 'normalized', 'Enable', bEnableImageLabeling, 'Position', [0 .5 1 .5]);
uicontrol(oProjectionButtonGroup_h, 'Style', 'radiobutton', 'String', 'Polygon',        ...
    'units', 'normalized', 'Enable', bEnableImageLabeling, 'Position', [0 0 1 .5]);

% Image context button group
uicontrol(oImagePanelOptions_h, 'Style', 'text', 'String', 'Image context', 'FontWeight', 'bold', 'HorizontalAlignment', ...
    'left', 'units', 'normalized', 'Position', [0 .35 .5 .15]);

fHeight = .3; fOffset = 0.65;
oImageContextButtonGroup_h = uibuttongroup(oImagePanelOptions_h, 'SelectionChangedFcn', @imageContextGroup_Callback, ...
    'Position', [0 1-fHeight-fOffset .5 fHeight], 'BorderType', 'line');

uicontrol(oImageContextButtonGroup_h, 'Style', 'radiobutton', 'String', 'Ray projection',        ...
    'units', 'normalized', 'Enable', bEnableImageLabeling, 'Position', [0 .5 1 .5]);
oImageObjectDefinitionButton_h = uicontrol(oImageContextButtonGroup_h, 'Style', 'radiobutton', 'String', 'Object definition', ...
    'units', 'normalized', 'Enable', bEnableImageLabeling, 'Position', [0 0 1 .5]);

%% Menus: Settings

oSettingsMenu_h = uimenu(oGUI_h, 'Label', '||   Settings');
oGeneralMenu_h  = uimenu(oSettingsMenu_h, 'Label', 'General');
oEnablePredictionMenu_h           = uimenu(oGeneralMenu_h, 'Label', 'Predict',                  ...
    'Checked', bPredictionActive, 'Callback', @enablePredictionMenu_Callback);
oAutoImportMenu_h                 = uimenu(oGeneralMenu_h, 'Label', 'Auto import objects',      ...
    'Checked', bAutoImport, 'Callback', @checkForNewTracksMenu_Callback);
oAutoDeleteMenu_h                 = uimenu(oGeneralMenu_h, 'Label', 'Auto delete objects',      ...
    'Checked', bAutoDelete, 'Callback', @autoDeleteMenu_Callback);
oInitEditedMenu_h                 = uimenu(oGeneralMenu_h, 'Label', 'Init from edited PCs',     ...
    'Checked', bInitFromEdited, 'Callback', @loadEditedMenu_Callback);
oEnableDebugInformationMenu_h     = uimenu(oGeneralMenu_h, 'Label', 'Debug information',        ...
    'Checked', bDebugInfo, 'Callback', @enableDebugInformationMenu_Callback);
oDisplayObjectDataMenu_h          = uimenu(oGeneralMenu_h, 'Label', 'Display object data',      ...
    'Checked', bDisplayObjectData, 'Callback', @displayObjectDataMenu_Callback);
% ICP menu
oICPMenu_h = uimenu(oSettingsMenu_h, 'Label', 'Optimization');
oEnableRegistrationMenu_h       = uimenu(oICPMenu_h, 'Label', 'Enable ICP',                  ...
    'Checked', bEnableICP,          'Callback', @enableRegistrationMenu_Callback);
oEnableFittingMenu_h            = uimenu(oICPMenu_h, 'Label', 'Enable fitting',              ...
    'Checked', bEnableFitting,      'Callback', @enableFittingMenu_Callback);
oEnableEdgesEstimationMenu_h    = uimenu(oICPMenu_h, 'Label', 'Enable edges estimation',     ...
    'Checked', bEnableEdges,        'Callback', @enableEdgesMenu_Callback);
oEnableOptimizationPlotsMenu_h  = uimenu(oICPMenu_h, 'Label', 'Enable debug plots',          ...
    'Checked', bEnableDebugPlots,   'Callback', @enableOptimizationDebugPlots_Callback);

%% Initialize GUI data

oGUIData = cGUIData();

% Menu settings
oGUIData.m_bEnableDebugInformation  = settingByMenu(oEnableDebugInformationMenu_h);
oGUIData.m_bInitFromEditedPC        = settingByMenu(oInitEditedMenu_h);
oGUIData.m_bAutoImport              = settingByMenu(oAutoImportMenu_h);
oGUIData.m_bAutoDelete              = settingByMenu(oAutoDeleteMenu_h);
oGUIData.m_bEnableOptimizationPlots = settingByMenu(oEnableOptimizationPlotsMenu_h);
oGUIData.m_bPredict                 = settingByMenu(oEnablePredictionMenu_h);
oGUIData.m_bDisplayObjectData       = settingByMenu(oDisplayObjectDataMenu_h);

% Modes
oGUIData.m_bEnableImageLabeling     = strcmp(bEnableImageLabeling, 'on');
oGUIData.m_bCameraCalibrationMode   = strcmp(bCameraCalibrationMode, 'on');

% Optimization
oGUIData.m_bEnableRegistration      = settingByMenu(oEnableRegistrationMenu_h);
oGUIData.m_bEnableFitting           = settingByMenu(oEnableFittingMenu_h);
oGUIData.m_bEnableEdgesEstimation   = settingByMenu(oEnableEdgesEstimationMenu_h);

% General settings
oGUIData.m_fDeltaLW                 = str2double(oDisLW_h.String);
oGUIData.m_fDeltaYaw                = str2double(oDisYaw_h.String);
oGUIData.m_fDeltaXY                 = str2double(oDisPos_h.String);
oGUIData.m_nNumICPIterations        = str2double(get(oIt_h, 'String'));
oGUIData.m_nSizeICPFifo             = str2double(get(oAccu_h, 'String'));
oGUIData.m_fZoomFactor              = 8;
oGUIData.m_bShowAccu                = 0;
oGUIData.m_fRelabelMargin           = 0.2;
oGUIData.m_nDeleteDelay             = nDeleteDelay;
oGUIData.m_nPointsDeleteThreshold   = 5;
oGUIData.m_bLogWarnings             = bLogWarnings;
oGUIData.m_nGlobalCorrIntervall     = nGlobalCorrInterv;
oGUIData.m_fMaxRange                = fMaxRange;
oGUIData.m_nMaxNumInHistory         = 15;

% Kalman settings
oGUIData.m_bEnablePrelabeling   = oPrelabeling.Value;
oGUIData.m_bEnableCorrections   = oCorrections_h.Value;
oGUIData.m_bEnableOptimization  = oOptimization_h.Value;

% Image and segmentation options
oGUIData.m_sProjectionMode  = oProjectionButtonGroup_h.SelectedObject.String;
oGUIData.m_sImageContext    = oImageContextButtonGroup_h.SelectedObject.String;
oGUIData.m_bUndistortImages = bUndistortImages;
oGUIData.m_bRandomColor     = bRandomColor;
oGUIData.m_bLoadImageLabels = bLoadImageLabels;
oGUIData.m_bDisplayPD       = bDisplayPD;

% Inits
oGUIData.m_bInit                = 0;
oGUIData.m_sRootDir             = sStdRootDir;
oGUIData.m_sRecording           = sStdRecording;
oGUIData.m_sDatasetDir          = sStdDatasetDir;
oGUIData.m_nSequenceID          = str2double(get(oSequenceID_h, 'String'));
oGUIData.m_nPCID                = str2double(get(oPCID_h, 'String'));
oGUIData.m_sView                = 'center';
oGUIData.m_bShowGround          = bShowGround;
oGUIData.m_bGUIZoomed           = 0;
oGUIData.m_nConsecutiveFrames   = 0;
oGUIData.m_nNumTargets          = nNumTargets;
oGUIData.m_sProjectionMode_prev = oProjectionButtonGroup_h.SelectedObject.String;

% Graphic objects
oGUIData.m_oSavedPanel_h             = oSavedPanel_h;
oGUIData.m_oSavedText_h              = oSavedText_h;
oGUIData.m_oImagePanel_h             = oImagePanel_h;
oGUIData.m_oShowFrontButton_h        = oShowFrontButton_h;
oGUIData.m_oShowBackButton_h         = oShowBackButton_h;
oGUIData.m_oPCAxes_h                 = oPCAxes_h;
oGUIData.m_oObjectListTable_h        = oObjectListTable_h;
oGUIData.m_oInfo_h                   = oInfo_h;
oGUIData.m_oGUI_h                    = oGUI_h;
oGUIData.m_oMetadataTable_h          = oMetadataTable_h;
oGUIData.m_hImageCallback            = @image_Callback;
oGUIData.m_voImageAxes_h             = gobjects(4,1);
oGUIData.m_hImageMenuCallback        = @imagePanelContextMenu_Callback;
oGUIData.m_oImages_h                 = oImages_h;

% GUI objects
oGUIData.m_voEdgePoints_h       = gobjects(4,1);
oGUIData.m_mfEdgePoints         = zeros(4,3);
oGUIData.m_nNumEdgePoints       = 0;
oGUIData.m_bConfirmBoxDeletion  = 0;
oGUIData.m_nNumPCObjects        = 0;
oGUIData.m_nIndexCurrentObject  = 0;
oGUIData.m_vnDeletedTrackIDs    = [];

% Camera handling
oGUIData.m_vfCameraTarget       = zeros(1,3);
oGUIData.m_bEnableCameraMov     = 0;
oGUIData.m_bEnableCameraRot     = 0;
oGUIData.m_bCameraTargetSet     = 0;
oGUIData.m_vfPrevPoint          = zeros(2,1);
oGUIData.m_bLeftMouseDown       = 0;
oGUIData.m_bRightMouseDown      = 0;
oGUIData.m_fCameraViewAngleUnzoomed = 6.6086;
oGUIData.m_fCameraViewAngleZoomed   = .827;

% Init state machine
oGUIData.m_sDefinitionState     = 'none';
oGUIData.m_bInDefinitionState   = 0;
oGUIData.m_sDefinitionMode      = 'none';

% Set current recording
oGUIData.m_sRecording = sStdRecording;

% Setup 
oGUIData.m_bConfigFilesLoaded = 0;
oGUIData.m_oEditorConfig  = cEditorConfig();
oGUIData.m_oPrelabelingConfig  = cPrelabelingConfig();
oGUIData.m_oEditorConfig.m_sRecording = sStdRecording;
oGUIData = setupEditor(oGUIData);

% Set data and GUI visible
setappdata(oGUI_h, 'GUIData',    oGUIData);
setappdata(oGUI_h, 'GUIObjects', cGUIObjects());
setappdata(oGUI_h, 'GUIHistory', []);
setappdata(oGUI_h, 'busy', 0);

set(oImages_h, 'Visible', 'on');
set(oGUI_h, 'Visible', 'on');

maximize(oImages_h);
maximize(oGUI_h);

%% Data callbacks

% Look automatically for first upcoming sequence in directory
    function oGUIData = setSequenceIDAutomatically(oGUIData)
        nPos = setInfoText(oInfo_h, 'Searching for sequences...', 1);
        
        sDatasetDir = oGUIData.m_sDatasetDir;
        if exist(sDatasetDir, 'dir') ~= 0 % directory exists
            vsSequences = dir(strcat(sDatasetDir, '\PCDataMatrices'));
            if strcmp(vsSequences(end).name(1,1), '.')
                setInfoText(oInfo_h, 'No sequences found. Please choose a valid directory.', 0);
                return;
            end
        else
            setInfoText(oInfo_h, 'Error: Directory does not exist.', 1);
            setInfoText(oInfo_h, 'Please choose a valid directory.', 1);
            return
        end
        i = 1;
        while strcmp(vsSequences(i,1).name(1,1), '.')
            i = i + 1;
        end
        
        % Process ID
        sFirstSeq    = num2str(str2double(vsSequences(i).name(1,5:end)));
        sPCDir       = strcat(sDatasetDir, '/PCDataMatrices', '/', vsSequences(i).name);
        vsPCIDs = dir(sPCDir); 
        i = 1;
        try
            while strcmp(vsPCIDs(i).name(1,1), '.')
                i = i + 1;
            end
            sFirstPCID = num2str(str2double(vsPCIDs(i).name(1,1:10)));
            oPCID_h.String          = sFirstPCID;
            oSequenceID_h.String    = sFirstSeq;
            oGUIData.m_nSequenceID  = str2double(sFirstSeq);
            oGUIData.m_nPCID        = str2double(sFirstPCID);
        catch
            setInfoText(oInfo_h, 'No point clouds found. Please choose a valid directory.', 1);
        end
        setInfoText(oInfo_h, 'Searching for sequences... done.', 1, nPos);
    end

% Setup editor by files in dataset root directory 
    function oGUIData = setupEditor(oGUIData)
        oGUIData.m_sDatasetDir = strcat(oGUIData.m_sRootDir, '\', oGUIData.m_sRecording);
        oInfo_h.String = {};
        
        % Read config files
        oGUIData.m_bConfigFilesLoaded = 0;
        oEConf  = oGUIData.m_oEditorConfig;
        oPConf  = oGUIData.m_oPrelabelingConfig;
        [~, nCodeE] = oEConf.load(oGUIData.m_sRootDir, oInfo_h);
        [~, nCodeR] = oPConf.load(oGUIData.m_sRootDir, oInfo_h);
        oEConf  = oEConf.compare(oPConf, oEConf.m_sRecording, oInfo_h);
        [oGUIData.m_voCalibration, nCodeC] = readCalibrationData(oGUIData.m_sRootDir);
        
        if ~nCodeE && ~nCodeR && ~nCodeC
            oGUIData.m_bConfigFilesLoaded = 1;
            oGUIData.m_MapClassToID = oEConf.getLabelMap();
            oGUIData.m_oClassesButtonGroup_h = generateClassSelection(oObjectPanel_h, oEConf, @classesGroup_Callback);
        end
        
        oGUIData.m_oEditorConfig        = oEConf;
        oGUIData.m_oPrelabelingConfig   = oPConf;
        
        oGUIData = setSequenceAndPCID(oGUIData);
    end

% Set sequence and PCID by editor config or automatically
    function oGUIData = setSequenceAndPCID(oGUIData)
        if oGUIData.m_bInitFromEditedPC && (oGUIData.m_oEditorConfig.m_LastEditedSequenceID ~= -1)
            oGUIData.m_nSequenceID  = oGUIData.m_oEditorConfig.m_LastEditedSequenceID;
            oGUIData.m_nPCID        = oGUIData.m_oEditorConfig.m_LastEditedPCID;
            
            oPCID_h.String       = oGUIData.m_oEditorConfig.m_LastEditedPCID;
            oSequenceID_h.String = oGUIData.m_oEditorConfig.m_LastEditedSequenceID;
            setInfoText(oInfo_h, 'Jumped to last edited sequence and PCID.', 1);
        else
            % Set to first sequence and PC in prelabeled directory
            oGUIData = setSequenceIDAutomatically(oGUIData);
        end 
    end

% Set base (root) directory by string
    function rootDir_Callback(hObject, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData = getappdata(oGUI_h, 'GUIData');
        
        % Read object and set directory
        oGUIData.m_sRootDir = get(hObject, 'String');
        
        % Load config files and set sequences
        oGUIData = setupEditor(oGUIData);
        
        % Set data
        setappdata(oGUI_h, 'GUIData', oGUIData);
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Set base (root) directory by selection dialog
    function rootDirSelect_Callback(~, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData = getappdata(oGUI_h, 'GUIData');
        
        % Selection
        sDirectory = uigetdir(get(oRootDir_h, 'String'));
        if sDirectory ~= 0
            set(oRootDir_h, 'String', sDirectory);
            oGUIData.m_sRootDir = sDirectory;
            oGUIData.m_sDatasetDir = strcat(sDirectory, '\', oGUIData.m_sRecording);
        end
        
        % Load config files and set sequences
        oGUIData = setupEditor(oGUIData);
        
        % Set data
        setappdata(oGUI_h, 'GUIData', oGUIData);
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Set recording directory by string
    function recordingDir_Callback(hObject, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData = getappdata(oGUI_h, 'GUIData');
        oInfo_h.String = {};
        
        % Read object and set sequence
        sRecording = get(hObject, 'String');
        oGUIData.m_sRecording = sRecording;
        oGUIData.m_sDatasetDir = strcat(oGUIData.m_sRootDir, '\', sRecording);
        oGUIData.m_oEditorConfig.m_sRecording = sRecording;
        oGUIData.m_oEditorConfig.compare(oGUIData.m_oPrelabelingConfig, sRecording, oInfo_h);
        
        oGUIData = setSequenceAndPCID(oGUIData);
       
        % Set data
        setappdata(oGUI_h, 'GUIData', oGUIData);
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Set config directory by dialog
    function recordingDirSelect_Callback(~, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData = getappdata(oGUI_h, 'GUIData');
        oInfo_h.String = {};
        
        % Selection
        sRecordingDir = uigetdir(get(oRecording_h, 'String'));
        if sRecordingDir ~= 0
            clDirectory = strsplit(sRecordingDir, '\');
            sRecording = clDirectory{1,end};
            set(oRecording_h, 'String', sRecording);
            oGUIData.m_sRecording = sRecording;
            oGUIData.m_sDatasetDir = strcat(oGUIData.m_sRootDir, '\', sRecording);
            oGUIData.m_oEditorConfig.m_sRecording = sRecording;
            oGUIData.m_oEditorConfig.compare(oGUIData.m_oPrelabelingConfig, sRecording, oInfo_h);
        end
        
        oGUIData = setSequenceAndPCID(oGUIData);
        
        % Set data
        setappdata(oGUI_h, 'GUIData', oGUIData);
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Set sequence ID by string
    function sequenceDir_Callback(hObject, ~, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData = getappdata(oGUI_h, 'GUIData');
        oGUIData.m_nSequenceID = str2double(get(hObject, 'String'));
        % Check point clouds in new sequence directory
        pcDir = strcat(oGUIData.m_sDatasetDir, '\', 'PCDataMatrices\', num2str(oGUIData.m_nSequenceID, 'Seq_%010u'), '\*.bin');
        pcIDs = dir(pcDir);
        i = 1;
        try
            while strcmp(pcIDs(i).name(1,1), '.')
                i = i + 1;
            end
            firstPCID = num2str(str2double(pcIDs(i).name(1,1:10)));
            oPCID_h.String = firstPCID;
            oGUIData.m_nPCID = str2double(firstPCID);
        catch
            setInfoText(oInfo_h, 'Sequence does not exist in folder. Choose a different one.', 0);
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Set point cloud ID by string
    function pcDir_Callback(hObject, ~, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        oGUIData.m_nPCID = str2double(get(hObject, 'String'));
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Load a point cloud and images. Reinitializes GUI.
    function loadButton_Callback(~, ~, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        oGUIHistory = getappdata(oGUI_h, 'GUIHistory');
        setSavedPanel(oSavedText_h, oSavedPanel_h, 'busy', 'y');
        
        oInfo_h.String = {};
        
        % Check if config files are loaded
        if ~oGUIData.m_bConfigFilesLoaded
            setInfoText(oInfo_h, 'Error: Could not load config files. Check standard root directory (TUBS).', 1);
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return
        end
        
        % Check if GUI is already initialized
        if oGUIData.m_bInit
            % Confirm reinit
            sButName = buttondlg(sprintf('GUI will be reinitialized. Current boxes and history will be lost.\n Confirm reinit operation.'), ...
                'Confirm', 'Reinit', 'Abort', struct('Default', 'Reinit', 'IconString', 'warn'));
            if strcmp(sButName, 'Abort')
                enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
                return
            end
            oGUIData.m_bInit = 0;
        end
        
        % Check for empty prelabeling directory (no data available)
        if checkForEmptyDirectory(buildPath(oGUIData.m_sDatasetDir, oGUIData.m_nSequenceID, oGUIData.m_nPCID, 12))
            setInfoText(oInfo_h, 'Error: Could not find file. Check IDs and directories.', 1);
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return
        end
        
        % Load to GUI
        bForward = 0;   % don't care for init case
        [oGUIData, oGUIObjects, oGUIHistory, ~] = loadToGUI(oGUIData, oGUIObjects, oGUIHistory, bForward);
        
        if oGUIHistory(end,1).m_bIsSaved == 1
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'saved', 'gr')
        else
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'unsaved', 'r')
        end
        
        % Enable view group
        oViewButtons_h = allchild(oViewButtonGroup_h);
        for i = 1 : size(oViewButtons_h, 1)
            oViewButtons_h(i).Enable = 'on';
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        setappdata(oGUI_h, 'GUIHistory', oGUIHistory);
        
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Load the next point cloud
    function loadNextButton_Callback(~, ~, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        oGUIHistory = getappdata(oGUI_h, 'GUIHistory');
        setSavedPanel(oSavedText_h, oSavedPanel_h, 'busy', 'y');
        oInfo_h.String = {};
        
        if ~oGUIData.m_bInit
            setInfoText(oInfo_h, 'Initialize GUI using "load" first!', 0);
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return;
        end
        
        nSequenceID = oGUIData.m_nSequenceID;
        nPCID       = oGUIData.m_nPCID;
        if oGUIData.m_oPCMetadata.m_bIsLastOfSequence
            nSequenceID = nSequenceID + 1;
            setInfoText(oInfo_h, sprintf('End of sequence reached. Proceeding to ID %d.', nSequenceID), 1);
        end
        nPCID = nPCID + 1;
        % Check for empty prelabeling directory (no data available)
        if checkForEmptyDirectory(buildPath(oGUIData.m_sDatasetDir, nSequenceID, nPCID, 12))
            setInfoText(oInfo_h, 'Error: Could not find file. Check IDs and prelabeled directory.', 0);
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return
        end
        
        oGUIData.m_nSequenceID  = nSequenceID;
        oGUIData.m_nPCID        = nPCID;
        oPCID_h.String          = num2str(nPCID);
        oSequenceID_h.String    = num2str(nSequenceID);
        
        bForward                = 1;
        [oGUIData, oGUIObjects, oGUIHistory, sStatus] = loadToGUI(oGUIData, oGUIObjects, oGUIHistory, bForward);
        
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        setappdata(oGUI_h, 'GUIHistory', oGUIHistory);
        
        if      strcmp(sStatus, 'saved')
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'saved', 'gr');
        elseif  strcmp(sStatus, 'unsaved')
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'unsaved', 'r')
        elseif  strcmp(sStatus, 'delta')
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'delta', 'y')
        end
        
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Load the previous point cloud
    function loadPreviousButton_Callback(~, ~, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        oGUIHistory = getappdata(oGUI_h, 'GUIHistory');
        oInfo_h.String = {};
        setSavedPanel(oSavedText_h, oSavedPanel_h, 'busy', 'y');
        
        if ~oGUIData.m_bInit
            setInfoText(oInfo_h, 'Initialize GUI using "load" first!', 0);
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return;
        end
        
        nSequenceID = oGUIData.m_nSequenceID;
        nPCID       = oGUIData.m_nPCID;
        if oGUIData.m_oPCMetadata.m_bIsFirstOfSequence
            nSequenceID = nSequenceID - 1;
            setInfoText(oInfo_h, sprintf('End of sequence reached. Changing sequence to ID %d.', nSequenceID), 1);
        end
        nPCID = nPCID - 1;
        % Check for empty prelabeling directory (no data available)
        if checkForEmptyDirectory(buildPath(oGUIData.m_sDatasetDir, nSequenceID, nPCID, 12))
            setInfoText(oInfo_h, 'Error: Could not find file. Check IDs and edited directory.', 1);
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return
        end
        
        bForward                = 0;
        oGUIData.m_nSequenceID  = nSequenceID;
        oGUIData.m_nPCID        = nPCID;
        
        [oGUIData, oGUIObjects, oGUIHistory, sStatus] = loadToGUI(oGUIData, oGUIObjects, oGUIHistory, bForward);
        
        if ~strcmp(sStatus, 'end')
            oPCID_h.String          = num2str(nPCID);
            oSequenceID_h.String    = num2str(nSequenceID);
        end
        
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        setappdata(oGUI_h, 'GUIHistory', oGUIHistory);
        
        if      strcmp(sStatus, 'saved')
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'saved', 'gr');
        elseif  strcmp(sStatus, 'unsaved')
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'unsaved', 'r')
        end
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Restores view on point cloud axes to default
    function restoreAxesButton_Callback(~, ~, ~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        if ~oGUIData.m_bInit
            return
        end
        
        % Axes are zoomed to object. Edge or cam definition
        if strcmp(oGUIData.m_sDefinitionState, 'edge') || strcmp(oGUIData.m_sDefinitionState, 'cam')
            % Axes are zoomed to object. Reset position and target.
            nObj = oGUIData.m_nIndexCurrentObject;
            if nObj > 0
                oPCAxes_h.CameraTarget   = [oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_x, ...
                    oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_y, 1];
                oPCAxes_h.CameraPosition = [oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_x, ...
                    oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_y, 1030];
            end
            oPCAxes_h.CameraUpVector = [0 1 0];
            oPCAxes_h.CameraViewAngle = oGUIData.m_fCameraViewAngleZoomed;
            % All other definition states
        elseif ~strcmp(oGUIData.m_sDefinitionState, 'none')
            oPCAxes_h.CameraTarget    = oGUIData.m_vfCameraTarget;
            oPCAxes_h.CameraPosition  = [oPCAxes_h.CameraTarget(1,1), oPCAxes_h.CameraTarget(1,2), 1030];
            oPCAxes_h.CameraUpVector  = [0 1 0];
            oPCAxes_h.CameraViewAngle = oGUIData.m_fCameraViewAngleZoomed;
            % Restore to standard view if not in definition state
        else
            setAxesSettings(oPCAxes_h, oGUIData.m_sView);
            oGUIData.m_bGUIZoomed = 0;
        end
        
        % Restore view to images
        for i = 1 : 4
            if isgraphics(oGUIData.m_voImageAxes_h(i,1))
                zoom(oGUIData.m_voImageAxes_h(i,1), 'out');
            end
        end
    end

% Show or unshow ground points
    function showGroundBox_Callback(hObject, ~, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if ~oGUIData.m_bInit
            setInfoText(oInfo_h, 'You have to load a PC before editing!', 0);
            set(hObject, 'Value', get(hObject, 'Min'));
            return;
        end
        
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            set(oGUIData.m_oGround_h, 'Visible', 'on');
            oGUIData.m_bShowGround = 1;
        elseif button_state == get(hObject, 'Min')
            set(oGUIData.m_oGround_h, 'Visible', 'off');
            oGUIData.m_bShowGround = 0;
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Set view mode
    function viewGroup_Callback(~, callbackdata)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        tag = callbackdata.NewValue.Tag;
        oGUIData.m_sView = tag;
        if strcmp(tag, 'global')
            oGUIData.m_fCameraViewAngleUnzoomed = 14.4678;
        else
            oGUIData.m_fCameraViewAngleUnzoomed = 6.6086;
        end
        setAxesSettings(oPCAxes_h, tag);
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

%% Editor settings callbacks

% Set position resolution
    function positioning_Callback(hObject, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        numString = get(hObject, 'String');
        num = str2double(numString);
        if ~isnan(num) % Clean number typed
            oGUIData.m_fDeltaXY = num;
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Set length resolution
    function lengthWidth_Callback(hObject, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        numString = get(hObject, 'String');
        num = str2double(numString);
        if ~isnan(num) % Clean number typed
            oGUIData.m_fDeltaLW = num;
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Set yaw resolution
    function yaw_Callback(hObject, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        numString = get(hObject, 'String');
        num = str2double(numString);
        if ~isnan(num) % Clean number typed
            oGUIData.m_fDeltaYaw = num;
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Set maximum number of ICP iterations
    function iterations_Callback(hObject, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        numString = get(hObject, 'String');
        num = str2double(numString);
        if ~isnan(num) % Clean number typed
            oGUIData.m_nNumICPIterations = num;
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Set size of ICP accumulator
    function accu_Callback(hObject, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        numString = get(hObject, 'String');
        num = str2double(numString);
        if ~isnan(num)
            oGUIData.m_nNumICPIterations = num;
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Set accumulator visible
    function showAccu_Callback(hObject, ~, ~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        
        voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
        
        button_state = get(hObject, 'Value');
        sState = 'off';
        if (button_state == get(hObject, 'Max'))
            oGUIData.m_bShowAccu = 1;
            sState = 'on';
        elseif(button_state == get(hObject, 'Min'))
            oGUIData.m_bShowAccu = 0;
        end
        
        for i = 1 : size(voPCMovableLabel,1)
            oAccu_h = findobj(voPCMovableLabel(i,1).m_Box_h, 'Tag', 'accu');
            if ~isempty(oAccu_h)
                set(oAccu_h, 'Visible', sState);
            end
        end
        
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        drawnow;
    end

% Enable kalman measurement update based on tracking information from prelabeling
    function enablePrelabeling_Callback(hObject, ~, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            oGUIData.m_bEnablePrelabeling = 1;
        elseif button_state == get(hObject, 'Min')
            oGUIData.m_bEnablePrelabeling = 0;
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Enable kalman measurement update based on manual edits
    function enableCorrectionUpdates_Callback(hObject, ~, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            oGUIData.m_bEnableCorrections = 1;
        elseif button_state == get(hObject, 'Min')
            oGUIData.m_bEnableCorrections = 0;
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Enable automatic optimization algorithms for kalman measurement updates
    function enableOptimization_Callback(hObject, ~, ~)
        enableDisableFig(oGUI_h, 'off');
        button_state = get(hObject, 'Value');
        oGUIData = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
        
        if button_state == get(hObject, 'Max')
            oGUIData.m_bEnableOptimization = 1;
            % Reinit
            for i = 1 : size(voPCMovableLabel, 1)
                delete(voPCMovableLabel(i,1).m_Accu_h);
                voPCMovableLabel(i,1).m_mfRegisteredPointMatrix = voPCMovableLabel.m_mfPointMatrix;
                voPCMovableLabel(i,1).m_Accu_h = gobjects(1,1);
            end
        elseif button_state == get(hObject, 'Min')
            oGUIData.m_bEnableOptimization = 0;
            % Reset ICP related matrices
            for i = 1 : size(voPCMovableLabel, 1)
                voPCMovableLabel(i,1).m_mfRegisteredPointMatrix = [];
                delete(voPCMovableLabel(i,1).m_Accu_h);
            end
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        drawnow;
        enableDisableFig(oGUI_h, 'on');
    end

%% Edtior modes

% New box mode: Create a new object
    function newBoxMode_Callback(hObject, ~, ~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        if ~oGUIData.m_bInit
            setInfoText(oInfo_h, 'You have to load a PC before editing!', 0);
            set(hObject, 'Value', get(hObject, 'Min'));
            return;
        end
        % Reset GUI to default state
        [oGUIData, oGUIObjects] = resetBoxWorkspace(oGUIData, oGUIObjects);
        oGUIData = leavingDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
            oGUIData, oGUIObjects);
        % Set state machine
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            % Set GUI mode
            oGUIData.m_sDefinitionMode = 'new';
            oGUIData.m_sDefinitionState = 'zoomToPoint';
            % Unset correction and pick box button
            oPickBoxButton_h.Value = 0; oCorrectionButton_h.Value = 0;
            setInfoText(oInfo_h, 'Entering new box mode. Click axes to zoom!', 0);
        elseif button_state == get(hObject, 'Min')
            oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable();
            
            setInfoText(oInfo_h, 'New box mode left.', 0);
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
    end

    % Correction mode: Editor will jump from box to box automatically
    function correctionMode_Callback(hObject, ~, ~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        if ~oGUIData.m_bInit
            setInfoText(oInfo_h, 'You have to load a PC before editing.', 0);
            set(hObject, 'Value', get(hObject, 'Min'));
            return;
        end
        % Reset GUI to default state
        [oGUIData, oGUIObjects] = resetBoxWorkspace(oGUIData, oGUIObjects);
        oGUIData = leavingDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
            oGUIData, oGUIObjects);
        % Set state machine
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            oGUIData.m_sDefinitionMode = 'correction';
            % Unset new and pick box buttons
            oNewBoxButton_h.Value = get(hObject, 'Min'); oPickBoxButton_h.Value = get(hObject, 'Min');
            oGUIData.m_nIndexCurrentObject = 0;

            if mod(oGUIData.m_nConsecutiveFrames, oGUIData.m_nGlobalCorrIntervall) == 0
                setInfoText(oInfo_h, '********** GLOBAL CORRECTION **********', 0);
                setInfoText(oInfo_h, 'All objects need to be edited.', 1);
            else
                setInfoText(oInfo_h, 'Entering correction mode. Editor chooses objects to be edited.', 0);
            end
            setInfoText(oInfo_h, 'Press [up] or [down] to move through boxes!', 1);
            setInfoText(oInfo_h, 'Press [t] to toggle to next box to be corrected!', 1);
            setInfoText(oInfo_h, 'Press [esc] to exit!', 1);
        elseif button_state == get(hObject, 'Min')
            oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable();
            setInfoText(oInfo_h, 'Correction mode left.', 0);
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
    end

% Pick box mode: Pick an object pick clicking the axes
    function pickBoxMode_Callback(hObject, ~, ~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        if ~oGUIData.m_bInit
            setInfoText(oInfo_h, 'You have to load a PC before editing.', 0);
            set(hObject, 'Value', get(hObject, 'Min'));
            return;
        end
        % Reset GUI to default state
        [oGUIData, oGUIObjects] = resetBoxWorkspace(oGUIData, oGUIObjects);
        oGUIData = leavingDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
            oGUIData, oGUIObjects);
        % Set state machine
        button_state = get(hObject, 'Value');
        if button_state == get(hObject, 'Max')
            oGUIData.m_sDefinitionMode = 'pick';
            oNewBoxButton_h.Value = 0; oCorrectionButton_h.Value = 0;
            setInfoText(oInfo_h, 'Entering pick box mode. Click axes to zoom to desired box!', 0);
            setInfoText(oInfo_h, 'Press [esc] to exit!', 1);
            setInfoText(oInfo_h, 'Press [t] to toggle to closest box!', 1);
        elseif button_state == get(hObject, 'Min')
            setInfoText(oInfo_h, 'Pick box mode left.', 0);
            oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable();
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
    end

% Change classes for an object
    function classesGroup_Callback(~, callbackdata)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        if ~strcmp(oGUIData.m_sDefinitionState, 'none')
            % Set class
            nObj = oGUIData.m_nIndexCurrentObject;
            oGUIObjects.m_voPCMovableLabel(nObj).m_sClassification = callbackdata.NewValue.String;
            % Mark as corrected
            markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h)
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
    end

% Change definition mode by GUI selection
    function definitionGroup_Callback(~, callbackdata)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if ~strcmp(oGUIData.m_sDefinitionState, 'none')
            tag = callbackdata.NewValue.Tag;
            switch tag
                case 'boxPosDefinition'
                    oGUIData.m_sDefinitionState = 'pos';
                case 'boxLengthDefinition'
                    oGUIData.m_sDefinitionState = 'length';
                case 'boxYawDefinition'
                    oGUIData.m_sDefinitionState = 'yaw';
                case 'boxWidthDefinition'
                    oGUIData.m_sDefinitionState = 'width';
                case 'boxCamDefinition'
                    oGUIData.m_sDefinitionState = 'cam';
            end
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Relabel the point cloud
    function relabelButton_Callback(hObject, ~, ~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
        if ~oGUIData.m_bInit
            setInfoText(oInfo_h, 'You have to load a PC before editing.', 0);
            set(hObject, 'Value', get(hObject, 'Min'));
            return;
        end
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        
        % Relabel to init point matrices in rectangles
        fMargin = 0; bUseHeight = 0; bSelective = 0;     % fMargin = 0 improves overhang algorithm
        posHeader                     = setInfoText(oInfo_h, 'Relabeling:', 0);
        [oGUIData, voPCMovableLabel]  = relabelPC(voPCMovableLabel, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h);
        setInfoText(oInfo_h, 'Relabeling: ... done!', 0, posHeader);
        
        % Complete box data
        posHeader           = setInfoText(oInfo_h, 'Completing object data: ... ', 1);
        voPCMovableLabel    = completeObjectData(oGUIData, voPCMovableLabel);
        setInfoText(oInfo_h, 'Completing box data: done!', 1, posHeader);
        
        % Relabel again using newly computed height information, auto deletion of objects is active
        fMargin    = oGUIData.m_fRelabelMargin;
        bUseHeight = 1;
        posHeader                     = setInfoText(oInfo_h, 'Relabeling:', 1);
        [oGUIData, voPCMovableLabel]  = relabelPC(voPCMovableLabel, oGUIData, fMargin, bUseHeight, bSelective, oInfo_h);
        setInfoText(oInfo_h, 'Relabeling: ... done!', 1, posHeader);
        
        % Auto delete
        if oGUIData.m_bAutoDelete
            oGUIObjects.m_voPCMovableLabel = voPCMovableLabel;
            [oGUIData, oGUIObjects] = autoDelete(oGUIData, oGUIObjects);
            voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
        end
        
        % Correction measurement update
        if oGUIData.m_bEnableCorrections
            posHeader           = setInfoText(oInfo_h, 'Correction measurement update: ...' , 1);
            voPCMovableLabel    = updateKalmanByCorrections(voPCMovableLabel, oGUIData);
            setInfoText(oInfo_h, 'Correction measurement update: ... done!' , 1, posHeader);
        end
        
        % Project objects
        if oGUIData.m_bEnableImageLabeling
            sShapeType  = oGUIData.m_sProjectionMode;
            oGUIObjects = projectPCObjects(oGUIData, oGUIObjects, sShapeType, oGUIData.m_bRandomColor);
        end
        
        oGUIObjects.m_voPCMovableLabel = voPCMovableLabel;
        [oGUIData, oGUIObjects] = redrawGUI(oGUIData, oGUIObjects);
        
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Save the current object list
    function saveButton_Callback(~, ~, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        oGUIHistory = getappdata(oGUI_h, 'GUIHistory');
        
        setSavedPanel(oSavedText_h, oSavedPanel_h, 'busy', 'y');
        
        if ~oGUIData.m_bInit
            setInfoText(oInfo_h, 'You have to load a point cloud before saving it.', 0);
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return
        end
        
        % Save objects
        [nCode, oGUIData, oGUIObjects, oGUIHistory] = saveObjectLists(oGUIData, oGUIObjects, oGUIHistory);
        if nCode
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return
        end
        
        % Redraw point cloud and objects
        [oGUIData, oGUIObjects] = redrawGUI(oGUIData, oGUIObjects);
        
        % Set saved panel
        setSavedPanel(oSavedText_h, oSavedPanel_h, 'saved', 'gr');
        
        % Set updated data
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        setappdata(oGUI_h, 'GUIHistory', oGUIHistory);
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

% Save the current object list and proceed to next frame
    function saveAndProceed_Callback(~, ~, ~)
        enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        oGUIHistory = getappdata(oGUI_h, 'GUIHistory');
        setSavedPanel(oSavedText_h, oSavedPanel_h, 'busy', 'y');
        
        if ~oGUIData.m_bInit
            setInfoText('You have to load a point cloud before saving it.', 0);
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return
        end
        
        % Save objects
        [nCode, oGUIData, oGUIObjects, oGUIHistory] = saveObjectLists(oGUIData, oGUIObjects, oGUIHistory);
        if nCode
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return
        end
        
        % Proceed to next frame
        nSequenceID = oGUIData.m_nSequenceID;
        nPCID       = oGUIData.m_nPCID;
        if oGUIData.m_oPCMetadata.m_bIsLastOfSequence
            nSequenceID = nSequenceID + 1;
            setInfoText(oInfo_h, sprintf('End of sequence reached. Proceeding to ID %d.', nSequenceID), 1);
        end
        nPCID = nPCID + 1;
        % Check for empty prelabeling directory (no data available)
        if checkForEmptyDirectory(buildPath(oGUIData.m_sDatasetDir, nSequenceID, nPCID, 12))
            setInfoText(oInfo_h, 'Error: Could not find file. Check IDs and prelabeled directory.', 0);
            enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            return
        end
        
        oGUIData.m_nSequenceID  = nSequenceID;
        oGUIData.m_nPCID        = nPCID;
        oPCID_h.String          = num2str(nPCID);
        oSequenceID_h.String    = num2str(nSequenceID);
        
        bForward = 1;
        [oGUIData, oGUIObjects, oGUIHistory, sStatus] = loadToGUI(oGUIData, oGUIObjects, oGUIHistory, bForward);
        
        % Set saved panel
        if      strcmp(sStatus, 'saved')
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'saved', 'gr');
        elseif  strcmp(sStatus, 'unsaved')
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'unsaved', 'r')
        elseif  strcmp(sStatus, 'delta')
            setSavedPanel(oSavedText_h, oSavedPanel_h, 'delta', 'y')
        end
        
        % Set updated data
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        setappdata(oGUI_h, 'GUIHistory', oGUIHistory);
        
        enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
    end

%% Point cloud axes callback
    function pcAxes_Callback(hObject, ~, ~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        if ~oGUIData.m_bInit
            return
        end
        pt =  get(hObject, 'CurrentPoint'); % get marked point on axes
        
        % Prestate before edges definition
        bPrestateEvaluated = 0;
        if strcmp(oGUIData.m_sDefinitionState, 'zoomToPoint')
            x = pt(2,1);
            y = pt(2,2);
            oGUIData.m_sDefinitionState = 'edge'; % go in edge definition state
            % Zoom to desired point
            [oGUIData, oGUIObjects] = zoomToObject(oGUIData, oGUIObjects, oPCAxes_h, x, y);
            setInfoText(oInfo_h, 'Click axes to define four BB edge points!', 1);
            setInfoText(oInfo_h, 'Press [esc] to exit!', 1);
            % Enable edge and cam definition states
            oEdgeButton_h.Value = 1; oEdgeButton_h.Enable = 'on';
            oCamButton_h.Value  = 0; oCamButton_h.Enable  = 'on';
            bPrestateEvaluated = 1;
        end
        
        % New box definition
        if strcmp(oGUIData.m_sDefinitionState, 'edge') && ~bPrestateEvaluated
            if strcmp(get(oGUI_h, 'Selectiontype'), 'normal') % left click: determine middle position
                oGUIData.m_nNumEdgePoints   = oGUIData.m_nNumEdgePoints + 1;
                nNumEdgePoints              = oGUIData.m_nNumEdgePoints;
                % Set edge point
                oGUIData.m_mfEdgePoints(nNumEdgePoints, 1:2)   = pt(2,1:2);
                oGUIData.m_mfEdgePoints(nNumEdgePoints, 3)     = -1.8;
                mfEdgePoints = oGUIData.m_mfEdgePoints;
                % Plot edge point
                vnColor = [1 0 0];
                oGUIData.m_voEdgePoints_h(nNumEdgePoints,1) = plot3(mfEdgePoints(nNumEdgePoints,1), mfEdgePoints(nNumEdgePoints,2), ...
                    mfEdgePoints(nNumEdgePoints,3), 'o', 'MarkerEdgeColor', vnColor, 'MarkerFaceColor', vnColor, ...
                    'MarkerSize', 5, 'Parent', oPCAxes_h);
                setInfoText(oInfo_h, 'An edge point has been set!', 1);
                if(nNumEdgePoints == 4)
                    setInfoText(oInfo_h, 'Indicate the front by clicking next to an edge!', 1);
                end
                if(nNumEdgePoints == 5)
                    setInfoText(oInfo_h, 'Definition complete. Estimating rectangle.', 1);
                    [mfEdgePoints, bSuccess] = estimateRectangle(mfEdgePoints);
                    if(bSuccess == 1)
                        % New box is defined
                        % Determine box parameters, 5th edge point indicates front edge
                        [oMovableLabel, bSuccess] = calculateBoxParameters(mfEdgePoints);
                        % Set information for completeObjectData(...) during Relabel & complete or forward pass
                        oMovableLabel.m_bIsUserDefined = 1;
                        oMovableLabel.m_bCompleteObjectData = 1;
                        if(bSuccess == 1)
                            setInfoText(oInfo_h, '... done!', 1);
                            % Increase number of objects
                            oGUIData.m_nNumPCObjects = oGUIData.m_nNumPCObjects + 1;
                            % Draw rectangle
                            oMovableLabel.m_Box_h = drawRectangle(oPCAxes_h, 'r', oMovableLabel, -1.8);
                            % Assign track ID
                            oGUIData.m_oEditorConfig.m_LastWrittenTrackID   = oGUIData.m_oEditorConfig.m_LastWrittenTrackID + 1;
                            oMovableLabel.m_nTrackID                        = oGUIData.m_oEditorConfig.m_LastWrittenTrackID;
                            % Update reference point
                            bInit = 1;
                            oMovableLabel.updateReferencePoint(bInit);
                            % Add to object vector
                            oGUIObjects.m_voPCMovableLabel(oGUIData.m_nNumPCObjects,1) = oMovableLabel;
                            % Change GUI
                            oGUIData.m_nIndexCurrentObject = oGUIData.m_nNumPCObjects;
                            oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable(oMovableLabel);
                            oMetadataTable_h.Data       = fillMetadataTable(oGUIData.m_oPCMetadata);
                            setSavedPanel(oSavedText_h, oSavedPanel_h, 'edited', 'y');
                            % Proceed
                            oEdgeButton_h.Value = 0;
                            oEdgeButton_h.Enable = 'off';
                            oUndef_h = findobj(oGUIData.m_oClassesButtonGroup_h, 'String', 'Undefined');
                            oUndef_h.Value = 1;
                            enteringDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
                                oGUIData, oGUIObjects);
                            setInfoText(oInfo_h, 'Click right to zoom out again and to define another new box!', 1);
                        else
                            setInfoText(oInfo_h, 'Box could not be calculated. Aborting.', 1);
                        end
                    elseif bSuccess == -2
                        setInfoText(oInfo_h, '... failed. Could not correct rectangle to 90°.', 1);
                    elseif bSuccess == -3
                        setInfoText(oInfo_h, '... failed. You have to define points clock- or counter clockwise!', 1);
                    end
                    % Reset edge points
                    delete(oGUIData.m_voEdgePoints_h); drawnow;
                    oGUIData.m_nNumEdgePoints   = 0;
                    oGUIData.m_mfEdgePoints     = zeros(4,3);
                end
            end
        end
        
        % Position definition
        if strcmp(oGUIData.m_sDefinitionState, 'pos')
            nObj = oGUIData.m_nIndexCurrentObject;
            
            if (nObj > oGUIData.m_nNumPCObjects) || (oGUIData.m_nNumPCObjects == 0)
                return;
            end
            deltaXY = (pt(2,1:2) - [oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_x, ...
                oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_y])';
            oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_x = pt(2,1);
            oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_y = pt(2,2);
            transformBox(oGUIObjects.m_voPCMovableLabel(nObj), 0, deltaXY);
            markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h);
        end
        
        % Right click in new box mode
        if strcmp(oGUIData.m_sDefinitionMode, 'new') && ~strcmp(oGUIData.m_sDefinitionMode, 'none') && ...
                strcmp(get(oGUI_h, 'SelectionType'), 'alt')
            % Reset workspace
            [oGUIData, oGUIObjects] = resetBoxWorkspace(oGUIData, oGUIObjects);
            oGUIData = leavingDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
                oGUIData, oGUIObjects);
            % Restore state machine
            oGUIData.m_sDefinitionMode = 'new';
            oGUIData.m_sDefinitionState = 'zoomToPoint';
            % Ummark current box
            nObj = oGUIData.m_nIndexCurrentObject;
            if nObj ~= 0 && nObj <= oGUIData.m_nNumPCObjects
                unmarkBox(oGUIObjects.m_voPCMovableLabel(nObj));
            end
        end
        
        % Pick box mode
        if strcmp(oGUIData.m_sDefinitionMode, 'pick')
            % Left click: Pick closest box and enter definition state, not in definition state
            if strcmp(get(oGUI_h, 'SelectionType'), 'normal') && strcmp(oGUIData.m_sDefinitionState, 'none')
                if oGUIData.m_nNumPCObjects == 0
                    return
                end
                % Get closest PC object
                x = pt(2,1); y = pt(2,2);
                nObj = determineClosestBox(oGUIObjects.m_voPCMovableLabel(:), [x;y]);
                oGUIData.m_nIndexCurrentObject  = nObj;
                % Zoom to object and enter definition state
                x = oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_x;
                y = oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_y;
                [oGUIData, oGUIObjects] = zoomToObject(oGUIData, oGUIObjects, oPCAxes_h, x, y);
                oGUIData = enteringDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
                    oGUIData, oGUIObjects);
                setInfoText(oInfo_h, 'Press [t] to toggle to closest box!', 1);
                oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable(oGUIObjects.m_voPCMovableLabel(nObj));
                
                % Right click: Exit definition state (not cam)
            elseif strcmp(get(oGUI_h, 'SelectionType'), 'alt') && ~strcmp(oGUIData.m_sDefinitionState, 'none') && ...
                    ~strcmp(oGUIData.m_sDefinitionState, 'cam')
                oGUIData.m_bGUIZoomed = 0;
                camzoom(oPCAxes_h, 1/oGUIData.m_fZoomFactor);
                setAxesSettings(oPCAxes_h, oGUIData.m_sView);
                setInfoText(oInfo_h, 'Entering pick box mode. Click axes to zoom to desired box!', 0);
                setInfoText(oInfo_h, 'Press [esc] to exit!', 1);
                % Ummark current box
                nObj = oGUIData.m_nIndexCurrentObject;
                if nObj ~= 0 && nObj <= oGUIData.m_nNumPCObjects
                    unmarkBox(oGUIObjects.m_voPCMovableLabel(nObj));
                end
                % Restore state machine
                oGUIData.m_sDefinitionState = 'none';
                oGUIData = leavingDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
                    oGUIData, oGUIObjects);
                oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable();
            end
        end
        
        % Camera control. Left click: Set Camera target.
        if ~strcmp(get(oGUI_h, 'SelectionType'), 'alt') && strcmp(oGUIData.m_sDefinitionState, 'none')
            oGUIData.m_vfCameraTarget = [pt(2,1:2), 1];
            oGUIData.m_bCameraTargetSet = 1;
        end
        
        % Set updated GUIData and GUIObjects
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
    end

%% Image callbacks

    function image_Callback(hObject, ~, ~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        
        sImageType  = get(gca, 'UserData');
        oCalib      = oGUIData.getCalibration(sImageType);
        nImgIdx     = oGUIObjects.getImageIndex(sImageType);   % Development
        
        if ~oGUIData.m_bInit
            return
        end
        
        % Get marked point on axes
        pt =  get(hObject, 'CurrentPoint');
        x = pt(2,1);
        y = pt(2,2);
        
        % Rays of sight
        if strcmp(oGUIData.m_sImageContext, 'Ray projection') && ~oGUIData.m_bCameraCalibrationMode
            % Left click: Draw ray
            if strcmp(get(oImages_h, 'SelectionType'), 'normal')
                mfMap_x = oGUIObjects.m_voImageData(nImgIdx,1).m_mfMap_x;
                mfMap_y = oGUIObjects.m_voImageData(nImgIdx,1).m_mfMap_y;
                [oRay_h, oPix_h] = drawRay(x, y, mfMap_x, mfMap_y, gca, oPCAxes_h, oCalib);
                oEntry = {oRay_h, oPix_h};
                nCtr = oGUIObjects.m_voImageData(nImgIdx).m_nRaysCtr;
                nCtr = nCtr + 1;
                oGUIObjects.m_voImageData(nImgIdx).m_nRaysCtr = nCtr;
                oGUIObjects.m_voImageData(nImgIdx).m_clRays{nCtr,1} = oEntry;
                
                % Right click: delete ray
            else
                nCtr = oGUIObjects.m_voImageData(nImgIdx).m_nRaysCtr;
                if nCtr > 0
                    oEntry = oGUIObjects.m_voImageData(nImgIdx).m_clRays{nCtr,1};
                    nCtr = nCtr - 1;
                    oGUIObjects.m_voImageData(nImgIdx).m_nRaysCtr = nCtr;
                    delete(oEntry{1,1});
                    delete(oEntry{1,2});
                end
            end
            drawnow;
        end
        
        % Camera calibration
        if oGUIData.m_bCameraCalibrationMode
            scatter(x, y, 50, [1 0 0], 'o', 'Parent', gca);
            nNumTargets = oGUIData.m_nNumTargets;   % number of objects to be defined
            
            % Init container
            if isempty(oGUIData.m_clPointContainer)
                for i = 1 : nNumTargets
                    mfPoints = zeros(4,2);
                    nPoints = 0;
                    if i == 1
                        mfPoints(1,:) = [x, y];
                        nPoints = 1;
                    end
                    oGUIData.m_clPointContainer{i,1} = mfPoints;    % points
                    oGUIData.m_clPointContainer{i,2} = nPoints;     % counter
                end
                oGUIData.m_nObj = 1;
                return;
            end
            
            % Fill container
            nObj = oGUIData.m_nObj;
            mfPoints = oGUIData.m_clPointContainer{nObj,1};
            nPoints = oGUIData.m_clPointContainer{nObj,2};
            
            nPoints = nPoints + 1;
            mfPoints(nPoints,:) = [x, y];
            
            oGUIData.m_clPointContainer{nObj,1} = mfPoints;    % points
            oGUIData.m_clPointContainer{nObj,2} = nPoints;     % counter
            
            % Calibrate
            if (nObj == nNumTargets) && (nPoints == 4)
                setInfoText(oInfo_h, 'Saving point container for calibration.', 1);
                clPointContainer = oGUIData.m_clPointContainer;
                voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
                oAxes_h = gca;
                save('Calibration_Container.mat', 'clPointContainer', 'voPCMovableLabel', 'oCalib', 'oAxes_h');
                % calibrateCamera(oGUIData.m_clPointContainer, oGUIObjects.m_voPCMovableLabel, oCalib, gca);
                return;
            end
            
            % Check for object increment in case four points are defined
            if nPoints == 4
                nObj = nObj + 1;
            end
            
            oGUIData.m_nObj = nObj;
        end
    end

    function imagePanelContextMenu_Callback(~, evnt)
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        vfPosition  = get(gca, 'CurrentPoint');
        sImageType  = get(gca, 'UserData');
        nImageIdx   = oGUIObjects.getImageIndex(sImageType);
        
        if evnt.Source.Position == 1    % license plate
            sClass      = 'Licence plate';
            vfPosition  = [vfPosition(1,1)-30, vfPosition(1,2)-5, 200, 80];  % x, y, width, height
        else
            sClass      = 'Face';
            vfPosition  = [vfPosition(1,1)-10, vfPosition(1,2)-10, 100, 100];
        end
        
        oImageData  = oGUIObjects.m_voImageData(nImageIdx,1);
        oImageLabel = createImageLabel([], gca, 'Rectangle', sClass, vfPosition, oImageData, [0 0 0]);
        
        if isempty(oGUIObjects.m_voImageData(nImageIdx,1).m_voImageLabels)
            oGUIObjects.m_voImageData(nImageIdx,1).m_voImageLabels(1,1) = oImageLabel;
        else
            oGUIObjects.m_voImageData(nImageIdx,1).m_voImageLabels(end+1,1) = oImageLabel;
        end
        
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
    end

% This function implements draggable 3D-Shapes within the image plane
    function imagePanelMouseMove_Callback(~,~)
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        if isempty(oGUIObjects) || isempty(oGUIObjects.m_voImageData)
            return
        end
        
        for i = 1:4
            if ~isempty(oGUIObjects.m_voImageData(i,1).m_voImageLabels)
                for j = 1:size(oGUIObjects.m_voImageData(i,1).m_voImageLabels, 1)
                    if oGUIObjects.m_voImageData(i,1).m_voImageLabels(j,1).isClicked()
                        oGUIObjects.m_voImageData(i,1).m_voImageLabels(j,1).move();
                    end
                end
            end
        end
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
    end

% This function implements draggable 3D-Shapes within the image plane
    function imagePanelMouseRelease_Callback(~,~)
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        if isempty(oGUIObjects) || isempty(oGUIObjects.m_voImageData)
            return
        end
        
        for i = 1:4
            if ~isempty(oGUIObjects.m_voImageData(i,1).m_voImageLabels)
                for j = 1:size(oGUIObjects.m_voImageData(i,1).m_voImageLabels, 1)
                    oGUIObjects.m_voImageData(i,1).m_voImageLabels(j,1).mouseRelease();
                end
            end
        end
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
    end

    function projectionGroup_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        oGUIData.m_sProjectionMode = oProjectionButtonGroup_h.SelectedObject.String;
        
        switch oGUIData.m_sProjectionMode
            case '3D'
                setInfoText(oInfo_h, '3D shapes will be projected.', 0);
                setInfoText(oInfo_h, 'Drag and drop 3D shapes at edges or corners to correct.', 1);
            case 'Polygon'
                setInfoText(oInfo_h, 'Polygon shapes will be projected.', 0);
                setInfoText(oInfo_h, 'Drag and drop vertices.', 1);
                setInfoText(oInfo_h, 'Press and hold [a] and click an edge to add a vertex', 1);
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

    function imageContextGroup_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        oGUIData.m_sImageContext = oImageContextButtonGroup_h.SelectedObject.String;
        
        % Add context menu to image
        if strcmp(oGUIData.m_sImageContext, oImageObjectDefinitionButton_h.String)
            oGUIData = addImagePanelContextMenu(oGUIData);
        else
            % Remove callback
            for i = 1 : 4
                if isgraphics(oGUIData.m_voImageAxes_h(i,1))
                    oGUIData.m_voImageAxes_h(i,1).UIContextMenu = [];
                end
            end
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Project point cloud objects into the image planes
    function projectObjectsIntoImages_Callback(~,~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        if getappdata(oGUI_h, 'busy')
            return
        end
        
        if ~oGUIData.m_bInit
            setInfoText(oInfo_h, 'Initialize GUI using "load" first!', 0);
            return;
        end
        
        voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
        if isempty(voPCMovableLabel)
            setInfoText(oInfo_h, 'No objects in point cloud to project.', 1);
            return;
        end
        
        % Projection mode changed. All projected objects will be deleted. Check up in dialog.
        if ~isempty(oGUIData.m_sProjectionMode_prev)
            % Change in projection mode
            if ~strcmp(oGUIData.m_sProjectionMode_prev, oGUIData.m_sProjectionMode)
                sName = buttondlg(sprintf('Projection mode changed.\nAll image labels will be deleted and reprojected.'), ...
                    'Confirm', 'Reproject', 'Abort', struct('Default', 'Reproject', 'IconString', 'warn'));
                if strcmp(sName, 'Abort')
                    return
                end
                
                % Delete projected objects
                for i = 1 : 4
                    voImageLabels = oGUIObjects.m_voImageData(i,1).m_voImageLabels;
                    vbKeep = true(size(voImageLabels,1) ,1);
                    for j = 1 : size(voImageLabels, 1)
                        if voImageLabels(j,1).m_bIsUserDefined
                            continue
                        end
                        oPoly_h = voImageLabels(j,1).m_oPoly_h;
                        % Delete children, not hggroup. They are linked by the callbacks (deleting hggroup deletes label object)
                        if ~isempty(oPoly_h)
                            delete(oPoly_h.Children)
                        end
                        vbKeep(j,1) = false;
                        
                        % Unset projection flag
                        if voImageLabels(j,1).m_nIdxPCObject > 0
                            oGUIObjects.m_voPCMovableLabel(voImageLabels(j,1).m_nIdxPCObject,1).m_bIsProjected = 0;
                        end
                    end
                    oGUIObjects.m_voImageData(i,1).m_voImageLabels = voImageLabels(vbKeep);
                end
            end
        end
        
        % Delete projected objects
        for i = 1 : 4
            voImageLabels = oGUIObjects.m_voImageData(i,1).m_voImageLabels;
            vbKeep = true(size(voImageLabels,1) ,1);
            for j = 1 : size(voImageLabels, 1)
                if voImageLabels(j,1).m_bIsProjected
                    vbKeep(j,1) = false;
                    delete(voImageLabels(j,1).m_oPoly_h.Children)
                end
            end
            oGUIObjects.m_voImageData(i,1).m_voImageLabels = voImageLabels(vbKeep);
        end
        
        % Unset deletion flag
        for i = 1 : size(voPCMovableLabel,1)
            voPCMovableLabel(i,1).m_bShapeDeleted = 0;
            voPCMovableLabel(i,1).m_bIsProjected = 0;
        end
        
        % Project according to selected projection mode
        oGUIData.m_sProjectionMode_prev = oGUIData.m_sProjectionMode;
        sShapeType = oProjectionButtonGroup_h.SelectedObject.String;
        oGUIObjects = projectPCObjects(oGUIData, oGUIObjects, sShapeType, oGUIData.m_bRandomColor);
        drawnow;
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

%% Scroll callback

% Use mouse wheel to change object
    function doScroll(~, eventdata, ~)
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;
        if ~oGUIData.m_bInit
            return;
        end
        if oGUIData.m_bInDefinitionState
            nObj                = oGUIData.m_nIndexCurrentObject;
            nNumGUIObjects      = oGUIData.m_nNumPCObjects;
            sDefState           = oGUIData.m_sDefinitionState;
            if (nObj > nNumGUIObjects) || (nNumGUIObjects == 0)
                return;
            end
            
            % Change an object's box
            oBox  = voPCMovableLabel(nObj,1);
            nSign = 1;
            if strcmp(sDefState, 'length')
                if(eventdata.VerticalScrollCount > 0 && (oBox.m_fBBLength >= oGUIData.m_fDeltaLW))
                    oBox.m_fBBLength = oBox.m_fBBLength - oGUIData.m_fDeltaLW;
                    nSign = -1;
                else
                    oBox.m_fBBLength = oBox.m_fBBLength + oGUIData.m_fDeltaLW;
                end
                updateLength(oBox, oGUIData.m_fDeltaLW*nSign);
            end
            if strcmp(sDefState, 'width')
                if(eventdata.VerticalScrollCount > 0 && (oBox.m_fBBWidth >= oGUIData.m_fDeltaLW))
                    oBox.m_fBBWidth = oBox.m_fBBWidth - oGUIData.m_fDeltaLW;
                    nSign = -1;
                else
                    oBox.m_fBBWidth = oBox.m_fBBWidth + oGUIData.m_fDeltaLW;
                end
                updateWidth(oBox, oGUIData.m_fDeltaLW*nSign);
            end
            if strcmp(sDefState, 'yaw')
                if(eventdata.VerticalScrollCount > 0)
                    oBox.m_fBBYaw = oBox.m_fBBYaw - oGUIData.m_fDeltaYaw;
                    nSign = -1;
                else
                    oBox.m_fBBYaw = oBox.m_fBBYaw + oGUIData.m_fDeltaYaw;
                end
                if(oBox.m_fBBYaw > 360) % Correct yaw
                    oBox.m_fBBYaw = oBox.m_fBBYaw - 360;
                elseif(oBox.m_fBBYaw < 0)
                    oBox.m_fBBYaw = oBox.m_fBBYaw + 360;
                end
                transformBox(oBox, oGUIData.m_fDeltaYaw*nSign, [0;0]);
            end
            % Mark box as corrected if not in definition state
            if ~strcmp(sDefState, 'cam')
                markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h);
            end
            
            oGUIObjects.m_voPCMovableLabel = voPCMovableLabel;
            oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable(oBox);
        end
        
        % Advanced camera handling
        if ~oGUIData.m_bInDefinitionState || (oGUIData.m_bInDefinitionState && strcmp(sDefState, 'cam'))
            oGUIData.m_bGUIZoomed = 1;
            fZoomFactor = 0.1;
            if(eventdata.VerticalScrollCount < 0)
                camzoom(oPCAxes_h, 1 + fZoomFactor);
            else
                camzoom(oPCAxes_h, 1 - fZoomFactor);
            end
        end
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
        setappdata(oGUI_h, 'GUIData',    oGUIData);
    end

%% Keyboard callback

% Different GUI states are differentiated within the keyboard callback
    function keyboard_Callback(hObject, ~, ~)
        if getappdata(oGUI_h, 'busy')
            return;
        end
        oGUIData    = getappdata(oGUI_h, 'GUIData');
        oGUIObjects = getappdata(oGUI_h, 'GUIObjects');
        nObj        = oGUIData.m_nIndexCurrentObject;
        key         = get(hObject, 'CurrentCharacter');
        
        if isempty(key)
            return
        end
        
        % Check for any capital letters
        if regexp(key,'\<[A-Z]\>')
            setInfoText(oInfo_h, 'Warning: CAPS LOCK may be active. Please check and reset.', 0);
        end
        
        % Definition state
        if  oGUIData.m_bInDefinitionState
            % Definition: Move definition state left using q or e
            if key == 'q'
                if      strcmp(oGUIData.m_sDefinitionState, 'yaw')
                    oGUIData.m_sDefinitionState = 'cam';
                    oCamButton_h.Value = 1;
                elseif  strcmp(oGUIData.m_sDefinitionState, 'cam')
                    % Position button disabled in edges definition
                    if strcmp(oPosButton_h.Enable, 'on')
                        oGUIData.m_sDefinitionState = 'pos';
                        oPosButton_h.Value = 1;
                    else
                        oGUIData.m_sDefinitionState = 'edge';
                        oEdgeButton_h.Value = 1;
                    end
                elseif strcmp(oGUIData.m_sDefinitionState, 'pos')
                    oGUIData.m_sDefinitionState = 'length';
                    oLengthButton_h.Value = 1;
                elseif strcmp(oGUIData.m_sDefinitionState, 'length')
                    oGUIData.m_sDefinitionState = 'width';
                    oWidthButton_h.Value = 1;
                elseif strcmp(oGUIData.m_sDefinitionState, 'width')
                    oGUIData.m_sDefinitionState = 'yaw';
                    oYawButton_h.Value = 1;
                elseif strcmp(oGUIData.m_sDefinitionState, 'edge')
                    oGUIData.m_sDefinitionState = 'cam';
                    oCamButton_h.Value = 1;
                end
                % Definition: Move definition state right
            elseif key == 'e'
                if      strcmp(oGUIData.m_sDefinitionState, 'pos')
                    oGUIData.m_sDefinitionState = 'cam';
                    oCamButton_h.Value = 1;
                elseif	strcmp(oGUIData.m_sDefinitionState, 'cam')
                    % Yaw disabled if edges definition
                    if strcmp(oYawButton_h.Enable, 'on')
                        oGUIData.m_sDefinitionState = 'yaw';
                        oYawButton_h.Value = 1;
                    else
                        oGUIData.m_sDefinitionState = 'edge';
                        oEdgeButton_h.Value = 1;
                    end
                elseif  strcmp(oGUIData.m_sDefinitionState, 'yaw')
                    oGUIData.m_sDefinitionState = 'width';
                    oWidthButton_h.Value = 1;
                elseif  strcmp(oGUIData.m_sDefinitionState, 'width')
                    oGUIData.m_sDefinitionState = 'length';
                    oLengthButton_h.Value = 1;
                elseif	strcmp(oGUIData.m_sDefinitionState, 'length')
                    oGUIData.m_sDefinitionState = 'pos';
                    oPosButton_h.Value = 1;
                elseif strcmp(oGUIData.m_sDefinitionState, 'edge')
                    oGUIData.m_sDefinitionState = 'cam';
                    oCamButton_h.Value = 1;
                end
            end
            
            % Definition: position using w, a, s, d
            if strcmp(sprintf('%s', key), 'w') || strcmp(sprintf('%s', key), 'a') || strcmp(sprintf('%s', key), 's') ...
                    || strcmp(sprintf('%s', key), 'd')
                fDeltaXY = oGUIData.m_fDeltaXY;
                if      (key == 'a') % left
                    fDeltaXY = fDeltaXY .* [-1; 0];
                elseif  (key == 'w') % up
                    fDeltaXY = fDeltaXY .* [0; 1];
                elseif  (key == 'd') % right
                    fDeltaXY = fDeltaXY .* [1; 0];
                elseif  (key == 's') % down
                    fDeltaXY = fDeltaXY .* [0; -1];
                end
                
                if (nObj > oGUIData.m_nNumPCObjects) || (nObj == 0)
                    return;
                end
                oObj = oGUIObjects.m_voPCMovableLabel(nObj);
                oObj.m_fBBMiddle_x = oObj.m_fBBMiddle_x + fDeltaXY(1,1);
                oObj.m_fBBMiddle_y = oObj.m_fBBMiddle_y + fDeltaXY(2,1);
                transformBox(oObj, 0, fDeltaXY);
                markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h);
            end
            
            % Definition: Enforce height recalculation during completeObjectData(...)
            if strcmp(sprintf('%s',key), 'h')
                oGUIObjects.m_voPCMovableLabel(nObj,1).m_bRecalcHeight = 1;
                markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h);
                setInfoText(oInfo_h, sprintf('Info %s %d: Height will be recalculated.', ...
                    oGUIObjects.m_voPCMovableLabel(nObj).m_sClassification, nObj), 1);
            end
            
            % Definition: Delete a box by pressing del (ENG keyboard) or entf (GER keyboard)
            nObj = oGUIData.m_nIndexCurrentObject;  % index of box to be deleted
            if (nObj > oGUIData.m_nNumPCObjects) || (nObj == 0)
                return;
            end
            
            % Delete using key 127
            if key == 127 % entf (GER-Layout)
                enableDisableFig(oGUI_h, 'off'); setappdata(oGUI_h, 'busy', 1);
                if ~oGUIData.m_bConfirmBoxDeletion
                    oGUIData.m_bConfirmBoxDeletion = 1;
                    setInfoText(oInfo_h, 'Confirm deletion by pressing [entf/del] again.', 1);
                else
                    % Delete box
                    [oGUIData, oGUIObjects] = deleteBox(oGUIData, oGUIObjects, nObj);
                    % Update GUI information tables
                    oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable();
                    oMetadataTable_h.Data       = fillMetadataTable(oGUIData.m_oPCMetadata);
                    [oGUIData, oGUIObjects]     = resetBoxWorkspace(oGUIData, oGUIObjects);
                    oGUIData = leavingDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
                        oGUIData, oGUIObjects);
                    oGUIData.m_bConfirmBoxDeletion = 0;
                    setInfoText(oInfo_h, 'Box deleted.', 1);
                    
                    % Restore previous mode
                    if  oCorrectionButton_h.Value == 1
                        oGUIData.m_sDefinitionMode = 'correction';
                    elseif  oNewBoxButton_h.Value == 1
                        oGUIData.m_sDefinitionMode = 'new';
                        oGUIData.m_sDefinitionState = 'zoomToPoint';
                    elseif  oPickBoxButton_h.Value == 1
                        oGUIData.m_sDefinitionMode = 'pick';
                    end
                    setSavedPanel(oSavedText_h, oSavedPanel_h, 'edited', 'y');
                end
                enableDisableFig(oGUI_h, 'on'); setappdata(oGUI_h, 'busy', 0);
            end
            
            % Definition: Class selection using hot keys
            nKey = str2double(key);
            if (nKey > 0) && (nKey <= 9)
                voClasses_h = oGUIData.m_oClassesButtonGroup_h.Children;
                for i = 1 : size(voClasses_h,1)
                    if ~isempty(voClasses_h(i,1).Tag)
                        if strcmp(voClasses_h(i,1).Tag, key)
                            oGUIObjects.m_voPCMovableLabel(nObj).m_sClassification = voClasses_h(i,1).String;
                            voClasses_h(i,1).Value = 1;
                            break;
                        end
                    end
                end
                markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h);
            end
            
            % Definition: Toggle active flag
            if key == 'f'
                oGUIObjects.m_voPCMovableLabel(nObj).m_bCompleteObjectData = 1;     % checked in completeObjectData(...)
                if oGUIObjects.m_voPCMovableLabel(nObj).m_bIsActive
                    oGUIObjects.m_voPCMovableLabel(nObj).m_bIsActive = 0;
                else
                    oGUIObjects.m_voPCMovableLabel(nObj).m_bIsActive = 1;
                end
                % markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h);
                fillObjectListTable(oGUIData, oGUIObjects);
            end
            
            % Definition: Reinit kalman filter variances (dynamics) using 'u'
            if key == 'u'
                % Reinit kalman filters
                oGUIObjects.m_voPCMovableLabel(nObj).m_oKalman.reset()
                setInfoText(oInfo_h, sprintf('Info %s %d: Reinitializing Kalman Filter.', ...
                    oGUIObjects.m_voPCMovableLabel(nObj).m_sClassification, nObj), 1);
                oGUIObjects.m_voPCMovableLabel(nObj).reassignData();
                markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h);
            end
            
            % Definition: Mark current object as corrected using 'x'
            if key == 'x' && ~strcmp(oGUIData.m_sDefinitionMode, 'new')
                markCurrentBoxAsCorrected(oGUIData, oGUIObjects, oPCAxes_h);
            end
        end
        
        % Pick box mode, toggeling with [t]
        if strcmp(oGUIData.m_sDefinitionMode, 'pick') && oGUIData.m_bInDefinitionState
            if strcmp(sprintf('%s',key), 't')
                x = oGUIObjects.m_voPCMovableLabel(nObj,1).m_fBBMiddle_x;
                y = oGUIObjects.m_voPCMovableLabel(nObj,1).m_fBBMiddle_y;
                % Sort out current box from vector
                select          = true(size(oGUIObjects.m_voPCMovableLabel,1),1);
                select(nObj,1)  = 0;
                vObjects = oGUIObjects.m_voPCMovableLabel(select,:);
                nNextObj = determineClosestBox(vObjects, [x;y]);
                if(nNextObj >= nObj)
                    nNextObj = nNextObj + 1;
                end
                
                % Zoom to new object
                x = oGUIObjects.m_voPCMovableLabel(nNextObj).m_fBBMiddle_x;
                y = oGUIObjects.m_voPCMovableLabel(nNextObj).m_fBBMiddle_y;
                oGUIData.m_nIndexCurrentObject = nNextObj;
                zoomToObject(oGUIData, oGUIObjects, oPCAxes_h, x, y);
                oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable(oGUIObjects.m_voPCMovableLabel(nNextObj));
                % Unmark former object
                unmarkBox(oGUIObjects.m_voPCMovableLabel(nObj));
            end
        end
        
        % Correction mode, toggle between objects using t
        if strcmp(oGUIData.m_sDefinitionMode, 'correction')
            % Toggle, 30 equals up arrow on keyboard, 31 equals down arrow
            if (strcmp(sprintf('%s', key), 't') || strcmp(sprintf('%d', key), '31') || strcmp(sprintf('%d', key), '30'))
                
                % Unmark current object
                if (nObj > 0 && nObj <= oGUIData.m_nNumPCObjects)
                    unmarkBox(oGUIObjects.m_voPCMovableLabel(nObj));
                end
                
                bProceed = 1;
                % Toggle to next box whose predicted flag is not set (uncertain prediction)
                if strcmp(sprintf('%s', key), 't')
                    
                    % Rely on the editors self assessment
                    if mod(oGUIData.m_nConsecutiveFrames, oGUIData.m_nGlobalCorrIntervall) ~= 0
                        % Get next uncertain object (if any)
                        nObj_next = oGUIObjects.getNextUncertainObject(nObj);
                        
                        % No object found: done with correction
                        if nObj_next == 0
                            bProceed = 0;
                        end
                        nObj = nObj_next;
                        % Global correction: Proceed through the object vector
                    else
                        bAllCorrected = oGUIObjects.checkForUncorrectedObjects();
                        if bAllCorrected
                            bProceed = 0;
                        end
                        if (nObj < oGUIData.m_nNumPCObjects)
                            nObj = nObj + 1;
                        else
                            nObj = 1;
                        end
                    end
                    % Up with arrows on keyboard
                elseif (key == 30)
                    if nObj > 1
                        nObj = nObj - 1;
                    else
                        nObj = oGUIData.m_nNumPCObjects;
                    end
                    % Down
                elseif (key == 31)
                    if nObj < oGUIData.m_nNumPCObjects
                        nObj = nObj + 1;
                    else
                        nObj = 1;
                    end
                end
                % In case there is no object:
                if (nObj > oGUIData.m_nNumPCObjects)
                    return;
                end
                
                if bProceed
                    oGUIData.m_nIndexCurrentObject = nObj;
                    % Enter definition state
                    oGUIData = enteringDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
                        oGUIData, oGUIObjects);
                    setInfoText(oInfo_h, 'Press [up] or [down] to move through boxes!', 1);
                    setInfoText(oInfo_h, 'Press [t] to proceed to next box!',    1);
                    setInfoText(oInfo_h, 'Press [space] to restore view!',       1);
                    % Zoom to corresponding point
                    x = oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_x;
                    y = oGUIObjects.m_voPCMovableLabel(nObj).m_fBBMiddle_y;
                    [oGUIData, oGUIObjects] = zoomToObject(oGUIData, oGUIObjects, oPCAxes_h, x, y);
                    oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable(oGUIObjects.m_voPCMovableLabel(nObj));
                else
                    setInfoText(oInfo_h, '********** CORRECTION MODE: DONE. **********', 0);
                    setInfoText(oInfo_h, 'Define new objects or proceed.', 1);
                    setInfoText(oInfo_h, 'Correction mode left.', 1);
                    [oGUIData, oGUIObjects] = resetBoxWorkspace(oGUIData, oGUIObjects);
                    oGUIData = leavingDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
                        oGUIData, oGUIObjects);
                    oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable();
                    oCorrectionButton_h.Value = 0;
                end
            end
        end
        
        % Exit editing modes by pressing ESC (27)
        if ~strcmp(oGUIData.m_sDefinitionMode, 'none')
            if key == 27
                if oGUIData.m_bGUIZoomed
                    camzoom(oPCAxes_h, 1/oGUIData.m_fZoomFactor);
                    setAxesSettings(oPCAxes_h, oGUIData.m_sView);
                    oGUIData.m_bGUIZoomed = 0;
                end
                setInfoText(oInfo_h, 'Mode left.', 0);
                oNewBoxButton_h.Value = 0;   oCorrectionButton_h.Value = 0; oPickBoxButton_h.Value = 0;
                oPosButton_h.Value    = 0;   oEdgeButton_h.Value       = 0; oLengthButton_h.Value  = 0;
                oYawButton_h.Value    = 0;   oWidthButton_h.Value      = 0;
                [oGUIData, oGUIObjects] = resetBoxWorkspace(oGUIData, oGUIObjects);
                oGUIData = leavingDefinitionState(oDefinitionButtonGroup_h, oGUIData.m_oClassesButtonGroup_h, oViewButtonGroup_h, ...
                    oGUIData, oGUIObjects);
                oCurrentBoxInfoTable_h.Data = fillCurrentBoxInfoTable();
            end
        end
        
        % Restore view by using space (32)
        keyStr = sprintf('%d', key);
        if oGUIData.m_bInDefinitionState && strcmp(keyStr, '32')
            restoreAxesButton_Callback();
        end
        
        % Set updated data
        setappdata(oGUI_h, 'GUIData', oGUIData);
        setappdata(oGUI_h, 'GUIObjects', oGUIObjects);
    end

%% Menu callbacks

% Enable core prediction functionality
    function enablePredictionMenu_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bPredict
            oEnablePredictionMenu_h.Checked = 'off';
        else
            oEnablePredictionMenu_h.Checked = 'on';
        end
        oGUIData.m_bPredict = ~oGUIData.m_bPredict;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Look for new point cloud objects within the objects lists from prelabeling
    function checkForNewTracksMenu_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bAutoImport
            oAutoImportMenu_h.Checked = 'off';
        else
            oAutoImportMenu_h.Checked = 'on';
        end
        oGUIData.m_bAutoImport = ~oGUIData.m_bAutoImport;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Prefer to load object lists from the 'edited' directory first.
    function loadEditedMenu_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bInitFromEditedPC
            oInitEditedMenu_h.Checked = 'off';
        else
            oInitEditedMenu_h.Checked = 'on';
        end
        oGUIData.m_bInitFromEditedPC = ~oGUIData.m_bInitFromEditedPC;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Enable automatic deletion of point cloud objects (depending on the number of points and range)
    function autoDeleteMenu_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bAutoDelete
            oAutoDeleteMenu_h.Checked = 'off';
        else
            oAutoDeleteMenu_h.Checked = 'on';
        end
        oGUIData.m_bAutoDelete = ~oGUIData.m_bAutoDelete;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Enable debug information within info panel
    function enableDebugInformationMenu_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bEnableDebugInformation
            oEnableDebugInformationMenu_h.Checked = 'off';
        else
            oEnableDebugInformationMenu_h.Checked = 'on';
        end
        oGUIData.m_bEnableDebugInformation = ~oGUIData.m_bEnableDebugInformation;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Enable debug information within info panel
    function displayObjectDataMenu_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bDisplayObjectData
            oDisplayObjectDataMenu_h.Checked = 'off';
        else
            oDisplayObjectDataMenu_h.Checked = 'on';
        end
        oGUIData.m_bDisplayObjectData = ~oGUIData.m_bDisplayObjectData;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Enable ICP registration for optimization
    function enableRegistrationMenu_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bEnableRegistration
            oEnableRegistrationMenu_h.Checked = 'off';
        else
            oEnableRegistrationMenu_h.Checked = 'on';
        end
        oGUIData.m_bEnableRegistration = ~oGUIData.m_bEnableRegistration;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Enable fitting algorithms (ellipse and rectangle)
    function enableFittingMenu_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bEnableFitting
            oEnableFittingMenu_h.Checked = 'off';
        else
            oEnableFittingMenu_h.Checked = 'on';
        end
        oGUIData.m_bEnableFitting = ~oGUIData.m_bEnableFitting;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Enable edge point estimation algorithm (part of fitting algorithms)
    function enableEdgesMenu_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bEnableEdgesEstimation
            oEnableEdgesEstimationMenu_h.Checked = 'off';
        else
            oEnableEdgesEstimationMenu_h.Checked = 'on';
        end
        oGUIData.m_bEnableEdgesEstimation = ~oGUIData.m_bEnableEdgesEstimation;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Enable optimization debug plots
    function enableOptimizationDebugPlots_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if oGUIData.m_bEnableOptimizationPlots
            oEnableOptimizationPlotsMenu_h.Checked = 'off';
        else
            oEnableOptimizationPlotsMenu_h.Checked = 'on';
        end
        oGUIData.m_bEnableOptimizationPlots = ~oGUIData.m_bEnableOptimizationPlots;
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

%% Advanced camera handling

% Switch camera mode by left or right click
    function windowButtonDown_Callback(hObject ,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if ~oGUIData.m_bInit
            return
        end
        oGUIData.m_vfPrevPoint = hObject.CurrentPoint';
        % Set mouse flags
        if strcmp(get(oGUI_h, 'SelectionType'), 'alt')
            oGUIData.m_bRightMouseDown = 1;
        else
            oGUIData.m_bLeftMouseDown = 1;
        end
        % Determine mode
        if      oGUIData.m_bRightMouseDown
            oGUIData.m_bEnableCameraMov = 1; % dolly
            oGUIData.m_bEnableCameraRot = 0;
        elseif	oGUIData.m_bLeftMouseDown
            oGUIData.m_bEnableCameraMov = 0;
            oGUIData.m_bEnableCameraRot = 1; % rotation
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Release mouse buttons
    function windowButtonUp_Callback(~,~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if ~oGUIData.m_bInit
            return
        end
        if strcmp(get(oGUI_h, 'SelectionType'), 'alt')
            oGUIData.m_bRightMouseDown = 0;
        else
            oGUIData.m_bLeftMouseDown = 0;
            oGUIData.m_bCameraTargetSet = 0;
        end
        
        % Deactivate modes
        if ~oGUIData.m_bRightMouseDown
            oGUIData.m_bEnableCameraMov = 0;
        end
        if ~oGUIData.m_bLeftMouseDown
            oGUIData.m_bEnableCameraRot = 0;
        end
        setappdata(oGUI_h, 'GUIData', oGUIData);
    end

% Evaluate mouse movement
    function windowButtonMotion_Callback(hObject, ~)
        oGUIData = getappdata(oGUI_h, 'GUIData');
        if ~oGUIData.m_bInit || (~oGUIData.m_bRightMouseDown && ~oGUIData.m_bLeftMouseDown)
            return
        end
        pt = get(hObject, 'CurrentPoint');
        % Check if location is within axes
        minx    = oPCPanel_h.Position(1);        miny = oPCPanel_h.Position(2);
        maxx    = minx + oPCPanel_h.Position(3); maxy = miny + oPCPanel_h.Position(4);
        hObject.Units = 'norm';
        x = pt(1,1); y = pt(1,2);
        % Point is within boundaries
        if x >= minx && x <= maxx && y >= miny && y <= maxy
            prevPoint = oGUIData.m_vfPrevPoint;
            xDiff = x - prevPoint(1,1);
            yDiff = y - prevPoint(2,1);
            oGUIData.m_vfPrevPoint = pt';
            
            % Rotation
            if (oGUIData.m_bEnableCameraRot) && ~(strcmp(oGUIData.m_sDefinitionState, 'pos') ...
                    || strcmp(oGUIData.m_sDefinitionState, 'edge'))
                % 0.5 frame = 180 deg
                norm = 1; map = 200;
                dPhi   = -xDiff/norm*map;
                dTheta = -yDiff/norm*map;
                camorbit(oPCAxes_h, dPhi, dTheta, 'direction');
                
                % Movement
            elseif oGUIData.m_bEnableCameraMov && ~(strcmp(oGUIData.m_sDefinitionState, 'pos') ...
                    || strcmp(oGUIData.m_sDefinitionState, 'edge'))
                scale = 200;
                if oGUIData.m_bInDefinitionState
                    scale = 50;
                end
                % Map xDiff, yDiff to camera perspective. yDiff is movement along camUpVector (planely).
                % xDiff is movement perpendicular to this vector
                camUpVec = oPCAxes_h.CameraUpVector;
                dMov_yDiff = camUpVec .* (yDiff * -scale);
                dMov_xDiff = [camUpVec(1,2), -camUpVec(1,1), camUpVec(1,3)] .* (xDiff * -scale);
                dMov       = dMov_yDiff + dMov_xDiff;
                camdolly(oPCAxes_h, dMov(1,1), dMov(1,2), 0, 'movetarget', 'data'); % moves target and cam position
            end
        end
    end
end

