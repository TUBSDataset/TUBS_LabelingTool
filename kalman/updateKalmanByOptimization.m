function [voPCMovableLabel] = updateKalmanByOptimization(voPCMovableLabel, oGUIData, oGUIHistory)
% ---------------------------------------------------------------------------------------------
% Function updateKalmanByOptimization(...) performs the measurement update for the kalman filters of all objects
% based on an point cloud registration via ICP and multiple optimization algorithms. 

% First, the registration is performed. Next, the object's given ground plane is fitted onto the corresponding point
% cloud by using RANSAC to fit lines for the point cloud's edges. The box's position and orientation is then optimized
% using Newton's algorithm. 

% If this procedure fails, an ellipse is fitted. If both rectangle and ellipse fitting fail,
% then an algorithm is used to optimize the box's position by estimating the point cloud's edge points. This is
% basically the same algorithm that estimates a box from the given user input when defining a new box. 
%
% INPUT:
%   voPCMovableLabel:       Vector of cPCMovableLabel objects to be corrected
%   oGUIData:               Object of class cGUIData containing metadata inforamtion
%   oGUIHistory:            Vector of class cHistory containing previous samples
%   
% OUTPUT:
%   voPCMovableLabel:       Corrected object vector
% --------------------------------------------------------------------------------------------- 
bEnableRegistration          = oGUIData.m_bEnableRegistration;           % enable ICP for point cloud registration
bEnableFitting               = oGUIData.m_bEnableFitting;                % first rectangle estimation via RANSAC & Newton's algorithm, 
                                                                         % fallback to ellipse estimation if falied
bEnableEdgesEstimation       = oGUIData.m_bEnableEdgesEstimation;        % fall back if rectangle & ellipse estimation failed
bDebug                       = oGUIData.m_bEnableDebugInformation;

% Internal settings
bUseRegistrationForUpdate    = 1;    % use middle and yaw estimated by ICP algorithm (propagates errors if not corrected manually).
bUseEdgesOnlyAsFallback      = 1;    % 0 = do it anyway
bCoupleEllipseMidYaw         = 1;    % 1: if mid or yaw estimation are inconsistent, both are ignored
bDoRoughRegis                = 1;    % 1: Do a rough registration before fine registering point clouds
bUsePointsThreshold          = 100;  % threshold of points in accumulator below which to use current points instead of accumulator for estimation
dqs                          = 0.01; % default variance scaling if registration was not done
fVarPosEstmThreshold         = 0.01; % threshold to unset (accurate) prediction flag if variance of position estimations exceeds this limit 
fVarYawEstmThreshold         = 1;    % threshold to unset (accurate) prediction flag if variance of yaw estimations exceeds this limit 
fAbsYawEstmThreshold         = 2;    % (deg). Absolute threshold if no variance for yaw estimations can be calculated

% Consistency settings               
fConsistencyYawThreshold     = 5;    % deg (15)
fDsPlausible                 = 1;    % distance between previous middle and estimated middle, 0.5 for ellipse fitting
fDeviationFrom90Threshold    = 10;   % RANSAC-Algorithm: How much the crossing angle of length and width edge is allowed to deviate from 90 deg

% Plot settings
bSelectivePlot               = 0;    % Plot only a certain object within the object vector
nObjIndex                    = 8;    % Object index to plot
bOnlyPlotResult              = 0;    % Plot only the result 

voHistory        = oGUIHistory;
oInfo_h          = oGUIData.m_oInfo_h;
nPosInHistory    = oGUIData.m_nPosInHistory;

voPCMovableLabel_HSRF = voHistory(nPosInHistory-1).m_oGUIObjects.m_voPCMovableLabel;
oPCMetadata_CSRF      = oGUIData.m_oPCMetadata;

nCtr = 0;
nPos = setInfoText(oInfo_h, 'Optimizing predictions:', 1, 'end', nCtr);

for i = 1 : size(voPCMovableLabel, 1)
    %% Plot settings
    if oGUIData.m_bEnableOptimizationPlots && bOnlyPlotResult && (~bSelectivePlot || (bSelectivePlot && i == nObjIndex))
        enableRegistrationPlot = 0;
        enablePlotRANSAC       = 0;
        enablePlotNewton       = 0;
        enablePlotEllipse      = 0;
        enablePlotEdges        = 0;
        enablePlotResult       = 1;
    elseif oGUIData.m_bEnableOptimizationPlots && (~bSelectivePlot || (bSelectivePlot && i == nObjIndex))
        enableRegistrationPlot = 1;
        enablePlotRANSAC       = 1;
        enablePlotNewton       = 1;
        enablePlotEllipse      = 1;
        enablePlotEdges        = 0;
        enablePlotResult       = 1;
    else
        enableRegistrationPlot = 0;
        enablePlotRANSAC       = 0;
        enablePlotNewton       = 0;
        enablePlotEllipse      = 0;
        enablePlotEdges        = 0;
        enablePlotResult       = 0;
    end
    
    %% Info
    setInfoText(oInfo_h, 'Optimizing predictions:', 1, nPos, nCtr);
    nCtr = nCtr + 1;

    %% Check for optimization conditions
    oPCMovableLabel_cur   = voPCMovableLabel(i,1);
    currentPoints         = oPCMovableLabel_cur.m_mfPointMatrix;
    bHighDeterminability  = 0;  % increases certainty if RANSAC algorithm succeeds with high conficence
    
    % Skip inactive objects to reduce runtime
    if ~oPCMovableLabel_cur.m_bIsActive
        oPCMovableLabel_cur.m_mfRegisteredPointMatrix = currentPoints;
        continue;
    end
    
    if isempty(currentPoints)
        if bDebug
            setInfoText(oInfo_h, sprintf('Warning %s %d: Empty current box occured, skipping.', ...
                oPCMovableLabel_cur.m_sClassification, i), 1);
        end
        voPCMovableLabel(i,1).m_bIsPredicted = 0;
        continue;
    end
    
    % Determine corresponding box, needed for registration and for consistency checks when applying estimation algorithms
    bCorrespondingBoxFound = 0;
    for j = 1 : size(voPCMovableLabel_HSRF, 1)
        if oPCMovableLabel_cur.m_nTrackID == voPCMovableLabel_HSRF(j,1).m_nTrackID
            bCorrespondingBoxFound = 1;
            oPCMovableLabel_pre = voPCMovableLabel_HSRF(j,1);
            break;
        end
    end
    
    if ~bCorrespondingBoxFound
        if bDebug 
            setInfoText(oInfo_h, sprintf('Warning %s %d: New box without history found, skipping update.', ...
                oPCMovableLabel_cur.m_sClassification, i), 1);
            continue;
        end
    end
    
    %% Calculate the time difference between the previous object position and the current one
    oPCMetadata_HSRF = voHistory(nPosInHistory-1).m_oPCMetadata;
    
    % Current reference frame
    phi_cur = atan2(oPCMovableLabel_cur.m_fBBMiddle_y, oPCMovableLabel_cur.m_fBBMiddle_x)*180/pi();
    if phi_cur < 0
        phi_cur = phi_cur + 360;
    end
    phi_cur_scan = 360 - phi_cur;   
    
    t_start_cur = oPCMetadata_CSRF.m_nFirstTimestamp_us/1000/1000;
    DT_cur      = (oPCMetadata_CSRF.m_nLastTimestamp_us - oPCMetadata_CSRF.m_nFirstTimestamp_us)/1000/1000;
    t_cur       = t_start_cur + phi_cur_scan/360*DT_cur;  
    
    % Previous reference frame
    phi_pre     = atan2(oPCMovableLabel_pre.m_fBBMiddle_y, oPCMovableLabel_pre.m_fBBMiddle_x)*180/pi();
    if phi_pre < 0
        phi_pre = phi_pre + 360;
    end
    phi_scan_pre = 360 - phi_pre;   % scanner rotates clockwise
    
    t_start_pre = oPCMetadata_HSRF.m_nFirstTimestamp_us/1000/1000;
    DT_pre      = (oPCMetadata_HSRF.m_nLastTimestamp_us - oPCMetadata_HSRF.m_nFirstTimestamp_us)/1000/1000;
    t_pre       = t_start_pre + phi_scan_pre/360*DT_pre;   % object timestamp in predecessor frame
    
    % Delta t
    DT = t_cur - t_pre;
    
    %% Rotate previous object into current reference frame using calculated DT
    fDeltaEgoYaw = oPCMetadata_HSRF.m_fEgoYawRate * DT * pi()/180;  % in rad
    
    previousMiddleXY(1,1)     = oPCMovableLabel_pre.m_fBBMiddle_x;
    previousMiddleXY(2,1)     = oPCMovableLabel_pre.m_fBBMiddle_y;
    previousMiddleXY(3,1)     = oPCMovableLabel_pre.m_fBBMiddle_z;
    R = [cos(-fDeltaEgoYaw) -sin(-fDeltaEgoYaw) 0; sin(-fDeltaEgoYaw) cos(-fDeltaEgoYaw) 0; 0 0 1];
    
    previousMiddleXY_CSRF  = R*previousMiddleXY;
    previousBBYaw_CSRF     = oPCMovableLabel_pre.m_fBBYaw - fDeltaEgoYaw*180/pi();
    
    %% Registration
    %  Set yawRegistered to yawPredicted for RANSAC algorithm if registration failed
    yawRegistered        = oPCMovableLabel_cur.m_fBBYaw;
    fifo                 = currentPoints;
    bSuccessRegistration = 0;
    
    if bEnableRegistration
        bRegisPossible = 1;
        if isempty(oPCMovableLabel_pre.m_mfRegisteredPointMatrix)
            if bDebug
                setInfoText(oInfo_h, sprintf('Warning %s %d: No points in accumulator, skipping registration.', ...
                    oPCMovableLabel_cur.m_sClassification, i), 1);
            end
            bRegisPossible = 0;
        end
        
        if size(currentPoints,1) < 10
            if bDebug
                setInfoText(oInfo_h, sprintf('Warning %s %d: Not enough current points, skipping ICP.', ...
                    oPCMovableLabel_cur.m_sClassification, i), 1);
            end
            bRegisPossible = 0;
        end
        
        if bRegisPossible
            % Rotate accumulator points to new CSRF
            previousRegisteredPoints = oPCMovableLabel_pre.m_mfRegisteredPointMatrix;
            previousRegisteredPoints_CSRF = R*previousRegisteredPoints';
            
            if enableRegistrationPlot
                figure('units', 'normalized', 'outerposition', [0 0 1 1]); 
                axRegis1 = subplot(1,2,1);                      % computation plot
                axRegis2 = subplot(1,2,2, 'Visible', 'off');    % result plot
                set(axRegis1, 'Projection', 'perspective', 'DataAspectRatio', [1 1 1], 'NextPlot', 'add');
                
                scatter3(previousRegisteredPoints_CSRF(1,:), previousRegisteredPoints_CSRF(2,:), previousRegisteredPoints_CSRF(3,:), ...
                    20, [0.3 0.3 .3], '.', 'Parent', axRegis1);
                % Plot current points
                cur_h = scatter3(currentPoints(:,1), currentPoints(:,2), currentPoints(:,3), 40, [0 .1 .9], '.', 'Parent', axRegis1);
                
                set(axRegis1, 'XLim', [oPCMovableLabel_cur.m_fBBMiddle_x-3 oPCMovableLabel_cur.m_fBBMiddle_x+3]);
                set(axRegis1, 'YLim', [oPCMovableLabel_cur.m_fBBMiddle_y-3 oPCMovableLabel_cur.m_fBBMiddle_y+3]);
                set(axRegis2, 'XLim', get(axRegis1, 'XLim') + [0 0]);
                set(axRegis2, 'YLim', get(axRegis1, 'YLim') + [0 0]);
                
                set(axRegis1, 'CameraPosition', [oPCMovableLabel_cur.m_fBBMiddle_x  oPCMovableLabel_cur.m_fBBMiddle_y   34.6554]);
                set(axRegis1, 'CameraTarget',   [oPCMovableLabel_cur.m_fBBMiddle_x  oPCMovableLabel_cur.m_fBBMiddle_y        0]);
                set(axRegis1, 'CameraUpVector', [0 1 0]);
                set(axRegis2, 'CameraPosition', get(axRegis1, 'CameraPosition'));
                set(axRegis2, 'CameraTarget',   get(axRegis1, 'CameraTarget'));
                set(axRegis2, 'CameraUpVector', get(axRegis1, 'CameraUpVector'));
            else
                axRegis1 = []; axRegis2 = []; cur_h = [];
            end
            
            % Rough registration of previous registered points via current predicted data
            predictedMid(1,1) = oPCMovableLabel_cur.m_fBBMiddle_x;
            predictedMid(2,1) = oPCMovableLabel_cur.m_fBBMiddle_y;
            predictedMid(3,1) = oPCMovableLabel_cur.m_fBBMiddle_z;
            
            DxRough = predictedMid(1,1) - previousMiddleXY_CSRF(1,1);
            DyRough = predictedMid(2,1) - previousMiddleXY_CSRF(2,1);
            DYawRough = (oPCMovableLabel_cur.m_fBBYaw - previousBBYaw_CSRF)*pi()/180; % rad
            
            % Rotate previous registered points, center is previousMiddle_CSRF (rough registration)
            previousMiddle_CSRF      = previousMiddleXY_CSRF;
            previousMiddle_CSRF(3,1) = predictedMid(3,1);   
            
            R = [cos(DYawRough) -sin(DYawRough) 0; sin(DYawRough) cos(DYawRough) 0; 0 0 1];
            prevPRough_CSRF = repmat(previousMiddle_CSRF, [1, size(previousRegisteredPoints_CSRF,2)]) ...
                + R*(previousRegisteredPoints_CSRF - repmat(previousMiddle_CSRF, [1, size(previousRegisteredPoints_CSRF,2)]));
            
            % Translate (rough registration)
            TRough = [DxRough; DyRough; 0];
            prevPRough_CSRF = prevPRough_CSRF + repmat(TRough, [1, size(prevPRough_CSRF,2)]);
            
            if enableRegistrationPlot 
                % Plot previous points
                if bDoRoughRegis
                    prev_h = scatter3(prevPRough_CSRF(1,:), prevPRough_CSRF(2,:), prevPRough_CSRF(3,:), 20, [.8 0 .2], ...
                        '.', 'Parent', axRegis1);
                else
                    prev_h = scatter3(previousRegisteredPoints_CSRF(1,:), previousRegisteredPoints_CSRF(2,:), ...
                        previousRegisteredPoints_CSRF(3,:), 20, [.8 0 .2], '.', 'Parent', axRegis1);
                end
            else
                prev_h = [];
            end
            
            % Rough registration of previous yaw and middle
            previousBBYawRough_CSRF    = previousBBYaw_CSRF   + DYawRough*180/pi();
            previousMiddleXYRough_CSRF = previousMiddleXY_CSRF + TRough;
            
            % Fine registration via ICP. Empirically, registering fewer (current) points with previous 
            % (accumulated) point cloud yields better results (P2C = 0) = previous 2 current
            nMaxIt = oGUIData.m_nNumICPIterations;
            if bDoRoughRegis
                [DxRegis, DyRegis, yawRegistered, middleRegistered, dqs, yawTrustable, registeredPoints] = ...
                    registerBoxesICP(currentPoints, prevPRough_CSRF', previousMiddleXYRough_CSRF, previousBBYawRough_CSRF, ...
                        0, nMaxIt, axRegis1, axRegis2, cur_h, prev_h);
            else
                [DxRegis, DyRegis, yawRegistered, middleRegistered, dqs, yawTrustable, registeredPoints] = ...
                    registerBoxesICP(currentPoints, previousRegisteredPoints_CSRF', previousMiddleXY_CSRF, previousBBYaw_CSRF, ...
                        0, nMaxIt, axRegis1, axRegis2, cur_h, prev_h);
            end
            
            if enableRegistrationPlot
                set(axRegis1, 'Title', title(sprintf('%s %d: ICP registration', oPCMovableLabel_cur.m_sClassification, i), ...
                    'FontWeight', 'normal'));
                set(axRegis2, 'Visible', 'on', 'cameraPosition', get(axRegis1, 'cameraPosition'), 'cameraTarget', ...
                    get(axRegis1, 'cameraTarget'), 'XLim', get(axRegis1, 'XLim'), 'YLim', get(axRegis1, 'YLim'));
                
                scatter3(middleRegistered(1,1), middleRegistered(2,1), middleRegistered(3,1), 30, [1 0 0], 'x', ...
                    'Parent', axRegis1, 'LineWidth', 3);
                scatter3(middleRegistered(1,1), middleRegistered(2,1), middleRegistered(3,1), 30, [1 0 0], 'x', ...
                    'Parent', axRegis2, 'LineWidth', 3);
                
                scatter3(predictedMid(1,1), predictedMid(2,1), predictedMid(3,1), 40, [.3 .3 .3], 'x', 'Parent', ... 
                    axRegis1, 'LineWidth', 4);
                scatter3(predictedMid(1,1), predictedMid(2,1), predictedMid(3,1), 40, [.3 .3 .3], 'x', 'Parent', ...
                    axRegis2, 'LineWidth', 4);
                
                % Left:     grey:    previous registered pionts. 
                %           blue:    current points
                %           red:     previous points (rough registered if applied)
                %           green:   current points registered (actively registered)
                
                % Right plot equals registration result.
                % Right:    dark gr: current points         
                %           li   gr: registerend points (transformed previous PC)
                %           red:     estimated mid by registration
                %           grey:    predicted object middle
                
                % Draw prediction and estimation by registration
                drawRectangle(axRegis2, [.3 .3 .3], oPCMovableLabel_cur, 0);
                estmRegis = copy(oPCMovableLabel_cur);
                estmRegis.m_fBBMiddle_x = middleRegistered(1,1);
                estmRegis.m_fBBMiddle_y = middleRegistered(2,1);
                estmRegis.m_fBBMiddle_z = middleRegistered(3,1);
                estmRegis.m_fBBYaw = yawRegistered;
                drawRectangle(axRegis2, [0 1 0], estmRegis, 0);
                
                pause(0.5);
            end

            % Consistency check: only update if Dyaw < cosistencyThreshold [deg]
            DyawRegistered = yawRegistered - previousBBYaw_CSRF;
            %  Ds             = sqrt(DxRegis^2+DyRegis^2);
            y_pred = oPCMovableLabel_cur.m_fBBMiddle_y;
            x_pred = oPCMovableLabel_cur.m_fBBMiddle_x;
            x_regi = middleRegistered(1,1);
            y_regi = middleRegistered(2,1);
            Ds = sqrt((y_pred - y_regi)^2 + (x_pred - x_regi)^2);
            bSuccessRegistration = 1;
            if (abs(DyawRegistered) > fConsistencyYawThreshold) || ~yawTrustable || Ds > fDsPlausible 
                if bDebug
                    setInfoText(oInfo_h, sprintf('Warning %s %d: Registration inconsistent. Resetting accumulator matrix.', ...
                        oPCMovableLabel_cur.m_sClassification, i), 1);
                    setInfoText(oInfo_h, 'Estimation only based on current points.', 1);
                end
                bSuccessRegistration = 0;
                oPCMovableLabel_cur.m_mfRegisteredPointMatrix = currentPoints;
            else
                % Success case
                % FIFO memory
                maxNum = oGUIData.m_nSizeICPFifo;
                fifo = [currentPoints; registeredPoints];
                if size(fifo,1) > maxNum
                    fifo = fifo(1:maxNum,:);     % prefer current points
                end
                oPCMovableLabel_cur.m_mfRegisteredPointMatrix = fifo;
            end
            
            % Consistency plot
            if enableRegistrationPlot
                text((axRegis2.XLim(1,1)+.2), (axRegis2.YLim(1,1)+.7), sprintf('yaw pred.:    \t%.2f', ...
                    oPCMovableLabel_cur.m_fBBYaw), 'Color', 'black', 'FontSize', 12)
                if bSuccessRegistration 
                    text((axRegis2.XLim(1,1)+.2), (axRegis2.YLim(1,1)+.5), sprintf('yaw regis:    \t%.2f', ...
                        yawRegistered), 'Color', 'green', 'FontSize', 12)
                    text((axRegis2.XLim(1,1)+.2), (axRegis2.YLim(1,1)+.3), sprintf('mid avail.    \t%.2f', ...
                        (Ds)), 'Color', 'green', 'FontSize', 12)
                else
                    text((axRegis2.XLim(1,1)+.2), (axRegis2.YLim(1,1)+.5), sprintf('yaw regis:    \t%.2f', ...
                        yawRegistered), 'Color', 'red', 'FontSize', 12)
                    text((axRegis2.XLim(1,1)+.2), (axRegis2.YLim(1,1)+.3), sprintf('mid avail.    \t%.2f', ...
                        (Ds)), 'Color', 'red', 'FontSize', 12)
                end
            end
        end
    end
    
    % Reset accumulator if registration is disabled or not possible
    if ~bRegisPossible || ~ bEnableRegistration
        if bDebug
            setInfoText(oInfo_h, sprintf('Warning %s %d: Resetting accumulator.', ...
                oPCMovableLabel_cur.m_sClassification, i), 1);
        end
        oPCMovableLabel_cur.m_mfRegisteredPointMatrix = currentPoints;
    end
    
    %% Fitting algorithms (Based on current points. If registration is enabled: Estimation is based on registered point cloud)
    %  Estimate yaw and middle from countour points via an optimization algorithm based on RANSAC and Newton's algorithm (rectangle fitting),
    %  ellipse fitting or an edge estimation algorithm
    
    bSuccessFitting         = 0;
    bMiddleFittingAvailable = 0;
    bYawFittingAvailable    = 0;
    
    if bEnableFitting
        if enablePlotRANSAC || enablePlotNewton
            figure('units', 'normalized', 'outerposition', [0 0 1 1]); axRAN = axes;
        else
            axRAN = [];
        end
        
        %% Rectangle fitting
        %  Prefer current points for estimation. If not enough points: Use registered points (fifo)
        if size(currentPoints,1) > bUsePointsThreshold
            [middleEstimatedRANSAC, yawEstimatedRANSAC, contourPoints, angleOfIntersection, bSuccessFitting] = ...
                fitRectangleToContour(oPCMovableLabel_cur, currentPoints, yawRegistered, axRAN, enablePlotRANSAC, enablePlotNewton);
        else
            if bDebug
                setInfoText(oInfo_h, sprintf('Warning %s %d: Not enough current points. Using accu for estimation.', ...
                    oPCMovableLabel_cur.m_sClassification, i), 1);
            end
            [middleEstimatedRANSAC, yawEstimatedRANSAC, contourPoints, angleOfIntersection, bSuccessFitting] = ...
                fitRectangleToContour(oPCMovableLabel_cur, fifo, yawRegistered, axRAN, enablePlotRANSAC, enablePlotNewton);
        end
        
        if enablePlotRANSAC || enablePlotNewton
            set(axRAN, 'Title', title(sprintf('%s %d: Box fitting via RANSAC and Newton', ...
                oPCMovableLabel_cur.m_sClassification, i), 'FontWeight', 'normal'));
            drawRectangle(axRAN, [0.3 0.3 0.3], oPCMovableLabel_cur, 0); hold on;
            pause(0.5);
        end
        
        middleFitting  = middleEstimatedRANSAC;
        yawFitting     = yawEstimatedRANSAC;
        
        if (bSuccessFitting)
            bMiddleFittingAvailable = 1; % RANSAC success: both are available
            bYawFittingAvailable    = 1;
        end
        
        DxRANSAC = middleEstimatedRANSAC(1,1) - oPCMovableLabel_cur.m_fBBMiddle_x;
        DyRANSAC = middleEstimatedRANSAC(2,1) - oPCMovableLabel_cur.m_fBBMiddle_y;
        
        % RANSAC consistency check
        DYawEstimatedRANSAC = yawEstimatedRANSAC - oPCMovableLabel_cur.m_fBBYaw;
        Ds       = sqrt(DxRANSAC^2+DyRANSAC^2);
        diffTo90 = abs(90-angleOfIntersection);     % use diff to 90 degrees as determinability
        consistencyYawThreshold_temp = fConsistencyYawThreshold;
        
        if (diffTo90 < 3)
            if bDebug
                setInfoText(oInfo_h, sprintf('Info %s %d: High determinability.', ...
                    oPCMovableLabel_cur.m_sClassification, i), 1);
            end
            consistencyYawThreshold_temp = 25;
            bHighDeterminability         = 1;
        end
        
        if (abs(DYawEstimatedRANSAC) > consistencyYawThreshold_temp) || (Ds > fDsPlausible) || diffTo90 > fDeviationFrom90Threshold
            if bDebug
                setInfoText(oInfo_h, sprintf('Warning %s %d: RANSAC results inconsistent, ignoring.', ...
                    oPCMovableLabel_cur.m_sClassification, i), 1);
            end
            bSuccessFitting         = 0;
            bMiddleFittingAvailable = 0;    % RANSAC failed: both are not available
            bYawFittingAvailable    = 0;
        end
        
        % Plot consistency results
        if (enablePlotRANSAC || enablePlotNewton)
            text((axRAN.XLim(1,1)+.2), (axRAN.YLim(1,1)+.7), sprintf('yaw pred.:    \t%.2f', oPCMovableLabel_cur.m_fBBYaw), ...
                'Color', 'black', 'FontSize', 12)
            if(bYawFittingAvailable)
                text((axRAN.XLim(1,1)+.2), (axRAN.YLim(1,1)+.5), sprintf('yaw fitted:    \t%.2f', yawFitting), 'Color', ...
                    'green', 'FontSize', 12)
            else
                text((axRAN.XLim(1,1)+.2), (axRAN.YLim(1,1)+.5), sprintf('yaw fitted:    \t%.2f', yawFitting), 'Color', ...
                    'red', 'FontSize', 12)
            end
            if(bMiddleFittingAvailable)
                text((axRAN.XLim(1,1)+.2), (axRAN.YLim(1,1)+.3), sprintf('mid avail.     \t%.2f', Ds), ...
                    'Color', 'green', 'FontSize', 12)
            else
                text((axRAN.XLim(1,1)+.2), (axRAN.YLim(1,1)+.3), sprintf('mid avail.     \t%.2f', Ds), ...
                    'Color', 'red', 'FontSize', 12)
            end
        end
        
        %% Ellipse fitting if rectangle fitting failed
        if (~bSuccessFitting)
            % Temporary decrease DsPlausible to 0.5: Algorithm estimates middle based on the visible point cloud
            % (suppress cases where acutal middle differs from visible middle)
            DsPlausible_temp        = .5;    
            bMiddleFittingAvailable = 1;
            bYawFittingAvailable    = 1;
            
            if (enablePlotEllipse && (size(contourPoints,1) > 5) && bEnableFitting)
                figure('units', 'normalized', 'outerposition', [0 0 1 1]); axEll = axes;
            else
                axEll = [];
            end
            
            bUseLS = 0;
            [ellipseParameters, bSuccessFitting] = fitEllipseToContour(oPCMovableLabel_cur, contourPoints, bUseLS, axEll);
            
            if (enablePlotEllipse && (size(contourPoints,1) > 5)  && bEnableFitting)
                set(axEll, 'Title', title(sprintf('%s %d: Box fitting via ellipse', oPCMovableLabel_cur.m_sClassification, i), ...
                    'FontWeight', 'normal'));
                drawRectangle(axEll, [0.3 0.3 0.3], oPCMovableLabel_cur, 0); hold on;
                pause(0.5);
            end
            
            if(bSuccessFitting)
                yawFitting         = ellipseParameters(1,1);
                middleFitting(1,1) = ellipseParameters(4,1);
                middleFitting(2,1) = ellipseParameters(5,1);
                
                DxEllipse = middleFitting(1,1) - oPCMovableLabel_cur.m_fBBMiddle_x;
                DyEllipse = middleFitting(2,1) - oPCMovableLabel_cur.m_fBBMiddle_y;
                Ds = sqrt(DxEllipse^2+DyEllipse^2);
                
                % Middle consistency check
                if (Ds > DsPlausible_temp)
                    bMiddleFittingAvailable = 0;
                    if bDebug
                        setInfoText(oInfo_h, sprintf('Warning %s %d: Ellipse middle result inconsistent, ignoring.', ...
                            oPCMovableLabel_cur.m_sClassification, i), 1);
                    end
                end
                
                % Yaw consistency check (implementation at the end of this file)
                [yawFitting, bYawFittingAvailable] = ellipseYawConsistencyCheck(oInfo_h, yawFitting, ...
                    oPCMovableLabel_cur.m_fBBYaw, fConsistencyYawThreshold, oPCMovableLabel_cur, i, bDebug);
            else
                bYawFittingAvailable     = 0;
                bMiddleFittingAvailable  = 0;
            end
            
            % Plot result and consistency
            text_hs = gobjects(3,1);
            if ~isempty(axEll)
                text_hs(1,1) = text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.7), ...
                    sprintf('yaw pred.:    \t%.2f', oPCMovableLabel_cur.m_fBBYaw), 'Color', 'black', 'FontSize', 12);
                if(bYawFittingAvailable)
                    text_hs(2,1) = text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.5), ...
                        sprintf('yaw fitted:    \t%.2f', yawFitting), 'Color', 'green', 'FontSize', 12);
                else
                    text_hs(2,1) = text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.5), ...
                        sprintf('yaw fitted:    \t%.2f', yawFitting), 'Color', 'red', 'FontSize', 12);
                end
                if(bMiddleFittingAvailable)
                    text_hs(3,1) = text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.3), ...
                        sprintf('mid avail.     \t%.2f', Ds), 'Color', 'green', 'FontSize', 12);
                else
                    text_hs(3,1) = text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.3), ...
                        sprintf('mid avail.     \t%.2f', Ds), 'Color', 'red', 'FontSize', 12);
                end
            end
            
            % Try LS algorithm if middle or yaw inconsistent
            if ~bYawFittingAvailable || ~bMiddleFittingAvailable
                delete(text_hs);
                bMiddleFittingAvailable = 1;
                bYawFittingAvailable    = 1;
                
                if bDebug
                    setInfoText(oInfo_h, sprintf('Warning %s %d: Falling back to LS ellipse fitting.', ...
                        oPCMovableLabel_cur.m_sClassification, i), 1);
                end
                
                bUseLS = 1;
                [ellipseParameters, bSuccessFitting] = fitEllipseToContour(oPCMovableLabel_cur, contourPoints, bUseLS, axEll);
                if bSuccessFitting
                    yawFitting         = ellipseParameters(1,1);
                    middleFitting(1,1) = ellipseParameters(4,1);
                    middleFitting(2,1) = ellipseParameters(5,1);
                    
                    DxEllipse = middleFitting(1,1) - oPCMovableLabel_cur.m_fBBMiddle_x;
                    DyEllipse = middleFitting(2,1) - oPCMovableLabel_cur.m_fBBMiddle_y;
                    Ds = sqrt(DxEllipse^2+DyEllipse^2);
                    
                    % Middle consistency check
                    if Ds > DsPlausible_temp
                        bMiddleFittingAvailable = 0;
                        if bDebug
                            setInfoText(oInfo_h, sprintf('Warning %s %d: Ellipse middle result inconsistent, ignoring.', ...
                                oPCMovableLabel_cur.m_sClassification, i), 1);
                        end
                    end
                    
                    % Yaw consistency check
                    [yawFitting, bYawFittingAvailable]  = ellipseYawConsistencyCheck(oInfo_h, yawFitting, ...
                        oPCMovableLabel_cur.m_fBBYaw, fConsistencyYawThreshold, oPCMovableLabel_cur, i, bDebug);
                else
                    bYawFittingAvailable     = 0;
                    bMiddleFittingAvailable  = 0;
                end
            end
            
            if (bCoupleEllipseMidYaw && (~bYawFittingAvailable || ~bMiddleFittingAvailable))
                bSuccessFitting = 0;
            end
            
            % All ellipse algorithms failed
            if (~bYawFittingAvailable && ~bMiddleFittingAvailable) || (~bSuccessFitting)
                if bDebug
                    setInfoText(oInfo_h, sprintf('Warning %s %d: Falling back to ellipse fitting failed, ignoring.', ...
                        oPCMovableLabel_cur.m_sClassification, i), 1);
                end
                bYawFittingAvailable     = 0;
                bMiddleFittingAvailable  = 0;
            end
            
            % Plot result and consistency
            if ~isempty(axEll)
                text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.7), ...
                    sprintf('yaw pred.:    \t%.2f', oPCMovableLabel_cur.m_fBBYaw), 'Color', 'black', 'FontSize', 12)
                if(bYawFittingAvailable)
                    text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.5), ...
                        sprintf('yaw fitted:    \t%.2f', yawFitting), 'Color', 'green', 'FontSize', 12)
                else
                    text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.5), ...
                        sprintf('yaw fitted:    \t%.2f', yawFitting), 'Color', 'red', 'FontSize', 12)
                end
                if(bMiddleFittingAvailable)
                    text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.3), ...
                        sprintf('mid avail.     \t%.2f', Ds), 'Color', 'green', 'FontSize', 12)
                else
                    text((axEll.XLim(1,1)+.2), (axEll.YLim(1,1)+.3), ...
                        sprintf('mid avail.     \t%.2f', Ds), 'Color', 'red', 'FontSize', 12)
                end
            end
        end
    end
    
    %% Estimate yaw and middle by finding edge points and applying a rectification (as done for a box definition)
    bSuccessEdges = 0;
    if (bEnableEdgesEstimation && ((bUseEdgesOnlyAsFallback && ~bSuccessFitting)) || ~bUseEdgesOnlyAsFallback)
        if (enablePlotEdges)
            figure('units', 'normalized', 'outerposition', [0 0 1 1]); axEdges = axes;
        else
            axEdges = [];
        end
        
        [middleEstimatedEdges, yawEstimatedEdges, bSuccessEdges] = fitRectangleByEdgePoints(oPCMovableLabel_cur, fifo, axEdges);
        
        if(enablePlotEdges)
            set(axEdges, 'Title', title(sprintf('%s %d: Box fitting via edge points', ...
                oPCMovableLabel_cur.m_sClassification, i),'FontWeight', 'normal'));
            pause(0.5);
        end
        
        % Consistency check
        if (bSuccessEdges == 1)
            DxEdges = middleEstimatedEdges(1,1) - oPCMovableLabel_cur.m_fBBMiddle_x;
            DyEdges = middleEstimatedEdges(2,1) - oPCMovableLabel_cur.m_fBBMiddle_y;
            Ds = sqrt(DxEdges^2+DyEdges^2);
            DYawEstimatedEdges = yawEstimatedEdges - oPCMovableLabel_cur.m_fBBYaw;
            
            if (abs(DYawEstimatedEdges) > fConsistencyYawThreshold) || (Ds > fDsPlausible)
                if bDebug
                    setInfoText(oInfo_h, sprintf('Warning %s %d: Edges fitting results inconsistent, ignoring.', ...
                        oPCMovableLabel_cur.m_sClassification, i), 1);
                end
                bSuccessEdges = 0;
            end
        end
    end
    
    %% Measurement update
    %  Build middle and yaw estimation
    weightMid   = 1;           weightYaw    = 1;    % number of applied algorithms to be averaged
    bbMiddle    = zeros(2,1);  bbMiddleInit = 0;
    bbYaw       = 0;           bbYawInit    = 0;
    
    % Registration
    if (bEnableRegistration && bSuccessRegistration && bUseRegistrationForUpdate && ~bHighDeterminability)
        bbMiddle = middleRegistered(1:2,:);
        bbMiddleInit = 1;
        
        bbYaw     = yawRegistered; % if successRegistration then yaw is trustable
        bbYawInit = 1;
    end
    
    % Rectangle and ellipse
    if (bEnableFitting)
        % Middle
        if(bbMiddleInit && bMiddleFittingAvailable)
            bbMiddle = bbMiddle + middleFitting;
            weightMid = weightMid + 1;
        elseif(bMiddleFittingAvailable)
            bbMiddle = middleFitting;
            bbMiddleInit = 1;
        end
        
        % Yaw
        if(bbYawInit && bYawFittingAvailable)
            bbYaw = bbYaw + yawFitting;
            weightYaw = weightYaw + 1;
        elseif(bYawFittingAvailable)
            bbYaw = yawFitting;
            bbYawInit = 1;
        end
    end
    % successEdges = 0 if fitting succeeded and onlyUseEdgesAsFallback = 1. 
    % If ~onlyUseEdgesAsFallback and successEdges = 1, then average further
    if (bEnableEdgesEstimation && bSuccessEdges)
        % Middle
        if(bbMiddleInit)
            bbMiddle = bbMiddle + middleEstimatedEdges;
            weightMid = weightMid + 1;
        else
            bbMiddle = middleEstimatedEdges;
            bbMiddleInit = 1;
        end
        
        % Yaw
        if(bbYawInit)
            bbYaw = bbYaw + yawEstimatedEdges;
            weightYaw = weightYaw + 1;
        else
            bbYaw = yawEstimatedEdges;
            bbYawInit = 1;
        end
    end
    
    bbMiddle = (1/weightMid) .* bbMiddle;
    bbYaw    = (1/weightYaw) .* bbYaw;
    
    % Sum up fitting result 
    if (bDebug && bbMiddleInit)
        setInfoText(oInfo_h, sprintf('Warning %s %d: FITTING: FOUND BB MIDDLE.', oPCMovableLabel_cur.m_sClassification, i), 1);
    end
    if (bDebug && bbYawInit)
        setInfoText(oInfo_h, sprintf('Warning %s %d: FITTING: FOUND BB YAW.', oPCMovableLabel_cur.m_sClassification, i), 1);
    end
    
    if ~bbYawInit
        if bDebug
            setInfoText(oInfo_h, sprintf('Warning %s %d: FITTING: NO BB YAW, SKIPPING.', oPCMovableLabel_cur.m_sClassification, i), 1);
        end
        voPCMovableLabel(i,1).m_bIsPredicted = 0;
        continue;
    end
    
    % Determine variances
    % dqs is the mean of squared distances of all point pairs (see ICP registration)
    % Used for variance estimation (if registration was done)
    varScale = 1;   % 20
    if(bHighDeterminability)
        varScale = 0.1; % 1
    end
    varYawOffset = 1;
    fVar         = dqs * varScale;
    if(fVar < 0.01)
        fVar = 0.01;
    end
    
    % Map measurement to reference point
    refPoint_ID = oPCMovableLabel_cur.m_vfReferencePoint(3,1);
    fBBYaw = oPCMovableLabel_cur.m_fBBYaw * pi()/180;
    lvec(1,1) = oPCMovableLabel_cur.m_fBBLength/2*cos(fBBYaw);
    lvec(2,1) = oPCMovableLabel_cur.m_fBBLength/2*sin(fBBYaw);
    wvec(1,1) = oPCMovableLabel_cur.m_fBBWidth/2*cos(fBBYaw + pi()/2);
    wvec(2,1) = oPCMovableLabel_cur.m_fBBWidth/2*sin(fBBYaw + pi()/2);
    switch refPoint_ID
        case 1
            refPoint = bbMiddle + lvec + wvec;
        case 2
            refPoint = bbMiddle + wvec;
        case 3
            refPoint = bbMiddle - lvec + wvec;
        case 4
            refPoint = bbMiddle + lvec - wvec;
        case 5
            refPoint = bbMiddle - wvec;
        case 6
            refPoint = bbMiddle - lvec - wvec;
    end
    
    % Update
    vfMeasurements = [refPoint(1,1); refPoint(2,1); bbYaw];
    vfVariances    = [fVar; fVar; fVar + varYawOffset];
    
    oPCMovableLabel_pred = copy(oPCMovableLabel_cur);
    bStable = oPCMovableLabel_cur.m_oKalman.update(vfMeasurements, vfVariances);
    
    if ~bStable && (bDebug || oGUIData.m_bLogWarnings )
        setInfoText(oInfo_h, sprintf('Warning %s %d: FILTER UNSTABLE.', oPCMovableLabel_cur.m_sClassification, i), 1);
    end

    [bConsistent, sStatus] = oPCMovableLabel_cur.m_oKalman.stateConsistencyCheck();

    if ~bConsistent
        % Reset to predecessor state and reinit model
        oPCMovableLabel_cur.m_oKalman.reset();     
        if bDebug || oGUIData.m_bLogWarnings 
            setInfoText(oInfo_h, sprintf('Warning %s %d: STATES INCONSISTENT. Cause: %s', ...
                oPCMovableLabel_cur.m_sClassification, i, sStatus), 1);
        end
    end         
    
    % Reassing data format entries
    oPCMovableLabel_cur.reassignData();
    voPCMovableLabel(i,1).updateReferencePoint();
    
    %% Unset accurate prediction flag
    %  Calculate the variance of position and yaw estimations regarding the prediction as GT
    vfValues_pos_estm   = [];
    vfValues_pos_GT     = [];
    vfValues_yaw_estm   = [];
    vfValues_yaw_GT     = [];
    
    if bSuccessRegistration
        vfValues_pos_estm   = [middleRegistered(1,1);  middleRegistered(2,1)];
        vfValues_pos_GT     = [oPCMovableLabel_pred.m_fBBMiddle_x; oPCMovableLabel_pred.m_fBBMiddle_y];
        
        vfValues_yaw_estm   = [yawRegistered];
        vfValues_yaw_GT     = [oPCMovableLabel_pred.m_fBBYaw];
    end
    
    if bSuccessFitting
        vfValues_pos_estm   = [vfValues_pos_estm;   middleFitting(1,1); middleFitting(2,1)];
        vfValues_pos_GT     = [vfValues_pos_GT;     oPCMovableLabel_pred.m_fBBMiddle_x; oPCMovableLabel_pred.m_fBBMiddle_y];
        
        vfValues_yaw_estm   = [vfValues_yaw_estm;   yawFitting];
        vfValues_yaw_GT     = [vfValues_yaw_GT;     oPCMovableLabel_pred.m_fBBYaw];
    end
    
    if ~isempty(vfValues_pos_estm)
        vfDiff_pos = (vfValues_pos_estm - vfValues_pos_GT);
        fVar_pos = var(vfDiff_pos);
    else
        fVar_pos = 100;
    end
    
    if ~isempty(vfValues_yaw_estm)
        vfDiff_yaw = (vfValues_yaw_estm - vfValues_yaw_GT);
        fVar_yaw = var(vfDiff_yaw);
        fAbs_yaw = mean(abs(vfDiff_yaw));
    else
        fVar_yaw = 100;
        fAbs_yaw = 100;
    end
    
    if bDebug
        fprintf('Info %s %d: pos variance: %0.5f | yaw variance: %0.5f.\n', oPCMovableLabel_cur.m_sClassification, i, fVar_pos, fVar_yaw);
    end
    
    if (fVar_pos > fVarPosEstmThreshold) || (fVar_yaw > fVarYawEstmThreshold)
        voPCMovableLabel(i,1).m_bIsPredicted = 0;
    end
    
    if (fVar_yaw == 0) && (fAbs_yaw > fAbsYawEstmThreshold)
        voPCMovableLabel(i,1).m_bIsPredicted = 0;
    end

    %% Result plots
    if (enablePlotResult)
        figure('units', 'normalized', 'outerposition', [0 0 1 1]);
        axResult = axes;
        set(axResult, 'Projection', 'perspective', 'DataAspectRatio', [1 1 1], 'NextPlot', 'add');
        set(axResult, 'XLim', [oPCMovableLabel_cur.m_fBBMiddle_x-3, oPCMovableLabel_cur.m_fBBMiddle_x+3]);
        set(axResult, 'YLim', [oPCMovableLabel_cur.m_fBBMiddle_y-3, oPCMovableLabel_cur.m_fBBMiddle_y+3]);
        set(axResult, 'CameraPosition', [oPCMovableLabel_cur.m_fBBMiddle_x  oPCMovableLabel_cur.m_fBBMiddle_y   34.6554]);
        set(axResult, 'CameraTarget',   [oPCMovableLabel_cur.m_fBBMiddle_x  oPCMovableLabel_cur.m_fBBMiddle_y        0]);
        set(axResult, 'CameraUpVector', [0 1 0]);
        title(axResult, sprintf('%s %d: Result', oPCMovableLabel_cur.m_sClassification, i))
        scatter3(currentPoints(:,1), currentPoints(:,2), currentPoints(:,3), 20, [0 0 0], '.', 'Parent', axResult);
       
        % Prediction
        drawRectangle(axResult, [.3 .3 .3], oPCMovableLabel_cur, 0);
        
        % Registration: red
        if (bEnableRegistration && bSuccessRegistration)
            estmRegis = copy(oPCMovableLabel_cur);
            estmRegis.m_fBBMiddle_x = middleRegistered(1,1);
            estmRegis.m_fBBMiddle_y = middleRegistered(2,1);
            estmRegis.m_fBBMiddle_z = 0;
            estmRegis.m_fBBYaw      = yawRegistered;
            drawRectangle(axResult, [1 0 0], estmRegis, 0);
        end
        
        % Fitting: blue
        if (bEnableFitting && bSuccessFitting && bMiddleFittingAvailable && bYawFittingAvailable)
            estmFit = copy(oPCMovableLabel_cur);
            estmFit.m_fBBMiddle_x = middleFitting(1,1);
            estmFit.m_fBBMiddle_y = middleFitting(2,1);
            estmFit.m_fBBMiddle_z = 0;
            estmFit.m_fBBYaw      = yawFitting;
            drawRectangle(axResult, [0 0 1], estmFit, 0);
        end
        
        % Merged: magenta
        if(bbMiddleInit && bbYawInit)
            estmMerge = copy(oPCMovableLabel_cur);
            estmMerge.m_fBBMiddle_x = bbMiddle(1,1);
            estmMerge.m_fBBMiddle_y = bbMiddle(2,1);
            estmMerge.m_fBBMiddle_z = 0;
            estmMerge.m_fBBYaw      = bbYaw;
            drawRectangle(axResult, [1 0 1], estmMerge, 0);
        end
        
        % Kalman: green
        estmKal = copy(oPCMovableLabel_cur);
        estmKal.m_fBBMiddle_x = oPCMovableLabel_cur.m_fBBMiddle_x;
        estmKal.m_fBBMiddle_y = oPCMovableLabel_cur.m_fBBMiddle_y;
        estmKal.m_fBBMiddle_z = 0;
        estmKal.m_fBBYaw      = oPCMovableLabel_cur.m_fBBYaw;
        drawRectangle(axResult, [0 1 0], estmKal, 0);
        
        % Predicted: grey
        oPCMovableLabel_pred.m_fBBMiddle_x = oPCMovableLabel_pred.m_fBBMiddle_x;
        oPCMovableLabel_pred.m_fBBMiddle_y = oPCMovableLabel_pred.m_fBBMiddle_y;
        oPCMovableLabel_pred.m_fBBMiddle_z = 0;
        oPCMovableLabel_pred.m_fBBYaw      = oPCMovableLabel_pred.m_fBBYaw;
        drawRectangle(axResult, [.3 .3 .3], oPCMovableLabel_pred, 0);
    end
    
end

end

%% EllipseYawConsistencyCheck
%  This functions implements a consistency check for the estimated yaw value by ellipse fitting
function [fYawFitting, bYawFittingAvailable]  = ellipseYawConsistencyCheck(     oInfo_h, ...
                                                                                fYawFitting, ...
                                                                                fPreviousBBYaw_CSRF, ...
                                                                                fConsistencyYawThreshold, ...
                                                                                oPCMovableLabel, ...
                                                                                i, ...
                                                                                bDebug)
bYawFittingAvailable = 1;
DYawEstimatedEllipse = fYawFitting - fPreviousBBYaw_CSRF;   % yaw difference

% Before angluar consistency check: Try to establish a mapping by turining the ellipse by 180°. 
if abs(DYawEstimatedEllipse) > 135
    if fYawFitting > oPCMovableLabel.m_fBBYaw
        if (bDebug)
            fprintf('Warning %s %d: Fitted ellipse measurement mapped by -180.\n', ...
                oPCMovableLabel.m_sClassification, i);
        end
        fYawFitting = fYawFitting - 180;
    else
        if bDebug
            fprintf('Warning %s %d: Fitted ellipse measurement mapped by +180.\n', ...
                oPCMovableLabel.m_sClassification, i);
        end
        fYawFitting = fYawFitting + 180;
    end
end

DYawEstimatedEllipse = fYawFitting - fPreviousBBYaw_CSRF;

% Ellipse might be roated by 90 deg (length of object point cloud is smaller than width 
if abs(DYawEstimatedEllipse) > 45
    if fYawFitting > oPCMovableLabel.m_fBBYaw
        if bDebug
            fprintf('Warning %s %d: Fitted ellipse turned -90 deg.\n', oPCMovableLabel.m_sClassification, i);
        end
        fYawFitting = fYawFitting - 90; 
    else
        if bDebug
            fprintf('Warning %s %d: Fitted ellipse turned +90 deg.\n', oPCMovableLabel.m_sClassification, i);
        end
        fYawFitting = fYawFitting + 90; 
    end
end

% Ellipse consistency check, treat middle and yaw separately
DYawEstimatedEllipse = fYawFitting - fPreviousBBYaw_CSRF;
if (abs(DYawEstimatedEllipse) > fConsistencyYawThreshold)
    bYawFittingAvailable = 0;
    if bDebug
        setInfoText(oInfo_h, sprintf('Warning %s %d: Ellipse yaw result inconsistent, ignoring.', ...
            oPCMovableLabel.m_sClassification, i), 1);
    end
end

end

