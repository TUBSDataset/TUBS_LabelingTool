classdef cPCMovableLabel < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cPCMovableLabel stores an object's size, position and dynamics. 
    %
    % Currently, our prelabeling system is providing 7 classes: 
    % car, van, truck, motorbike, pedestrian, bicycle and a general class movable.
    % You can configure more classes using EditorConfig.xml. The probability vector will be set accordingly.  
    
    % The editor uses three prediction models. For pedestrians a point mass model with constant velocity is used.
    % Hence, no acceleartion is provided for pedesetrians.
    % All other classes use an IMM-eKF filter structre incorporating a point mass model and a circular path model.
    % The model outputs are fused according to the model's probability. See class cKalman for more information.
    
    % A reference point model with six points is used for object tracking. Additionally to the four edge points, 
    % we define the two points lying on the length line (same direction as the object's heading or yaw, respectively) 
    % as reference points. The point closest to the ego vehicle is chosen as reference for tracking. Each reference point
    % has an ID (see below). 
    %
    %   Front (fBBYaw = 90° in Velodyne Reference Frame)
    %  1 ___ 4
    %   |   |
    %  2|   |5
    %   |___|
    %  3     6
    %    Rear 
    % ---------------------------------------------------------------------------------------------
    
    properties
        %% Data format entries
        % General information
        m_nTrackID               = 0;    % unique object ID (only) within a sensor recording
        m_bIsActive              = 0;    % used to distinguish between parking and driving cars, sitting and walking pedestrians, unmanned and manned bicycles
        m_fExistenceLikelihood   = 0;    % estimated by our tracking system used for prelabeling
        m_sClassification        = 'Undefined';
        m_nTimestamp             = 0;    % prediction timestamp for the object's variables 
        m_voProbabilityVector    = [];   % struct array containing all class prababilies according to the editor configuration. 
                                        
        % Bounding box data
        m_fBBMiddle_x 	= 0;    % cartesian coordinates specified in the velodyne's sensor reference frame
        m_fBBMiddle_y   = 0;
        m_fBBMiddle_z   = 0;
        m_fBBHeight     = 0;
        m_fBBWidth      = 0;
        m_fBBLength     = 0;
        m_fBBYaw        = 0;    % BB yaw related to the x axis in degree. 
        
        % Note that Kalman filters are initialized independently. Filter values are applied after first prediction step.
        % Dynamics
        m_fVxAbs = 0;               % absolute velocity in x direction (including ego movement) m/s
        m_fVyAbs = 0;               % absolute velocity in y direction m/s
        m_fAxAbs = 0;               % absolute acceleration in x direction m/s^2
        m_fAyAbs = 0;               % absolute acceleration in y direction m/s^2
        m_fBBYawRatePerDist = 0;    % yaw rate per distance deg/m

        % Variances 
        m_fVarBBMiddle_x = 100;
        m_fVarBBMiddle_y = 100;
        m_fVarBBYaw = 100;
        
        m_fVarVxAbs = 100;
        m_fVarVyAbs = 100;
        m_fVarAxAbs = 100;
        m_fVarAyAbs = 100;        
        m_fVarBBYawRatePerDist = 100;
        
        % Filter initialization. Takes place during completeObjectData(...)
        m_fV_init = 2;
        m_fA_init = 0.1;

        m_fVarPos_init  = 100;
        m_fVarV_init    = 5;
        m_fVarA_init    = 2.5;
        m_fVarK_init    = 1;
        
        %% Helper: Variables needed for internal processes within the editor
        % Core
        m_bIsNew                    = 1;    % true if object is new in frame
        m_bIsCorrected              = 0;    % true if object is corrected
        m_bIsPredicted              = 0;    % true if prediction is accurate and needs no reviews
        m_oKalman                   = [];   % object of class cKalman representing the object's filter
        m_mfPointMatrix             = [];   % all points from the current PC assigned to this object
        m_mfRegisteredPointMatrix   = [];   % accumulated points assigned to this object
        m_vfGroundLevels            = [];   % ground level information of points in m_mfPointMatrix
        m_vfReferencePoint          = zeros(3,1); % current reference point used for tracking (x;y;ID)
        m_Box_h                     = gobjects(0,0);    % box graphic
        m_Accu_h                    = gobjects(0,0);    % accu graphic  (via ICP accumulated points)
        m_Polygon_h                 = gobjects(0,0);    % image object if projection was done
        
        % Editing
        m_nEmptyCounter        = 0;         % increases in each scan if a box empty. Used to determine the moment of deletion
        m_bCounterIncreased    = 0;         % avoid increasing empty counter multiple times in same scan
        m_bIsUserDefined       = 0;         % true for an user defined object (in new box mode)
        m_bCompleteObjectData  = 0;         % true if object data is to be completed (toggeling active flag, enforcing height recalculation)
        m_bNewMeasurement      = 0;         % true if correction was performed. Resets after updateKalmanByCorrections(...)
        m_bFirstFrame          = 1;         % true if object is present for the first time
        m_bIsProjected         = 0;         % true if object is currently projected onto the image planes
        m_bScanBreak           = 0;         % true if object is predicted onto the scan's break (at 0°)
        m_bShapeDeleted        = 0;         % true if corresponding image shape was deleted by user
        
        % Height calculation
        m_bRecalcHeight        = 0;         % set by keyboard callback. Forces recalculation of height parameter if case of faults
    end
    
    methods
        % Constructor
        function obj = cPCMovableLabel()
            obj.m_oKalman = cKalman();
        end
        
        % This function initializes necessary helper variables
        function obj = init(obj)
            % Init reference point
            bInit = 1;
            obj.updateReferencePoint(bInit);
            
            % Init Kalman filter
            obj.m_oKalman.initModels(obj);
            
            % Init registration matrix (point matrix initialized during relabling)
            obj.m_mfRegisteredPointMatrix = obj.m_mfPointMatrix;
        end
        
        function obj = updateReferencePoint(obj, bInit_In)
            bInit = 0;
            if nargin > 1
               bInit = bInit_In; 
            end
            % Compute all reference points
            ref_old = obj.m_vfReferencePoint(1:2,1);
            ID_old  = obj.m_vfReferencePoint(3,1);
            
            fBBYaw = obj.m_fBBYaw * pi()/180;
            
            % Create set of reference points
            vfPoints = zeros(6,3);
            
            vfMiddle(1,1) = obj.m_fBBMiddle_x;
            vfMiddle(2,1) = obj.m_fBBMiddle_y;
            lvec(1,1) = obj.m_fBBLength/2*cos(fBBYaw);
            lvec(2,1) = obj.m_fBBLength/2*sin(fBBYaw);
            wvec(1,1) = obj.m_fBBWidth/2*cos(fBBYaw + pi()/2);
            wvec(2,1) = obj.m_fBBWidth/2*sin(fBBYaw + pi()/2);
            
            % Reference points
            vfPoints(1,1:2) =  lvec + wvec + vfMiddle;
            vfPoints(1,3)   =  1;
            
            vfPoints(2,1:2) =  wvec + vfMiddle;
            vfPoints(2,3)   =  2;
            
            vfPoints(3,1:2) = -lvec + wvec + vfMiddle;
            vfPoints(3,3)   =  3;
            
            vfPoints(4,1:2) =  lvec - wvec + vfMiddle;
            vfPoints(4,3)   =  4;
            
            vfPoints(5,1:2) = -wvec + vfMiddle;
            vfPoints(5,3)   =  5;
            
            vfPoints(6,1:2) = -lvec - wvec + vfMiddle;
            vfPoints(6,3)   =  6;
            
            % Determine closest reference point to ego vehicle
            cells         = num2cell(vfPoints, 2);
            distFnc       = @(v) sqrt(v(1,1)^2 + v(1,2)^2);
            distance      = cellfun(distFnc, cells);
            vfPoints(:,4) = distance;
            vfPoints      = sortrows(vfPoints,4);

            ID_new = vfPoints(1,3);
            
            bReferencePointChanged = 0;
            if ID_new ~= ID_old
                bReferencePointChanged = 1;
            end
            
            % Set new reference point
            obj.m_vfReferencePoint = vfPoints(1,1:3)';

            if bInit
                return
            end
            
            if ~bReferencePointChanged
                return
            end
            
            % Detect length side change in order to update yawRate for CVCP model
            sChangeState  = 'none';  
            if      (ID_old <= 3) && (ID_new >= 4)
                sChangeState = 'right';
            elseif  (ID_old >= 4) && (ID_new <= 3)
                sChangeState = 'left';
            end
            
            % Update models
            delta = obj.m_vfReferencePoint(1:2,1) - ref_old;
            
            obj.m_oKalman.changeReferencePoint(delta, sChangeState, obj.m_fBBWidth);
        end
        
        % Function to update the probability vector in a PCMovablelLabel additional classes in EditorConfig.xml compared to 
        % available classes in the prelabeling stage
        function [nCode, sMessage] = updateClassesVector(obj, oEditorConfig)
            nCode       = 0; % 1 = new classes found
            sMessage    = '';
            % Check for new classes
            for i = 1 : size(oEditorConfig.m_voMovableClasses,1)
                bNewFound = 1;
                for j = 1 : size(obj.m_voProbabilityVector.Class,1)
                    if strcmp(oEditorConfig.m_voMovableClasses(i,1).Name, obj.m_voProbabilityVector.Class(j,1).Name)
                        bNewFound = 0;
                        break;
                    end
                end
                % Update vector
                if bNewFound
                    nCode = 1; sMessage = strcat(sMessage, oEditorConfig.m_voMovableClasses(i,1).Name, '||');  
                    obj.m_voProbabilityVector.Class(end+1,1).Name           = oEditorConfig.m_voMovableClasses(i,1).Name;
                    obj.m_voProbabilityVector.Class(end,1).Probability      = 0;
                end
            end
        end
        
        function reassignData(obj)
            pFused = obj.m_oKalman.getFusedState();
            PFused = obj.m_oKalman.getFusedCovariance();
            
            if strcmp(obj.m_sClassification, 'Pedestrian')
               [pFused, PFused] = obj.m_oKalman.getCVPM(); 
            end
            
            obj.m_vfReferencePoint(1,1) = pFused(1,1);
            obj.m_vfReferencePoint(2,1) = pFused(4,1);
            
            % Orientation
            yaw = pFused(7,1);
            while yaw < 0 
                yaw = yaw + 360;
            end
            while yaw > 360
                yaw = yaw - 360; 
            end
            obj.m_fBBYaw = yaw;
            
            % Calculate middle
            refPoint_ID = obj.m_vfReferencePoint(3,1);
            refPoint    = obj.m_vfReferencePoint(1:2,1);
            
            fBBYaw = obj.m_fBBYaw * pi()/180;
            lvec(1,1) = obj.m_fBBLength/2*cos(fBBYaw);
            lvec(2,1) = obj.m_fBBLength/2*sin(fBBYaw);
            wvec(1,1) = obj.m_fBBWidth/2*cos(fBBYaw + pi()/2);
            wvec(2,1) = obj.m_fBBWidth/2*sin(fBBYaw + pi()/2);
            
            switch refPoint_ID
                case 1
                    refPoint = refPoint - lvec - wvec;
                case 2
                    refPoint = refPoint - wvec;
                case 3
                    refPoint = refPoint + lvec - wvec;
                case 4
                    refPoint = refPoint - lvec + wvec;
                case 5
                    refPoint = refPoint + wvec;
                case 6
                    refPoint = refPoint + lvec + wvec;
            end
            
            % Position
            obj.m_fBBMiddle_x   = refPoint(1,1);
            obj.m_fBBMiddle_y   = refPoint(2,1);
            
            [~, ~, fProb_CVCP] = obj.m_oKalman.getCVCP();

            % Dynamics
            if obj.m_bIsActive
                obj.m_fVxAbs = pFused(2,1);
                obj.m_fVyAbs = pFused(5,1);
                obj.m_fAxAbs = pFused(3,1);
                obj.m_fAyAbs = pFused(6,1);
                if fProb_CVCP > 0.8
                    obj.m_fBBYawRatePerDist = pFused(8,1);
                else
                    obj.m_fBBYawRatePerDist = 0;
                end
            else
                obj.m_fVxAbs = 0;
                obj.m_fVyAbs = 0;
                obj.m_fAxAbs = 0;
                obj.m_fAyAbs = 0;
                obj.m_fBBYawRatePerDist = 0;
            end
            
            
            % Variances
            obj.m_fVarBBMiddle_x        = PFused(1,1);
            obj.m_fVarBBMiddle_y        = PFused(4,4);
            obj.m_fVarVxAbs             = PFused(2,2);
            obj.m_fVarVyAbs             = PFused(5,5);
            obj.m_fVarAxAbs             = PFused(3,3);
            obj.m_fVarAyAbs             = PFused(6,6);
            obj.m_fVarBBYaw             = PFused(7,7);
            obj.m_fVarBBYawRatePerDist  = PFused(8,8);
        end
        
        % This function averages the ground level information for all points within the object's bounding box
        function fGroundLevel = getGroundLevel(obj)
            nPoints = size(obj.m_vfGroundLevels, 1);
            fAvergage = 0.0;
            fDefault = -1.5;
            if nPoints  > 0
                for i = 1 : nPoints;
                    fAvergage = fAvergage + obj.m_vfGroundLevels(i,1);
                end
                fGroundLevel = fAvergage/nPoints;
            else
                fGroundLevel = fDefault;
            end
            
            if fGroundLevel > 0
               fGroundLevel = fDefault;
            end
        end
        
        % This function computes bounding box's 3D edge and middle points 
        function mfPos = get3DPoints(obj)
            % Create 3D points to project
            x = obj.m_fBBMiddle_x;
            y = obj.m_fBBMiddle_y;
            z = obj.m_fBBMiddle_z;
            fBBYaw      = obj.m_fBBYaw *pi()/180;
            fBBLength   = obj.m_fBBLength;
            fBBWidth    = obj.m_fBBWidth;
            fBBHeight   = obj.m_fBBHeight;
            
            vfMiddlePlane(1,1) = x;
            vfMiddlePlane(2,1) = y;
            
            lvec(1,1) = fBBLength/2*cos(fBBYaw);
            lvec(2,1) = fBBLength/2*sin(fBBYaw);
            wvec(1,1) = fBBWidth/2*cos(fBBYaw + pi()/2);
            wvec(2,1) = fBBWidth/2*sin(fBBYaw + pi()/2);
            
            % Order of points: [downside] front left, front right, back right, back left [upside] ..., [mid]
            mfPos = zeros(9,3);
            mfPos(9,:) = [x y z];
            
            mfPos(1,1:2)  =  (lvec + wvec + vfMiddlePlane)';
            mfPos(2,1:2)  =  (lvec - wvec + vfMiddlePlane)';
            mfPos(3,1:2)  =  (-lvec - wvec + vfMiddlePlane)';
            mfPos(4,1:2)  =  (-lvec + wvec + vfMiddlePlane)';
            
            for j = 1 : 4
                mfPos(j,3) = z - fBBHeight/2;
            end
            
            mfPos(5:8,:) = mfPos(1:4,:);
            
            for j = 5 : 8
                mfPos(j,3) = z + fBBHeight/2;
            end
        end      
    end
    
    methods (Access = protected)
       function oCopy = copyElement(obj)
            oCopy = copyElement@matlab.mixin.Copyable(obj);
            oCopy.m_oKalman = copy(obj.m_oKalman);
        end 
    end
end

