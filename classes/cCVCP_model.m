classdef cCVCP_model < iModel
    % ---------------------------------------------------------------------------------------------
    % Class cCVPM_model assumes a constant velocity (CV) and a circular path (CP).
    % ---------------------------------------------------------------------------------------------
    properties
        m_fV_init      = 2;      % m/s
        m_fK_init      = 0;
        
        m_fVarPos_init = 100;
        m_fVarV_init   = 10;     % (m/s)^2
        m_fVarK_init   = 1;      % m^2 / deg^2
    end
    
    methods
        function obj = cCVCP_model()
            obj.m_nNumOfStates  = 5;
            obj.m_vfState       = zeros(5,1);  % x, y, yaw, yawRate, vAbs
            obj.m_mfCov         = zeros(5,5);
            
            obj.m_vfState_pre   = obj.m_vfState;
            obj.m_mfCov_pre     = obj.m_mfCov;
        end
        
        function obj = init(obj, oPCMovableLabel, nNumIMM, bDefault)
            % Init model probability
            obj.m_fModelProb = 1/nNumIMM;
            obj.m_fModelProb_init = obj.m_fModelProb;
            
            fRef = oPCMovableLabel.m_vfReferencePoint;
            obj.m_vfState(1,1) = fRef(1,1);
            obj.m_vfState(2,1) = fRef(2,1);
            obj.m_vfState(3,1) = oPCMovableLabel.m_fBBYaw;
                
            if bDefault
                % Init state vector
                obj.m_vfState(4,1) = obj.m_fK_init;
                obj.m_vfState(5,1) = obj.m_fV_init;
                
                 % Init covariance
                obj.m_mfCov(1,1) = obj.m_fVarPos_init;
                obj.m_mfCov(2,2) = obj.m_fVarPos_init;
                obj.m_mfCov(3,3) = obj.m_fVarPos_init;            
                obj.m_mfCov(4,4) = obj.m_fVarK_init; 
                obj.m_mfCov(5,5) = obj.m_fVarV_init;
            else
                % Init state vector
                obj.m_vfState(4,1) = oPCMovableLabel.m_fBBYawRatePerDist;
                obj.m_vfState(5,1) = sqrt(oPCMovableLabel.m_fVxAbs^2 + oPCMovableLabel.m_fVyAbs^2);
                
                % Init covariance
                obj.m_mfCov(1,1) = oPCMovableLabel.m_fVarBBMiddle_x;
                obj.m_mfCov(2,2) = oPCMovableLabel.m_fVarBBMiddle_y;
                obj.m_mfCov(3,3) = oPCMovableLabel.m_fVarBBYaw;             % in deg
                obj.m_mfCov(4,4) = oPCMovableLabel.m_fVarBBYawRatePerDist;  % in deg/m
                obj.m_mfCov(5,5) = oPCMovableLabel.m_fVarVxAbs + oPCMovableLabel.m_fVarVyAbs;   % approximation
            end

            obj.updateSharedSpace();
        end

        function p = getSharedState(obj)
            p = obj.m_vfSharedState;
        end

        function P = getSharedCovariance(obj)
            P =  obj.m_mfSharedCov;
        end
        
        function r = getModelProbability(obj)
            r = obj.m_fModelProb;
        end
        
        function r = getUniqueStateFlags(~)
            r       = zeros(8,1);
            r(8,1)  = 1;
        end
        
        function obj = setMixedSpace(obj, pMixed, PMixed)
            % Set state and covarance (only x and y for stability)
            obj.m_vfState(1,1) = pMixed(1,1);
            obj.m_vfState(2,1) = pMixed(4,1);
            
            varX    = PMixed(1,1);
            varY    = PMixed(4,4);
            covXY   = PMixed(1,4);
            
            obj.m_mfCov(1,1) = varX;
            obj.m_mfCov(2,2) = varY;
            
            obj.m_mfCov(1,2) = covXY;
            obj.m_mfCov(2,1) = covXY;

            % Set shared space
            obj.m_vfSharedState     = pMixed;
            obj.m_mfSharedCov       = PMixed;
        end

        function updateSharedSpace(obj)
            % Shared state
            x   = obj.m_vfState(1,1);
            y   = obj.m_vfState(2,1);
            yaw = obj.m_vfState(3,1) * pi()/180;
            k   = obj.m_vfState(4,1);
            v   = obj.m_vfState(5,1);
            c   = 360/(2*pi());
            
            if k > 0 
                yawA = pi()/2;    % direction of zentripetal acceleration
            else
                yawA = -pi()/2;
            end
            
            % x, vx, ax, y, vy, ax, yaw, k
            obj.m_vfSharedState(1,1) = x;  
            obj.m_vfSharedState(2,1) = v * cos(yaw);   
            obj.m_vfSharedState(3,1) = k*v^2/c * cos(yaw + yawA);
            obj.m_vfSharedState(4,1) = y;
            obj.m_vfSharedState(5,1) = v * sin(yaw);
            obj.m_vfSharedState(6,1) = k*v^2/c * sin(yaw + yawA);
            obj.m_vfSharedState(7,1) = yaw*180/pi();
            obj.m_vfSharedState(8,1) = k;

            % Transform covariance matrix into shared space via jacobian of transformation function
            J = [ 1, 0,                          0,                       0,                         0;
                  0, 0,                -v*sin(yaw),                       0,                  cos(yaw);
                  0, 0, -(k*v^2*sin(yaw + yawA))/c, (v^2*cos(yaw + yawA))/c, (2*k*v*cos(yaw + yawA))/c;
                  0, 1,                          0,                       0,                         0;
                  0, 0,                 v*cos(yaw),                       0,                  sin(yaw);
                  0, 0,  (k*v^2*cos(yaw + yawA))/c, (v^2*sin(yaw + yawA))/c, (2*k*v*sin(yaw + yawA))/c;
                  0, 0,                          1,                       0,                         0;
                  0, 0,                          0,                       1,                         0];
                  
            obj.m_mfSharedCov = J*obj.m_mfCov*J';       
        end

        function obj = reinitVariance(obj, fVarInit)
            obj.m_mfCov(4,4) = fVarInit;
            obj.m_mfCov(5,5) = fVarInit;
        end
        
        function obj = reinit(obj)
            % Reset dynamics
            obj.m_vfState(4,1) = obj.m_fK_init;
            obj.m_vfState(5,1) = obj.m_fV_init;
            
            % Reset covariance
            obj.m_mfCov = zeros(5,5);
            
            obj.m_mfCov(1,1) = obj.m_fVarPos_init;
            obj.m_mfCov(2,2) = obj.m_fVarPos_init;
            obj.m_mfCov(3,3) = obj.m_fVarPos_init;
            obj.m_mfCov(4,4) = obj.m_fVarK_init;
            obj.m_mfCov(5,5) = obj.m_fVarV_init;
            
            obj.m_fModelProb = obj.m_fModelProb_init;
            obj.updateSharedSpace();
        end
        
        % Predict current state
        function obj = predict(obj, fDT)
            %% Save state and cov
            obj.m_vfState_pre   = obj.m_vfState;
            obj.m_mfCov_pre     = obj.m_mfCov;
            
            %% System transistion
            x   = obj.m_vfState(1,1);
            y   = obj.m_vfState(2,1);
            yaw = obj.m_vfState(3,1);
            k   = obj.m_vfState(4,1);
            v   = obj.m_vfState(5,1);
            
            yaw_rad = yaw*pi()/180;
            % Compute delta yaw
            fDyaw       = v*k*fDT;
            fDyaw_rad   = fDyaw*pi()/180;
            fDyaw_rad   = fDyaw_rad + 10^-12; % avoid devision by zero
            
            SW = sin(fDyaw_rad)/fDyaw_rad;
            CW = (1-cos(fDyaw_rad))/fDyaw_rad;
            fFac_x = (SW*cos(yaw_rad) - CW*sin(yaw_rad));
            fFac_y = (CW*cos(yaw_rad) + SW*sin(yaw_rad));
            
            % Turn rate (yawRatePerDist) and velocity constant
            x_p   = x + v*fDT*fFac_x;
            y_p   = y + v*fDT*fFac_y;
            yaw_p = yaw + fDyaw;
            
            % Update state vector
            obj.m_vfState(1,1) = x_p;
            obj.m_vfState(2,1) = y_p;
            obj.m_vfState(3,1) = yaw_p;
            
            %% Process noise as diagonal matrix
            fSigX   = .1;      % m
            fSigY   = .1;      % m
            fSigYaw = .02;      % deg       .04
            fSigK   = .02;      % deg/m     .04
            fSigV   = .05;      % m/s       .08

            % X and y are not correlated (point mass)
            Q = zeros(5,5);
            Q(1,1) = fSigX^2;
            Q(2,2) = fSigY^2;
            Q(3,3) = fSigYaw^2;
            Q(4,4) = fSigK^2;
            Q(5,5) = fSigV^2;
            
            %% Jacobi matrix
            % Avoid devision by zero
            if k < 10^-6
                k = 10^-6;
            end
            if fDyaw_rad < 10^-6
                fDyaw_rad = 10^-6;
            end
            
            % Build up matrix
            SW = sin(fDyaw_rad)/fDyaw_rad;
            CW = (1-cos(fDyaw_rad))/fDyaw_rad;
            AW = 1/k*(cos(fDyaw_rad) - sin(fDyaw_rad)/fDyaw_rad);
            BW = 1/k*(sin(fDyaw_rad) - (1-cos(fDyaw_rad))/fDyaw_rad);
            
            yaw_p_rad = yaw_p * pi()/180;
            fF13 = -fDT*v*(SW*sin(yaw_p_rad) + CW*cos(yaw_p_rad));
            fF14 = fDT*v*(AW*cos(yaw_p_rad) - BW*sin(yaw_p_rad));
            fF15 = fDT*(cos(yaw_p_rad)*cos(fDyaw_rad) - sin(yaw_p_rad)*sin(fDyaw_rad));
            fF23 = fDT*v*(SW*cos(yaw_p_rad) - CW*sin(yaw_p_rad));
            fF24 = fDT*v*(AW*sin(yaw_p_rad) + BW*cos(yaw_p_rad));
            fF25 = fDT*(cos(yaw_p_rad)*sin(fDyaw_rad) + sin(yaw_p_rad)*cos(fDyaw_rad));
            
            F = [
                1   0     fF13    fF14    fF15;
                0   1     fF23    fF24    fF25;
                0   0     1       fDT        0;
                0   0     0       1          0;
                0   0     0       0          1;
                ];
            
            %% Update covariance
            P = obj.m_mfCov;
            obj.m_mfCov = F*P*F' + Q;
            
            obj.updateSharedSpace();
        end
        
        % Update state vector by position and yaw: vfMeasurements = [x; y; yaw]
        function obj = update(obj, vfMeasurements, vfVariances)
            % Save state and cov
            obj.m_bStable       = 1;
            obj.m_vfState_pre   = obj.m_vfState;
            obj.m_mfCov_pre     = obj.m_mfCov;
            
            % Measurement
            yaw_measure = vfMeasurements(3,1);
            yaw_state   = obj.m_vfState(3,1);
            yaw_measure = adaptYawMeasurement(yaw_measure, yaw_state);
            vfMeasurements(3,1) = yaw_measure;
            
            y = vfMeasurements;
            R = zeros(3,3);     % measurement covariance
            R(1,1) = vfVariances(1,1);
            R(2,2) = vfVariances(2,1);
            R(3,3) = vfVariances(3,1);
            
            % Observer matrix
            C = zeros(3,5);
            C(1,1) = 1;
            C(2,2) = 1;
            C(3,3) = 1;
            
            % Kalman Gain
            P = obj.m_mfCov;
            S = R + C*P*C';
            Sinv = inv(S);
            K = P*C'*Sinv;
            
            % Update state vector
            p = obj.m_vfState;
            r = y - C*p;    % residuum
            obj.m_vfState = p + K*(r);
            
            % Update covariance, Joseph form
            P = obj.m_mfCov;
            
            I = eye(5,5);
            obj.m_mfCov = (I-K*C)*P*(I-K*C)'+K*R*K';
            
            % Limit
            obj.limitStateEntries();
            
            % Update model plausibility
            obj.m_fModelPl = 1/sqrt(norm(2*pi().*S))*exp(-.5*r'*Sinv*r);
            obj.m_fModelPl = obj.m_fModelPl + 10^-12;   % avoid division by 0
            
            % Test for stability
            if sum(sum(isnan(obj.m_mfCov))) 
                obj.reinit();
                obj.m_fModelPl = 10^-15;
                obj.m_bStable  = 0;
            end
            
            % Update shared space
            obj.updateSharedSpace();
        end
  
        function obj = changeReferencePoint(obj, vfDelta, sChangeState, fWidth)
            obj.m_vfState(1,1) = obj.m_vfState(1,1) + vfDelta(1,1);
            obj.m_vfState(2,1) = obj.m_vfState(2,1) + vfDelta(2,1);
            
            k = obj.m_vfState(4,1);

            k = k + 10^-12;
            % Change turn rate according to new radius
            delta_radius = 0;
            if      k > 0
                if      strcmp(sChangeState, 'right')
                    delta_radius = fWidth;
                elseif  strcmp(sChangeState, 'left')
                    delta_radius = -fWidth;
                end
            elseif  k < 0
                if      strcmp(sChangeState, 'right')
                    delta_radius = -fWidth;
                elseif  strcmp(sChangeState, 'left')
                    delta_radius = fWidth;
                end
            end
            
            % Update radius
            radius = 360/(2*pi()*k) + delta_radius;
            
            % Update yaw rate per distance
            obj.m_vfState(4,1) = 360/(2*pi()*radius);
            
            obj.updateSharedSpace();
        end
        
        function obj = setPosition(obj, x, y, yaw)
            yaw = adaptYawMeasurement(yaw, obj.m_vfState(3,1));
            obj.m_vfState(1,1) = x;
            obj.m_vfState(2,1) = y;
            obj.m_vfState(3,1) = yaw;
            
            obj.updateSharedSpace();
        end
        
        function obj = compensateEgoMovement(obj, fDT, vfEgoMov)
            % Parse ego movement vector
            vel_ego = [vfEgoMov(1,1); vfEgoMov(2,1)];
            acc_ego = [vfEgoMov(3,1); vfEgoMov(4,1)];
            k_ego   =  vfEgoMov(5,1);   % ego yaw rate
            
            % Delta ego yaw 
            yaw_ego = -k_ego*fDT*pi()/180;
            
            % Get state
            pos = [obj.m_vfState(1,1); obj.m_vfState(2,1)];
            yaw = obj.m_vfState(3,1);
            
            % Compensate translation
            pos = pos - (vel_ego.*fDT + .5*fDT^2.*acc_ego); 
            
            % Compensate rotation
            R_ego = [cos(yaw_ego) -sin(yaw_ego); sin(yaw_ego) cos(yaw_ego)];
            
            pos = R_ego*pos;
            yaw = yaw + yaw_ego*180/pi();
            
            % Set state
            obj.m_vfState(1,1) = pos(1,1);
            obj.m_vfState(2,1) = pos(2,1);
            obj.m_vfState(3,1) = yaw;
            
            obj.updateSharedSpace();
        end
        
        function obj = limitStateEntries(obj)
            k_lim = 10;     % deg/m
            v_lim = 27;     % m/s
            if abs(obj.m_vfState(4,1)) > k_lim     
                obj.m_vfState(4,1) = (obj.m_vfState(4,1) / abs(obj.m_vfState(4,1))) * k_lim;
            end
            
            if obj.m_vfState(5,1) > v_lim
               obj.m_vfState(5,1) = v_lim;
            end
        end
        
        function obj = setDynamicsToZero(obj)
            vfDynamicStates = [4;5];
            for i = 1 : size(vfDynamicStates, 1)
                obj.m_vfState(i,1) = 0;
                obj.m_mfCov(i,i) = 0;
            end
        end
    end
end

