classdef cCVPM_model < iModel
    % ---------------------------------------------------------------------------------------------
    % Class cCVPM_model assumes a point mass (PM) with constant velocity (CV). 
    % The bounding box yaw is measured but not used for predictions.
    % ---------------------------------------------------------------------------------------------
    properties
        m_fV_init      = 1;      % m/s
        m_fVarV_init   = 5;      % (m/s)^2
        m_fVarPos_init = 100;    % m^2
    end
    
    methods 
        function obj = cCVPM_model()
            obj.m_nNumOfStates  = 5;      
            obj.m_vfState       = zeros(5,1);  % x, vXAbs, y, vYAbs, yaw
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
            obj.m_vfState(3,1) = fRef(2,1);
            obj.m_vfState(5,1) = oPCMovableLabel.m_fBBYaw;
                
            if bDefault
                % Init state vector
                obj.m_vfState(2,1) = obj.m_fV_init;
                obj.m_vfState(4,1) = obj.m_fV_init;

                % Init covariance
                obj.m_mfCov(1,1) = obj.m_fVarPos_init;
                obj.m_mfCov(2,2) = obj.m_fVarV_init;
                obj.m_mfCov(3,3) = obj.m_fVarPos_init;
                obj.m_mfCov(4,4) = obj.m_fVarV_init;
                obj.m_mfCov(5,5) = obj.m_fVarPos_init;
            else
                % Init state vector
                obj.m_vfState(2,1) = oPCMovableLabel.m_fVxAbs;
                obj.m_vfState(4,1) = oPCMovableLabel.m_fVyAbs;
                
                % Init covariance
                obj.m_mfCov(1,1) = oPCMovableLabel.m_fVarBBMiddle_x;
                obj.m_mfCov(2,2) = oPCMovableLabel.m_fVarVxAbs;
                obj.m_mfCov(3,3) = oPCMovableLabel.m_fVarBBMiddle_y;
                obj.m_mfCov(4,4) = oPCMovableLabel.m_fVarVyAbs;
                obj.m_mfCov(5,5) = oPCMovableLabel.m_fVarBBYaw;
            end
            
            obj.updateSharedSpace();
        end
        
        function p = getSharedState(obj)
            p = obj.m_vfSharedState;
        end
        
        function P = getSharedCovariance(obj)
            P = obj.m_mfSharedCov;
        end
        
        function r = getModelProbability(obj)
            r = obj.m_fModelProb;
        end
        
        function r = getUniqueStateFlags(~)
            r = zeros(8,1);
        end
        
        function obj = setMixedSpace(obj, pMixed, PMixed)
            % not implemented
        end
        
        function obj = reinitVariance(obj, fVarInit)
            obj.m_mfCov(2,2) = fVarInit;
            obj.m_mfCov(4,4) = fVarInit;
        end
        
        function obj = reinit(obj)
            % Reset covariance
            obj.m_mfCov = zeros(5,5);
            
            obj.m_mfCov(1,1) = obj.m_fVarPos_init;
            obj.m_mfCov(2,2) = obj.m_fVarV_init;
            obj.m_mfCov(3,3) = obj.m_fVarPos_init;
            obj.m_mfCov(4,4) = obj.m_fVarV_init;
            obj.m_mfCov(5,5) = obj.m_fVarPos_init;
            
            % Reset dynamics
            fYaw = obj.m_vfState(5,1)*pi()/180;
            
            obj.m_vfState(2,1) = obj.m_fV_init * cos(fYaw);
            obj.m_vfState(4,1) = obj.m_fV_init * sin(fYaw);
            
            obj.m_fModelProb = obj.m_fModelProb_init;
            obj.updateSharedSpace();
        end
        
        function obj = updateSharedSpace(obj)
            % Shared state
            obj.m_vfSharedState(1,1)  = obj.m_vfState(1,1);
            obj.m_vfSharedState(2,1)  = obj.m_vfState(2,1);
            obj.m_vfSharedState(3,1)  = 0;
            obj.m_vfSharedState(4,1)  = obj.m_vfState(3,1);
            obj.m_vfSharedState(5,1)  = obj.m_vfState(4,1);
            obj.m_vfSharedState(6,1)  = 0;
            obj.m_vfSharedState(7,1)  = obj.m_vfState(5,1);
            obj.m_vfSharedState(8,1)  = 0;    % k = 0
            
            % Shared covariance
            fSigK = 0.2; 
            fSigA = 0.2;
            
            varX    = obj.m_mfCov(1,1);
            varVx   = obj.m_mfCov(2,2);
            varY    = obj.m_mfCov(3,3); 
            varVy   = obj.m_mfCov(4,4);
            varYaw  = obj.m_mfCov(5,5);
            
            covXVx  = obj.m_mfCov(1,2);
            covYVy  = obj.m_mfCov(3,4);
            
            obj.m_mfSharedCov(1,1) = varX;
            obj.m_mfSharedCov(2,2) = varVx;
            obj.m_mfSharedCov(4,4) = varY;
            obj.m_mfSharedCov(5,5) = varVy;
            obj.m_mfSharedCov(7,7) = varYaw;
            
            obj.m_mfSharedCov(1,2) = covXVx;    obj.m_mfSharedCov(2,1) = covXVx;
            obj.m_mfSharedCov(4,5) = covYVy;    obj.m_mfSharedCov(5,4) = covYVy;
            
            obj.m_mfSharedCov(3,3) = fSigA^2;
            obj.m_mfSharedCov(6,6) = fSigA^2;
            obj.m_mfSharedCov(8,8) = fSigK^2;
        end
        
        % Predict current state
        function obj = predict(obj, fDT)
            %% Save state and cov
            obj.m_vfState_pre   = obj.m_vfState;
            obj.m_mfCov_pre     = obj.m_mfCov;
            
            %% System transistion
            x   = obj.m_vfState(1,1);
            vx  = obj.m_vfState(2,1);
            y   = obj.m_vfState(3,1);
            vy  = obj.m_vfState(4,1);
            % Acceleration is constant
            x_p   = x + vx*fDT;
            y_p   = y + vy*fDT;
            % Update state vector
            obj.m_vfState(1,1) = x_p;
            obj.m_vfState(3,1) = y_p;
            
            %% Process noise
            fSigV   = .25;   % m/s
            fSigYaw = .5;    % deg
            % Discrete Wiener Process Acceleration Model
            Q = zeros(4,4);
            Qco = [
                    1/4*fDT^4   1/2*fDT^3;
                    1/2*fDT^3       fDT^2;
                  ];
            % X and y are not correlated (point mass)
            Q(1:2,1:2) = Qco;
            Q(3:4,3:4) = Qco;
            Q = fSigV^2 .* Q;
            
            %% Jacobi matrix
             F = [
                    1   fDT     0     0;  
                    0     1     0     0;
                    0     0     1    fDT;
                    0     0     0     1;
                 ];
             
             %% Update covariance
             P = obj.m_mfCov(1:4, 1:4);
             obj.m_mfCov(1:4,1:4) = F*P*F' + Q; 
             obj.m_mfCov(5,5)     = obj.m_mfCov(5,5) + fSigYaw^2;
             
             obj.updateSharedSpace();
        end
        
        % Update state vector by position: vfMeasurements = [x; y]
        function obj = update(obj, vfMeasurements, vfVariances)
            % Save state and cov
            obj.m_vfState_pre   = obj.m_vfState;
            obj.m_mfCov_pre     = obj.m_mfCov;
            obj.m_bStable       = 1;
            % Measurement
            y = vfMeasurements(1:3,1);
            R = zeros(3,3);     % measurement covariance
            R(1,1) = vfVariances(1,1);
            R(2,2) = vfVariances(2,1);
            R(3,3) = vfVariances(3,1);
            
            % Observer matrix
            C = zeros(3,5);
            C(1,1) = 1;
            C(2,3) = 1;
            C(3,5) = 1;
            
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
            obj.m_fModelPl = 1/sqrt(norm((2*pi()).*S))*exp(-.5*r'*Sinv*r);
            obj.m_fModelPl = obj.m_fModelPl + 10^-12;   % avoid division by 0
            obj.updateSharedSpace();
            
            % Test for stability
            if sum(sum(isnan(obj.m_mfCov))) 
                obj.reinit();
                obj.m_fModelPl = 10^-15;
                obj.m_bStable  = 0;
            end
        end
        
        function obj = setPosition(obj, x, y, yaw)
            yaw = adaptYawMeasurement(yaw, obj.m_vfState(5,1));
            obj.m_vfState(1,1) = x;
            obj.m_vfState(3,1) = y;
            obj.m_vfState(5,1) = yaw;
            
            obj.updateSharedSpace();
        end
        
        function obj = changeReferencePoint(obj, vfDelta, ~, ~)
            obj.m_vfState(1,1) = obj.m_vfState(1,1) + vfDelta(1,1);
            obj.m_vfState(3,1) = obj.m_vfState(3,1) + vfDelta(2,1);
            
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
            pos = [obj.m_vfState(1,1); obj.m_vfState(3,1)];
            vel = [obj.m_vfState(2,1); obj.m_vfState(4,1)];
            yaw = obj.m_vfState(5,1);
            
            % Compensate translation
            pos = pos - (vel_ego.*fDT + .5*fDT^2.*acc_ego); 
            
            % Compensate rotation
            R_ego = [cos(yaw_ego) -sin(yaw_ego); sin(yaw_ego) cos(yaw_ego)];
            
            pos = R_ego*pos;
            vel = R_ego*vel;
            yaw = yaw + yaw_ego*180/pi();
            
            % Set state
            obj.m_vfState(1,1) = pos(1,1);
            obj.m_vfState(3,1) = pos(2,1);
            obj.m_vfState(2,1) = vel(1,1);
            obj.m_vfState(4,1) = vel(2,1);
            obj.m_vfState(5,1) = yaw;
            
            obj.updateSharedSpace();
        end
        
        function obj = limitStateEntries(obj)
            v_lim = 27; % m/s
            if abs(obj.m_vfState(2,1)) > v_lim     
                obj.m_vfState(2,1) = (obj.m_vfState(2,1) / abs(obj.m_vfState(2,1))) * v_lim;
            end
            if abs(obj.m_vfState(4,1)) > v_lim     
                obj.m_vfState(4,1) = (obj.m_vfState(4,1) / abs(obj.m_vfState(4,1))) * v_lim;
            end
        end
        
        function obj = setDynamicsToZero(obj)
            vfDynamicStates = [2;4];
            for i = 1 : size(vfDynamicStates, 1)
                obj.m_vfState(i,1) = 0;
                obj.m_mfCov(i,i) = 0;
            end
        end
    end
end

