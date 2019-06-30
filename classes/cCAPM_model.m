classdef cCAPM_model < iModel
    % ---------------------------------------------------------------------------------------------
    % Class cCAPM_model assumes a point mass (PM) with constant acceleration (CA).
    % ---------------------------------------------------------------------------------------------
    properties
        m_fV_init      = 2;      % m/s
        m_fA_init      = .5;     % m/s^2
        
        m_fVarV_init   = 5;      % (m/s)^2
        m_fVarA_init   = 2.5;    % m^2/s^4
        m_fVarPos_init = 100;    % m^2 / deg^2
    end
    
    methods 
        function obj = cCAPM_model()
            obj.m_nNumOfStates  = 7;      
            obj.m_vfState       = zeros(7,1);  % x, vXAbs, aXAbs, y, vYAbs, aYAbs, yaw
            obj.m_mfCov         = zeros(7,7);  
            
            obj.m_vfState_pre   = obj.m_vfState;
            obj.m_mfCov_pre     = obj.m_mfCov;
        end
        
        function obj = init(obj, oPCMovableLabel, nNumIMM, bDefault)
            % Init model probability 
            obj.m_fModelProb = 1/nNumIMM;
            obj.m_fModelProb_init = obj.m_fModelProb;
            
            fRef = oPCMovableLabel.m_vfReferencePoint;
            obj.m_vfState(1,1) = fRef(1,1);
            obj.m_vfState(4,1) = fRef(2,1);
            obj.m_vfState(7,1) = oPCMovableLabel.m_fBBYaw;
                
            if bDefault
                % Init state vector
                yaw_rad = obj.m_vfState(5,1) * pi()/180;
                
                obj.m_vfState(2,1) = obj.m_fV_init * cos(yaw_rad);
                obj.m_vfState(3,1) = obj.m_fA_init * cos(yaw_rad);
                obj.m_vfState(5,1) = obj.m_fV_init * sin(yaw_rad);
                obj.m_vfState(6,1) = obj.m_fA_init * sin(yaw_rad);
                
                % Init covariance
                obj.m_mfCov(1,1) = obj.m_fVarPos_init;
                obj.m_mfCov(2,2) = obj.m_fVarV_init;
                obj.m_mfCov(3,3) = obj.m_fVarA_init;
                obj.m_mfCov(4,4) = obj.m_fVarPos_init;
                obj.m_mfCov(5,5) = obj.m_fVarV_init;
                obj.m_mfCov(6,6) = obj.m_fVarA_init;
                obj.m_mfCov(7,7) = obj.m_fVarPos_init;
            else
                % Init state vector
                obj.m_vfState(2,1) = oPCMovableLabel.m_fVxAbs;
                obj.m_vfState(3,1) = oPCMovableLabel.m_fAxAbs;
                obj.m_vfState(5,1) = oPCMovableLabel.m_fVyAbs;
                obj.m_vfState(6,1) = oPCMovableLabel.m_fAyAbs;
                
                % Init covariance
                obj.m_mfCov(1,1) = oPCMovableLabel.m_fVarBBMiddle_x;
                obj.m_mfCov(2,2) = oPCMovableLabel.m_fVarVxAbs;
                obj.m_mfCov(3,3) = oPCMovableLabel.m_fVarAxAbs;
                obj.m_mfCov(4,4) = oPCMovableLabel.m_fVarBBMiddle_y;
                obj.m_mfCov(5,5) = oPCMovableLabel.m_fVarVyAbs;
                obj.m_mfCov(6,6) = oPCMovableLabel.m_fVarAyAbs;
                obj.m_mfCov(7,7) = oPCMovableLabel.m_fVarBBYaw;
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
            r = zeros(8,1);
        end

        function obj = setMixedSpace(obj, pMixed, PMixed)
            % Set state and covarance (only x and y for stability)
            obj.m_vfState(1,1) = pMixed(1,1);
            obj.m_vfState(4,1) = pMixed(4,1);
            
            obj.m_mfCov(1,1) = PMixed(1,1);
            obj.m_mfCov(4,4) = PMixed(4,4);
            obj.m_mfCov(1,4) = PMixed(1,4);
            obj.m_mfCov(4,1) = PMixed(4,1); 
 
            % Set shared space
            obj.m_vfSharedState     = pMixed;
            obj.m_mfSharedCov       = PMixed;
        end
        
        function obj = updateSharedSpace(obj)
            % Shared state
            obj.m_vfSharedState(1:7,1)  = obj.m_vfState;
            
            % Shared covariance
            obj.m_mfSharedCov(1:7,1:7)  = obj.m_mfCov;
        end
        
        function obj = reinitVariance(obj, fVarInit)
            obj.m_mfCov(2,2) = fVarInit;
            obj.m_mfCov(3,3) = fVarInit;
            obj.m_mfCov(5,5) = fVarInit;
            obj.m_mfCov(6,6) = fVarInit;
        end
        
        function obj = reinit(obj)
            % Reset covariance
            obj.m_mfCov = zeros(7,7);
            
            obj.m_mfCov(1,1) = obj.m_fVarPos_init;
            obj.m_mfCov(2,2) = obj.m_fVarV_init;
            obj.m_mfCov(3,3) = obj.m_fVarA_init;
            obj.m_mfCov(4,4) = obj.m_fVarPos_init;
            obj.m_mfCov(5,5) = obj.m_fVarV_init;
            obj.m_mfCov(6,6) = obj.m_fVarA_init;
            obj.m_mfCov(7,7) = obj.m_fVarPos_init;
            
            % Reset dynamics
            fYaw = obj.m_vfState(7,1)*pi()/180;
            obj.m_vfState(2,1) = obj.m_fV_init * cos(fYaw);
            obj.m_vfState(3,1) = obj.m_fA_init * cos(fYaw);
            obj.m_vfState(5,1) = obj.m_fV_init * sin(fYaw);
            obj.m_vfState(6,1) = obj.m_fA_init * sin(fYaw);
            
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
            vx  = obj.m_vfState(2,1);
            ax  = obj.m_vfState(3,1);
            y   = obj.m_vfState(4,1);
            vy  = obj.m_vfState(5,1);
            ay  = obj.m_vfState(6,1);
            
            % Acceleration is constant
            x_p   = x + vx*fDT + .5*ax*fDT^2;
            y_p   = y + vy*fDT + .5*ay*fDT^2;
            vx_p  = vx + ax*fDT;
            vy_p  = vy + ay*fDT;
            
            % Update state vector
            obj.m_vfState(1,1) = x_p;
            obj.m_vfState(2,1) = vx_p;
            obj.m_vfState(4,1) = y_p;
            obj.m_vfState(5,1) = vy_p;
            
            %% Process noise
            fSigA       = 0.05;
            fSigYaw     = 0.1;
            
            % Discrete Wiener Process Acceleration Model
            Q = zeros(6,6);
            Qco = [
                    1/4*fDT^4   1/2*fDT^3   1/2*fDT^2;
                    1/2*fDT^3       fDT^2       fDT;
                    1/2*fDT^2       fDT           1;
                  ];
            % X and y are not correlated (point mass)
            Q(1:3,1:3) = Qco;
            Q(4:6,4:6) = Qco;
            Q = fSigA^2 .* Q;
            
            %% Jacobi matrix
             F = [
                    1   fDT     1/2*fDT^2       0       0       0;
                    0     1           fDT       0       0       0;
                    0     0             1       0       0       0;
                    0     0             0       1       fDT     1/2*fDT^2;
                    0     0             0       0       1       fDT;
                    0     0             0       0       0       1;
                 ];
             
             %% Update covariance
             P = obj.m_mfCov(1:6,1:6);
             obj.m_mfCov(1:6,1:6)   = F*P*F' + Q;
             obj.m_mfCov(7,7)       = obj.m_mfCov(7,7) + fSigYaw^2; 
             
             obj.updateSharedSpace();
        end
        
        % Update state vector by position: vfMeasurements = [x; y]
        function obj = update(obj, vfMeasurements, vfVariances)
            % Save state and cov
            obj.m_bStable       = 1;
            obj.m_vfState_pre   = obj.m_vfState;
            obj.m_mfCov_pre     = obj.m_mfCov;
            
            % Measurement
            yaw_measure = vfMeasurements(3,1);
            yaw_state   = obj.m_vfState(7,1);
            yaw_measure = adaptYawMeasurement(yaw_measure, yaw_state);
            vfMeasurements(3,1) = yaw_measure;
            
            y = vfMeasurements(1:3,1);   % update by x, y, yaw
            R = zeros(3,3);              % measurement covariance
            R(1,1) = vfVariances(1,1);
            R(2,2) = vfVariances(2,1);
            R(3,3) = vfVariances(3,1);
            
            % Observer matrix
            C = zeros(3,7);
            C(1,1) = 1;
            C(2,4) = 1;
            C(3,7) = 1;
            
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
            
            I = eye(7,7);
            obj.m_mfCov = (I-K*C)*P*(I-K*C)'+K*R*K';
            
            % Limit
            obj.limitStateEntries();

            % Update model plausibility
            obj.m_fModelPl = 1/sqrt(norm((2*pi()).*S))*exp(-.5*r'*Sinv*r);           
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
       
        function obj = changeReferencePoint(obj, vfDelta, ~, ~)
            obj.m_vfState(1,1) = obj.m_vfState(1,1) + vfDelta(1,1);
            obj.m_vfState(4,1) = obj.m_vfState(4,1) + vfDelta(2,1);
            
            obj.updateSharedSpace();
        end
        
        function obj = setPosition(obj, x, y, yaw)
            yaw = adaptYawMeasurement(yaw, obj.m_vfState(7,1));
            obj.m_vfState(1,1) = x;
            obj.m_vfState(4,1) = y;
            obj.m_vfState(7,1) = yaw;
            
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
            pos = [obj.m_vfState(1,1); obj.m_vfState(4,1)];
            vel = [obj.m_vfState(2,1); obj.m_vfState(5,1)];
            acc = [obj.m_vfState(3,1); obj.m_vfState(6,1)];
            yaw = obj.m_vfState(7,1);
            
            % Compensate translation
            pos = pos - (vel_ego.*fDT + .5*fDT^2.*acc_ego); 
            
            % Compensate rotation
            R_ego = [cos(yaw_ego) -sin(yaw_ego); sin(yaw_ego) cos(yaw_ego)];
            
            pos = R_ego*pos;
            vel = R_ego*vel;
            acc = R_ego*acc;
            yaw = yaw + yaw_ego*180/pi();
            
            % Set state
            obj.m_vfState(1,1) = pos(1,1);
            obj.m_vfState(4,1) = pos(2,1);
            obj.m_vfState(2,1) = vel(1,1);
            obj.m_vfState(5,1) = vel(2,1);
            obj.m_vfState(3,1) = acc(1,1);
            obj.m_vfState(6,1) = acc(2,1);
            obj.m_vfState(7,1) = yaw;
            
            obj.updateSharedSpace();
        end
        
        function obj = limitStateEntries(obj)
            a_lim = 10; % m/s^2
            v_lim = 27; % m/s
            if abs(obj.m_vfState(2,1)) > v_lim     
                obj.m_vfState(2,1) = (obj.m_vfState(2,1) / abs(obj.m_vfState(2,1))) * v_lim;
            end
            if abs(obj.m_vfState(5,1)) > v_lim     
                obj.m_vfState(4,1) = (obj.m_vfState(4,1) / abs(obj.m_vfState(4,1))) * v_lim;
            end
            
            if abs(obj.m_vfState(3,1)) > a_lim     
                obj.m_vfState(3,1) = (obj.m_vfState(3,1) / abs(obj.m_vfState(3,1))) * a_lim;
            end
            if abs(obj.m_vfState(6,1)) > a_lim     
                obj.m_vfState(6,1) = (obj.m_vfState(6,1) / abs(obj.m_vfState(6,1))) * a_lim;
            end
        end
        
        function obj = setDynamicsToZero(obj)
            vfDynamicStates = [2;3;5;6;];
            for i = 1 : size(vfDynamicStates, 1)
                obj.m_vfState(i,1) = 0;
                obj.m_mfCov(i,i) = 0;
            end
        end
    end
end

