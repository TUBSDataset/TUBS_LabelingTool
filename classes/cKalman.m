classdef cKalman < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cKalman implements an interacting, multiple model extended Kalman Filter (IMM-eKF).
    % For pedestrians a CVPM (constant velocity, point mass) model is used. All other movable classes
    % rely on the IMM-eKF algorithm utalizing a CAPM (constant acceleration, point mass) and a CVCP model
    % (constant velocity, circular path). Currently, the models interact only with x and y states for stability reasons.
    % In order to interact more states, an unscented transform of the CVCP model into the shared state space may be considered.
    % The IMM output is fused using all states.
    % ---------------------------------------------------------------------------------------------
    properties
        m_vfFusedState;           % fused state in shared space (IMM filtered)
        m_mfFusedCovariance;      % fused cov matrix
        
        m_vfFusedState_pre;       % predecessor state
        m_mfFusedCovariance_pre;  % predecessor covariance
        
        m_bInit;                  % init flag
        m_nNumModels;             % number of models available, length of m_States
        m_clModelVec;             % vector or estimated models (iModel) (cell array)
        m_nIdxIMM_First;          % index of frist IMM-Model
        m_nIdxIMM_Last;           % index of last IMM-Model
        m_nNumIMM;                % number of IMM models
        m_nIdxCVPM;               % index of CVPM model for pedestrians
        m_nIdxCVCP;               % index of CVPM model 
        
        m_mfUT;                   % matrix of model transistion probabilities. Notation: m_mfUT(i,j) means "into i | given j"
        m_mfUT_AP;                % matrix of apriori transistion probabilities
        m_vfC;                    % vector of normalization values
    end
    
    methods
        function obj = cKalman()
            obj.m_vfFusedState        = zeros(8,1);     % Shared state space for IMM-models: x, vXAbs, aXAbs, y, vYAbs, aYAbs, yaw, k
            obj.m_mfFusedCovariance   = zeros(8,8);
            
            obj.m_vfFusedState_pre        = obj.m_vfFusedState;
            obj.m_mfFusedCovariance_pre   = obj.m_mfFusedCovariance;
            
            obj.m_bInit               = 0;
            obj.m_clModelVec          = {cCVPM_model(); cCAPM_model(); cCVCP_model()};
            obj.m_nIdxCVPM            = 1;
            obj.m_nIdxCVCP            = 3;
            obj.m_nIdxIMM_First       = 2;
            obj.m_nIdxIMM_Last        = 3;
            
            obj.m_nNumIMM       = obj.m_nIdxIMM_Last - obj.m_nIdxIMM_First + 1;
            obj.m_nNumModels    = size(obj.m_clModelVec,1);
            obj.m_mfUT          = zeros(obj.m_nNumModels, obj.m_nNumModels);
            obj.m_mfUT_AP       = zeros(obj.m_nNumModels, obj.m_nNumModels);    % m_mfUT_AP(i,j)
            
            % Init apriori and transistion probability matrix
            nFirst = obj.m_nIdxIMM_First;
            nLast  = obj.m_nIdxIMM_Last;
            
            % All models have the same transistion probability, initially
            obj.m_mfUT(nFirst:nLast, nFirst:nLast) = ones(obj.m_nNumIMM, obj.m_nNumIMM) .* 1/obj.m_nNumIMM;
            
            % Apriori transistion probability is low for stability
            fProbRemain = 0.9;                      % probability for a model to remain in its state
            for i = nFirst : nLast
                obj.m_mfUT_AP(i,:) = (1-fProbRemain)/obj.m_nNumIMM;
                obj.m_mfUT_AP(i,i) = fProbRemain;
            end
            
            for i = obj.m_nIdxIMM_First  : obj.m_nIdxIMM_Last
                obj.m_clModelVec{i,1}.m_fModelProb = 1/obj.m_nNumIMM;
            end
            
            % Initialize normalization
            obj.m_vfC = zeros(obj.m_nNumModels, 1);
            for j = obj.m_nIdxIMM_First  : obj.m_nIdxIMM_Last
                fC = 0;
                for i = obj.m_nIdxIMM_First  : obj.m_nIdxIMM_Last
                    fModelProb = obj.m_clModelVec{i,1}.m_fModelProb;
                    fC = fC + obj.m_mfUT_AP(i,j)*fModelProb;
                end
                obj.m_vfC(j,1) = fC;
            end
        end
        
        % Init all models
        function obj = initModels(obj, oPCMovableLabel, bDefaultDynamics_in)
            bDefaultDynamics = 0;
            if nargin > 2
                bDefaultDynamics = bDefaultDynamics_in;
            end
            
            for i = 1 : obj.m_nNumModels
                obj.m_clModelVec{i,1} = obj.m_clModelVec{i,1}.init(oPCMovableLabel, obj.m_nNumIMM, bDefaultDynamics);
            end
            obj.m_bInit = 1;
            obj.fuseModels();
        end
        
        % Interaction step (state mixing) for IMM-eKF
        function interactModels(obj)
            % Mix state vectors
            for j = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                % Compute mixed state by transistion probabilities
                pMixed = zeros(8,1);
                for i = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                    pShared_i  = obj.m_clModelVec{i,1}.getSharedState();
                    fUTij      = obj.m_mfUT(i,j);
                    pMixed     = pMixed + fUTij .* pShared_i;
                end
                
                % Compute mixed covariance
                PMixed = zeros(8,8);
                for i = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                    pShared_i  = obj.m_clModelVec{i,1}.getSharedState();
                    PShared_i  = obj.m_clModelVec{i,1}.getSharedCovariance();
                    fUTij      = obj.m_mfUT(i,j);
                    pDelta     = pShared_i - pMixed;
                    PMixed     = PMixed + fUTij .* (PShared_i + pDelta*pDelta');
                end
                
                for i = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                    vUniqueFlags = obj.m_clModelVec{i,1}.getUniqueStateFlags();
                    pShared_i  = obj.m_clModelVec{i,1}.getSharedState();
                    PShared_i  = obj.m_clModelVec{i,1}.getSharedCovariance();
                    for s = 1 : 8
                        if vUniqueFlags(s,1) == 1
                            pMixed(s,1) = pShared_i(s,1);
                            PMixed(s,:) = PShared_i(s,:);
                            PMixed(:,s) = PShared_i(:,s);
                        end
                    end
                end
                
                obj.m_clModelVec{j,1} = obj.m_clModelVec{j,1}.setMixedSpace(pMixed, PMixed);
            end
        end
        
        % Predict all models for time gap fDT
        function obj = predict(obj, fDT, vfEgoMov)
            % Compute interaction step (state mixing) for IMM-eKF
            obj.interactModels();
            
            % Perform prediction
            for i = 1 : obj.m_nNumModels
                obj.m_clModelVec{i,1} = obj.m_clModelVec{i,1}.predict(fDT);
            end
            
            % Compensate ego movement
            for i = 1 : obj.m_nNumModels
                obj.m_clModelVec{i,1} = obj.m_clModelVec{i,1}.compensateEgoMovement(fDT, vfEgoMov);
            end
            
            % Compute fused output
            obj.fuseModels();
        end
        
        % Update all models by given measurements
        function bStable = update(obj, vfMeasurements, vfVariances)
            % Perform measurement update
            for i = 1 : obj.m_nNumModels
                obj.m_clModelVec{i,1} = obj.m_clModelVec{i,1}.update(vfMeasurements, vfVariances);
            end
            % Check for stability
            bStable = 1;
            for i = 1 : obj.m_nNumModels
                if ~obj.m_clModelVec{i,1}.m_bStable
                    bStable = 0;
                end
            end
            
            % Perform probability update
            obj.probabilityUpdate();
            
            % Compute fused output
            obj.fuseModels();
        end
        
        % Update probabilites for all models
        function probabilityUpdate(obj)
            % Model probability
            fNorm = 0;
            for j = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                fPl = obj.m_clModelVec{j,1}.m_fModelPl;
                fC  = obj.m_vfC(j,1);
                fNorm = fNorm + fPl*fC;
            end
            
            for j = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                fPl = obj.m_clModelVec{j,1}.m_fModelPl;
                fC  = obj.m_vfC(j,1);
                obj.m_clModelVec{j,1}.m_fModelProb = fPl*fC/fNorm;
            end
            
            % Normalization and transistion
            for j = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                % Normalization factor
                fC_j = 0;
                for i = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                    fUT_AP  = obj.m_mfUT_AP(i,j);
                    fUM     = obj.m_clModelVec{i,1}.getModelProbability();
                    fC_j    = fC_j + fUT_AP*fUM;
                end
                obj.m_vfC(j,1) = fC_j;
                
                % Transistion probabilities
                for i = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                    fUT_AP  = obj.m_mfUT_AP(i,j);
                    fUM     = obj.m_clModelVec{i,1}.getModelProbability();
                    obj.m_mfUT(i,j) = 1/fC_j*fUT_AP*fUM;
                end
            end
        end
        
        % Generate IMM output
        function fuseModels(obj)
            % Save previous state and covariance
            obj.m_vfFusedState_pre        = obj.m_vfFusedState;
            obj.m_mfFusedCovariance_pre   = obj.m_mfFusedCovariance;
            
            pFused  = zeros(8,1);    % fused state
            PFused  = zeros(8,8);    % fused covariance
            
            % Compute fused state by model probabilities
            for j = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                pShared     = obj.m_clModelVec{j,1}.getSharedState();
                fUMj        = obj.m_clModelVec{j,1}.getModelProbability();
                pFused      = pFused + fUMj .* pShared;
            end
            
            % Covariance
            for j = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                fUMj        = obj.m_clModelVec{j,1}.getModelProbability();
                covShared   = obj.m_clModelVec{j,1}.getSharedCovariance();
                pShared     = obj.m_clModelVec{j,1}.getSharedState();
                pDelta      = pShared - pFused;
                PFused      = PFused + fUMj .* (covShared + pDelta*pDelta');
            end
            
            for i = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                vUniqueFlags = obj.m_clModelVec{i,1}.getUniqueStateFlags();
                pShared  = obj.m_clModelVec{i,1}.getSharedState();
                PShared  = obj.m_clModelVec{i,1}.getSharedCovariance();
                for s = 1 : 8
                    if vUniqueFlags(s,1) == 1
                        pFused(s,1) = pShared(s,1);
                        PFused(s,:) = PShared(s,:);
                        PFused(:,s) = PShared(:,s);
                    end
                end
            end
            
            obj.m_vfFusedState      = pFused;
            obj.m_mfFusedCovariance = PFused;
        end
        
        function r = getFusedState(obj)
            r = obj.m_vfFusedState;
        end
        
        function r = getFusedCovariance(obj)
            r = obj.m_mfFusedCovariance;
        end
        
        % Reinitialze covariance
        function obj = reinitVariance(obj, fVarInit)
            for i = 1 : obj.m_nNumModels
                obj.m_clModelVec{i,1} = obj.m_clModelVec{i,1}.reinitVariance(fVarInit);
            end
        end
        
        function [p, P] = getCVPM(obj)
            p = obj.m_clModelVec{obj.m_nIdxCVPM,1}.getSharedState();
            P = obj.m_clModelVec{obj.m_nIdxCVPM,1}.getSharedCovariance();
        end
        
        function vUM = getModelProbabilities(obj)
            vUM = zeros(obj.m_nNumModels,1);
            for j = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                vUM(j,1) = obj.m_clModelVec{j,1}.getModelProbability();
            end
        end
        
        function vMP = getModelPlausibilities(obj)
            vMP = zeros(obj.m_nNumModels,1);
            for j = obj.m_nIdxIMM_First : obj.m_nIdxIMM_Last
                vMP(j,1) = obj.m_clModelVec{j,1}.m_fModelPl;
            end
        end
        
        function [p, P, u] = getCVCP(obj)
            p = obj.m_clModelVec{obj.m_nIdxCVCP,1}.m_vfState;
            P = obj.m_clModelVec{obj.m_nIdxCVCP,1}.m_mfCov;
            u = obj.m_clModelVec{obj.m_nIdxCVCP,1}.getModelProbability();
        end
        
        function obj = changeReferencePoint(obj, vfDelta, sChangeState, fWidth)
            for i = 1 : size(obj.m_clModelVec, 1)
                obj.m_clModelVec{i,1}.changeReferencePoint(vfDelta, sChangeState, fWidth);
            end
            obj.fuseModels();
        end
        
        function obj = setPosition(obj, x, y, yaw)
            for i = 1 : size(obj.m_clModelVec, 1)
                obj.m_clModelVec{i,1}.setPosition(x, y, yaw);
            end
            obj.fuseModels();
        end
        
        function obj = setDynamicsToZero(obj)
            for i = 1 : size(obj.m_clModelVec, 1)
                obj.m_clModelVec{i,1}.setDynamicsToZero();
            end
            obj.fuseModels();
        end
        
        function obj = reset(obj)
            obj.m_vfFusedState        = obj.m_vfFusedState_pre;
            obj.m_mfFusedCovariance   = obj.m_mfFusedCovariance_pre;
            
            for i = 1 : size(obj.m_clModelVec, 1)
                obj.m_clModelVec{i,1}.reinit();
            end
            
            % Reinit apriori and transistion probability matrix
            nFirst = obj.m_nIdxIMM_First;
            nLast  = obj.m_nIdxIMM_Last;
            
            obj.m_mfUT(nFirst:nLast, nFirst:nLast) = ones(obj.m_nNumIMM, obj.m_nNumIMM) .* 1/obj.m_nNumIMM;
            
            fProbRemain = 0.9;                   
            for i = nFirst : nLast
                obj.m_mfUT_AP(i,:) = (1-fProbRemain)/obj.m_nNumIMM;
                obj.m_mfUT_AP(i,i) = fProbRemain;
            end
            
            obj.fuseModels();
        end   
        
        function [bConsistent, sStatus] = stateConsistencyCheck(obj)
            % Thresholds
            dist_threshold  = 3;   % m
            yaw_threshold   = 10;  % in deg
            v_threshold     = 20;  % m/s
            k_threshold     = 20;  % deg/s
            
            % Check for consistency
            bConsistent = 1;
            sStatus     = '';   % position, velocity, yaw, yaw rate (k)
            
            p_estm = obj.m_vfFusedState;
            p_prev = obj.m_vfFusedState_pre;
            
            % Distance to previous reference point
            dist = sqrt((p_estm(1,1)-p_prev(1,1))^2 + (p_estm(4,1)-p_prev(4,1))^2);
            if dist > dist_threshold
                bConsistent = 0;
                sStatus = 'position';
            end
            
            % Velocities
            vAbs_estm = sqrt(p_estm(2,1)^2+p_estm(5,1)^2);
            vAbs_prev = sqrt(p_prev(2,1)^2+p_prev(5,1)^2);
            
            if abs(vAbs_estm - vAbs_prev) > v_threshold
                bConsistent = 0;
                sStatus = 'velocity';
            end
            
            % Yaw
            yaw_estm = p_estm(7,1);
            yaw_prev = p_prev(7,1);
            
            if abs(yaw_estm - yaw_prev) > yaw_threshold
                bConsistent = 0;
                sStatus = 'yaw';
            end
            
            % Yaw rate
            k_estm = p_estm(8,1);
            k_prev = p_prev(8,1);
            
            if abs(k_estm - k_prev) > k_threshold
                bConsistent = 0;
                sStatus = 'yaw rate';
            end
        end
    end
    
    methods (Access = protected)
        function oCopy = copyElement(obj)
            oCopy = copyElement@matlab.mixin.Copyable(obj);
            for i = 1 : obj.m_nNumModels
                oCopy.m_clModelVec{i,1} = copy(obj.m_clModelVec{i,1});
            end
        end
    end
end

