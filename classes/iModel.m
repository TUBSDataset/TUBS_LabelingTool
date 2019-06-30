classdef (Abstract) iModel < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % iModel is an interface used to implement an IMM-model structure for kalman filtering
    % New models need to be derived from iModel.
    % ---------------------------------------------------------------------------------------------
    properties
        m_nNumOfStates;      % number of states
        m_vfState;           % state vector
        m_mfCov;             % covariance matrix
        m_vfState_pre;       % former state
        m_mfCov_pre;         % former covariance 
             
        m_vfSharedState = zeros(8,1);     % shared state vector
        m_mfSharedCov   = zeros(8,8);     % shared
        
        m_fModelProb = 0;    % model probability
        m_fModelPl   = 0;    % model plausibility
        m_fModelProb_init;   % inital model probability
        
        m_fVarInit   = 100;  % initial variance
        m_bStable    = 1;
    end
    
    methods (Abstract)
        getSharedState(obj)
        getSharedCovariance(obj)
        getModelProbability(obj)
        getUniqueStateFlags(obj)
        obj = setMixedSpace(obj, pMixed, PMixed)
        obj = updateSharedSpace(obj)
        obj = init(obj, oPCMovableLabel, nNumIMM, bDefault)
        obj = reinitVariance(obj, fVarInit)
        obj = predict(obj, fDT)
        obj = update(obj, vfMeasurements, vfVariances)
        obj = changeReferencePoint(obj, vfDelta, sChangeState, fWidth)
        obj = setPosition(obj, x, y, yaw)
        obj = reinit(obj)
        obj = compensateEgoMovement(obj, fDT, vfEgoMov)
        obj = limitStateEntries(obj)
        obj = setDynamicsToZero(obj)
    end   
end

