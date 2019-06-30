function [voPCMovableLabel] = updateKalmanByCorrections(voPCMovableLabel, oGUIData)
% ---------------------------------------------------------------------------------------------
% Function updateKalmanByCorrections(...) performs the measurement update for the kalman filters of all objects
% based on manual correction inputs. 
%
% INPUT:
%   voPCMovableLabel:       Vector of cPCMovableLabel objects to be corrected
%   oGUIData:               Object of class cGUIData containing metadata inforamtion
%   
% OUTPUT:
%   voPCMovableLabel:       Corrected object vector
% --------------------------------------------------------------------------------------------- 
oInfo_h = oGUIData.m_oInfo_h;

% sigXY   = oGUIData.m_fDeltaXY;
% sigYaw  = oGUIData.m_fDeltaYaw;
sigXY   = 0.1;
sigYaw  = 0.1;
bDebug  = oGUIData.m_bEnableDebugInformation;

for i = 1 : size(voPCMovableLabel, 1)
    %% Perform measurement update
    yaw_measure = voPCMovableLabel(i,1).m_fBBYaw;
    pos_measure = [voPCMovableLabel(i,1).m_vfReferencePoint(1,1); voPCMovableLabel(i,1).m_vfReferencePoint(2,1)];
    
    % Skip if it's the first frame (prediction -> correction)
    if voPCMovableLabel(i,1).m_bFirstFrame
        if bDebug
            setInfoText(oInfo_h, sprintf('Info %s %d: First frame, skipping.', voPCMovableLabel(i,1).m_sClassification, i), 1);
        end
        % Apply measured position for first prediction
        voPCMovableLabel(i,1).m_oKalman.setPosition(pos_measure(1,1), pos_measure(2,1), yaw_measure);
        continue
    end
    
    %  Skip unmeasured voPCMovableLabel(i,1)
    if ~voPCMovableLabel(i,1).m_bNewMeasurement
        if bDebug
            setInfoText(oInfo_h, sprintf('Info %s %d: Unmeasured object, skipping.', voPCMovableLabel(i,1).m_sClassification, i), 1);
        end
        continue
    end

    % Create measurement
    voPCMovableLabel(i,1).m_bNewMeasurement = 0;
    
    vfMeasurements = [pos_measure; yaw_measure];
    vfVariances    = [sigXY^2; sigXY^2; sigYaw^2;];
    
    % Update
    bStable = voPCMovableLabel(i,1).m_oKalman.update(vfMeasurements, vfVariances);
    
    if ~bStable && (bDebug || oGUIData.m_bLogWarnings )
        setInfoText(oInfo_h, sprintf('Warning %s %d: FILTER UNSTABLE.', voPCMovableLabel(i,1).m_sClassification, i), 1);
    end

    [bConsistent, sStatus] = voPCMovableLabel(i,1).m_oKalman.stateConsistencyCheck();

    if ~bConsistent
        % Reset to predecessor state and reinit model
        voPCMovableLabel(i,1).m_oKalman.reset();     
        if bDebug || oGUIData.m_bLogWarnings 
            setInfoText(oInfo_h, sprintf('Warning %s %d: STATES INCONSISTENT. Cause: %s', ...
                voPCMovableLabel(i,1).m_sClassification, i, sStatus), 1);
        end
    end   
        
    % Overwrite position 
    voPCMovableLabel(i,1).m_oKalman.setPosition(pos_measure(1,1), pos_measure(2,1), yaw_measure);
    
    % Set dynamics (state and cov) to zero for unactive objects (apriori knowledge)
    if ~voPCMovableLabel(i,1).m_bIsActive
        voPCMovableLabel(i,1).m_oKalman.setDynamicsToZero();
    end
    
    % Reassing data format entries
    voPCMovableLabel(i,1).reassignData();
end

end




