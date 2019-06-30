function [voPCMovableLabel] = updateKalmanByPrelabeling(voPCMovableLabel, voPCMovableLabel_Loaded, oGUIData)
% ---------------------------------------------------------------------------------------------
% Function updateKalmanByPrelabeling(...) performs the measurement update for the kalman filters of 
% all objects based on associated objects from the prelabeling stage.
%
% INPUT:
%   voPCMovableLabel:           Vector of cPCMovableLabel objects to be corrected
%   voPCMovableLabel_Loaded:    Vector of cPCMOvableLabel loaded from the prelabeling stage
%   oGUIData:                   Object of class cGUIData containing metadata inforamtion
%   
% OUTPUT:
%   voPCMovableLabel:           Corrected object vector
% --------------------------------------------------------------------------------------------- 

fSigXY   = 0.3;
fSigYaw  = 0.3;
fDist_thres = 1.5;     % association threshold

oInfo_h = oGUIData.m_oInfo_h;
bDebug  = oGUIData.m_bEnableDebugInformation;

for i = 1 : size(voPCMovableLabel, 1)
    %% Find associated object
    
    x = voPCMovableLabel(i,1).m_fBBMiddle_x;
    y = voPCMovableLabel(i,1).m_fBBMiddle_y;
        
    bAssociationFound = 0;
    for j = 1 : size(voPCMovableLabel_Loaded, 1)
        x_l = voPCMovableLabel_Loaded(j,1).m_fBBMiddle_x;
        y_l = voPCMovableLabel_Loaded(j,1).m_fBBMiddle_y;
        
        fDist = sqrt((x - x_l)^2 + (y - y_l)^2);
        if fDist < fDist_thres
            bAssociationFound = 1;
            oPCMovableLabel_assoc = voPCMovableLabel_Loaded(j,1);
            break;
        end
    end
    
    if ~bAssociationFound
        continue;
    end
    
    if bDebug
        setInfoText(oInfo_h, sprintf('Info %s %d: Association found.', voPCMovableLabel(i,1).m_sClassification, i), 1);
    end
    
    %% Perform measurement update
    %  Update refernce point of association
    bInit = 1;
    oPCMovableLabel_assoc.updateReferencePoint(bInit);
    
    % Build measurement
    yaw_measure = oPCMovableLabel_assoc.m_fBBYaw;
    pos_measure = [oPCMovableLabel_assoc.m_vfReferencePoint(1,1); oPCMovableLabel_assoc.m_vfReferencePoint(2,1)];
    
    vfMeasurements = [pos_measure; yaw_measure];
    vfVariances    = [fSigXY^2; fSigXY^2; fSigYaw^2;];
    
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
    
    % Reassing data format entries
    voPCMovableLabel(i,1).reassignData();
end

end

