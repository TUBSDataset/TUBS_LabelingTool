function [voPCMovableLabel] = predictKalman(voPCMovableLabel, oGUIData, oGUIHistory)
% ---------------------------------------------------------------------------------------------
% Function predictKalman(...) ...
%
% INPUT:
%   voPCMovableLabel:   Vector of class cPCMovableLabel to be predicted
%   oGUIData:           Object of class cGUIData containing metadata information
%   oGUIHistory:        Vector of class cHistory containing previous samples
%
% OUTPUT:
%   voPCMovableLabel:   Predicted object vector
% ---------------------------------------------------------------------------------------------
voHistory = oGUIHistory;

oPCMetadata_CSRF    = oGUIData.m_oPCMetadata;               % metadata of current point cloud (current sensor reference frame)
nPos                = oGUIData.m_nPosInHistory;
oPCMetadata_HSRF    = voHistory(nPos-1,1).m_oPCMetadata;    % metadata of previous point cloud (history ")

DT_scan = (oPCMetadata_CSRF.m_nTimestamp_us - oPCMetadata_HSRF.m_nTimestamp_us)/1000/1000;
egoK    = oPCMetadata_HSRF.m_fEgoYawRate;

for i = 1 : size(voPCMovableLabel,1)
    % Unset prediction flag for uncertain predictions
    if voPCMovableLabel(i,1).m_bFirstFrame 
        voPCMovableLabel(i,1).m_bIsPredicted = 0;
    end
    
    %% 1. Calculate prediction interval DT. Note that this intervall is not constant if a rotating sensor is used as in this case.
    %  Timestamp in predecessor frame
    phi_pre = atan2(voPCMovableLabel(i,1).m_fBBMiddle_y, voPCMovableLabel(i,1).m_fBBMiddle_x)*180/pi();
    if phi_pre < 0
        phi_pre = phi_pre + 360;
    end
    phi_scan_pre = 360 - phi_pre;   % scanner rotates clockwise
    
    t_start_pre = oPCMetadata_HSRF.m_nFirstTimestamp_us/1000/1000;
    DT_pre      = (oPCMetadata_HSRF.m_nLastTimestamp_us - oPCMetadata_HSRF.m_nFirstTimestamp_us)/1000/1000;
    
    t_pre = t_start_pre + phi_scan_pre/360*DT_pre;   % object timestamp in predecessor frame
    
    % Predict the object's azimuth in current reference frame
    vfPos = [voPCMovableLabel(i,1).m_fBBMiddle_x; voPCMovableLabel(i,1).m_fBBMiddle_y];
    vfVel = [voPCMovableLabel(i,1).m_fVxAbs; voPCMovableLabel(i,1).m_fVyAbs];
    vfAcc = [voPCMovableLabel(i,1).m_fAxAbs; voPCMovableLabel(i,1).m_fAyAbs];

    vfPos = vfPos + DT_scan .* vfVel + .5*DT_scan^2 .* vfAcc;
    
    % Compensate ego translation
    vfVel_ego = [oPCMetadata_HSRF.m_fEgoVx; oPCMetadata_HSRF.m_fEgoVy];
    vfAcc_ego = [oPCMetadata_HSRF.m_fEgoAx; oPCMetadata_HSRF.m_fEgoAy];

    vfPos = vfPos - (DT_scan.*vfVel_ego + .5*DT_scan^2.*vfAcc_ego);
    
    % Compensate ego rotation
    yaw_ego = -egoK * DT_scan * pi()/180;
    R_ego = [cos(yaw_ego) -sin(yaw_ego); sin(yaw_ego) cos(yaw_ego)];
    
    vfPos = R_ego*vfPos;
    
    % Current reference frame
    phi_cur = atan2(vfPos(2,1), vfPos(1,1))*180/pi();
    if phi_cur < 0
        phi_cur = phi_cur + 360;
    end
    phi_cur_scan = 360 - phi_cur;   
    
    t_start_cur = oPCMetadata_CSRF.m_nFirstTimestamp_us/1000/1000;
    DT_cur      = (oPCMetadata_CSRF.m_nLastTimestamp_us - oPCMetadata_CSRF.m_nFirstTimestamp_us)/1000/1000;
    
    t_cur = t_start_cur + phi_cur_scan/360*DT_cur;   
    
    % Prediction interval
    DT = t_cur - t_pre;
    % In case of quadrant change from I to IV
    if DT < .05 && ~voPCMovableLabel(i,1).m_bScanBreak;
        voPCMovableLabel(i,1).m_bScanBreak = 1;
    elseif DT < .05
        voPCMovableLabel(i,1).m_bScanBreak = 0;
        DT = (oPCMetadata_CSRF.m_nTimestamp_us - oPCMetadata_HSRF.m_nTimestamp_us)/1000/1000 + DT;
    end
    
    % Set timestamp
    voPCMovableLabel(i,1).m_nTimestamp = t_cur*1000*1000;
   
    %% 2. Perform update
    vfEgoMov = [vfVel_ego; vfAcc_ego; egoK];
    voPCMovableLabel(i,1).m_oKalman.predict(DT, vfEgoMov);
    
    voPCMovableLabel(i,1).reassignData();
    voPCMovableLabel(i,1).updateReferencePoint();
end

end

