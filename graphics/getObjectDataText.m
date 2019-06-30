function [sObjectData] = getObjectDataText(oPCMovableLabel)
% ---------------------------------------------------------------------------------------------
% Function getObjectDataText(...) generates a text field containing state vector and variances information.
%
% INPUT:
%   oPCMovableLabel:    Object of class cPCMovableData containing the object's information
%   
%
% OUTPUT:
%   sObjectData:        Generated text field
% ---------------------------------------------------------------------------------------------
p = oPCMovableLabel.m_oKalman.getFusedState();
P = oPCMovableLabel.m_oKalman.getFusedCovariance();

mPs = oPCMovableLabel.m_oKalman.getModelProbabilities();

sObjectData = sprintf('[Label]                     [State]        [Variances] \n');
sObjectData = sprintf('%s%s', sObjectData, sprintf('Ref_x = %8.3f       %8.3f       %6.4f\n', ...
    oPCMovableLabel.m_vfReferencePoint(1,1), p(1,1), P(1,1)));
sObjectData = sprintf('%s%s', sObjectData, sprintf('Ref_y = %8.3f       %8.3f       %6.4f\n', ...
    oPCMovableLabel.m_vfReferencePoint(2,1), p(4,1), P(4,4)));
sObjectData = sprintf('%s%s', sObjectData, sprintf('Vel_x = %8.3f       %8.3f       %6.4f\n', ...
    oPCMovableLabel.m_fVxAbs, p(2,1), P(2,2)));
sObjectData = sprintf('%s%s', sObjectData, sprintf('Vel_y = %8.3f       %8.3f       %6.4f\n', ...
    oPCMovableLabel.m_fVyAbs, p(5,1), P(5,5)));
sObjectData = sprintf('%s%s', sObjectData, sprintf('Acc_x = %8.3f       %8.3f       %6.4f\n', ...
    oPCMovableLabel.m_fAxAbs, p(3,1), P(3,3)));
sObjectData = sprintf('%s%s', sObjectData, sprintf('Acc_y = %8.3f       %8.3f       %6.4f\n', ...
    oPCMovableLabel.m_fAyAbs, p(6,1), P(6,6)));
sObjectData = sprintf('%s%s', sObjectData, sprintf('yaw  = %8.3f       %8.3f    %6.4f\n', ...
    oPCMovableLabel.m_fBBYaw, p(7,1), P(7,7)));
sObjectData = sprintf('%s%s', sObjectData, sprintf('k      = %8.3f       %8.3f       %6.4f\n', ...
    oPCMovableLabel.m_fBBYawRatePerDist, p(8,1), P(8,8)));
sObjectData = sprintf('%s%s', sObjectData, sprintf('P(CAPM) = %8.3f \nP(CVCP) = %8.3f \n', ...
    mPs(2,1), mPs(3,1)));
end

