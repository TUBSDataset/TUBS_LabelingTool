function [yaw_measure_adapted] = adaptYawMeasurement(yaw_measure, yaw_state)
% ---------------------------------------------------------------------------------------------
% Function adaptYawMeasurement(...) adapts a yaw measurement according to the corresponding state entry
% in order to avoid discontinuities.
%
% INPUT:
%   yaw_measure:   Input angle, 0 < yaw_measure < 360
%   yaw_state:     State angle, -unlim < yaw_state < +unlim
%
% OUTPUT:
%   yaw_adapted:   Adapted yaw measurement
% ---------------------------------------------------------------------------------------------

yaw_state_save = yaw_state;

% Map to value range 0 ... 360
nCtr = 0;
while (yaw_state < 0) && (nCtr < 30)
    yaw_state = yaw_state + 360;
    nCtr = nCtr + 1;
end
nCtr = 0;
while (yaw_state > 360) && (nCtr < 30)
    yaw_state = yaw_state - 360;
    nCtr = nCtr + 1;
end

% Calculate difference
dYaw = calculateDeltaYaw360(yaw_measure, yaw_state);

% Generate new measurement value
yaw_measure_adapted = yaw_state_save + dYaw;

end

