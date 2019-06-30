function [fDyaw] = calculateDeltaYaw360(fYawDest, fYawSource)
% ---------------------------------------------------------------------------------------------
% Function calculateDyaw360(...) computes the angular delta between fYawSource with respect to fYawDest.
%
% INPUT:
%   fYawDest:       Target angle,  0 < fYawDest     < 360
%   fYawSource:     Origin angle,  0 < fYawSource   < 360
%
% OUTPUT:
%   fDyaw:          Calculated difference.
% ---------------------------------------------------------------------------------------------

if      (fYawDest < 45) && (fYawSource > 315)
    fDyaw = 360 - fYawSource + fYawDest;
elseif  (fYawDest > 315) && (fYawSource < 45)
    fDyaw = -(fYawSource + (360-fYawDest));
else
    fDyaw = fYawDest - fYawSource;
end

end

