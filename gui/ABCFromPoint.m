function [A,B,C] = ABCFromPoint(p, q) 
% ---------------------------------------------------------------------------------------------
% Function ABCFromPoint(...) computes line coefficients for a line from point p to q.
%
% INPUT:
%   p:          Orgin of line
%   q:          Target of line
% OUTPUT:
%   A, B, C:    Computed line coefficients
% ---------------------------------------------------------------------------------------------

A = -(q(2,1) - p(2,1));
B = q(1,1) - p(1,1);
C = -(A*p(1,1) + B*p(2,1));

end

