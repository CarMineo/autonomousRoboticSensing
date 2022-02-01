function c = F_vectorDot3D(a,b)
%F_VECTORDOT3D dot product of two vectors
%   c = F_vectorDot3D(a,b)
%
%   Inputs:
%       a - (1 x 3 double) First vector
%       b - (1 x 3 double) Second vector
%
%   Outputs:
%       c - (1 x 1 double) Dot product
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_vectorDot3D v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

c = sum(a.*b,2);

end

