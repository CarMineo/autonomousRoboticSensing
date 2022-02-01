function [u,v] = F_uvRodrigues(w)
%F_UVRODRIGUES Generates two versors perpendicular to a given versor, using
%Rodrigues formula, in order to create an orthogonal reference system that
%respects the right hand rule.
%   [us,vs] = F_uvRodrigues(ns)
%
%   Inputs:
%       ns - A unitary vector (3x1)
%
%   Outputs:
%       us - First perpendicular versor (vector with unitary modulus) (3x1)
%       vs - Second perpendicular versor (vector with unitary modulus) (3x1)
%       Note - us and vs are also perpendicular to each other.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_uvRodrigues v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.

%------------- BEGIN CODE --------------


u = nan(size(w));
v = nan(size(w));

%compute rotate theta and axis
zaxis = [0 0 1];

x = 1;
y = 0;
z = 0;

s = size(w,1);

for j=1:s
    w(j,:) = w(j,:)./norm(w(j,:));
    if isequal(w(j,:),zaxis)
        fx = x;
        fy = y;
        fz = z;
    elseif isequal(-w(j,:),zaxis)
        fx = -x;
        fy = -y;
        fz = -z;
    else
        ang = acos(dot(zaxis,w(j,:)));
        axis = cross(zaxis,w(j,:))/norm(cross(zaxis,w(j,:)));
        % A skew symmetric representation of the normalized axis
        axis_skewed = [ 0 -axis(3) axis(2) ; axis(3) 0 -axis(1) ; -axis(2) axis(1) 0];
        % Rodrigues formula for the rotation matrix
        R = eye(3) + sin(ang)*axis_skewed + (1-cos(ang))*axis_skewed*axis_skewed;
        fx = R(1,1)*x + R(1,2)*y + R(1,3)*z;
        fy = R(2,1)*x + R(2,2)*y + R(2,3)*z;
        fz = R(3,1)*x + R(3,2)*y + R(3,3)*z;
    end
    
    u(j,:) = [fx fy fz];
    v(j,:) = cross(w(j,:),u(j,:));
end

end

