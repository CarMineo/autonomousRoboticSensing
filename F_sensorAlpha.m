function alpha = F_sensorAlpha(R)
%F_SENSORALPHA Computes a scalar number (double), indicating the
%orientation of the sensor reference system, with respect to the global
%x-axis direction. This is required to the execution of the autonomous
%robotic sensing framework described into the paper titled:
%"Autonomous Robotic Sensing for Simultaneous Geometric and Volumetric
%Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   alpha = F_sensorAlpha(R)
%
%   Inputs:
%       R - Rotation matrix of the sensor pose (3x3 double).
%
%   Outputs:
%       alpha - A scalar number (double) indicating the orientation of the
%               initial sensor reference system, with respect to the global
%               x-axis direction. This is used in other functions to keep
%               the orientation of the sensor frame consistent throughout
%               the autonomous robotic inspection of a part. alpha is given
%               in degrees.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_sensorAlpha v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

u = R(:,1)';
uRef = -[cos(atan2(-R(1,3),R(3,3))) 0 sin(atan2(-R(1,3),R(3,3)))];
v = cross(uRef,R(:,3)');
Rref = [uRef' v' R(:,3)];
testPt = u*Rref;
alpha = atan2(testPt(:,2),testPt(:,1));

end

