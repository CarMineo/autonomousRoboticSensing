function [P,corrMag] = F_corrStoff(P,R,stoff,tStoff)
%F_CORRSTOFF corrects the Cartesian coordinates of the pose (P), to bring
%the sensor standoff to the target standoff.
%   R = F_da2R(da,rotAxis)
%
%   Inputs:
%       P - Sensor pose (1x6) P=(x,y,z,a,b,c), with X, Y and Z being the
%           Cartesian coordinates and A, B and C being the angular Euler
%           coordinates.
%       R - Rotation matrix of the current pose (3x3 double). This could be
%           retrieved inside this function, from the first input through:
%           R = eul2rotm(deg2rad(P(1,4:6)),'ZYX').
%           However, R is provided for convenience, in order to not
%           duplicate the computation effort undertaken before the
%           execution of this function.
%       stoff - (1 x 1 double) Current sensor standoff.
%       tStoff - (1 x 1 double) A scalar number indicating the target 
%                sensor standoff.
%
%   Outputs:
%       P - Corrected sensor pose.
%       corrMag - (1 x 1 double) A scalar number indicating the magnitude
%                 of the correction.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_corrStoff v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

corrMag = stoff-tStoff;
vCorr = R(:,3).*corrMag;
P(1:3) = P(1:3) + vCorr';

end

