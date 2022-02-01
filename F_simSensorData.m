function [amp,stoff] = F_simSensorData(P,R,TR,FN,n,tStoff)
%F_SIMSENSORDATA simulates a sensor signal to support the execution of the
%autonomous robotic sensing framework described into the paper titled:
%"Autonomous Robotic Sensing for Simultaneous Geometric and Volumetric
%Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   [amp,standoff] = F_simSensorData(P,R,TR,FN,n,tStoff)
%
%   Inputs:
%       P = Initial pose (1x6) P=(x,y,z,a,b,c), with X, Y and Z being the
%           Cartesian coordinates and A, B and C being the angular Euler
%           coordinates.
%       R - Rotation matrix of the current pose (3x3 double). This could be
%           retrieved inside this function, from the first input through:
%           R = eul2rotm(deg2rad(P(1,4:6)),'ZYX'). However, R is provided
%           for convenience, in order to not duplicate the computation
%           effort undertaken outside this function.
%       TR = Triangulation of part surface. This is only for simulation
%            purpose, to support the generation of simulated sensor signals.
%            The part is unknown in real applications.
%       FN = Noprmals of triangulation faces. This is only for simulation
%            purpose, to support the generation of simulated sensor signals.
%            The part is unknown in real applications.
%       n = A scalar number indicating the level of sensor signal noise.
%       tStoff = A scalar number indicating the target sensor standoff.
%
%   Outputs:
%       amp - (1 x 1 double) Simulated sensor amplitude value.
%       stoff - (1 x 1 double) Simulated sensor standoff.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_simSensorData v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

amp = 0;
stoff = nan;

[intersectionPoints, distFromOrigin, intersectedFaces] = F_intersectLineMesh(P(1,1:3),R(:,3)',TR.Points,TR.ConnectivityList);
if ~isempty(distFromOrigin)
    ns = FN(intersectedFaces(1),:);
    vd = (R(:,3)').*tStoff;
    ve = intersectionPoints(1,:) - P(1,1:3);
    vOffset = vd+ve;
    stoff = norm(vOffset);
    
    cosTheta = abs(dot(ns,R(:,3)'));
    amp = cosTheta + ((rand-0.5).*n);
end

