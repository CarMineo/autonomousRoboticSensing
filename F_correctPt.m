function [scanData,R] = F_correctPt(scanData,R,amp,stoff,tStoff,n,da,TR,FN)
%F_CORRECTPT corrects a sensor pose, to maximize the signal amplitude and
%avoid deviation of the sensor standoff from the target standoff.
%This is crucial to the execution of the autonomous robotic sensing
%framework described into the paper titled: "Autonomous Robotic Sensing for
%Simultaneous Geometric and Volumetric Inspection of Free-Form Parts",
%by C. Mineo, D. Cerniglia and A. Poole.
%   [scanData,R] = F_correctPt(scanData,R,amp,stoff,tStoff,n,da,TR,FN)
%
%   Inputs:
%       scanData - Structure array of the data structure, containing the 
%                  following fields:
%               .nPts - (1 x 1 uint32) Number of distinct visited points.
%               .pts - (nPts x 6 double) A matrix, where each row contains
%                      the Cartesian coordinates of one robot pose (X,Y,Z)
%                      and its Eulerian angular coordinates (A,B,C). The
%                      Eulerian angles are given in degrees. Each distinct
%                      pose is listed only once, to minimize memory consumption.
%               .isDataPt - (nPts x 1 bool) A Boolean value for each point.
%                           TRUE (1) = sensor data has been collected at
%                           the point (the part has been detected). 
%               .isOverlapPt - (nPts x 1 bool) A Boolean value for each
%                              point. TRUE (1) = the point overlaps with a
%                              previously inspected region.
%               .amp - (nPts x 1 double) A value for each point, to store
%                      the amplitude value measured by the sensor.
%               .standoff - (nPts x 1 double) A value for each point, to
%                           store the measured sensor standoff.
%               .nViaPts - (1 x 1 uint32) Number of points  constituting
%                          the robotic trajectory.
%               .iViaPts - (nViaPts x 1 uint32) Indices of the trajectory
%                          points (the indices refer to the points in pts).
%               .isAcquPt - (nViaPts x 1 bool) A Boolean value for each
%                           trajectory point. TRUE (1) = sensor data
%                           acquisition has been attempted at the relative
%                           trajectory point.
%               .nTri - (1 x 1 uint32) Number of triangles in the geometry
%                       reconstruction tessellated surface.
%               .iTri - (nTri x 3 uint32) Each row contains the indices of
%                       the vertices of one triangle. The indices refer to
%                       the points in pts (with no repetitions).
%               .nEdges - (1 x 1 uint32) Number of distinct triangle edges
%                         in the whole triangulated surface (with no
%                         repetitions).
%               .iTriEdges - (nEdges x 2 uint32) Each row contains the
%                            indices of the extremities of one edge. The
%                            indices refer to the points in pts. 
%               .isOutEdge - (nEdges x 1 bool) A Boolean value for each
%                            edge of the triangulation. TRUE (1) = the
%                            relative edge is on the perimeter of the
%                            triangulation.
%       R - Rotation matrix of the current pose (3x3 double). This could be
%           retrieved inside this function, from the first input through:
%           R = eul2rotm(deg2rad(scandata.pts(scanData.nPts,4:6)),'ZYX').
%           However, R is provided for convenience, in order to not
%           duplicate the computation effort undertaken before the
%           execution of this function.
%       amp - (1 x 1 double) Current sensor amplitude value.
%       stoff - (1 x 1 double) Current sensor standoff.
%       tStoff - (1 x 1 double) A scalar number indicating the target 
%                sensor standoff.
%       n = A scalar number indicating the level of sensor signal noise.
%       da = A scalar number indicating the angular increment to use in the
%            amplitude mapping process for pose correction.
%       TR = Triangulation of part surface. This is only for simulation
%            purpose, to support the generation of simulated sensor signals.
%            The part is unknown in real applications.
%       FN = Noprmals of triangulation faces. This is only for simulation
%            purpose, to support the generation of simulated sensor signals.
%            The part is unknown in real applications.
%
%   Outputs:
%       scanData - Updated structure array of the data structure.
%       R - Updated sensor rotation matrix of the current pose (3x3 double).
%           This could be retrieved outside this function, through:
%           R = eul2rotm(deg2rad(scandata.pts(scanData.nPts,4:6)),'ZYX').
%           However, R is provided for convenience, in order to not
%           duplicate the computation effort undertaken by this function.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_correctPt v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

P = scanData.pts(scanData.nPts,:);

rotAxis = '';
amp0 = -inf;

counter = 0;

if amp > (1.5*n)
    while ((amp-amp0) > n) || (counter<2)
        amp0 = amp;
        
        nx = abs(R(1,3)); ny = abs(R(2,3)); nz = abs(R(3,3));
        
        if (nz >= nx) && (nz >= ny)
            if (ny >= nx)
                if ~strcmp(rotAxis,'X')
                    rotAxis = 'X';
                else
                    rotAxis = 'Y';
                end
            else
                if ~strcmp(rotAxis,'Y')
                    rotAxis = 'Y';
                else
                    rotAxis = 'X';
                end
            end
        elseif (ny > nx) && (ny > nz)
            if (nz >= nx)
                if ~strcmp(rotAxis,'X')
                    rotAxis = 'X';
                else
                    rotAxis = 'Z';
                end
            else
                if ~strcmp(rotAxis,'Z')
                    rotAxis = 'Z';
                else
                    rotAxis = 'X';
                end
            end
        else
            if (nz >= ny)
                if ~strcmp(rotAxis,'Y')
                    rotAxis = 'Y';
                else
                    rotAxis = 'Z';
                end
            else
                if ~strcmp(rotAxis,'Z')
                    rotAxis = 'Z';
                else
                    rotAxis = 'Y';
                end
            end
        end
        
        [R,amp,stoff,~,~,~] = F_findRotToMaxAmp(P,R,amp,stoff,n,da,rotAxis,tStoff,TR,FN);
        counter = counter + 1;
    end
    
    % Compute correction for sensor standoff
    [Pcorr, corrMagnitude] = F_corrStoff(P,R,stoff,tStoff);
    
    % Note corrMagnitude is the correction distance. This value can be used
    % in a conditional statemement, if one wants to limit the maximum
    % allowed correction (e.g. to prevent unintentional collision
    % avoidance, due to unreliable sensor data).
    
    %if (abs(corrMagnitude)<100)
    P = Pcorr;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%         Simulate sensor amplitude and standoff at first pose    %%%
    [amp,stoff] = F_simSensorData(P,R,TR,FN,n,tStoff);                  %%%
    %%% This must be replaced with real sensor acquisition and          %%%
    %%% processing in real applications.                                %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %end
   
    P(1,4:6) = rad2deg(rotm2eul(R,'ZYX'));
    
    scanData.pts(scanData.nPts,:) = P;
    scanData.isDataPt(scanData.nPts,:) = true;
    scanData.amp(scanData.nPts) = amp;
    scanData.standoff(scanData.nPts) = stoff;
end


