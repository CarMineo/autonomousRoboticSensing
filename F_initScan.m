function [scanData,R,alpha] = F_initScan(P,n,da,tStoff,r,theta,rDir,TR,FN)
%F_INITSCAN Initializes the data structure required to the execution of the
%autonomous robotic sensing framework described into the paper titled:
%"Autonomous Robotic Sensing for Simultaneous Geometric and Volumetric
%Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%If the part is detected at the first pose (amp>1.5n), the function is also
%responsible for visiting the first pose, computing and visiting the second
%and third pose. Pose correction is carried out at each pose. Finally, the
%function creates the first triangle of the mesh.
%   [scanData,R,alpha] = F_initScan(P,n,da,tStoff,r,theta,rDir,TR,FN)
%
%   Inputs:
%       P = Initial pose (1x6) P=(x,y,z,a,b,c), with X, Y and Z being the
%           Cartesian coordinates and A, B and C being the angular Euler
%           coordinates.
%       n = A scalar number indicating the level of sensor signal noise.
%       da = A scalar number indicating the angular increment to use in the
%            amplitude mapping process for pose correction.
%       tStoff = A scalar number indicating the target sensor standoff.
%                This is the angular increment to use in the distance
%                between the sensor and the part surface that is mantained
%                by the pose correction algorithm.
%       r = A scalar number indicating the target inspection resolution.
%       theta = A scalar number indicating the angle (in degrees) to use
%               for the computation of the second sensor pose.
%       rDir = Boolean value indicating the preferred rotation direction
%              (TRUE = clockwise, FALSE = anticlockwise).
%       TR = Triangulation of part surface. This is only for simulation
%            purpose, to support the generation of simulated sensor signals.
%            The part is unknown in real applications.
%       FN = Noprmals of triangulation faces. This is only for simulation
%            purpose, to support the generation of simulated sensor signals.
%            The part is unknown in real applications.
%
%   Outputs:
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
%           The initialization of the data structure (scanData) is
%           carrie out by the function F_initScanData. This allocate enough
%           memory space to store up to 100 poses, 100 trajectory points,
%           100 meah triangles and 300 tringle edges.
%       R - Rotation matrix of the current pose (third pose). This could be
%           retrieved outside this function, from the first output through:
%           R = eul2rotm(deg2rad(scanData.pts(3,4:6)),'ZYX'). However, R is
%           explicitly output for convenience.
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
%   F_initScan v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

scanData = [];
R = eul2rotm(deg2rad(P(1,4:6)),'ZYX');
alpha = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%         Simulate sensor amplitude and standoff at first pose        %%%
[amp,stoff] = F_simSensorData(P,R,TR,FN,n,tStoff);                      %%%
%%% This must be replaced with real sensor acquisition and processing   %%%
%%% in real applications.                                               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if amp > (1.5*n)
    % Initialize data structure
    scanData = F_initScanData(P,100);
    
    % Get the angle (in gedrees), indicating the orientation of the
    % initial sensor reference system, with respect to the global x-axis direction.
    alpha = F_sensorAlpha(R);
    
    % Correct first pose
    [scanData,~] = F_correctPt(scanData,R,amp,stoff,tStoff,n,da,TR,FN);
    
    % Get second pose
    [P,R,scanData] = F_getSecondPt(scanData,r,theta);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%      Simulate sensor amplitude and standoff at second pose      %%%
    [amp,stoff] = F_simSensorData(P,R,TR,FN,n,tStoff);                  %%%
    %%% This must be replaced with real sensor acquisition and          %%%
    %%% processing in real applications.                                %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Correct second pose
    [scanData,~] = F_correctPt(scanData,R,amp,stoff,tStoff,n,da,TR,FN);
    
    % Get third pose
    [P,R,scanData] = F_getThirdPt(scanData,r,rDir,alpha);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%      Simulate sensor amplitude and standoff at third pose       %%%
    [amp,stoff] = F_simSensorData(P,R,TR,FN,n,tStoff);                  %%%
    %%% This must be replaced with real sensor acquisition and          %%%
    %%% processing in real applications.                                %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Correct third pose
    [scanData,R] = F_correctPt(scanData,R,amp,stoff,tStoff,n,da,TR,FN);
    
    % Create first mesh triangle
    scanData.nTri = 1;
    scanData.nEdges = 3;
    scanData.isOutEdge(1:3,1) = true;
    if rDir
        scanData.iTri(1,:) = [2 1 3];
        scanData.iTriEdges(1:3,:) = [2 1;
            1 3;
            3 2];
    else
        scanData.iTri(1,:) = [2 3 1];
        scanData.iTriEdges(1:3,:) = [1 2;
            2 3;
            3 1];
    end
else
    R = [];
end

