function [P,R,scanData] = F_getSecondPt(scanData,r,theta)
%F_GETSECONDPT computes the second sensor pose to visit in the execution of
%the autonomous robotic sensing framework described into the paper titled:
%"Autonomous Robotic Sensing for Simultaneous Geometric and Volumetric
%Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   [P,R,scanData] = F_getSecondPt(scanData,r,theta)
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
%       r = A scalar number indicating the target inspection resolution.
%       theta = A scalar number indicating the angle (in degrees) to use
%               for the computation of the second sensor pose.
%
%   Outputs:
%       P - (1 x 6 double) Second pose.
%       R - (3 x 3 double) Rotation matrix (ZYX convention) relative to the
%           second pose.
%       scanData - updated data structure.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_getSecondPt v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

P = scanData.pts(scanData.nPts,:);
R = eul2rotm(deg2rad(P(1,4:6)),'ZYX');

uRef = -[cos(atan2(-R(1,3),R(3,3))) 0 sin(atan2(-R(1,3),R(3,3)))];
vRef = -[0 cos(atan2(-R(2,3),R(3,3))) sin(atan2(-R(2,3),R(3,3)))];

P(1) = P(1) + uRef(1).*r.*cosd(theta) + vRef(1).*r.*sind(theta);
P(2) = P(2) + uRef(2).*r.*cosd(theta) + vRef(2).*r.*sind(theta);
P(3) = P(3) + uRef(3).*r.*cosd(theta) + vRef(3).*r.*sind(theta);

scanData.nPts = scanData.nPts + 1;
scanData.pts(scanData.nPts,:) = P;
scanData.nViaPts = scanData.nViaPts + 1;
scanData.iViaPts(scanData.nViaPts) = scanData.nPts;
scanData.isAcquPt(scanData.nViaPts) = true;

end

