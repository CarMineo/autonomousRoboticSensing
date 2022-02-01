function scanData = F_initScanData(P,n)
%F_INITSCANDATA Initializes the data structure required to the execution of
%the autonomous robotic sensing framework described into the paper titled:
%"Autonomous Robotic Sensing for Simultaneous Geometric and Volumetric
%Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   scanData = F_initScanData(P,n)
%
%   Inputs:
%       P = Initial pose (1x6) P=(x,y,z,a,b,c), with X, Y and Z being the
%           Cartesian coordinates and A, B and C being the angular Euler
%           coordinates.
%       n = Number of elements to allocate the memory for.
%
%   Outputs:
%       scanData - Structure array of the data structure, containing the 
%                  following fields:
%               .nPts - (1 x 1 uint32) Number of distinct visited points.
%                       This is initialized as 1.
%               .pts - (n x 6 double) A matrix, where each row contains
%                      the Cartesian coordinates of one robot pose (X,Y,Z)
%                      and its Eulerian angular coordinates (A,B,C). The
%                      Eulerian angles are given in degrees. Each distinct
%                      pose is listed only once, to minimize memory consumption.
%                       This is initialized as an array of zeros, but the
%                       first row is set equal to the first pose (P).
%               .isDataPt - (n x 1 bool) A Boolean value for each point.
%                           TRUE (1) = sensor data has been collected at
%                           the point (the part has been detected). This is
%                           initialized as an array of FALSE. 
%               .isOverlapPt - (nPts x 1 bool) A Boolean value for each
%                              point. TRUE (1) = the point overlaps with a
%                              previously inspected region. This is
%                           initialized as an array of FALSE.
%               .amp - (n x 1 double) A value for each point, to store
%                      the amplitude value measured by the sensor. This is
%                           initialized as an array of zeros.
%               .standoff - (n x 1 double) A value for each point, to
%                           store the measured sensor standoff. This is
%                           initialized as an array of zeros.
%               .nViaPts - (1 x 1 uint32) Number of points  constituting
%                          the robotic trajectory. This is initialized as 1.
%               .iViaPts - (n x 1 uint32) Indices of the trajectory
%                          points (the indices refer to the points in pts).
%                          This is initialized as an array of zeros, but
%                          the first element is set to 1.
%               .isAcquPt - (n x 1 bool) A Boolean value for each
%                           trajectory point. TRUE (1) = sensor data
%                           acquisition has been attempted at the relative
%                           trajectory point. This is initialized as an
%                           array of FALSE, but the first element is set to
%                           TRUE.
%               .nTri - (1 x 1 uint32) Number of triangles in the geometry
%                       reconstruction tessellated surface. This is
%                            initialized as zero.
%               .iTri - (n x 3 uint32) Each row contains the indices of
%                       the vertices of one triangle. The indices refer to
%                       the points in pts (with no repetitions). This is
%                       initialized as a 2D array of zeros.
%               .nEdges - (1 x 1 uint32) Number of distinct triangle edges
%                         in the whole triangulated surface (with no
%                         repetitions). This is initialized as zero.
%               .iTriEdges - (3n x 2 uint32) Each row contains the
%                            indices of the extremities of one edge. The
%                            indices refer to the points in pts. This is
%                            initialized as a 2D array of zeros.
%               .isOutEdge - (3n x 1 bool) A Boolean value for each
%                            edge of the triangulation. TRUE (1) = the
%                            relative edge is on the perimeter of the
%                            triangulation. This is initialized as an array
%                            of FALSE values.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_initScanData v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

scanData = [];
scanData.nPts = uint32(1);
scanData.pts = [P; zeros(n-1,6)];
scanData.isDataPt = false(n,1);
scanData.isOverlapPt = false(n,1);
scanData.amp = zeros(n,1);
scanData.standoff = zeros(n,1);
scanData.nViaPts = uint32(ones(1));
scanData.iViaPts = uint32([1;zeros(n-1,1)]);
scanData.isAcquPt = [true; false(n-1,1)];
scanData.nTri = uint32(zeros(1));
scanData.iTri = uint32(zeros(n,3));
scanData.nEdges = uint32(zeros(1));
scanData.iTriEdges = uint32(zeros(3*n,2));
scanData.isOutEdge = false(3*n,1);

end

