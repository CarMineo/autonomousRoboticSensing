function scanData = F_removePaddingInScanData(scanData)
%F_REMOVEPADDINGINSCANDATA releases unused allocated space in scanData.
%   scanData = F_removePaddingInScanData(scanData)
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
%
%   Outputs:
%       scanData - unpadded data structure.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_removePaddingInScanData v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

scanData.pts = scanData.pts(1:scanData.nPts,:);
scanData.isDataPt = scanData.isDataPt(1:scanData.nPts,1);
scanData.isOverlapPt = scanData.isOverlapPt(1:scanData.nPts,1);
scanData.amp = scanData.amp(1:scanData.nPts,1);
scanData.standoff = scanData.standoff(1:scanData.nPts,1);

scanData.iViaPts = scanData.iViaPts(1:scanData.nViaPts,1);
scanData.isAcquPt = scanData.isAcquPt(1:scanData.nViaPts,1);

scanData.iTri = scanData.iTri(1:scanData.nTri,:);
scanData.iTriEdges = scanData.iTriEdges(1:scanData.nEdges,:);
scanData.isOutEdge = scanData.isOutEdge(1:scanData.nEdges,1);

end

