function [scanData,iAdEdges] = F_meshGrowth(scanData,i0,iA,iB,iABedge,betaAng,rDir)
%F_MESHGROWTH extends the mesh for each new added sensor pose.
%For a detailed explanation of the function see the paper titled: "Autonomous
%Robotic Sensing for Simultaneous Geometric and Volumetric Inspection of
%Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   [scanData,iAdEdges] = F_meshGrowth(scanData,i0,iA,iB,iABedge,betaAng,rDir)
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
%       i0 - (1 x 1 uint32) Index of current pose.
%       iA - (1 x 1 uint32) Index of pose A.
%       iB - (1 x 1 uint32) Index of pose B.
%       iABedge - (1 x 1 uint32) Index of mesh edge linking A and B.
%       betaAng - (1 x 1 double) a scalar value representing the angle
%                 formed by the vector from point A to the current pose and
%                 the vector from A to B (see explanation in paper). This
%                 angle is given in degrees and plays a fundamental role in
%                 deciding if either one or two triangles need to be added
%                 to the mesh.
%       iAdEdges - (2 x 1 uint32) Indices of the boundary mesh edges linked
%                  to the current pose.
%
%   Outputs:
%       iA - index of pose A
%       rDir - updated Boolean value indicating the preferred rotation direction
%              (TRUE = clockwise, FALSE = anticlockwise).
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
%   F_meshGrowth v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

scanData.nTri = scanData.nTri+1;
scanData.nEdges = scanData.nEdges + 2;
if rDir
    scanData.iTri(scanData.nTri,:) = [i0 iA scanData.nPts];
    scanData.iTriEdges(scanData.nEdges-1:scanData.nEdges,:) = [scanData.nPts i0;...
                                                             iA scanData.nPts];
else
    scanData.iTri(scanData.nTri,:) = [i0 scanData.nPts iA];
    scanData.iTriEdges(scanData.nEdges-1:scanData.nEdges,:) = [i0 scanData.nPts;...
                                                             scanData.nPts iA];
end
scanData.isOutEdge(scanData.nEdges-1:scanData.nEdges,1) = true(2,1);
iAdEdges = [scanData.nEdges-1; scanData.nEdges];

if betaAng<(pi/2)
    scanData.isOutEdge(iABedge) = false;
    scanData.isOutEdge(scanData.nEdges) = false;
    scanData.nTri = scanData.nTri+1;
    scanData.nEdges = scanData.nEdges+1;
    
    if rDir
        scanData.iTri(scanData.nTri,:) = [scanData.nPts iA iB];
        scanData.iTriEdges(scanData.nEdges,:) = [iB scanData.nPts];
    else
        scanData.iTri(scanData.nTri,:) = [scanData.nPts iB iA];
        scanData.iTriEdges(scanData.nEdges,:) = [scanData.nPts iB];
    end
    
    scanData.isOutEdge(scanData.nEdges,1) = true(1,1);
    iAdEdges(end) = scanData.nEdges;
end

end

