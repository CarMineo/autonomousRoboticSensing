function [iA,rDir,scanData] = F_getA(scanData,iAdEdges,rDir)
%F_GETA finds the index of pose A, required for the computation of the next
%best target sensor pose to visit in the execution of the autonomous
%robotic sensing framework described into the paper titled: "Autonomous
%Robotic Sensing for Simultaneous Geometric and Volumetric Inspection of
%Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   [iA,rDir,scanData] = F_getA(scanData,iAdEdges,rDir)
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
%       iAdEdges - (2 x 1 uint32) Indices of the boundary mesh edges linked
%                  to the current pose.
%       rDir - Boolean value indicating the preferred rotation direction
%              (TRUE = clockwise, FALSE = anticlockwise).
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
%   F_getA v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------


if isempty(iAdEdges)
    iA = [];
    return;
end

iCurr = scanData.iViaPts(scanData.nViaPts);
iPrev = scanData.iViaPts(scanData.nViaPts-1);

iPts = scanData.iTriEdges(iAdEdges,:);
iValidEdges = find((scanData.isDataPt(iPts(:,1)) & ~scanData.isOverlapPt(iPts(:,1))) |...
                   (scanData.isDataPt(iPts(:,2)) & ~scanData.isOverlapPt(iPts(:,2))));

l = length(iValidEdges);
if (l == 2)
    iForwEdge = find((iPts(:,1) ~= iPrev) & (iPts(:,2) ~= iPrev));
    iAedge = iForwEdge;
    
    scanData.isOutEdge(iAdEdges(iAedge)) = false;
    iEdgePts = iPts(iAedge,:);
    iA = iEdgePts(iEdgePts ~= iCurr);
elseif (l == 1)
    iForwEdge = find((iPts(:,1) ~= iPrev) & (iPts(:,2) ~= iPrev));
    if iValidEdges == iForwEdge
        iAedge = iForwEdge;
    else
        iAedge = iValidEdges;
        rDir = ~rDir;
    end
    
    scanData.isOutEdge(iAdEdges(iAedge)) = false;
    iEdgePts = iPts(iAedge,:);
    iA = iEdgePts(iEdgePts ~= iCurr);
else
    iA = [];
end


end

