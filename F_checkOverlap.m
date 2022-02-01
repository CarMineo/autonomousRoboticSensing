function [scanData,d,intTri] = F_checkOverlap(scanData,r)
%F_CHECKOVERLAP check if the current pose overlaps to any mesh triangles.
%This function is important to avoid inspecting already explored regions.
%For further details, see the paper titled: "Autonomous Robotic Sensing for
%Simultaneous Geometric and Volumetric Inspection of Free-Form Parts", by
%C. Mineo, D. Cerniglia and A. Poole.
%   [scanData,d,intTri] = F_checkOverlap(scanData,r)
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
%       r - A scalar number indicating the target inspection resolution.
%
%   Outputs:
%       scanData - updated data structure, including bridge edge (if overlap
%                  exists).
%       d - minimum distance of intersections.
%       intTri - index of intersected triangle
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_checkOverlap v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

d = [];
intTri = [];

iCurr = scanData.iViaPts(scanData.nViaPts);
% isNotParent = all(scanData.iTri(1:scanData.nTri,:) ~= iCurr,2);
% isClose = all(sqrt([sum(([scanData.pts(scanData.iTri(1:scanData.nTri,1),1)-scanData.pts(iCurr,1) ...
%                     scanData.pts(scanData.iTri(1:scanData.nTri,1),2)-scanData.pts(iCurr,2) ...
%                     scanData.pts(scanData.iTri(1:scanData.nTri,1),3)-scanData.pts(iCurr,3)].^2),2) ...
%               sum(([scanData.pts(scanData.iTri(1:scanData.nTri,2),1)-scanData.pts(iCurr,1) ...
%                     scanData.pts(scanData.iTri(1:scanData.nTri,2),2)-scanData.pts(iCurr,2) ...
%                     scanData.pts(scanData.iTri(1:scanData.nTri,2),3)-scanData.pts(iCurr,3)].^2),2) ...
%               sum(([scanData.pts(scanData.iTri(1:scanData.nTri,3),1)-scanData.pts(iCurr,1) ...
%                     scanData.pts(scanData.iTri(1:scanData.nTri,3),2)-scanData.pts(iCurr,2) ...
%                     scanData.pts(scanData.iTri(1:scanData.nTri,3),3)-scanData.pts(iCurr,3)].^2),2)])<(2*r),2);
% iTri = find(isNotParent & isClose);


% iTri = find((scanData.iTri(1:scanData.nTri,1) ~= scanData.iViaPts(scanData.nViaPts)) &...
%     (scanData.iTri(1:scanData.nTri,2) ~= scanData.iViaPts(scanData.nViaPts)) &...
%     (scanData.iTri(1:scanData.nTri,3) ~= scanData.iViaPts(scanData.nViaPts)));
iTri = find(all(scanData.iTri(1:scanData.nTri,:) ~= iCurr,2));

if ~isempty(iTri)
    R = eul2rotm(deg2rad(scanData.pts(iCurr,4:6)),'ZYX');
    [~, d, intTri] = F_intersectMultipleLineMesh(scanData.pts(iCurr,1:3),R(:,3)',scanData.pts(1:scanData.nPts,1:3),scanData.iTri(iTri,:));
    if d(1)<r
        scanData.isOverlapPt(iCurr,1) = true;
        triPts = [scanData.pts(scanData.iTri(iTri(intTri(1)),1),1:3);
                  scanData.pts(scanData.iTri(iTri(intTri(1)),2),1:3);
                  scanData.pts(scanData.iTri(iTri(intTri(1)),3),1:3)];
        dists = sqrt(sum([((triPts(:,1) - scanData.pts(iCurr,1)).^2) ...
                          ((triPts(:,2) - scanData.pts(iCurr,2)).^2) ...
                          ((triPts(:,3) - scanData.pts(iCurr,3)).^2)],2));
        iPt = find(dists == min(dists),1);
        scanData.nEdges = scanData.nEdges + 1;
        scanData.iTriEdges(scanData.nEdges,:) = [scanData.iTri(iTri(intTri(1)),iPt) iCurr];
    end
end


end

