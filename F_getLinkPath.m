function [scanData,iA,rDir] = F_getLinkPath(scanData,rDir)
%F_GETLINKPATH finds the index of pose B, required for the computation of the next
%best target sensor pose to visit in the execution of the autonomous
%robotic sensing framework described into the paper titled: "Autonomous
%Robotic Sensing for Simultaneous Geometric and Volumetric Inspection of
%Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   [scanData,iA,rDir] = F_getLinkPath(scanData,rDir)
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
%       rDir - Boolean value indicating the preferred rotation direction
%              (TRUE = clockwise, FALSE = anticlockwise).
%
%   Outputs:
%       scanData - updated data structure.
%       iA - index of pose A
%       rDir - updated Boolean value indicating the preferred rotation direction
%              (TRUE = clockwise, FALSE = anticlockwise).
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_getLinkPath v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------


pathLength = inf;
iA = [];
iLinkPath = [];
G = digraph([scanData.iTriEdges(1:scanData.nEdges,1)';scanData.iTriEdges(1:scanData.nEdges,2)'],[scanData.iTriEdges(1:scanData.nEdges,2)';scanData.iTriEdges(1:scanData.nEdges,1)']);
iOutEdges = scanData.iTriEdges(scanData.isOutEdge(1:scanData.nEdges,1) & xor(scanData.isDataPt(scanData.iTriEdges(1:scanData.nEdges,1)) & ~scanData.isOverlapPt(scanData.iTriEdges(1:scanData.nEdges,1)),scanData.isDataPt(scanData.iTriEdges(1:scanData.nEdges,2)) & ~scanData.isOverlapPt(scanData.iTriEdges(1:scanData.nEdges,2))),:);

%                 hLink = [];
for i=1:size(iOutEdges,1)
    %                     hb = plot3([scanData.pts(iFreeEdges(i,1),1) scanData.pts(iFreeEdges(i,2),1)],...
    %                         [scanData.pts(iFreeEdges(i,1),2) scanData.pts(iFreeEdges(i,2),2)],...
    %                         [scanData.pts(iFreeEdges(i,1),3) scanData.pts(iFreeEdges(i,2),3)],...
    %                         '-b','linewidth',3);
    
    %         hFigGraph = figure(2);
    %         hp = plot(G,'XData',[pts(visitedPts.i(1:end-1),1);P(1)],'YData',[pts(visitedPts.i(1:end-1),2);P(2)]);axis equal;grid on;
    
    if scanData.isDataPt(iOutEdges(i,1)) && ~scanData.isOverlapPt(iOutEdges(i,1))
        [iLinkPath0, pathLength0] = shortestpath(G,scanData.iViaPts(scanData.nViaPts),iOutEdges(i,2));
        
        if pathLength0 < pathLength
            iLinkPath = iLinkPath0;
            pathLength = pathLength0;
            if rDir == false
                rDir = true;
            end
            
            %                             delete(hLink);
            %                             hLink = plot3(scanData.pts(iLinkPath,1),scanData.pts(iLinkPath,2),scanData.pts(iLinkPath,3),'-m','linewidth',3);
        end
    else
        [iLinkPath0, pathLength0] = shortestpath(G,scanData.iViaPts(scanData.nViaPts),iOutEdges(i,1));
        
        if pathLength0 < pathLength
            iLinkPath = iLinkPath0;
            pathLength = pathLength0;
            if rDir == true
                rDir = false;
            end
            
            %                             delete(hLink);
            %                             hLink = plot3(scanData.pts(iLinkPath,1),scanData.pts(iLinkPath,2),scanData.pts(iLinkPath,3),'-m','linewidth',3);
        end
    end
    
    %                     delete(hb);
end
nLink = length(iLinkPath);

if nLink>0
    scanData.iViaPts(scanData.nViaPts+1:scanData.nViaPts+nLink+1) = [iLinkPath'; iLinkPath(end)];
    scanData.isAcquPt(scanData.nViaPts+1:scanData.nViaPts+nLink+1) = [false(nLink,1); true];
    scanData.nViaPts = scanData.nViaPts + nLink + 1;
    
    % Find triangulation edges linked to last via point
    iEdge = find(scanData.isOutEdge(1:scanData.nEdges,1) & ...
        ((((scanData.iTriEdges(1:scanData.nEdges,1) == scanData.iViaPts(scanData.nViaPts))) | ...
        ((scanData.iTriEdges(1:scanData.nEdges,2) == scanData.iViaPts(scanData.nViaPts))))) & ...
        ((scanData.isDataPt(scanData.iTriEdges(1:scanData.nEdges,1)) & ~scanData.isOverlapPt(scanData.iTriEdges(1:scanData.nEdges,1))) | (scanData.isDataPt(scanData.iTriEdges(1:scanData.nEdges,2)) & ~scanData.isOverlapPt(scanData.iTriEdges(1:scanData.nEdges,2)))) & ...
        ((scanData.iTriEdges(1:scanData.nEdges,1) ~= scanData.iViaPts(scanData.nViaPts-2)) & (scanData.iTriEdges(1:scanData.nEdges,2) ~= scanData.iViaPts(scanData.nViaPts-2))), 1);
    
    if isempty(iEdge)
        iEdge = find(scanData.isOutEdge(1:scanData.nEdges,1) & ...
            ((((scanData.iTriEdges(1:scanData.nEdges,1) == scanData.iViaPts(scanData.nViaPts))) | ...
            ((scanData.iTriEdges(1:scanData.nEdges,2) == scanData.iViaPts(scanData.nViaPts))))) & ...
            ((scanData.isDataPt(scanData.iTriEdges(1:scanData.nEdges,1)) & ~scanData.isOverlapPt(scanData.iTriEdges(1:scanData.nEdges,1))) | (scanData.isDataPt(scanData.iTriEdges(1:scanData.nEdges,2)) & ~scanData.isOverlapPt(scanData.iTriEdges(1:scanData.nEdges,2)))));
    end
    
    [iA,~,scanData] = F_getA(scanData,iEdge,rDir);
end


end

