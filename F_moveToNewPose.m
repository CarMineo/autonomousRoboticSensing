function handles = F_moveToNewPose(handles,scanData,plotPath,nPtToPlot,vidOut,vidObj)
%F_MOVETONEWPOSE updates the figure that is used to demonstrate the 
%execution of the autonomous robotic sensing framework described into the
%paper titled: "Autonomous Robotic Sensing for Simultaneous Geometric and
%Volumetric Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and
%A. Poole.
%   handles = F_moveToNewPose(handles,scanData,plotPath,nPtToPlot,vidOut,vidObj)
%
%   Inputs:
%       handles - Figure handles
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
%       plotPath - a boolean variable indicating if the user wants the
%                  inspection trajectory to be highlighted through a solid
%                  black line.
%       nPtToPlot - (1 x 1 uint32) number of trajectory points to lighlight 
%                   through a shading line. If np==Inf, the whole
%                   trajectory is highlighted.
%       vidObj - Video writer object.
%       vidOut - A boolean variable indicating if the user wants to save a
%                video.
%
%   Outputs:
%       handles - Updated figure handles
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_moveToNewPose v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

nPtToPlot = nPtToPlot + 1;

indEnd = scanData.nViaPts-2;
indStart = indEnd;
if ~scanData.isAcquPt(indStart)
    while ~scanData.isAcquPt(indStart-1)
        indStart = indStart-1;
    end
    
    indEnd = indEnd + 1;
    
    for i=indStart:indEnd
        set(handles.PoI,'XData',scanData.pts(scanData.iViaPts(i),1),'YData',scanData.pts(scanData.iViaPts(i),2),'ZData',scanData.pts(scanData.iViaPts(i),3));
        
        if plotPath
            if isinf(nPtToPlot)
                inspPath = scanData.pts(scanData.iViaPts(1:i),:);
                iLinks = find(~scanData.isAcquPt(1:i));
                colorMat = uint8(zeros(4,i));
            else
                alphaLine = uint8(linspace(0,255,nPtToPlot));
                if i<nPtToPlot
                    inspPath = scanData.pts(scanData.iViaPts(1:i),:);
                    alphaLine = alphaLine(end-i+1:end);
                    iLinks = find(~scanData.isAcquPt(1:i));
                    colorMat = [uint8(zeros(3,i)); alphaLine];
                else
                    inspPath = scanData.pts(scanData.iViaPts(i-nPtToPlot+1:i),:);
                    iLinks = find(~scanData.isAcquPt(i-nPtToPlot+1:i));
                    colorMat = [uint8(zeros(3,nPtToPlot)); alphaLine];
                end
            end
            
            colorMat(1,iLinks) = 255;
            colorMat(3,iLinks) = 255;
            set(handles.inspPath,'XData',inspPath(:,1),'YData',inspPath(:,2),'ZData',inspPath(:,3));
            set(handles.inspPath.Edge, 'ColorBinding', 'interpolated', 'ColorData', colorMat,'ColorType', 'truecoloralpha');
        else
            set(handles.inspPath,'XData',nan,'YData',nan,'ZData',nan);
        end
        drawnow;
        F_appendFrameToVideo(vidOut,vidObj,handles,1);
    end
end







