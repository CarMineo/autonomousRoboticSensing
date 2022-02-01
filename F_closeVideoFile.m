function F_closeVideoFile(vidOut,saveDir,vidObj,handles,scanData,alpha,plotPath,nPtToPlot,nFrames,playVideo)
%F_CLOSEVIDEOFILE appends a frame of the current figure to the video
%file.
%   F_closeVideoFile(vidOut,saveDir,vidObj,handles,scanData,alpha,plotPath,nPtToPlot,nFrames,playVideo)
%
%   Inputs: 
%       vidOut - A boolean variable indicating if the user wants to save a
%                video.
%       saveDir - String containing the full path of the saving directory. 
%       vidObj - Video writer object.
%       handles - Figure handles.
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
%       alpha - A scalar number (double) indicating the orientation of the
%               initial sensor reference system, with respect to the global
%               x-axis direction. This is used in other functions to keep
%               the orientation of the sensor frame consistent throughout
%               the autonomous robotic inspection of a part. alpha is given
%               in degrees.
%       plotPath - a boolean variable indicating if the user wants the
%                  inspection trajectory to be highlighted through a solid
%                  black line.
%       nPtToPlot - (1 x 1 uint32) number of trajectory points to lighlight 
%                   through a shading line. If np==Inf, the whole
%                   trajectory is highlighted.
%       nFrames - Number of frames of the current figure to append to the
%                 video file.
%       playVideo - If TRUE, play video soon after saving and closing the file.
%
%   Outputs:
%       void
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_appendFrameToVideo v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------


if vidOut==1
    if plotPath && ~isinf(nPtToPlot)
        nPtToPlot = nPtToPlot - 1;
        while nPtToPlot>0
            handles = F_refreshPlots(handles,scanData,alpha,plotPath,nPtToPlot);
            F_appendFrameToVideo(vidOut,vidObj,handles,1);
            nPtToPlot = nPtToPlot - 1;
        end
        plotPath = false;
        handles = F_refreshPlots(handles,scanData,alpha,plotPath,nPtToPlot);
    end
    
    F_appendFrameToVideo(vidOut,vidObj,handles,nFrames);
    close(vidObj);
    
    if playVideo
        close all;
        winopen([saveDir '\\video.mp4'])
    end
end

end

