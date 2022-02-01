%
% This matlab script demonstrates the work related to the paper titled:
% "Autonomous Robotic Sensing for Simultaneous Geometric and Volumetric
% Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%
% INSTRUCTIONS TO USE THIS DEMONSTRATION SCRIPT:
% In order to demonstrate the autonomous robotic inspection framework,
% without a real inspection system, the present script allows generating a
% random geometry. The generated geometry is displayed in the figure, to
% inform the user and the function responsible to simulate sensor data. The
% incremental exploration of the part geometry, through the proposed
% framework, leads to the generation of a surface reconstruction mesh and
% textured 3D maps of the sensor amplitude and sensor standoff. The little
% variability of the amplitude and of the sensor standoff, despite of the
% part surface curvature, are a demonstration of the good functionality of
% the autonomous pose correction and navigation algorithms. The user can
% control the type of surface to generate, through some key geometry
% properties, as well as indicating the key input parameters for the
% inspection (stepping angle for amplitude mapping, sensor noise amplitude,
% target sensor standoff target resolution, angle for selection of second
% sensor pose, and preferred initial rotation direction). Instead, the
% initial inspection pose is randomly selected in this demonstration
% script, in order to prove the robustness of the framework. A new random
% shape and starting pose are generated for each execution of the script.
% The authors of this work hope this demonstration script can help the full
% understanding of the framework, which is fully described in the linked
% scientific publication, and guide the adaptation of the framework
% functions to deploy it to real applications. Do not hesitate to contact
% the author, if you need any clarifications.
%
% DISCLAIMER: 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
% AUTHOR OF THIS DEMONSTRATION SCRIPT: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
% Acknowledgements:
%    Per-Olof Persson and Gilbert Strang - Authors of "A Simple Mesh
%                                          Generator in MATLAB", 
%                                          SIAM Review Vol. 46 (2) 2004.
%    David Legland - Author of NORMALIZEVECTOR3D, VECTORNORM3D and
%                    INTERSECTLINEMESH3D.
%    Sven Holcombe - Author of VECTORCROSS3D.
%    Pariterre - Author of AVERAGERT, FROMANGLETOMATRIX and
%                FROMMATRIXTOANGLE.
%    Aslak Grinsted - Author of SUBAXIS and PARSEARGS.

%--------------------------------------------------------------------------
clear; clc; close all;  % Clear enviroment
%--------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                    DEFINITION OF INPUT PARAMETERS                   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters for autonomous sensor pose correction 
da = 1.00;      % Amplitude mapping step angle [degrees]
n = 0.1;        % Sensor noise amplitude (for signal simulation)
tStoff = 30;    % Target sensor standoff

% Parameters for autonomous geometry exploration 
r = 2;          % Target inspection resolution
% P             % Starting pose is randomly selected in this example
theta = 0;      % Angle for selection of second sensor pose
rDir = false;   % Preferred initial rotation direction [TRUE - clockwise,
                % FALSE - anticlockwise]

% Parameters for generation of random part surface geometry
geomProp = struct;                  % Structure array
geomProp.surfType = 'spherical';    % .surfType - Type of surface curvature
                                    %            flat - Flat surface 
                                    %     cylindrical - Cylindrical surface
                                    %       spherical - Spherical surface
geomProp.curvRad = 21;              % .curvRad - Surface radius of curvature
geomProp.hollow = false;            % .hollow - Boolean variable indicating
                                    %           if the surface contains an
                                    %           hole. See desription of 
                                    %           F_randomShapeGen function.
geomProp.inRad = 10;                % .inRad - Minimum distance of surface
                                    %          boundary from origin.
geomProp.outRad = 20;               % .outRad - Maximum distance of surface
                                    %           boundary from origin.
geomProp.baseThickness = 1;         % .baseThickness - Heigth of part base

% Parameters for controlling the output
plotPath = true;    % TRUE - Path is highlighted through solid line
nPtToPlot = 10;     % Number of last visited points for path plotting
figOut = true;      % TRUE - Save final figure
vidOut = false;     % TRUE - Save animation video
frameRate = 10;     % Video frame rate
playVideo = false;  % TRUE - Play video soon after saving

%--------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  GENERATION OF RANDOM GEOMETRY AND RANDOM INSPECTION STARTING POSE  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[TR,FN,IC,shapeBoundary,P,TRside] = F_randomShapeGen(geomProp);

% Random selection of a starting pose. 
P = P(ceil(rand*size(P,1)),:); 

%--------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%
%%% ALGORITHM BODY %%%
%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%         INITIALIZATION OF AUTONOMOUS GEOMETRY EXPLORATION           %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[scanData,R,alpha] = F_initScan(P,n,da,tStoff,r,theta,rDir,TR,FN);
if ~isempty(scanData)
    iAdEdges = [2;3];
    saveDir = F_makeSaveDir();
    handles = F_initFigure(TR,TRside,shapeBoundary);
    vidObj = F_initVideoRecording(saveDir,vidOut,frameRate,handles,5);
    handles = F_refreshPlots(handles,scanData,alpha,plotPath,nPtToPlot);
    F_appendFrameToVideo(vidOut,vidObj,handles,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%       AUTONOMOUS POSE CORRECTION AND PART SURFACE EXPLORATION       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~isempty(scanData)
    while true
        scanData = F_makeSpaceInScanData(scanData,100);
        [P,R,scanData,rDir,iAdEdges] = F_getNextPt(scanData,r,rDir,alpha,iAdEdges);

        if isempty(P)
            break; % Terminate if there are no more points to visit
        end
        handles = F_moveToNewPose(handles,scanData,plotPath,nPtToPlot,vidOut,vidObj);
        [amp,stoff] = F_simSensorData(P,R,TR,FN,n,tStoff);
        [scanData,R] = F_correctPt(scanData,R,amp,stoff,tStoff,n,da,TR,FN);
        handles = F_refreshPlots(handles,scanData,alpha,plotPath,nPtToPlot);
        F_appendFrameToVideo(vidOut,vidObj,handles,1);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                             SAVE RESULTS                            %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scanData = F_removePaddingInScanData(scanData);

save([saveDir '\\scanData.mat'],'scanData');

if figOut
    savefig(handles.f, [saveDir '\\figure.fig']);
end

F_closeVideoFile(vidOut,saveDir,vidObj,handles,scanData,alpha,plotPath,nPtToPlot,20,playVideo);





