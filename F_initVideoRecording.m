function vidObj = F_initVideoRecording(saveDir,vidOut,frameRate,handles,nFrames)
%F_INITVIDEORECORDING initializes a video file to save the frames of the
%figure demonstrating the progress and the results of the autonomous
%robotic sensing framework described into the paper titled:
%"Autonomous Robotic Sensing for Simultaneous Geometric and Volumetric
%Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   vidObj = F_initVideoRecording(saveDir,vidOut,frameRate,handles,nFrames)
%
%   Inputs:
%       saveDir - String containing the full path of the saving directory. 
%       vidOut - A boolean variable indicating if the user wants to save a
%                video.
%       frameRate - Video frame rate. 
%       handles - Figure handles
%       nFrames - Number of initial frames of the current figure to append
%                 to the video file.
%
%   Outputs:
%       vidObj - Video writer object.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_initVideoRecording v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------


vidObj = [];

if vidOut
    vidObj = VideoWriter([saveDir '\\video'],'MPEG-4');
    vidObj.FrameRate = frameRate;
    open(vidObj);
end

if vidOut && (nFrames>0)
    for i=1:nFrames
        writeVideo(vidObj, getframe(handles.f));
    end
end

end

