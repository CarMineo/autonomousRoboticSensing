function F_appendFrameToVideo(vidOut,vidObj,handles,nFrames)
%F_APPENDFRAMETOVIDEO appends a frame of the current figure to the video
%file.
%   F_appendFrameToVideo(vidOut,vidObj,handles,nFrames)
%
%   Inputs: 
%       vidOut - A boolean variable indicating if the user wants to save a
%                video.
%       vidObj - Video writer object.
%       handles - Figure handles
%       nFrames - Number of initial frames of the current figure to append
%                 to the video file.
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


if vidOut && (nFrames>0)
    for i=1:nFrames
        writeVideo(vidObj, getframe(handles.f));
    end
end

end

