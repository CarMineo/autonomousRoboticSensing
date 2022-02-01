function R = F_da2R(da,rotAxis)
%F_DA2R converts an angular rotation (da) around a given rotation axis into
%a rotational matrix.
%   R = F_da2R(da,rotAxis)
%
%   Inputs:
%       da = A scalar number indicating the angular increment to use in the
%            amplitude mapping process for pose correction.
%       rotAxis = A string value indicating the axis of rotation for
%                 amplitude mapping. 'X' = x-axis, 'Y' = y-axis and 
%                 'Z' = z-axis.
%
%   Outputs:
%       R - (3 x 3 double) Rotation matrix (ZYX convention).
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_da2R v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

switch rotAxis
    case 'X'
        R = [1 0         0;
            0 cosd(da) -sind(da);
            0 sind(da)  cosd(da)];
    case 'Y'
        R = [cosd(da) 0  sind(da);
            0        1  0;
            -sind(da) 0  cosd(da)];
    case 'Z'
        R = [cosd(da) -sind(da) 0;
            sind(da)  cosd(da) 0;
            0         0        1];
end

end

