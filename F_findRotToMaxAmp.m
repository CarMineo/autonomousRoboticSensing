function [RpMax,ampMax,stoffMax,angles,amps,stoffs] = F_findRotToMaxAmp(P,RpStart,ampStart,stoffStart,noise,da,rotAxis,tStoff,TR,FN)
%F_FINDROTTOMAXAMP corrects a sensor pose orientation, to maximize the 
%signal amplitude. This is crucial to the execution of the autonomous
%robotic sensing framework described into the paper titled: "Autonomous
%Robotic Sensing for Simultaneous Geometric and Volumetric Inspection of
%Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   [RpMax,ampMax,stoffMax,angles,amps,stoffs] = F_findRotToMaxAmp(P,RpStart,ampStart,stoffStart,noise,da,rotAxis,tStoff,TR,FN)
%
%   Inputs:
%       RpStart - Rotation matrix of the current pose (3x3 double). This
%                 could be retrieved inside this function, from the first
%                 input through:
%                 RpStart = eul2rotm(deg2rad(P(1,4:6)),'ZYX').
%                 However, RpStart is provided for convenience, in order to
%                 not duplicate the computation effort undertaken before
%                 the execution of this function.
%       ampStart - (1 x 1 double) Current sensor amplitude value.
%       stoffStart - (1 x 1 double) Current sensor standoff.
%       noise = A scalar number indicating the level of sensor signal noise.
%       da = A scalar number indicating the angular increment to use in the
%            amplitude mapping process for pose correction.
%       rotAxis = A string value indicating the axis of rotation for
%                 amplitude mapping. 'X' = x-axis, 'Y' = y-axis and 
%                 'Z' = z-axis.
%       tStoff = A scalar number indicating the target sensor standoff.
%       TR = Triangulation of part surface. This is only for simulation
%            purpose, to support the generation of simulated sensor signals.
%            The part is unknown in real applications.
%       FN = Noprmals of triangulation faces. This is only for simulation
%            purpose, to support the generation of simulated sensor signals.
%            The part is unknown in real applications.
%
%   Outputs:
%       RpMax - Rotation matrix of the pose (3x3 double), where the maximum
%               sensor signal amplitude was registered.
%       ampMax - (1 x 1 double) Maximum registered sensor signal amplitude.
%       stoffMax - (1 x 1 double) Sensor standoff relative to the pose,
%                  where the maximum sensor signal amplitude was registered.
%       angles - Array of explored angles. The angles are given in degrees
%                and their value is given with respect to the initial
%                orientation.
%       amps - Array of all registered amplitudes, corresponding to the
%              explored angles.
%       stoffs - Array of all registered sensor standoffs, corresponding to
%                the explored angles.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_findRotToMaxAmp v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------


amp = ampStart;
standoff = stoffStart;
RpMax = RpStart;
ampMax = ampStart;
stoffMax = stoffStart;
amp0 = ampStart;

signCorrectRot = nan;
n = ceil(180/da) + 1;
deltaAngles = zeros(1,n);
deltaAmps = zeros(1,n);
deltaAmps(1) = amp;
deltaStandoffs = zeros(1,n);
deltaStandoffs(1) = standoff;
angles = 0;
amps = amp;
stoffs = standoff;

counter = 1;

if amp<=(1.5*noise)
    return;
end

switch rotAxis
    case 'X'
        xaxis = [1 0 0];
        if (acosd(dot(xaxis,abs(RpStart(:,3)))) <= da)
            return;
        end
    case 'Y'
        yaxis = [0 1 0];
        if (acosd(dot(yaxis,abs(RpStart(:,3)))) <= da)
            return;
        end
    case 'Z'
        zaxis = [0 0 1];
        if (acosd(dot(zaxis,abs(RpStart(:,3)))) <= da)
            return;
        end
end

while true
    if amp>(amp0+noise)
        if isnan(signCorrectRot)
            signCorrectRot = sign(da);
        end
        
        amp0 = amp;
        da0 = da;
    elseif amp<(amp0-noise)
        if isnan(signCorrectRot)
            da = -da;
            signCorrectRot = sign(da);
            
            amp0 = ampStart;
            
            if (sign(da) < 0)
                da0 = -angles(end) + da;
            else
                da0 = -angles(1) + da;
            end
        else
            break;
        end
    else
        if (angles(end)-angles(1) < 360)
            da0 = da;
        else
            break;
        end
    end
    
    counter = counter + 1;
    deltaAngles(counter) = da0;
    angles = cumsum(deltaAngles(1:counter));
    [angles,is] = sort(angles);
    
    if (sign(da0) < 0)
        R = F_da2R(angles(1),rotAxis);
    else
        R = F_da2R(angles(end),rotAxis);
    end
        
    Rp = R*RpStart;
    
    [amp,standoff] = F_simSensorData(P,Rp,TR,FN,noise,tStoff);
    
    deltaAmps(counter) = amp;
    deltaStandoffs(counter) = standoff;
    
    if amp>ampMax
        ampMax = amp;
        stoffMax = standoff;
        RpMax = Rp;
    end
end

amps = deltaAmps(is);
stoffs = deltaStandoffs(is);

fitCurve = fit(angles', amps','poly2');
polyCoeff = coeffvalues(fitCurve);
da = roots(polyder(polyCoeff));
maxVal = fitCurve(da);
if (polyCoeff(1)<0) || (abs(maxVal-ampMax)<(noise/2))
    R = F_da2R(da,rotAxis);
    RpMax = R*RpStart;
    
    [ampMax,stoffMax] = F_simSensorData(P,RpMax,TR,FN,noise,tStoff);
end


end

