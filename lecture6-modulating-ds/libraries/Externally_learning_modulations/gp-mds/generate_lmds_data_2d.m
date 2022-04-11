function [ data ] = generate_lmds_data_2d( demPos,demVel,orgVel,varargin )
% A function that computes velocity scaling and rotation by comparing
% demVel and orgVel at the input points demPos
distance_threshold = 0.3;
if nargin>3
    distance_threshold = varargin{1};
end
% first we must get rid of data close to the origin
% compute columnwise norm
distance_to_origin = sqrt(sum(demPos.*demPos,1));
inds_to_remove = find(distance_to_origin < distance_threshold);
demPos(:, inds_to_remove) = [];
demVel(:,inds_to_remove) = [];

nbData = size(demVel,2);
data = zeros(4,nbData);
prevAngle = 0;
for n =1:nbData
    speedDes = norm(demVel(:,n));
    normDesDir = demVel(:,n)/speedDes;
    speedOrg = norm(orgVel(:,n));
    normOrgDir = orgVel(:,n)/speedOrg;
    angle = atan2(normDesDir(2),normDesDir(1))-atan2(normOrgDir(2),normOrgDir(1));
    
    % put in a good range
    if(angle > pi)
        angle = -(2*pi-angle);
    elseif(angle < -pi)
        angle = 2*pi+angle;
    end
    % because we are using regression, angles flippign between -pi and pi
    % are a problem. This is handled here by findConsistentAngle
    angle = findConsistentAngle(angle,prevAngle);
    prevAngle = angle;
    
    speed_fact = speedDes/speedOrg - 1;
    data(:,n) = [demPos(:,n);angle;speed_fact];
end

end

function a = findConsistentAngle(angle,prevAngle)
% a little function that ensures that prevent jittering from +/- pi for
% large angles. 
a = angle;
if(prevAngle > pi/2 && angle < - pi/2)
    a = angle + 2*pi;
elseif (prevAngle < -pi/2 && angle > pi/2)
    a = angle - 2*pi;
end
end

