function Ans = getXi(g)
% Recieves input of g and extracts Twist out

% Find exp(wthet)
r = g(1:3,1:3);
theta = acos((trace(r)-1)/2);
if(theta == 0)
    w2 = [0;0;0];
    v = g(1:3,4);
else
    % w = 1/(2*sin(theta))*[r(3,2)-r(2,3);r(1,3)-r(3,1);r(2,1)-r(1,2)];
    % wwedge = SKEW3(w);
    wwedge = 1/(2*sin(theta))*(r-r');
    w2 = theta*[wwedge(3,2); wwedge(1,3); wwedge(2,1)];
    % w = [wwedge(3,2); wwedge(1,3); wwedge(2,1)];
    wwedge = logm(r);
    w = [wwedge(3,2); wwedge(1,3); wwedge(2,1)];



    wmag = sqrt(sum(w2.^2));
    expP = g(1:3,4);
    % v = inv((eye(3)-exprot)*wwedge + w*w')*expP

    Ainv = eye(3) - 1/2*wwedge + ((2*sin(wmag)-wmag*(1+cos(wmag)))/(2*wmag^2*sin(wmag))*wwedge*wwedge);
    v = Ainv*expP;
end
% find the log of the given matrix

Ans = [v;w2];






