function Ans = manipulability(J,measure)
% Calculates the mainpulability from the Body Jacobian, and identifies
% measure "string" to find what method to use to solve said manipulability.


if( strcmp(measure,'sigmamin') )
    Ans = svds(J,1,'smallest');
elseif(strcmp(measure,'detjac'))
    Ans = det(J);
elseif(strcmp(measure,'invcond'));
    Ans = svds(J,1,'smallest')/svds(J,1,'largest');
else
    error('Invalid measure string.');
end
