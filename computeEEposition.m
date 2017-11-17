% Compute end effector position based on the current configuration

% EEpos = computeEEposition([0, 0, 0])

% Fill in the necessary inputs
function posEE = computeEEposition()
global qs % configuration (NOTE: This is only 3 angles now)


% Fill this in
EEconfig = updateQ(qs);
posEE = EEconfig(4,:);

end