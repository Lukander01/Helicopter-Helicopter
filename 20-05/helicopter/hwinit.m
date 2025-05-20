%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DSCS FPGA interface board: init and I/O conversions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Sensor calibration
pitchoffs = -[0];
pitchgain = [1];

adinoffs = [pitchoffs 0 0 0];    % input offset
adingain = [pitchgain 1 1 1];     % input gain (to radians)

