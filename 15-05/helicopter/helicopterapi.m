% Helicopter USB interface - RT API specific definitions
% (c) Robert Babuska, 2012

#define init RT API                                 % initialize USB API for the Helicopter
ua = [0;0];
speed1 = 0;
speed2 = 0;
try
    helicopter('CloseAll');
    H = helicopter('open', 'spixo1');               % open port
%     [S1,S2] = spine('setget', H, [0 0 1], [0 0 1]);
    helicopter('setlimbparams', H, 'angleoffset', 0.0, 0.0);
    helicopter('write', H, [0.0 0.0]);
    HeliConnected = 1;
catch
    disp('Warning: the setup is not connected to the computer.');
    HeliConnected = 0;
end;

#define close RT API                                % close USB API
if HeliConnected
    helicopter('write', H, [0.0 0.0]);              % reset actuator
    helicopter('CloseAll');                         % close the port
end;

#define sensor
if HeliConnected
    ys = helicopter('read', H);
else
    ys = zeros(1,4);
end;

#define actuator
if HeliConnected
    helicopter('write', H, ua);                     % send control input to process
end;
