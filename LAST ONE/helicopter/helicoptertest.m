% Measurement parameters
steps = 2000;
sampletime = 0.01;
filterfreq = 5;

% Memory initialization
pitchmem = zeros(1,steps);
pspeedmem = zeros(1,steps);
yawmem = zeros(1,steps);
yspeedmem = zeros(1,steps);
xtime = zeros(1,steps);

% Open connection to the device
helicopter('closeall');
h = helicopter('open', 'spixo1');

% Set speed filter
alpha =  1 / (1 + (1 / (2 * pi * filterfreq * sampletime)));
helicopter('setlimbparams', h, 'filteralpha', alpha, alpha);

% Determine and set angle offset
helicopter('setlimbparams', h, 'angleoffset', 0.0, 0.0);
helicopter('write', h, [0.0 0.0]);
for X=1:100
    y = helicopter('read', h);
    pitchsum(X) = y(1);
    yawsum(X) = y(3);
end;
helicopter('setlimbparams', h, 'angleoffset', mean(pitchsum), mean(yawsum));

% Open figure for real-time plot
figure(1);
fa = subplot(211);
stripchart(fa, 1/sampletime, (steps * sampletime), 2); ylim(fa, [-1.5 1.5]); ylabel(fa, 'Angles');
fs = subplot(212);
stripchart(fs, 1/sampletime, (steps * sampletime), 2); ylim(fs, [-100 100]); ylabel(fs, 'Speeds');

% Measure and loop until all measurements done
tic;
bt = toc;
lastX = 0;
for X=1:steps
    helicopter('write', h, [0.4+0.2*sin(X/100) 0*cos(X/125)]);
    y = helicopter('read', h);
    pitchmem(X) = y(1);
    pspeedmem(X) = y(2);
    yawmem(X) = y(3);
    yspeedmem(X) = y(4);
    t = bt + (sampletime * X);
    while (xtime(X) < t);
        xtime(X) = toc;
        % Update the chart, but not too often to keep the CPU load
        % reasonable
        if ((mod(X,5) == 0) && (X ~= lastX))
            stripchart(fa, [pitchmem(X-4:X); yawmem(X-4:X)]);
            stripchart(fs, [pspeedmem(X-4:X); yspeedmem(X-4:X)]);
            drawnow;
            lastX = X;
        end;
    end;
end;
toc;

% Turn of the motors
helicopter('write', h, [0.0 0.0]);
