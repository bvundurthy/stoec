%% ergodicity, optimization and figure params

ergs.s=1.5; % parameter of soblev norm

ergs.Nkx = 50;
ergs.Nky = 50;

%% Parameters for computing C_K and B_j in SMC update

ergs.KX = (0:ergs.Nkx-1)' * ones(1,ergs.Nky);
ergs.KY = ones(ergs.Nkx,1) * (0:ergs.Nky-1);
ergs.LK = 1.0 ./ ((1.0 + ergs.KX.^2 + ergs.KY.^2).^ergs.s);

% % ???? These might have to be moved to within the ergodicity function 
% % Check code right after run() X3 in main.m
% ergs.HK=[]; % normalizer of fourier basis functions, will be assigned in GetFourierCoeff and used in multiple places
% ergs.muk=[];

%% simulation parameters
opts.Nsteps = 3000; %10000
opts.dt = 0.1;

%% Figure parameters
figs.image = true; 
figs.color = false; 
figs.video = false; 
figs.animate = false; 
figs.ergodicity = false; 
