function [ pose,opt ] = initialization()
%Initialized the coverage problem. Domain, speed limits, number of agents...

opt=[];
%% Domain bounds
DomainBounds.xmin = 0.0;
DomainBounds.xmax = 100.0; % DrB 150
DomainBounds.ymin = 0.0;
DomainBounds.ymax = 100.0;  % DrB 150
Lx = DomainBounds.xmax - DomainBounds.xmin;
Ly = DomainBounds.ymax - DomainBounds.ymin;

opt.DomainBounds = DomainBounds;
opt.L = [Lx;Ly];
opt.reflectThreshold = 2; 

%% Agents params: Velocity Bounds, number of agents... 

% number of agents
opt.nagents = 1;

% forward velocity lower bound
if ~isfield(opt, 'vlb')
    opt.vlb = .1*ones(opt.nagents,1);
end

% forward velocity upper bound
if ~isfield(opt, 'vub')
    opt.vub = 2*ones(opt.nagents,1); %DrB 5*ones...
end


% angular velocity lower bound
if ~isfield(opt, 'wlb')
    opt.wlb = -0.4*ones(opt.nagents,1); % DrB -0.2
end


% angular velocity upper bound
if ~isfield(opt, 'wub')
    opt.wub = 0.4*ones(opt.nagents,1); % DrB 0.2
end


%% Initializing agent locations
pose.x = [60,20,25]; % DrB [70,20,25](slides) [140,20,25];
pose.y = [30,80,25]; % DrB [20,80,25](slides) [20,120,125];
pose.theta = 120*pi/180*ones(opt.nagents,1);
%% ergodicity params

opt.erg.s=1.5;%parameter of soblev norm

opt.erg.Nkx = 50;
opt.erg.Nky = 50;
%%%used in calculation CK and Bj in SMC update
opt.erg.KX = (0:opt.erg.Nkx-1)' * ones(1,opt.erg.Nky);
opt.erg.KY = ones(opt.erg.Nkx,1) * (0:opt.erg.Nky-1);
opt.erg.LK = 1.0 ./ ((1.0 + opt.erg.KX.^2 + opt.erg.KY.^2).^opt.erg.s);
%%%
opt.erg.HK=[];% normalizer of fourier basis functions, will ber assigned in GetFourierCoeff and used in multiple places
opt.erg.muk=[];

%% simulation params
opt.sim.Nsteps = 8000; %10000
opt.sim.dt = 0.1;

end

