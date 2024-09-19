%% ergodicity and optimization params

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
