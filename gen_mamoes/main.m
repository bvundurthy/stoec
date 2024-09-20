%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Author: Bhaskar Vundurthy(bvundurthy@outlook.com)
%%Based on the works of Elif Ayvali (eayvali@gmail.com) / Hadi Salman (hadicsalman@gmail.com)
%%Biorobotics lab, The Robotics Institute, Carnegie Mellon University
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run the parameters file - prevents data transfer between functions
close all; clear variables; clc;
tic

run("S1_agentParams.m"); 
run("S2_mapParams.m"); 
run("S3_ergOptParams.m"); 

%% Continuing older code here...



ergs.mu=reshape(infoMap,size(X));
ergs.mu = ergs.mu/sum(sum(ergs.mu));% normalize iformation distribution
[ergs.muk, ergs.HK] = GetFourierCoeff(opt,X,Y);
opts.kdOBJ = KDTreeSearcher([X(:),Y(:)]);%to eval traj cost by closest point

%% Plot utility 
figure(1);set(gcf,'color','w'); hold on
% surface(X,Y,Z,reshape(infoMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
surface(X,Y,Z,reshape(drawMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
colormap('gray'); 
axis tight
axis equal
% colorbar;


%% run simulation
Nsteps = opt.sim.Nsteps;
dt = opt.sim.dt;

% Initializing Fourier coefficients of coverage distribution
Ck = zeros(ergs.Nkx, ergs.Nky);
figure(1);
colors= {'k','g','m'};
Ergodicity_Metric = zeros(Nsteps,1);
BhattDistance = zeros(Nsteps,1);

for iagent = 1:opt.nagents
    scatter(pose.x(iagent),pose.y(iagent),'filled','MarkerFaceColor',[0,0,0],'MarkerEdgeColor',[1,1,1]);%plot posistion of agents initially
%     text(pose.x,pose.y,'\leftarrow start','Color','r')
    h(iagent) = plot(pose.x(iagent),pose.y(iagent), 'Marker', 'o', 'MarkerSize', 2,'Color', [0,0,0]);    
end

traj = zeros(Nsteps, opt.nagents, 3);%%(iteration * agent * [x,y])        
% saveas(gcf,'initial_SMC.fig')

%%
% pause(5);%pause to have time to take video
tic
v = VideoWriter('peaks.mp4', 'MPEG-4'); % DrB
open(v) %DrB
for it = 1:Nsteps
    % if(it == Nsteps/2)
    %     saveas(gcf,'halfway_SMC.fig')
    % end
    time = (it) * dt;
    [pose, Ck] = SMC_Update(pose, Ck, time, opt);
    traj_stat = zeros(size(ergs.mu(:)));
    for iagent = 1:opt.nagents
        traj(it,iagent,1)= pose.x(iagent);
        traj(it,iagent,2)= pose.y(iagent);
        set(h(iagent),'XData',traj(1:it,iagent,1),'YData',traj(1:it,iagent,2),'Color','y');
    
        traj_stat = traj_stat + timeAverageStatisticsDistribution( reshape(traj(:,iagent,1:2),Nsteps,2), opt); 
    end
    %%just for speed, draw every bla iterations
    if( mod(it,10) == 0)
        drawnow
        frame = getframe(gcf); %DrB
        writeVideo(v,frame); %DrB
    end
    
    disp(it);
    %%%%%%%%%%%%%%%    metric evaluations  %%%%%%%%%%%%%%%%%%%%
    %%%ergodicity
    ck = Ck/opt.nagents/time;
    [Ergodicity_Metric(it)] = Calculate_Ergodicity(ck, opt);
    
    %%%Bhatttacharyya distance
    BhattDistance(it) = evaluateBhattacharyyaDist( traj_stat, opt );
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
close (v); % DrB
% simulationTime = toc;
% saveas(gcf,'end SMC.fig')
%plot positions of agents at the end of the trajectory
% for iagent = 1:opt.nagents
%         scatter(pose.x(iagent),pose.y(iagent),colors(iagent),'fill');
% %         text(pose.x,pose.y,'\leftarrow start','Color','r')
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          PLOTS                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot metric of ergodicity as a function of time
% time=dt:dt:dt*Nsteps;
% figure;
% % axis([0.001 5 0.0001,1])
% xlabel('Time (sec)');
% ylabel('Coverage Metric, \phi(t)');
% title('metric of ergodicity as a function of time')
% % axis([-10 5010 10e-5 10]);
% % Dr. B
% % Ergodicity_Metric = Ergodicity_Metric.*10000; 
% pause(1); 
% 
% w = VideoWriter('ergodicity.mp4', 'MPEG-4'); % DrB
% open(w) %DrB
% for i = 1:20:Nsteps
%     loglog(time(1:i),Ergodicity_Metric(1:i),'LineWidth',2); 
% 
%         frame = getframe(gcf); %DrB
%         writeVideo(w,frame); %DrB
% end
% close (w); % DrB

% 
%% plot "Bhatttacharyya distance" (between information distribution and time averaged statistics of trajectories) as a function of time
% time=dt:dt:dt*Nsteps;
% figure;plot(time(1:end),BhattDistance(1:end))
% % axis([0.001 5 0.0001,1])
% xlabel('Time (sec)');
% ylabel(',Bhatttacharyya distance (BhDist(t))');
% title('Bhatttacharyya distance as a function of time')



% 
% %%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%         Trajectory plots for IROS paper    %%%%%%%%%%%%%%%%%%%%%%%  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear; close all;
% load SMC1.mat
% 
% Nsteps = opt.sim.Nsteps;
% 
% 
% %t = 0
% figure(1);set(gcf,'color','w'); hold on
% surface(X,Y,Z,reshape(infoMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
% axis tight
% axis equal
% colorbar;
% scatter(140,20,'r','fill')
% title('t = 0 sec')
% 
% %t = 500 sec
% figure(2);set(gcf,'color','w'); hold on
% surface(X,Y,Z,reshape(infoMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
% axis tight
% axis equal
% colorbar;
% scatter(140,20,'r','fill')
% plot3(traj(1:Nsteps/2,:,1),traj(1:Nsteps/2,:,2),traj(1:Nsteps/2,:,3),'r-','LineWidth', 2, 'MarkerSize', 5);
% title('t = 500 sec')
% 
% %t = 1000 sec
% figure(3);set(gcf,'color','w'); hold on
% surface(X,Y,Z,reshape(infoMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
% axis tight
% axis equal
% colorbar;
% scatter(140,20,'r','fill')
% plot3(traj(1:Nsteps,:,1),traj(1:Nsteps,:,2),traj(1:Nsteps,:,3),'r-','LineWidth', 2, 'MarkerSize', 5);
% title('t = 1000 sec')
% 
% 
% % plot "Bhatttacharyya distance" (between information distribution and time averaged statistics of trajectories) as a function of time
% figure(4)
% dt = opt.sim.dt;
% time=dt:dt:dt*Nsteps;
% plot(time(1:Nsteps),BhattDistance(1:Nsteps));
% xlabel('Time (sec)');
% ylabel(',Bhatttacharyya distance (BhDist(t))');
% title('Bhatttacharyya distance as a function of time')
