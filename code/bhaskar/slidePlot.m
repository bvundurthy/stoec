close all
clear variables
clc

opt=[];
%% Domain bounds
DomainBounds.xmin = 0.0;
DomainBounds.xmax = 100.0;
DomainBounds.ymin = 0.0;
DomainBounds.ymax = 100.0;
Lx = DomainBounds.xmax - DomainBounds.xmin;
Ly = DomainBounds.ymax - DomainBounds.ymin;

opt.DomainBounds = DomainBounds;
opt.L = [Lx;Ly];

% number of agents
opt.nagents = 1;

xdel=1;%resolution in x
ydel=1;%resolution in y
xRange=opt.DomainBounds.xmin:xdel:opt.DomainBounds.xmax-xdel;
yRange=opt.DomainBounds.ymin:ydel:opt.DomainBounds.ymax-ydel;

[X,Y] = meshgrid(xRange,yRange);

%% infotrmation map presnted in howie's office last time
peaks = [30 30; 
         35 35; 
         25 25; 
         25 35; 
         35 25; 
         70 70; 
         65 65; 
         75 75;
         65 75; 
         75 65;
         ]; 

for i = 1:size(peaks,1)
    close all; 
    m=peaks(i,:); %[30 30];
    s=30*eye(2);
    % m2=[100 90];
    % s2=100*eye(2);
    % m3=[60 100];
    % s3=60*eye(2);
    
    G1 = mvnpdf([X(:), Y(:)],m,s);
    % G2 = mvnpdf([X(:), Y(:)],m2,s2);
    % G3 = mvnpdf([X(:), Y(:)],m3,s3);
    if i==1
        pdfMap = G1; 
    else 
        pdfMap=(pdfMap + G1);
    end
    
    infoMap = i*pdfMap; 
    infoMap=max(infoMap,0); %crop below 0
    infoMap=infoMap./max(infoMap); %normalize
    infoMap = infoMap./sum(sum(infoMap));
    
    Z = zeros(size(X));
    opt.erg.mu=reshape(infoMap,size(X));
    
    figure(1); 
    % hold on
    surf(X,Y,reshape(infoMap,size(X)));  % Bhaskar
    view([-31.06163 21.91969]); 
    zlim([0 0.0054]); 
    % colormap('gray');
    % saveas(gcf, strcat('plot3d_',num2str(i),'.png')); 
    exportgraphics(gca,strcat('plot3d_',num2str(i),'.png')); 

    figure(2);
    % set(gcf,'color','w'); 
    % hold on
    surface(X,Y,Z,reshape(infoMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
    colormap('gray');
    axis tight
    axis equal
    set(gca,'visible','off')
    % saveas(gcf, strcat('plot2d_',num2str(i),'.png')); 
    exportgraphics(gca,strcat('plot2d_',num2str(i),'.png'))
end

% m=[30 30];
% s=10*eye(2);
% % m2=[100 90];
% % s2=100*eye(2);
% % m3=[60 100];
% % s3=60*eye(2);
% 
% G1 = mvnpdf([X(:), Y(:)],m,s);
% % G2 = mvnpdf([X(:), Y(:)],m2,s2);
% % G3 = mvnpdf([X(:), Y(:)],m3,s3);
% infoMap=(G1);
% 
% infoMap=max(infoMap,0); %crop below 0
% infoMap=infoMap./max(infoMap); %normalize
% infoMap = infoMap./sum(sum(infoMap));
% 
% Z = zeros(size(X));
% opt.erg.mu=reshape(infoMap,size(X));
% 
% figure(1);
% % set(gcf,'color','w'); 
% hold on
% surface(X,Y,Z,reshape(infoMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
% colormap('gray');
% axis tight
% axis equal
% 
% figure(2); 
% hold on
% surf(X,Y,reshape(infoMap,size(X)));  % Bhaskar
% view([-31.06163 21.91969]); 
% zlim([0 0.02]); 
% % colormap('gray');