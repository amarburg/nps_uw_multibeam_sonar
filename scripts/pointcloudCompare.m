clear;clc;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
titletext = 'Sonar Image vs Point Cloud';
clims_base = [-60 -0];
nBeams = 256;
FOV = 120/180*pi();
maxRange = 4;
xPlotRange = 4;
yPlotRange = xPlotRange*cos(45/180*pi());
filenumber = "000002";
filename = strcat("SonarRawData_", filenumber, ".csv");
filenamePC = strcat("PointCloud_", filenumber, ".csv");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bw = 29.9e3; % bandwidth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Data = csvread(filename,4,0); clearvars Beams dist plotData
plotSkips = 1;
iIndex = 0;
[Beams,dist] = ndgrid(1:length(1:plotSkips:nBeams), (100:length(Data(:,1)))/1500);
for i=2:plotSkips:nBeams+1
    iIndex = iIndex + 1;
    jIndex = 0;
    for j=1:length(Data(:,1))
        jIndex = jIndex + 1;
        plotData(iIndex,jIndex) = Data(j,i)*sqrt(3);
    end
end

range_vector = Data(:,1)';

fl = nBeams / (2.0 * tan(FOV/2.0));
sonarBeams = atan2( ((1:nBeams)-1) - 0.5 *(nBeams-1), fl);

x = range_vector.*cos(sonarBeams');
y = range_vector.*sin(sonarBeams');

figure;
ax1 = subplot(2,2,1);
scatterPointSize = 8;
scatter(x(:),y(:),scatterPointSize,20*log10(abs(plotData(:))),'filled')
clims = clims_base + 20*log10(max(max(abs(plotData))));
caxis(clims);colorbar;title('sonar image');
xlabel('X [m]');ylabel('Y [m]');h = colorbar;ylabel(h,'Echo Level');
axis equal;axis tight;colormap(ax1,hot);set(gca,'Color','k')
xlim(1.02*[0 xPlotRange]);ylim(0.90*[-yPlotRange yPlotRange])

% caxis([10 65])

% figure;
% iPlots = 1:30:nBeams;
% nPlots = length(1:30:nBeams);
% for i=2:nPlots-1
%     for j=1:length(Data(:,1))
%         temp(j) = Data(j,iPlots(i));
%     end
%     subplot(1,nPlots-2,i-1);
%     plot(abs(temp(1:length(range_vector))),range_vector);
%     ylim(1.02*[0 xPlotRange])
% end

% Read point cloud
PointCloud = csvread(filenamePC,1,0);

% Plot point cloud
subplot(2,2,3);
scatter3(PointCloud(:,1),PointCloud(:,2),PointCloud(:,3),5,'filled','k')
xlabel('X'),ylabel('Y'),zlabel('Z'); axis equal;
title('point cloud')

ax2 = subplot(2,2,[2 4 ]);
scatter(x(:),y(:),50,20*log10(abs(plotData(:))),'filled'); hold on;
% clims = clims_base + 20*log10(max(max(abs(plotData))));
title('sonar image');colormap(ax2,hsv);set(gca,'Color','k')
scatter(PointCloud(:,3),PointCloud(:,1),8,'filled','k')
scatter(PointCloud(:,3),PointCloud(:,1),4,'filled','w')
xlabel('X'),ylabel('Y'),zlabel('Z'); axis equal;
title('overlap');axis equal;
xlim(0.4*[2.5 xPlotRange]);ylim(0.15*[-yPlotRange yPlotRange])

sgtitle(titletext);
