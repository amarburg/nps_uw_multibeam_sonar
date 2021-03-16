clear;clc;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nBeams = 256;
clims_base = [0 50];
maxDataset = 1;
simulationGain = 0.1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
freqMode = 'HF';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% caseSet = 'Background';
caseSet = 'Horiz_Cylinder';
% caseSet = 'Vert_Cylinder';
% caseSet = 'Plate';
% caseSet = 'Plate2';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
yPlotRange = 2.0;
xPlotRange = yPlotRange*cos(pi()/4);

fig = figure('position',[100,100,1100,450]);
set(gcf,'color','w');
for k=1:2
    % Construct casename
    if k == 1 % --------- Simulation
        if strcmp(caseSet,'Plate2')
            casename = ['Recordings/record_' freqMode '_Plate'];
        else
            casename = ['Recordings/record_' freqMode '_' caseSet];
        end
        subtitletext = 'Simulation'; disp('------ Simulation -------')
        Gain = simulationGain;
    else % -------------- Dataset
        casename = ['Dataset/dataset_' freqMode '_' caseSet];
        %%%%%%%%%%%%%%%%%%%%%%%%%%% STRANGE %%%%%%%%%%%%%%%%%%%%%%%%%%
        nBeams = nBeams;
        %%%%%%%%%%%%%%%%%%%%%%%%%%% STRANGE %%%%%%%%%%%%%%%%%%%%%%%%%%
        subtitletext = 'Experiment'; disp('------ Experiment -------')
        Gain = 1;
    end
    % Set FOV according to frequency mode
    if strcmp(freqMode,'HF')
        FOV = 60/180*pi();
        titletext = [caseSet '@ 2.1 MHz'];
    elseif strcmp(freqMode,'LF')
        FOV = 130/180*pi();
        titletext = [caseSet '@ 1.2 MHz'];
    end
    titletextList = split(titletext,'_');

    % number of datasets for averaging
    all_files = dir(casename);
    all_dir = all_files([all_files(:).isdir]);
    num_dir = numel(all_dir)-2;
    if num_dir > maxDataset
        num_dir = maxDataset;
    end
    % ---------- Read data ------------ %
    opts = delimitedTextImportOptions("NumVariables", 1);
    opts.DataLines = [1, Inf]; opts.VariableNames = "data"; opts.VariableTypes = "double";
    opts.Delimiter = ","; opts.ExtraColumnsRule = "ignore"; opts.EmptyLineRule = "read";
    % Read ranges
    filename = convertCharsToStrings([casename '/' num2str(1) '/ranges']);
    ranges = readtable(filename, opts).data;
    % Read azimuth_angles
    filename = convertCharsToStrings([casename '/' num2str(1) '/azimuth_angles']);
    azimuth_angles = readtable(filename, opts).data;
    % Read azimuth_beamwidth
    filename = convertCharsToStrings([casename '/' num2str(1) '/azimuth_beamwidth']);
    azimuth_beamwidth = readtable(filename, opts).data;
    % Read elevation_beamwidth
    filename = convertCharsToStrings([casename '/' num2str(1) '/elevation_beamwidth']);
    elevation_beamwidth = readtable(filename, opts).data;
    for n = 1:num_dir
        % Read intensities
        filename = convertCharsToStrings([casename '/' num2str(n) '/intensities']);
        intensities_tot{n} = readtable(filename, opts).data;
    end
    for s = 1:length(intensities_tot{1})
        intensities(s) = 0;
        for n = 1:num_dir
            intensities(s) = intensities(s)+intensities_tot{n}(s)/num_dir;
        end
    end

%     figure;set(gcf,'color','w');
%     for i=2:length(azimuth_angles)
%         increments(i) = (azimuth_angles(i)-azimuth_angles(i-1))/pi()*180;
%     end
%     bar(increments);
%     legend('azimuth angle spacing [deg]');
%     title(['azimuth beamwidth = ' num2str(azimuth_beamwidth/pi()*180)]);
%     clearvars increments

%     figure;set(gcf,'color','w');
%     for i=2:length(ranges)
%         increments(i) = (ranges(i)-ranges(i-1));
%     end
%     bar(increments);
%     legend('range spacing [m]');
%     clearvars increments

    clearvars Beams dist plotData
    plotSkips = 1;
    iIndex = 0;
    [Beams,dist] = ndgrid(1:length(1:plotSkips:nBeams), (100:length(intensities(:,1)))/1500);

    for i=1:nBeams
        iIndex = iIndex + 1;
        jIndex = 0;
        for j=1:length(ranges)
            jIndex = jIndex + 1;
            plotData(iIndex,jIndex) = intensities((j-1)*nBeams + i);
        end
    end

    range_vector = ranges';

    fl = nBeams / (2.0 * tan(FOV/2.0));
    sonarBeams = atan2( ((1:nBeams)-1) - 0.5 *(nBeams-1), fl);

    x = range_vector.*cos(azimuth_angles);
    y = range_vector.*sin(azimuth_angles);

    disp(['Range data length  :' num2str(length(ranges))])
    disp(['Range Resolution   :' num2str(ranges(10)-ranges(9))])
    disp(['Number of datasets :' num2str(num_dir)])
    disp('')

    subplot(1,2,k)
    scatterPointSize = 8;
    scatter(y(:),x(:),scatterPointSize,20*log10(abs(plotData(:)*Gain)),'filled')
%     scatter(y(:),x(:),scatterPointSize,abs(plotData(:)),'filled')
    % clims = clims_base + 20*log10(max(max(abs(plotData))));
    xlabel('X [m]'); ylabel('Y [m]')
    clims = clims_base; caxis(clims); colorbar; h = colorbar;
    ylabel(h,'Echo Level')
%     ylabel(h,'Intensities')

    colormap(hot)
    % set(gca,'Color','k')
    axis equal; axis tight; grid on; set(gca,'FontSize',12)
    title(subtitletext);
    ylim(1.02*[0 yPlotRange])
    xlim(1.02*[-xPlotRange xPlotRange])
end
sgtitle([titletextList{1} ' ' titletextList{2}]); set(gca,'FontSize',12)