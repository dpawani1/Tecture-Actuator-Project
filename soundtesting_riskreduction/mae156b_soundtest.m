clc;clear;close all;
hearTHRE = 10^-12; %reference threshold of hearing, W/m^2

control = 35;
without_iso = 55;
with_iso = without_iso - 5;

controlINT = hearTHRE * 10^(control/10);
without_isoINT = hearTHRE * 10^(without_iso/10);
with_isoINT = hearTHRE * 10^(with_iso/10);

levels = [control, without_iso, with_iso];
levels2 = [controlINT, without_isoINT, with_isoINT];

%% decibels
figure(1)
bar(levels)
xticklabels({'Control','Rigid Mount','With Isolators'})
ylabel('Sound Level (dB)')
title('Actuator Noise Comparison')
fontsize(32,'points')
grid on

%% sound intensity - unused
figure(2)
bar(levels2)
xticklabels({'Control','Rigid Mount','With Isolators'})
ylabel('Sound Intensity (W/m^2)')
title('Actuator Noise Intensity Comparison')
fontsize(32,'points')
grid on

