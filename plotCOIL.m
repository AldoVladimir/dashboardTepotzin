function p=plotCOIL(state,READINGS,id_robot,no_readings)

index=READINGS.CABECERA.Robot_ID==id_robot;

subtable_bands=table(zeros(no_readings,11),zeros(no_readings,11),'VariableNames',["BandsDetected" "BandsEmitted"]);
subtable_bands_2=READINGS.BANDS_BOBINA(index,:);
[M,~]=size(subtable_bands_2);
subtable_bands(1:M,:)=subtable_bands_2(1:M,:);
bands_number=1:11;

if READINGS.CABECERA.Robot_ID(1)==id_robot & READINGS.NAVIGATION.CurrentState(1)==state
subplot(5,3,1+(id_robot-1).*3)
%bar(bands_number,[READINGS.BOBINA{1,["PSD_Bands" ]}; READINGS.COILSIGMAS{1,"Thld_B"}],0.8);
bar(bands_number,READINGS.BOBINA{1,["PSD_Bands" ]},0.8,'FaceColor',rgb('LightSkyBlue'),'EdgeColor','none');
hold on
%bar(bands_number,READINGS.COILSIGMAS{1,"Thld_B"},0.4);
plot(bands_number,READINGS.COILSIGMAS{1,"Thld_B"},'_r','MarkerSize',7);
yline(READINGS.BOBINA.PSD_Baseline(1),'-','Baseline','LabelVerticalAlignment','bottom','LineWidth',2,'Color',rgb('Indigo'))
hold off

xlabel("Banda")
ylabel("PSD (V^2/Hz)")
grid
title("R"+id_robot)
set(gca, 'YScale', 'log','XGrid','off')
ylim([1E-5 1])

subplot(5,3,2+(id_robot-1).*3)
h1=heatmap(subtable_bands.BandsEmitted(1:no_readings,:));
xlabel("Bandas emitidas")
ylabel("Últimas lecturas")
%title("R"+id_robot)
caxis([0 1])
h1.Colormap=copper;

subplot(5,3,3+(id_robot-1).*3)
h2=heatmap(subtable_bands.BandsDetected(1:no_readings,:));
xlabel("Bandas recibidas")
ylabel("Últimas lecturas")
caxis([0 1])
h2.Colormap=bone;
end
    
end

