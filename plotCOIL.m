function p=plotCOIL(state,READINGS,id_robot,no_readings,baselineGain)

index=READINGS.CABECERA.Robot_ID==id_robot;
%subtable_Bobina=READINGS.BOBINA(index,:);
subtable_bands=table(zeros(no_readings,11),zeros(no_readings,11),'VariableNames',["BandsDetected" "BandsEmitted"]);
subtable_bands_2=READINGS.BANDS_BOBINA(index,:);
[M,~]=size(subtable_bands_2);
subtable_bands(1:M,:)=subtable_bands_2(1:M,:);

if READINGS.CABECERA.Robot_ID(1)==id_robot & READINGS.NAVIGATION.CurrentState(1)==state
subplot(5,3,1+(id_robot-1).*3)
bar(READINGS.BOBINA{1,3:13});
hold on
yline(READINGS.BOBINA.PSD_Base(1),'--','Bl');
yline(READINGS.BOBINA.PSD_Base(1).*baselineGain,'--','Th');
hold off
xlabel("Banda")
ylabel("PSD (V^2/Hz)")
grid
title("R"+id_robot)
set(gca, 'YScale', 'log')
ylim([1E-6 1])

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
