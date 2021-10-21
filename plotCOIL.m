function p=plotCOIL(state,READINGS,id_robot,no_readings)

index=READINGS.CABECERA.Robot_ID==id_robot;

subtable_bands=table(zeros(no_readings,11),zeros(no_readings,11),'VariableNames',["BandsDetected" "BandsEmitted"]);
subtable_bands_2=READINGS.BANDS_BOBINA(index,:);
[M,~]=size(subtable_bands_2);
subtable_bands(1:M,:)=subtable_bands_2(1:M,:);
bands_number=1:11;
submatrixDBG=[READINGS.CABECERA.DBG_MSG(READINGS.CABECERA.Robot_ID==id_robot);0];

if READINGS.CABECERA.Robot_ID(1)==id_robot & READINGS.NAVIGATION.CurrentState(1)==state
subplot(5,3,1+(id_robot-1).*3)
bar(bands_number,READINGS.BOBINA{1,"PSD_Bands"},0.8,'FaceColor',rgb('LightSkyBlue'),'EdgeColor','none');
hold on
%bar(bands_number,READINGS.COILSIGMAS{1,"Thld_B"},0.4);
plot(bands_number,READINGS.COILSIGMAS{1,"Thld_B"},'_r','MarkerSize',7);
plot(bands_number,READINGS.COILSIGMAS{1,"SM_B"},'_','MarkerSize',7,'Color',rgb('Navy'));
yline(READINGS.BOBINA.PSD_Baseline(1),'-','Baseline','LabelVerticalAlignment','bottom','LineWidth',2,'Color',rgb('Indigo'))
%text(1:11,repmat(1E-5,11,1),num2str(READINGS.COILSIGMAS.N_B(1,:)),'horiz','center'); 
box off
hold off

%text(1:length(Y),Y,num2str(Y'),'vert','bottom','horiz','center'); 

xlabel("Banda")
ylabel("PSD (V^2/Hz)")
grid
title("R"+id_robot)
set(gca, 'YScale', 'log','XGrid','off')
ylim([1E-5 10])

subplot(5,3,2+(id_robot-1).*3)
h1=heatmap(subtable_bands.BandsEmitted(1:no_readings,:));
xlabel("Bandas emitidas")
ylabel("Últimas lecturas")
%title("R"+id_robot)
caxis([0 1])
h1.Colormap=copper;
h1.ColorbarVisible = 'off';
title("Actual DBG: "+submatrixDBG(1)+"        Past: "+submatrixDBG(2))

subplot(5,3,3+(id_robot-1).*3)
h2=heatmap(subtable_bands.BandsDetected(1:no_readings,:));
xlabel("Bandas recibidas")
ylabel("Últimas lecturas")
caxis([0 1])
h2.Colormap=bone;
h2.ColorbarVisible = 'off';
end
    
end

