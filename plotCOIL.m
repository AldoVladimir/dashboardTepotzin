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
plot(bands_number,READINGS.COILSIGMAS{1,"Thld_B"},'_r','MarkerSize',7);
plot(bands_number,READINGS.COILSIGMAS{1,"SM_B"},'_','MarkerSize',7,'Color',rgb('Navy'));
yline(READINGS.BOBINA.PSD_Baseline(1),'-','Baseline','LabelVerticalAlignment','bottom','LineWidth',2,'Color',rgb('Indigo'))
hold off
%text(1:11,repmat(1E-5,11,1),num2str(READINGS.COILSIGMAS.N_B(1,:)),'horiz','center'); 
box off
%ylabel("(V^2/Hz)")
grid
title(["PSD  R"+id_robot;"DBG      Act: "+submatrixDBG(1)+"  Pst: "+submatrixDBG(2)])
set(gca, 'YScale', 'log','XGrid','off')
set(gca, 'XTickLabel', ["1" "2" "3" "4" "5" "I" "II" "III" "IV" "V" "RST"])
ylim([1E-5 10])
yticks = get(gca,'YTick');
set(gca,'YTickLabel',yticks);

subplot(5,3,2+(id_robot-1).*3)
h1=heatmap(subtable_bands.BandsEmitted(1:no_readings,:));
%xlabel("Bandas emitidas")
%ylabel("Últimas lecturas")
caxis([0 1])
h1.Colormap=copper;
h1.ColorbarVisible = 'off';
h1.YDisplayLabels=nan(size(h1.YDisplayData));
h1.XDisplayLabels=["1" "2" "3" "4" "5" "I" "II" "III" "IV" "V" "RST"];
title(["Últimas recibidas";"Clust. Sz.: "+READINGS.NAVIGATION.ACS(1)+"     R. lost: "+READINGS.NAVIGATION.RCSL_counter(1)])

subplot(5,3,3+(id_robot-1).*3)
h2=heatmap(subtable_bands.BandsDetected(1:no_readings,:));
%xlabel("Bandas recibidas")
%ylabel("Últimas lecturas")
caxis([0 1])
h2.Colormap=bone;
h2.ColorbarVisible = 'off';
h2.YDisplayLabels=nan(size(h2.YDisplayData));
h2.XDisplayLabels=["1" "2" "3" "4" "5" "I" "II" "III" "IV" "V" "RST"];
title("Últimas emitidas")

end
    
end

