function p=plotCOIL(state,READINGS,id_robot,no_readings)
nCol=4;
index=READINGS.CABECERA.Robot_ID==id_robot;


subtable_bands=table(zeros(no_readings,11),zeros(no_readings,11),'VariableNames',["BandsDetected" "BandsEmitted"]);
subtable_bands_2=READINGS.BANDS_BOBINA(index,:);
subtable_CH=READINGS.RSSI.CH_Tx(index);

[M,~]=size(subtable_bands_2);
[N,~]=size(subtable_CH);
subtable_bands(1:M,:)=subtable_bands_2(1:M,:);
bands_number=1:11;
submatrixDBG=[READINGS.CABECERA.DBG_MSG(READINGS.CABECERA.Robot_ID==id_robot);0];

submatrixCH=zeros(no_readings,6);
for i=1:min(N,no_readings)
    submatrixCH(i,subtable_CH(i)+1)=1;
end
%Hora
[H,m,s]=hms(datetime("now"));

%if READINGS.CABECERA.Robot_ID(1)==id_robot & READINGS.NAVIGATION.CurrentState(1)==state
if READINGS.CABECERA.Robot_ID(1)==id_robot
subplot(5,nCol,1+(id_robot-1).*nCol)
bar(bands_number,READINGS.BOBINA{1,"PSD_Bands"},0.8,'FaceColor',rgb('LightSkyBlue'),'EdgeColor','none');
hold on
plot(bands_number,READINGS.COILSIGMAS{1,"Thld_B"},'_r','MarkerSize',7);
plot(bands_number,READINGS.COILSIGMAS{1,"SM_B"},'_','MarkerSize',7,'Color',rgb('Navy'));
yline(READINGS.BOBINA.PSD_Baseline(1),'-','Baseline','LabelVerticalAlignment','bottom','LineWidth',2,'Color',rgb('Indigo'))
hold off
ylabel("\bf R"+id_robot)
%text(1:11,repmat(1E-5,11,1),num2str(READINGS.COILSIGMAS.N_B(1,:)),'horiz','center'); 
box off
grid
title(["PSD";"DBG      Act: "+submatrixDBG(1)+"  Pst: "+submatrixDBG(2)])
set(gca, 'YScale', 'log','XGrid','off')
set(gca, 'XTickLabel', ["1" "2" "3" "4" "5" "I" "II" "III" "IV" "V" "RST"])
ylim([1E-5 10])
yticks = get(gca,'YTick');
set(gca,'YTickLabel',yticks);

subplot(5,nCol,2+(id_robot-1).*nCol)
h1=heatmap(subtable_bands.BandsDetected(1:no_readings,:),'CellLabelColor','none');
caxis([0 1])
h1.Colormap=copper;
h1.ColorbarVisible = 'off';
h1.YDisplayLabels=nan(size(h1.YDisplayData));
h1.XDisplayLabels=["1" "2" "3" "4" "5" "I" "II" "III" "IV" "V" "RST"];
title(["Últimas recibidas";"Clust. Sz.: "+READINGS.NAVIGATION.ACS(1)+"     N. C.: "+READINGS.NAVIGATION.NC_counter(1);"BCF Count: "+READINGS.NAVIGATION.BCF_counter(1)])

subplot(5,nCol,3+(id_robot-1).*nCol)
h2=heatmap(subtable_bands.BandsEmitted(1:no_readings,:),'CellLabelColor','none');
caxis([0 1])
h2.Colormap=bone;
h2.ColorbarVisible = 'off';
h2.YDisplayLabels=nan(size(h2.YDisplayData));
h2.XDisplayLabels=["1" "2" "3" "4" "5" "I" "II" "III" "IV" "V" "RST"];
title(["Últimas emitidas";"Curr. State: "+READINGS.DICTIONARY_STATES.Meaning(READINGS.NAVIGATION.CurrentState(1)==READINGS.DICTIONARY_STATES.Number)])

subplot(5,nCol,4+(id_robot-1).*nCol)
h3=heatmap(submatrixCH,'CellLabelColor','none');
caxis([0 1])
h3.Colormap=pink;
h3.ColorbarVisible = 'off';
h3.YDisplayLabels=nan(size(h3.YDisplayData));
h3.XDisplayLabels=["N/A" "1" "2" "3" "4" "5"];
title(["RF TX Ch.";"Update: "+H+":"+m+":"+s])


end
    
end

