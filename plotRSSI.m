function p=plotRSSI(state,READINGS,id_robot)

%Plotea primer y segundo sondeo con maximos y umbral
if READINGS.NAVIGATION.CurrentState(1)==state & READINGS.CABECERA.Robot_ID(1)==id_robot
%if READINGS.CABECERA.Robot_ID(1)==id_robot

Son1_lin=10.^(READINGS.RSSI.RSSI_1(1,:)'./10);
Son2_lin=10.^(READINGS.RSSI.RSSI_2(1,:)'./10);
N_1Son=numel(Son1_lin);

%Cambiar 1 por cero
Son1_lin(Son1_lin>=1)=0;
Son2_lin(Son2_lin>=1)=0;

%figure
subplot(5,1,id_robot)
p=plot(1:N_1Son,Son1_lin,'.-');
hold on
plot(N_1Son-numel(Son2_lin)+1:N_1Son,flipud(Son2_lin),'.-')
yline(READINGS.RSSI.Thld(1),'k--','Threshold','Color',rgb('Crimson'))
yline(READINGS.RSSI.Max(1,1),'b--','Max 1','Color',rgb('DarkBlue'))
yline(READINGS.RSSI.Max(1,2),'b--','Max 2','Color',rgb('DodgerBlue'))
yline(READINGS.RSSI.Max(1,3),'b--','Max 3','Color',rgb('DeepSkyBlue'))
hold off
ylabel("RSSI")
%legend("Primer sondeo","Segundo sondeo",'Location','southoutside','Orientation',"horizontal")
xlabel("Step")
title(["Combinación lineal de RSSI";"Id: "+READINGS.CABECERA.Robot_ID(1)+" DBG MSG: "+READINGS.CABECERA.DBG_MSG(1)+" State: "+READINGS.DICTIONARY_STATES.Meaning(READINGS.NAVIGATION.CurrentState(1)==READINGS.DICTIONARY_STATES.Number)])
xlim([0.5 N_1Son+0.5])
grid on
set(gca,'xtick',0:N_1Son)

end
    
end

