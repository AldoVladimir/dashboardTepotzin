function p=plotRSSI(state,READINGS,N_1Son,N_2Son,id_robot)

%Plotea primer y segundo sondeo con maximos y umbral
if READINGS.NAVIGATION.CurrentState(1)==state & READINGS.CABECERA.Robot_ID(1)==id_robot
idx_readings_1=1:N_1Son;
idx_readings_2=1:N_2Son;


Son1_lin=sum(10.^(READINGS.RSSI_1Son{idx_readings_1,["CH1" "CH2" "CH3" "CH4"]}./10),2);
Son2_lin=sum(10.^(READINGS.RSSI_2Son{idx_readings_2,["CH1" "CH2" "CH3" "CH4"]}./10),2);
%Cambiar 1 por cero
Son1_lin(Son1_lin>=1)=0;
Son2_lin(Son2_lin>=1)=0;

%figure
p=plot(10*log10(Son1_lin),'.-');
hold on
plot(N_1Son-numel(Son2_lin)+1:N_1Son,10*log10(flipud(Son2_lin)),'.-')
yline(10*log10(READINGS.THRESHOLD.Thld(1)),'k--','Threshold')
yline(10*log10(READINGS.THRESHOLD.Max1(1)),'b--','Max 1')
yline(10*log10(READINGS.THRESHOLD.Max2(1)),'b--','Max 2')
yline(10*log10(READINGS.THRESHOLD.Max3(1)),'b--','Max 3')
hold off
ylabel("RSSI [db]")
legend("Primer sondeo","Segundo sondeo",'Location','southoutside','Orientation',"horizontal")
xlabel("Step")
title(["Combinación lineal de RSSI";"Id: "+READINGS.CABECERA.Robot_ID(1)+" DBG MSG: "+READINGS.CABECERA.DBG_MSG(1)+" State: "+READINGS.DICTIONARY_STATES.Meaning(READINGS.NAVIGATION.CurrentState(1)==READINGS.DICTIONARY_STATES.Number)])
xlim([0.5 N_1Son+0.5])
%ylim([-70 -20])
grid on
set(gca,'xtick',0:N_1Son)
else
p=[];
end

    
end

