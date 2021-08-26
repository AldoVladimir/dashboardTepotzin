function p=plotRecentReading(state,READINGS,N_1Son,N_2Son)

%Plotea primer y segundo sondeo con maximos y umbral
if READINGS.NAVIGATION.CurrentState(1)==state
idx_readings_1=1:N_1Son;
idx_readings_2=1:N_2Son;


Son1_lin=sum(10.^(READINGS.RSSI_1Son{idx_readings_1,["CH1" "CH2" "CH3" "CH4"]}./10),2);
Son2_lin=sum(10.^(READINGS.RSSI_2Son{idx_readings_2,["CH1" "CH2" "CH3" "CH4"]}./10),2);
%Cambiar 1 por cero
Son1_lin(Son1_lin>=1)=0;
Son2_lin(Son2_lin>=1)=0;

%figure
p=plot(Son1_lin,'.-');
hold on
plot(N_1Son-numel(Son2_lin)+1:N_1Son,flipud(Son2_lin),'.-')
yline(READINGS.THRESHOLD.Thld(1),'k--','Threshold')
yline(READINGS.THRESHOLD.Max1(1),'b--','Max 1')
yline(READINGS.THRESHOLD.Max2(1),'b--','Max 2')
yline(READINGS.THRESHOLD.Max3(1),'b--','Max 3')
hold off
ylabel("RSSI")
legend("Primer sondeo","Segundo sondeo",'Location','southoutside','Orientation',"horizontal")
xlabel("Step")
title(["Combinación lineal de RSSI";"Id: "+READINGS.CABECERA.Robot_ID(1)+" DBG MSG: "+READINGS.CABECERA.DBG_MSG(1)+" State: "+READINGS.DICTIONARY_STATES.Meaning(READINGS.NAVIGATION.CurrentState(1)==READINGS.DICTIONARY_STATES.Number)])
xlim([0.5 N_1Son+0.5])
grid on
set(gca,'xtick',0:N_1Son)
else
p=[];
end

    
end

