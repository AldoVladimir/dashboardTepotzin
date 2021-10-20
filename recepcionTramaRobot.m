%% Objeto Serial
baudRate=115200;
%serialportlist
%UsarACM0 para recibir datos de la Launchpad
%ACM1 no envía datos. Se usa para debug propio.
s=serialport("/dev/ttyACM0",baudRate,'Timeout',60)
%s=serialport("/dev/ttyUSB0",baudRate,'Timeout',30);
configureTerminator(s,"CR/LF")

%% Inicializacion de variables
%Número de muestras a recordar
ind=500;

%Número de muestras por sondeo
N_1Son=12; N_2Son=12; % N_Son1+N_Son2<=35!!

%Inicializacion de variables
dataMatrix = zeros(ind,254);
[READINGS] = initTables(ind,N_1Son,N_2Son);
baselineGain=20;

%Limpia el buffer
flush(s)
%% Inicializar figura
%Cargar archivos dummy
%load RFData.mat

% %Inicializar figuras
% RSSI_figure=figure('Name','RSSI','NumberTitle','off');
% subplot(2,3,1)
% RSSI_1=plot(1);
% 
% subplot(2,3,2)
% RSSI_2=plot(1);
% 
% subplot(2,3,3)
% RSSI_3=plot(1);
% 
% subplot(2,3,4)
% RSSI_4=plot(1);
% 
% subplot(2,3,5)
% RSSI_5=plot(1);


COIL_figure=figure('Name','Bobinas','NumberTitle','off');
% subplot(5,3,1)
% COIL_1=plot(1);
% 
% subplot(5,3,2)
% COIL_2=plot(1);
% 
% subplot(5,3,3)
% COIL_3=plot(1);
% 
% subplot(5,3,4)
% COIL_4=plot(1);
% 
% subplot(5,3,5)
% COIL_5=plot(1);

%% Lectura
N=1;
i=0;
while 1
%while i<N
%Mover todos las lecturas un espacio
dataMatrix=circshift(dataMatrix,1);
READINGS=circShiftAll(READINGS);

%Lectura del serial y llenado de tablas
dataMatrix(1,:)=read(s,254,'uint8');
READINGS=fillTables(dataMatrix,READINGS,N_1Son,N_2Son);

% % %Ploteo de lecturas
% figure(RSSI_figure)
% subplot(2,3,1)
% RSSI_1=plotRSSI_db(1,READINGS,N_1Son,N_2Son,1);
% subplot(2,3,2)
% RSSI_2=plotRSSI_db(1,READINGS,N_1Son,N_2Son,2);
% subplot(2,3,3)
% RSSI_3=plotRSSI_db(1,READINGS,N_1Son,N_2Son,3);
% subplot(2,3,4)
% RSSI_4=plotRSSI_db(1,READINGS,N_1Son,N_2Son,4);
% subplot(2,3,5)
% RSSI_5=plotRSSI_db(1,READINGS,N_1Son,N_2Son,5);

figure(COIL_figure)
plotCOIL(2,READINGS,1,30,baselineGain)
plotCOIL(2,READINGS,2,30,baselineGain)
plotCOIL(2,READINGS,3,30,baselineGain)
plotCOIL(2,READINGS,4,30,baselineGain)
plotCOIL(2,READINGS,5,30,baselineGain)

% if READINGS.NAVIGATION.CurrentState(1)==1
%     i=i+1;d
%     pause(0.75)    
%     %write(s,READINGS.DICTIONARY_COMMANDS.Stop,"uint8"); 
% end
%write(s,READINGS.DICTIONARY_COMMANDS.Stop,"uint8");

%write(s,READINGS.DICTIONARY_COMMANDS.Resume,"uint8");
end
pause(0.75)

%write(s,READINGS.DICTIONARY_COMMANDS.Stop,"uint8");

%% Resume
write(s,READINGS.DICTIONARY_COMMANDS.Resume,"uint8")
%% Guardado de lecturas
%writetable(T,"./logs/.xlsx",'Sheet',1)


%% Prueba de recibir-enviar
% clear ans
% flush(s)
% read(s,254,'uint8');
% pause(1)
% write(s,stopHeader,"uint8");