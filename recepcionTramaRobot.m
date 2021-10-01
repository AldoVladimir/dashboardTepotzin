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
baselineGain=100;

%Limpia el buffer
flush(s)

%Cargar archivos dummy
%load RFData.mat

%Inicializar figuras
%Ploteos recientes
figure
subplot(2,3,1)
RSSI_1=plot(1);

subplot(2,3,2)
RSSI_2=plot(1);

subplot(2,3,3)
RSSI_3=plot(1);

subplot(2,3,4)
RSSI_4=plot(1);

subplot(2,3,5)
RSSI_5=plot(1);


figure
subplot(2,3,1)
COIL_1=plot(1);

subplot(2,3,2)
COIL_2=plot(1);

subplot(2,3,3)
COIL_3=plot(1);

subplot(2,3,4)
COIL_4=plot(1);

subplot(2,3,5)
COIL_5=plot(1);

%% Lectura
N=1;
i=0;
%while 1
%while i<N
%Mover todos las lecturas un espacio
dataMatrix=circshift(dataMatrix,1);
READINGS=circShiftAll(READINGS,N_1Son,N_2Son);

%Lectura del serial y llenado de tablas
dataMatrix(1,:)=read(s,254,'uint8');
READINGS=fillTables(dataMatrix,READINGS,N_1Son,N_2Son);

%Ploteo de lecturas
subplot(2,3,1)
RSSI_1=plotRSSI_db(1,READINGS,N_1Son,N_2Son,1);
subplot(2,3,2)
RSSI_2=plotRSSI_db(1,READINGS,N_1Son,N_2Son,2);
subplot(2,3,3)
RSSI_3=plotRSSI_db(1,READINGS,N_1Son,N_2Son,3);
subplot(2,3,4)
RSSI_4=plotRSSI_db(1,READINGS,N_1Son,N_2Son,4);
subplot(2,3,5)
RSSI_5=plotRSSI_db(1,READINGS,N_1Son,N_2Son,5);


subplot(2,3,1)
COIL_1=bar(READINGS.BOBINA{1,3:13})
hold on
yline(READINGS.BOBINA.PSD_Base(1),'--','Bl');
yline(READINGS.BOBINA.PSD_Base(1).*baselineGain,'--','Th');
hold off
xlabel("Banda")
ylabel("PSD (V^2/Hz)")
grid
title("R"+READINGS.CABECERA.Robot_ID(1))

% subplot(2,3,2)
% COIL_2=plotCOIL(1,READINGS,N_1Son,N_2Son,2);
% subplot(2,3,3)
% COIL_3=plotCOIL(1,READINGS,N_1Son,N_2Son,3);
% subplot(2,3,4)
% COIL_4=plotCOIL(1,READINGS,N_1Son,N_2Son,4);
% subplot(2,3,5)
% COIL_5=plotCOIL(1,READINGS,N_1Son,N_2Son,5);

if READINGS.NAVIGATION.CurrentState(1)==1
    i=i+1;d
    pause(0.75)    
    %write(s,READINGS.DICTIONARY_COMMANDS.Stop,"uint8"); 
end
%write(s,READINGS.DICTIONARY_COMMANDS.Stop,"uint8");

%write(s,READINGS.DICTIONARY_COMMANDS.Resume,"uint8");
%end
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