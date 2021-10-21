%% Objeto Serial
baudRate=115200;
%serialportlist
%UsarACM0 para recibir datos de la Launchpad
%ACM1 no envía datos. Se usa para debug propio.
s=serialport("/dev/ttyACM0",baudRate,'Timeout',60)
%s=serialport("/dev/ttyUSB0",baudRate,'Timeout',60);
%s_TTDM=serialport("/dev/ttyUSB0",baudRate,'Timeout',60);
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

RSSI_figure=figure('Name','RSSI','NumberTitle','off');
COIL_figure=figure('Name','Bobinas','NumberTitle','off');
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

%Ploteo de lecturas
figure(RSSI_figure)
plotRSSI(1,READINGS,1);
plotRSSI(1,READINGS,2);
plotRSSI(1,READINGS,3);
plotRSSI(1,READINGS,4);
plotRSSI(1,READINGS,5);

figure(COIL_figure)
plotCOIL(2,READINGS,1,30)
plotCOIL(2,READINGS,2,30)
plotCOIL(2,READINGS,3,30)
plotCOIL(2,READINGS,4,30)
plotCOIL(2,READINGS,5,30)

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

%% Debug TTDM
deb=read(s,254,'uint8');
%TTDM=read(s_TTDM,254,'uint8')
%% Guardado de lecturas
%writetable(T,"./logs/.xlsx",'Sheet',1)


%% Prueba de recibir-enviar
% clear ans
% flush(s)
% read(s,254,'uint8');
% pause(1)
% write(s,stopHeader,"uint8");