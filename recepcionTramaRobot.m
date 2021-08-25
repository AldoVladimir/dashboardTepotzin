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
N_1Son=23; N_2Son=12; % N_Son1+N_Son2<=35!!

%Inicializacion de variables
dataMatrix = zeros(ind,254);
[READINGS] = initTables(ind,N_1Son,N_2Son);

%Limpia el buffer
flush(s)

%Cargar archivos dummy
%load RFData.mat

%Inicializar figuras
%Ploteos recientes
figure
recentReadings=plot(1);


stopheader=[0x76 0x89 zeros(1,44)];
%% Lectura
N=1;
i=0;
%while 1
while i<N
%Mover todos las lecturas un espacio
dataMatrix=circshift(dataMatrix,1);
READINGS=circShiftAll(READINGS,N_1Son,N_2Son);

%Lectura del serial y llenado de tablas
dataMatrix(1,:)=read(s,254,'uint8');
READINGS=fillTables(dataMatrix,READINGS,N_1Son,N_2Son);

%Ploteo de lecturas
recentReadings=plotRecentReading(1,READINGS,N_1Son,N_2Son);

if READINGS.NAVIGATION.CurrentState(1)==1
    i=i+1;
end
%write(s,READINGS.DICTIONARY_COMMANDS.Stop,"uint8");

%write(s,READINGS.DICTIONARY_COMMANDS.Resume,"uint8");
end
pause(0.75)

write(s,READINGS.DICTIONARY_COMMANDS.Stop,"uint8");


%% Resume
write(s,READINGS.DICTIONARY_COMMANDS.Resume,"uint8");
%% Guardado de lecturas
%writetable(T,"./logs/.xlsx",'Sheet',1)


%% Prueba de recibir-enviar
% clear ans
% flush(s)
% read(s,254,'uint8');
% pause(1)
% write(s,stopHeader,"uint8");