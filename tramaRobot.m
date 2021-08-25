%% Recepcion trama robot

%% Objeto serial
baudRate=115200;
%serialportlist
%UsarACM0 para recibir datos de la Launchpad
%ACM1 no envía datos. Se usa para debug propio.
s=serialport("/dev/ttyACM0",baudRate,'Timeout',30)
configureTerminator(s,"CR/LF")

%% Tablas de variables y datos parciales

%D=size(dataMatrix);
%Lecturas parciales
D=20;

%Matriz de datos parciales
dataMatrix=zeros(20,254);

%Tabla de cabecera
Cabecera=array2table(zeros(D(1),3));
Cabecera_names=["Header" "Robot_ID" "DBG_MSG"];
Cabecera.Properties.VariableNames=Cabecera_names;
Cabecera.Header=string(Cabecera.Header);

%Tabla de navegación por RF
RF_NAV=array2table(zeros(D(1),12));
RF_NAV_names=["Mx_CH1" "My_CH1" "Mx_CH2" "My_CH2" "Mx_CH3" "My_CH3" "Mx_CH4" "My_CH4" "Mx_CH5" "My_CH5" "Mx_Res" "My_Res"];
RF_NAV.Properties.VariableNames=RF_NAV_names;

%Tabla de motores
MOTORS=array2table(zeros(D(1),4));
MOTORS_names=["Imot" "Isys" "MotorStatus" "IsysStatus"];
MOTORS.Properties.VariableNames=MOTORS_names;

%Tabla de bobina
BOBINA=array2table(zeros(D(1),15));
BOBINA_names=["ClusterSize" "BandsDetected" "PSD_B1" "PSD_B2" "PSD_B3" "PSD_B4" "PSD_B5" ...
    "PSD_B6" "PSD_B7" "PSD_B8" "PSD_B9" "PSD_B10" "PSD_B11" "PSD_Base" "BandsEmitted"];
BOBINA.Properties.VariableNames=BOBINA_names;

%Tabla navigation
NAVIGATION=array2table(zeros(D(1),9));
NAVIGATION_names=["CurrentState" "NextState" "nav_Cycs" "ACS" "LCS" "ClusterChanged" ...
     "BetterClusterFound" "MovStatus" "EscStrategy"];
NAVIGATION.Properties.VariableNames=NAVIGATION_names;

%Tabla RSSI
RSSI=array2table(zeros(12*D(1),7));
RSSI_names=["CH1" "CH2" "CH3" "CH4" "CH5" "Reading" "Angle"];
RSSI.Properties.VariableNames=RSSI_names;


%Preparar subplot
%Tabla de datos
%READING_DATA=table(Cabecera,RF_NAV,MOTORS,BOBINA,NAVIGATION,RSSI);
%Por cada lectura

%Estadistica RSSI
RSSI_statistics_data=zeros(12,5);
RSSI_statistics_median=zeros(D,5);
RSSI_statistics_3Q=zeros(D,5);
RSSI_statistics_1Q=zeros(D,5);

%% Lectura

flush(s)

%while 1
dataRecieved=read(s,254,'uint8');

%Pasar datos viejos y actualizar lecturas
dataMatrix(1:end-1,:)=dataMatrix(2:end,:);
dataMatrix(end,:)=dataRecieved;


%Actualizar los datos
Cabecera{1:end-1,:}=Cabecera{2:end,:};
RF_NAVI{1:end-1,:}=RF_NAVI{2:end,:};
MOTORS{1:end-1,:}=MOTORS{2:end,:};
BOBINA{1:end-1,:}=BOBINA{2:end,:};
NAVIGATION{1:end-1,:}=NAVIGATION{2:end,:};
RSSI{1:end-1,:}=RSSI{2:end,:};

%Actualizar la primera lectura de las tablas
    
    %Cabecera
    Cabecera{end,"Header"}=string(char(uint8(dataMatrix(1,1:3))));       
    Cabecera{end,"Robot_ID"}=uint8(dataMatrix(end,4));    
    Cabecera{end,"DBG_MSG"}=typecast(uint8(dataMatrix(end,5:6)),'uint16');
                
    %RF_NAV
    for j=1:12    
        RF_NAV{end,j}=typecast(uint8(dataMatrix(end,(9:12)+4.*(j-1))),'single');
    end
    
    %Motores
    for j=1:4 
        if j<=2
        MOTORS{end,j}=typecast(uint8(dataMatrix(end,(57:60)+4.*(j-1))),'single');     
        else
        MOTORS{end,j}=typecast(uint8(dataMatrix(end,(65:66)+2.*(j-3))),'uint16');    
        end
    end
    
    %Tabla de bobina    
    BOBINA{end,"ClusterSize"}=typecast(uint8(dataMatrix(end,69:70)),'uint16');
    BOBINA{end,"BandsDetected"}=typecast(uint8(dataMatrix(end,71:72)),'uint16');
    for j=3:14
        BOBINA{end,j}=typecast(uint8(dataMatrix(end,(73:76)+4.*(j-3))),'single');     
    end
    BOBINA{end,"BandsEmitted"}=typecast(uint8(dataMatrix(end,121:122)),'uint16');
    
    %Navigaion
    NAVIGATION{end,"CurrentState"}=typecast(uint8(dataMatrix(end,125:128)),'int32');
    NAVIGATION{end,"NextState"}=typecast(uint8(dataMatrix(end,129:132)),'int32');
    NAVIGATION{end,"nav_Cycs"}=typecast(uint8(dataMatrix(end,133:134)),'uint16');
    NAVIGATION{end,"ACS"}=uint8(dataMatrix(end,135));
    NAVIGATION{end,"LCS"}=uint8(dataMatrix(end,136));
    NAVIGATION{end,"ClusterChanged"}=logical(dataMatrix(end,137));
    NAVIGATION{end,"BetterClusterFound"}=logical(dataMatrix(end,138));
    NAVIGATION{end,"MovStatus"}=uint8(dataMatrix(end,139));
    NAVIGATION{end,"ACS"}=uint8(dataMatrix(end,140));
    
    %Tabla RSSI
    RSSI{(1:12)+(end-1)*12,1:end-2}=reshape(uint8(dataMatrix(end,141:200)),5,12)';
    RSSI{(1:12)+(end-1)*12,"Reading"}=repmat(typecast(uint8(dataMatrix(end,5:6)),'uint16'),12,1);
    RSSI{(1:12)+(end-1)*12,"Angle"}=(1:12)';
    
    %Estadistica de RSSI
    for j=1:20
    RSSI_statistics_data=RSSI{(1:12)+12*(j-1),["CH1" "CH2" "CH3" "CH4" "CH5"]};
    RSSI_statistics_median(j,:)=median(RSSI_statistics_data);    
    RSSI_statistics_3Q(j,:)=quantile(RSSI_statistics_data,0.75);
    RSSI_statistics_1Q(j,:)=quantile(RSSI_statistics_data,0.25);
    end
    %% Ploteo
    
    
    %Lectura actual
    figure
    subplot(2,1,1)   
    %figure
    plot(RSSI.Angle(end-11:end),-RSSI{end-11:end,1:5},'-o')
    legend(["CH1" "CH2" "CH3" "CH4" "CH5"])
    ylabel("RSSI [dB]")
    xlabel("Angulo")
    ylim([-130 -10])
    title("RSSI por canal actual")
    
    %Lecturas pasadas
    subplot(2,1,2)
   %figure
    plot(-RSSI_statistics_median)
    hold on
    plot(-RSSI_statistics_3Q,'--')
    plot(-RSSI_statistics_1Q,'--')
    hold off
    ylabel("RSSI [dB]")
    title("RSSI de muestras pasadas")
    ylim([-130 -10])
 %end