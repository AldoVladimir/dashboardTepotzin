function [READINGS] = fillTables(dataMatrix,READINGS,N_1Son,N_2Son)
%Inicializa todas las tablas con un conjunto de ceros
%y los guarda en una estructura 


i=1;
    %Cabecera
    READINGS.CABECERA{i,"Header"}=string(char(uint8(dataMatrix(1,1:3))));       
    READINGS.CABECERA{i,"Robot_ID"}=uint8(dataMatrix(i,4));    
    READINGS.CABECERA{i,"DBG_MSG"}=typecast(uint8(dataMatrix(i,5:6)),'uint16');
        
    %Motores
    READINGS.MOTORS{i,"Imot"}=typecast(uint8(dataMatrix(i,9:12)),'single');
    READINGS.MOTORS{i,"Isys"}=typecast(uint8(dataMatrix(i,13:16)),'single');
    READINGS.MOTORS{i,"MotorStatus"}=typecast(uint8(dataMatrix(i,17:18)),'uint16');
    READINGS.MOTORS{i,"IsysStatus"}=typecast(uint8(dataMatrix(i,19:20)),'uint16');


    %Tabla de bobina    
    READINGS.BOBINA{i,"ClusterSize"}=typecast(uint8(dataMatrix(i,21:22)),'uint16');
    for j=3:14
        READINGS.BOBINA{i,j}=typecast(uint8(dataMatrix(i,(25:28)+4.*(j-3))),'single');     
    end
    
    
    temp_detected=flip(dec2bin(typecast(uint8(dataMatrix(i,23:24)),'uint16')));
    temp_emitted=flip(dec2bin(typecast(uint8(dataMatrix(i,73:74)),'uint16')));
    
    for j=1:min([numel(temp_detected) 11])
    READINGS.BOBINA{i,"BandsDetected"}{1}(j)=temp_detected(j);
    end
    
    for j=1:min([numel(temp_emitted) 11])
    READINGS.BOBINA{i,"BandsEmitted"}{1}(j)=temp_emitted(j);
    end
    
    %Bandas de bobina
    for j=1:11
    READINGS.BANDS_BOBINA{i,"BandsDetected"}(j)=str2double(READINGS.BOBINA{i,"BandsDetected"}{1}(j));
    READINGS.BANDS_BOBINA{i,"BandsEmitted"}(j)=str2double(READINGS.BOBINA{i,"BandsEmitted"}{1}(j));
    end
    
    %Navigaion
    READINGS.NAVIGATION{i,"CurrentState"}=typecast(uint8(dataMatrix(i,77:80)),'int32');
    READINGS.NAVIGATION{i,"NextState"}=typecast(uint8(dataMatrix(i,81:84)),'int32');
    READINGS.NAVIGATION{i,"nav_Cycs"}=typecast(uint8(dataMatrix(i,85:86)),'uint16');
    READINGS.NAVIGATION{i,"ACS"}=uint8(dataMatrix(i,87));
    READINGS.NAVIGATION{i,"LCS"}=uint8(dataMatrix(i,88));
    READINGS.NAVIGATION{i,"RCSL_counter"}=uint8(dataMatrix(i,89));
    READINGS.NAVIGATION{i,"ResetCluster"}=logical(dataMatrix(i,90));
    READINGS.NAVIGATION{i,"BCF_counter"}=uint8(dataMatrix(i,91));
    READINGS.NAVIGATION{i,"EscapeFromCluster"}=logical(dataMatrix(i,91));
    
    %Step vector
    stepVector_1Son=(1:N_1Son)';
    stepVector_2Son=(1:N_2Son)';
    
    %THRESHOLD
    READINGS.THRESHOLD{i,"Max1"}=typecast(uint8(dataMatrix(i,93:96)),'single'); 
    READINGS.THRESHOLD{i,"Max2"}=typecast(uint8(dataMatrix(i,97:100)),'single'); 
    READINGS.THRESHOLD{i,"Max3"}=typecast(uint8(dataMatrix(i,101:104)),'single'); 
    READINGS.THRESHOLD{i,"Thld"}=typecast(uint8(dataMatrix(i,105:108)),'single'); 
    %READINGS.THRESHOLD{i,"StepSelect"}=uint8(dataMatrix(i,229)); 
    
    Son1_start=108; % El inicial -1
    %RSSI primer sondeo
    READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]}=-double(reshape(uint8(dataMatrix(i,(1:(N_1Son*4))+Son1_start)),4,N_1Son)');
    READINGS.RSSI_1Son{stepVector_1Son,"DBG_MSG"}=repmat(READINGS.CABECERA.DBG_MSG(i),N_1Son,1);
    READINGS.RSSI_1Son{stepVector_1Son,"Robot_Id"}=repmat(READINGS.CABECERA.Robot_ID(i),N_1Son,1);
    READINGS.RSSI_1Son{stepVector_1Son,"Step"}=stepVector_1Son;
    READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]}(READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]}==0)=NaN;
    
    
    %RSSI segundo sondeo
    READINGS.RSSI_2Son{stepVector_2Son,["CH1" "CH2" "CH3" "CH4"]}=-double(reshape(uint8(dataMatrix(i,Son1_start+N_1Son*4+1:Son1_start+N_1Son*4+N_2Son*4)),4,N_2Son)');
    READINGS.RSSI_2Son{stepVector_2Son,"DBG_MSG"}=repmat(READINGS.CABECERA.DBG_MSG(i),N_2Son,1);
    READINGS.RSSI_2Son{stepVector_2Son,"Robot_Id"}=repmat(READINGS.CABECERA.Robot_ID(i),N_2Son,1);
    READINGS.RSSI_2Son{stepVector_2Son,"Step"}=stepVector_2Son;
    READINGS.RSSI_2Son{stepVector_2Son,["CH1" "CH2" "CH3" "CH4"]}(READINGS.RSSI_2Son{stepVector_2Son,["CH1" "CH2" "CH3" "CH4"]}==0)=NaN;

    %Estadistica de RSSI primer sondeo
    READINGS.STAT_RSSI.Son1_median(i,:)=median(READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]},'omitnan');    
    READINGS.STAT_RSSI.Son1_1Q(i,:)=quantile(READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]},0.25);
    READINGS.STAT_RSSI.Son1_3Q(i,:)=quantile(READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]},0.75);

    %Estadistica de RSSI segundo sondeo
    READINGS.STAT_RSSI.Son2_median(i,:)=median(READINGS.RSSI_2Son{stepVector_2Son,["CH1" "CH2" "CH3" "CH4"]},'omitnan');    
    READINGS.STAT_RSSI.Son2_1Q(i,:)=quantile(READINGS.RSSI_2Son{stepVector_2Son,["CH1" "CH2" "CH3" "CH4"]},0.75);
    READINGS.STAT_RSSI.Son2_3Q(i,:)=quantile(READINGS.RSSI_2Son{stepVector_2Son,["CH1" "CH2" "CH3" "CH4"]},0.25);
end

