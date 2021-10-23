function [READINGS] = fillTables(dataMatrix,READINGS,N_1Son,N_2Son)
%Inicializa todas las tablas con un conjunto de ceros
%y los guarda en una estructura 


i=1;
    %Cabecera
    READINGS.CABECERA{i,"Header"}=string(char(uint8(dataMatrix(1,1:3))));       
    READINGS.CABECERA{i,"Robot_ID"}=uint8(dataMatrix(i,4));    
    READINGS.CABECERA{i,"DBG_MSG"}=typecast(uint8(dataMatrix(i,5:6)),'uint16');
        
    %Motores
    READINGS.MOTORS{i,"Imot"}=0;
    READINGS.MOTORS{i,"Isys"}=typecast(uint8(dataMatrix(i,9:12)),'single');
    READINGS.MOTORS{i,"MotorStatus"}=0;
    READINGS.MOTORS{i,"IsysStatus"}=0;


    %Tabla de bobina    
    READINGS.BOBINA{i,"ClusterSize"}=typecast(uint8(dataMatrix(i,13:14)),'uint16');
    READINGS.BOBINA{i,"PSD_Bands"}=typecast(uint8(dataMatrix(i,17:60)),'single');   
    READINGS.BOBINA{i,"PSD_Baseline"}=typecast(uint8(dataMatrix(i,61:64)),'single'); 
    
    temp_detected=flip(dec2bin(typecast(uint8(dataMatrix(i,15:16)),'uint16')));
    temp_emitted=flip(dec2bin(typecast(uint8(dataMatrix(i,65:66)),'uint16')));
    
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
    
    %Threshold bandas    
    READINGS.COILSIGMAS{i,"Thld_B"}=typecast(uint8(dataMatrix(i,69:112)),'single');     
    READINGS.COILSIGMAS{i,"SM_B"}=typecast(uint8(dataMatrix(i,113:156)),'single');
    READINGS.COILSIGMAS{i,"N_B"}=uint8(dataMatrix(i,157:167));  

    
    %Navigation
    READINGS.NAVIGATION{i,"CurrentState"}=typecast(uint8(dataMatrix(i,169:172)),'int32');
    READINGS.NAVIGATION{i,"NextState"}=typecast(uint8(dataMatrix(i,173:176)),'int32');
    READINGS.NAVIGATION{i,"nav_Cycs"}=typecast(uint8(dataMatrix(i,177:178)),'uint16');
    READINGS.NAVIGATION{i,"ACS"}=uint8(dataMatrix(i,179));
    READINGS.NAVIGATION{i,"LCS"}=uint8(dataMatrix(i,180));
    READINGS.NAVIGATION{i,"RCSL_counter"}=uint8(dataMatrix(i,181));
    READINGS.NAVIGATION{i,"ResetCluster"}=logical(dataMatrix(i,182));
    READINGS.NAVIGATION{i,"BCF_counter"}=uint8(dataMatrix(i,183));
    READINGS.NAVIGATION{i,"EscapeFromCluster"}=logical(dataMatrix(i,184));
    
    
    RSSI_start=200; % El inicial -1
    
    %RSSI 
    READINGS.RSSI{i,"RSSI_1"}=-dataMatrix(i,(1:N_1Son)+RSSI_start);
    READINGS.RSSI{i,"RSSI_2"}=-dataMatrix(i,(1:N_2Son)+RSSI_start+N_1Son); 
    READINGS.RSSI{i,"Max"}=typecast(uint8(dataMatrix(i,185:196)),'single');
    READINGS.RSSI{i,"Thld"}=typecast(uint8(dataMatrix(i,197:200)),'single');
    READINGS.RSSI{i,"CH_Tx"}=dataMatrix(i,225);
    
    READINGS.RSSI{i,"RSSI_1"}(READINGS.RSSI{i,"RSSI_1"}==0)=NaN;
    READINGS.RSSI{i,"RSSI_2"}(READINGS.RSSI{i,"RSSI_2"}==0)=NaN;
    

end

