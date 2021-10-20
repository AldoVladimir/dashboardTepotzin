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
    
%     for j=3:14
%         READINGS.COILSIGMAS{i,j-2}=typecast(uint8(dataMatrix(i,(69:72)+4.*(j-3))),'single');     
%     end
%     %SM
%     for j=3:14
%         READINGS.COILSIGMAS{i,j+9}=typecast(uint8(dataMatrix(i,(113:116)+4.*(j-3))),'single');     
%     end 
%     
%     %N
%     for j=1:11
%         READINGS.COILSIGMAS{i,j+22}=uint8(dataMatrix(i,j+156)); 
%     end
    
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
    
%     %Step vector
%     stepVector_1Son=(1:N_1Son)';
%     stepVector_2Son=(1:N_2Son)';
    

    %READINGS.THRESHOLD{i,"StepSelect"}=uint8(dataMatrix(i,229)); 
    
     RSSI_start=200; % El inicial -1
%     %RSSI primer sondeo
%     READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]}=-double(reshape(uint8(dataMatrix(i,(1:(N_1Son*4))+Son1_start)),4,N_1Son)');
%     READINGS.RSSI_1Son{stepVector_1Son,"DBG_MSG"}=repmat(READINGS.CABECERA.DBG_MSG(i),N_1Son,1);
%     READINGS.RSSI_1Son{stepVector_1Son,"Robot_Id"}=repmat(READINGS.CABECERA.Robot_ID(i),N_1Son,1);
%     READINGS.RSSI_1Son{stepVector_1Son,"Step"}=stepVector_1Son;
%     READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]}(READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]}==0)=NaN;
    
    %RSSI 
    READINGS.RSSI{i,"RSSI_1"}=-uint8(dataMatrix(i,(1:N_1Son)+RSSI_start));
    READINGS.RSSI{i,"RSSI_2"}=-uint8(dataMatrix(i,(1:N_1Son)+RSSI_start+N_1Son)); 
    READINGS.RSSI{i,"Max"}=typecast(uint8(dataMatrix(i,185:196)),'single');
    READINGS.RSSI{i,"Thld"}=typecast(uint8(dataMatrix(i,197:200)),'single');
    
    READINGS.RSSI{i,"RSSI_1"}(READINGS.RSSI{i,"RSSI_1"}==0)=NaN;
    READINGS.RSSI{i,"RSSI_2"}(READINGS.RSSI{i,"RSSI_2"}==0)=NaN;
    
%     %THRESHOLD
%     READINGS.THRESHOLD{i,"Max1"}=typecast(uint8(dataMatrix(i,185:188)),'single'); 
%     READINGS.THRESHOLD{i,"Max2"}=typecast(uint8(dataMatrix(i,189:192)),'single'); 
%     READINGS.THRESHOLD{i,"Max3"}=typecast(uint8(dataMatrix(i,193:196)),'single'); 
%     READINGS.THRESHOLD{i,"Thld"}=typecast(uint8(dataMatrix(i,197:200)),'single'); 
% 
%     %Estadistica de RSSI primer sondeo
%     READINGS.STAT_RSSI.Son1_median(i,:)=median(READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]},'omitnan');    
%     READINGS.STAT_RSSI.Son1_1Q(i,:)=quantile(READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]},0.25);
%     READINGS.STAT_RSSI.Son1_3Q(i,:)=quantile(READINGS.RSSI_1Son{stepVector_1Son,["CH1" "CH2" "CH3" "CH4"]},0.75);
% 
%     %Estadistica de RSSI segundo sondeo
%     READINGS.STAT_RSSI.Son2_median(i,:)=median(READINGS.RSSI_2Son{stepVector_2Son,["CH1" "CH2" "CH3" "CH4"]},'omitnan');    
%     READINGS.STAT_RSSI.Son2_1Q(i,:)=quantile(READINGS.RSSI_2Son{stepVector_2Son,["CH1" "CH2" "CH3" "CH4"]},0.75);
%     READINGS.STAT_RSSI.Son2_3Q(i,:)=quantile(READINGS.RSSI_2Son{stepVector_2Son,["CH1" "CH2" "CH3" "CH4"]},0.25);
end

