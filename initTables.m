function [READINGS] = initTables(ind,N_Son1,N_Son2)

%Tabla de cabecera
CABECERA=array2table(zeros(ind,3),'VariableNames',["Header" "Robot_ID" "DBG_MSG"]);
CABECERA.Header=string(CABECERA.Header);

%Tabla de motores
MOTORS=array2table(zeros(ind,4),'VariableNames',["Imot" "Isys" "MotorStatus" "IsysStatus"]);

%Tabla de bobina
BOBINA=table(zeros(ind,1),zeros(ind,1),zeros(ind,1),zeros(ind,11),zeros(ind,1),'VariableNames',["ClusterSize" "BandsDetected" "BandsEmitted" "PSD_Bands" "PSD_Baseline"]);
BOBINA.BandsDetected=cellstr(repmat('00000000000',ind,1));
BOBINA.BandsEmitted=cellstr(repmat('00000000000',ind,1));

%Tabla SigmaBobinas
COILSIGMAS=table(zeros(ind,11),zeros(ind,11),zeros(ind,11),'VariableNames',["Thld_B" "SM_B" "N_B"]);


%Tabla bandas dectadas y emitidas
BANDS_BOBINA=table(zeros(ind,11),zeros(ind,11),'VariableNames',["BandsDetected" "BandsEmitted"]);


%Tabla navigation
NAVIGATION=array2table(zeros(ind,9),'VariableNames', ...
    ["CurrentState" "NextState" "nav_Cycs" "ACS" "LCS" "RCSL_counter" ...
     "ResetCluster" "BCF_counter" "EscapeFromCluster"]);

%Tabla RSSI
RSSI=table(zeros(ind,N_Son1),zeros(ind,N_Son2),zeros(ind,3),zeros(ind,1),zeros(ind,1),'VariableNames',["RSSI_1" "RSSI_2" "Max" "Thld" "CH_Tx"]);

%Diccionario de estados
statesDictionary=table((0:4)',(["BOOT" "SEARCH ROT" "SEARCH ST" "ORIENT N' MOVE" "ESCAPE STRATEGY"])','VariableNames',["Number" "Meaning"]);

%Diccionario de interrupciones
commandsDictionary=table((uint8([0x76 0x89 zeros(1,44)]))',(uint8([0x76 0x8A zeros(1,44)]))','VariableNames',["Stop" "Resume"]);

READINGS=struct("COILSIGMAS",COILSIGMAS,"CABECERA",CABECERA,"MOTORS",MOTORS,"BOBINA",BOBINA,"BANDS_BOBINA",BANDS_BOBINA,"NAVIGATION",NAVIGATION,"RSSI",RSSI,"DICTIONARY_STATES",statesDictionary,"DICTIONARY_COMMANDS",commandsDictionary);


end

