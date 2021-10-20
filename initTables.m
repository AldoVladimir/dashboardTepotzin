function [READINGS] = initTables(ind,N_Son1,N_Son2)

%Tabla de cabecera
CABECERA=array2table(zeros(ind,3),'VariableNames',["Header" "Robot_ID" "DBG_MSG"]);
CABECERA.Header=string(CABECERA.Header);

%Tabla de motores
MOTORS=array2table(zeros(ind,4),'VariableNames',["Imot" "Isys" "MotorStatus" "IsysStatus"]);

%Tabla de bobina
BOBINA=array2table(zeros(ind,15),'VariableNames',["ClusterSize" "BandsDetected" "PSD_B1" "PSD_B2" "PSD_B3" "PSD_B4" "PSD_B5" ...
    "PSD_B6" "PSD_B7" "PSD_B8" "PSD_B9" "PSD_B10" "PSD_B11" "PSD_Base" "BandsEmitted"]);
BOBINA.BandsDetected=cellstr(repmat('00000000000',ind,1));
BOBINA.BandsEmitted=cellstr(repmat('00000000000',ind,1));

%Tabla thresholds
THRESHOLDS=array2table(zeros(ind,11),'VariableNames',["Thld_B"+(1:11)]);

%Tabla Sigma/Max
SM=array2table(zeros(ind,11),'VariableNames',["SM_B"+(1:11)]);

%Tabla N Sigma
NSIGMA=array2table(zeros(ind,11),'VariableNames',["N_"+(1:11)]);


%Tabla bandas dectadas y emitidas
BANDS_BOBINA=table(zeros(ind,11),zeros(ind,11),'VariableNames',["BandsDetected" "BandsEmitted"]);


%Tabla navigation
NAVIGATION=array2table(zeros(ind,9),'VariableNames', ...
    ["CurrentState" "NextState" "nav_Cycs" "ACS" "LCS" "RCSL_counter" ...
     "ResetCluster" "BCF_counter" "EscapeFromCluster"]);

%Tabla RSSI primer sondeo
RSSI_1Son=array2table(zeros(ind,N_Son1+3),'VariableNames',["RSSI_"+(1:N_Son1) "DBG_MSG" "Robot_Id" "Step"]);

%Tabla RSSI segundo sondeo
RSSI_2Son=array2table(zeros(ind,N_Son2+3),'VariableNames',["RSSI_"+(1:N_Son2) "DBG_MSG" "Robot_Id" "Step"]);

%Tabla THRESHOLD segundo sondeo
THRESHOLD=array2table(zeros(ind,4),'VariableNames',["Max1" "Max2" "Max3" "Thld"]);

%Statistics
dummyVariable=zeros(ind,4);
STAT_RSSI=table(dummyVariable,dummyVariable,dummyVariable,dummyVariable,dummyVariable,dummyVariable, ...
    'VariableNames',["Son1_median" "Son1_1Q" "Son1_3Q" "Son2_median" "Son2_1Q" "Son2_3Q"]);

%Diccionario de estados
statesDictionary=table((0:4)',(["BOOT" "SEARCH ROT" "SEARCH ST" "ORIENT N' MOVE" "ESCAPE STRATEGY"])','VariableNames',["Number" "Meaning"]);

%Diccionario de interrupciones
commandsDictionary=table((uint8([0x76 0x89 zeros(1,44)]))',(uint8([0x76 0x8A zeros(1,44)]))','VariableNames',["Stop" "Resume"]);

READINGS=struct("NSIGMA",NSIGMA,"SM",SM,"THRESHOLDS",THRESHOLDS,"CABECERA",CABECERA,"MOTORS",MOTORS,"BOBINA",BOBINA,"BANDS_BOBINA",BANDS_BOBINA,"NAVIGATION",NAVIGATION,"RSSI_1Son",RSSI_1Son,"RSSI_2Son",RSSI_2Son,"THRESHOLD",THRESHOLD,"STAT_RSSI",STAT_RSSI,"DICTIONARY_STATES",statesDictionary,"DICTIONARY_COMMANDS",commandsDictionary);


end

