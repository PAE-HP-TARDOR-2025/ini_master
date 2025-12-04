#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "CANopen.h"
#include "CO_driver_target.h"
#include "CO_LSSmaster.h"
#include "CO_NMT_Heartbeat.h"
#include "OD.h"
/*
#include <time.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
*/

/*app_main > ESP-IDF redefineix el punt d'entrada del programa
Embeded Systems amb freeRTOS cal inicialitzar el sistema i cridar app main
Framework > Controla l'arranque del sistema. */

/*Perquè funcioni LSSMaster he hagut de canviar una cosa al driver. Una definició. Revisar!!!*/

/* ---------------- Configuració Macros ------------------------- */
#define CAN_INTERFACE          "can0"
/*Linux -> Les interfícies CAN apareixen així
can0: indica la interfície física sobre la que reballarà el màster
    Generalment Rasberry Pi té interfície can0 activada
can1
vcan0 (CAN virtual)
*/
#define MASTER_NODE_ID         0x01        // Node ID del master
#define MASTER_BITRATE         250    // Bitrate CAN
#define FIRST_ASSIGNED_ID      0x10        // IDs començaran als 0x10 -> Això potser ho podem canviar
#define MAX_SLAVES_TO_CONFIGURE 10          //nombre màxim de nodes per configurar > Això ho he posat de cara als ACKs
#define ASSIGNED_BITRATE       250000      //Bit rate que assignem als esclaus > Vull assignar-ho des d'aquí. 
#define NMT_CONTROL (CO_NMT_PRE_OPERATIONAL | CO_NMT_STARTUP_TO_OPERATIONAL)
//El que hem fa dubtar més es quan li passo algo relatiu a NMT_CONTROL. Què li estic passant realment???
#define FIRST_HB_TIME          1000        //temps del primer heartbeat. 1000 -> ms


/*Defineixo el time_out. No sé si algú l'ha posat però hauriem de revisar les macros*/

/* -------------------- Globals ----------------------------------*/
static CO_t *CO = NULL;
static CO_config_t *config_ptr = 0; 
static uint32_t heapMemoryUsed = 0; 
volatile bool emergencyDetected = false;


static const char *TAG = "master";
//CO_LSSmaster_return_t discoveredSlaves[MAX_SLAVES_TO_CONFIGURE]; 
CO_LSS_address_t adresses[MAX_SLAVES_TO_CONFIGURE];
CO_LSSmaster_fastscan_t fastScan;


/* --------------------- Assignar NodeID --------------------------*/
uint32_t calcularNodeID(CO_LSS_address_t identity) {
    //Aquí he de mirar com assignar un nodeID. 
    /*Copilot em diu que ho faci amb el serialNumber
    Recordar que la CO_LSS_address_t en principi és única
    De moment deixo això. */
    return (identity.identity.serialNumber % 127) + 1;
    /*redueixo en un rand de 0-126. Sumi 1 per ajustar de 1-127 qye es el rang valid per node ID. */
    /*accedeixo així perquè tinc identity, que es pensa que es el union complet, no el struct intern. 
    Serial number, si mireu definició, no està dins de union directament sinó dins de struct identity. */
}

/* --------------------- Imprimir tots sensors --------------------------*/
void imprimirSensores() {
    // Sensores digitales (cada nodo = 1 sensor digital)
    for(int i = 0; i < OD_CNT_ARR_6000; i++){
        printf("Nodo %d : Digital Input = %d\n", i+1, OD_RAM.x6000_readDigitalInput8_bit[i]);
    }
}


/* --------------------- Callback EMCY ----------------------------*/
#if (CO_CONFIG_EM) & CO_CONFIG_EM_CONSUMER
void emergencyCallback(const uint16_t ident, 
                       const uint16_t errorCode,
                       const uint8_t errorRegister,
                       const uint8_t errorBit,
                       const uint32_t infoCode)
{
    emergencyDetected = true; 
    printf("[MASTER] EMCY received from node 0x%02X\n", ident & 0x7F);
    printf("[MASTER] Error Code: 0x%04X, Error Register: 0x%02X, Error Bit: %d, Info: 0x%08X\n",
           errorCode, errorRegister, errorBit, infoCode);
}
#endif

/* --------------------- Estats del máster -----------------------*/
typedef enum {
    MASTER_INIT,
    MASTER_LSS_DISCOVERY,
    MASTER_OPERATIONAL,
    MASTER_ERROR
} MasterState_t;

/*-------------- Funcions Complementàries ---------------*/

uint64_t get_time_linux(){
    struct timespec ts; 
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000; 
}

int main(int argc, char **argv){

    CO_ReturnError_t err;
    CO_LSSmaster_return_t  err_master; 
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT; //tipus de reinicialització donada
    void* CANptr = NULL; 
    uint64_t lastTime_us = 0; 
    MasterState_t masterState = MASTER_INIT;
    uint8_t idSlave = 0; 

    config_ptr = NULL;
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        printf("[MASTER] Can't create stack open");    
        return; 
    } else {
        printf("[MASTER] Allocated bytes for CANopen objects");
    }

    //aquí hauria de configurar la memòria????? > aL node ho vai fer

    while (reset != CO_RESET_APP) {
        
        printf("[MASTER] Reboot MASTER has started.... Config mode");
        
        CO -> CANmodule -> CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        err = CO_CANinit(CO, CAN_INTERFACE, MASTER_BITRATE);
        if (err != CO_ERROR_NO) {
            printf("[MASTER] Error al inicializar CAN\n");
            break;
            //return -2 > Revisar tipus errors
        }

        // Inicializar CANopen
        err = CO_CANopenInit(CO, NULL, NULL, OD, NULL, NMT_CONTROL,
                             FIRST_HB_TIME, 1000, 1000, false, MASTER_NODE_ID, NULL);
        if (err != CO_ERROR_NO) {
            printf("[MASTER] Error al inicializar CANopen\n");
            break;
            //return -3 > Revisar tipus errors
        }

        //Les adresses se les ha inventat paqui
        err = CO_LSSmaster_init(CO->LSSmaster, CO_LSSmaster_DEFAULT_TIMEOUT, CO->CANmodule, 0, 0x7E5, CO->CANmodule, 0,  0x7E4);
        /*Els camps d'aquesta funció son els seguents: CO_LSSMASTER_t*, timeout_ms, CANdevRX, CANdevRxIdx, CANidLSSSlave, CANdevTx, CANdevTxIdx, CANidLSSmaster
        Els id s'haurien de canviar segons el diccionari d'objectes
        IDX_RX i IDX_TX són 0 perquè només tinc un màster. Les seves posicions ja són la única, per dir-ho d'alguna forma.
        Es com dir "en quina posició de la taula interna de missatges RX/TX he de registrar LSS. "*/
        if (err != CO_ERROR_NO) {
            printf("[MASTER] Error al inicializar CANopen\n");
            break;
            //return -4 > Revisar tipus errors
        }

        uint32_t errInfo = 0;

        // Inicializar SYNC
        err = CO_SYNC_init(
        CO->SYNC, CO->em,
        OD_ENTRY_H1005, OD_ENTRY_H1006, OD_ENTRY_H1007, OD_ENTRY_H1019,
        CO->CANmodule, 0,
        #if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_PRODUCER
        CO->CANmodule, 1,
        #endif
        &errInfo);
        if (err != CO_ERROR_NO) {
            printf("[MASTER] Error al inicializar SYNC\n");
            break;
        }

        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        lastTime_us = get_time_linux();

        
        printf("[MASTER] Master node initiated\n");
       
        /* -------------------- Inicializar callback EMCY ------------------- */
        #if (CO_CONFIG_EM) & CO_CONFIG_EM_CONSUMER
        CO_EM_initCallbackRx(&CO->em, emergencyCallback);
        #endif
        
        // Bucle principal
        while (reset == CO_RESET_NOT) {
            uint64_t nowTime_us = get_time_linux(); 
            uint32_t timeDifference_us = (uint32_t)(nowTime_us - lastTime_us);
            lastTime_us = nowTime_us; 

            reset = CO_process(CO, false, timeDifference_us, NULL);

            /* -------------------- Procesar EMCY ----------------------------- */
            #if (CO_CONFIG_EM) & CO_CONFIG_EM_CONSUMER
            CO_EM_process(&CO->em, true, timeDifference_us, NULL);
            #endif
            
            //Hem sembla que aquest switch podria anar dins una funció a part. 
            switch (masterState){
                case MASTER_INIT:
                    masterState = MASTER_LSS_DISCOVERY;
                    break; 
                case MASTER_LSS_DISCOVERY:
                    /*no em cal posar enter pre-operational perquè els nodes, si no estan configurats, passen automaticamernt a LSS_WAIT
                    i a'aquí a NMT_PreOp sense que jo hagi de fer res.*/
                    CO_LSSmaster_return_t fastScanErr = CO_LSSmaster_IdentifyFastscan(CO->LSSmaster,timeDifference_us, &fastScan); 
                    if (fastScanErr != CO_LSSmaster_OK) {
                        printf("[MASTER] Error al buscar adressa LSS del node\n");
                        break;
                    }
                    while(fastScanErr == CO_LSSmaster_OK){
                        //Encara tinc algun node pendent d'escanejar. 
                        idSlave ++; 
                        //Aquí em sembla que hauria de preguntar per l'estat de l'esclau, no del master. 
                        err_master = CO_LSSmaster_InquireLssAddress(CO->LSSmaster, timeDifference_us, &adresses[idSlave]);
                        //Em guardo la adressa LSS a adresses -> De cadascun dels esclaus, es un llistat d'adresses
                        if (err_master != CO_LSSmaster_OK) {
                            printf("[MASTER] Error al buscar adressa LSS del node");
                            break;
                        }

                        err_master = CO_LSSmaster_swStateSelect(CO->LSSmaster, timeDifference_us, &adresses[idSlave]);
                        if (err_master != CO_LSSmaster_OK) {
                            printf("[MASTER] Error al buscar adressa LSS del node");
                            break;
                        }
                        /*Cada vegada que hagi de seleccionar un esclau ho he de fer així. Sinó no xutarà*/

                        /*A partir de l'adressa LSS puc trobar la identitat*/
                        if (adresses[idSlave].identity.vendorID != 0) {
                            uint8_t newNodeID = calcularNodeID(adresses[idSlave]);
                            CO_LSSmaster_configureNodeId(CO->LSSmaster, timeDifference_us, newNodeID);
                            CO_LSSmaster_configureStore(CO->LSSmaster, timeDifference_us);
                        }
                    }

                    if(idSlave <= MAX_SLAVES_TO_CONFIGURE){
                        printf("[MASTER] S'han configurat tots els nodes, en total %d", idSlave);
                        masterState = MASTER_OPERATIONAL;
                    }else{
                        printf("[MASTER] Hi ha alguna cosa que ha funcionat malament.");
                    }
                break; 

                case MASTER_OPERATIONAL:
                    static bool nmtSent = false;

                    if (!nmtSent) {
                        printf("[MASTER] Enviant NMT_ENTER_OPERATIONAL a tots els nodes");
                        CO_NMT_sendCommand(CO->NMT, CO_NMT_ENTER_OPERATIONAL, 0);
                        nmtSent = true;
                    }
                   
                        /*OJO!!! El 0 es el broadcast de tots els NMT. Quan envio a "0" realment estic enviant a tots els nodes.*/
                    uint32_t nextSync_us = UINT32_MAX;
                    CO_SYNC_status_t syncStatus;

                    if(!emergencyDetected){ //no hi ha emergencia

                        syncStatus = CO_SYNC_process(CO->SYNC, true, timeDifference_us, &nextSync_us);
                        if (syncStatus == CO_SYNC_RX_TX) {
                            imprimirSensores();
                        }       
                
                        /*He tingut una idea. Aquí podriem posar una cosa tipo que quan algo va malament intentar aplicar un heartbeat i 
                        sinó el que hauriem de fer és passar a l'estat de MASTER_ERROR i reiniciar a saco*/
                        uint8_t failedNodes = 0;

                        #if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE
                            uint32_t hbNext_us = UINT32_MAX;

                            // Procesar el consumidor de heartbeat
                            CO_HBconsumer_process(CO->HBcons, true, timeDifference_us, &hbNext_us);

                            // Recorrer los nodos monitorizados directamente desde CO->HBcons
                            for (uint8_t i = 0; i < CO->HBcons->numberOfMonitoredNodes; i++) {

                                // trobem quin node no ha respost dins el temps establert
                                if (CO->HBcons->monitoredNodes[i].HBstate == CO_HBconsumer_TIMEOUT) {

                                    uint8_t nodeId = CO->HBcons->monitoredNodes[i].nodeId;
                                    printf("[MASTER] Heartbeat perdut del node %d", nodeId);
                                    // Intento de recuperación suave
                                    failedNodes++;
                                    CO_NMT_sendCommand(CO->NMT, CO_NMT_RESET_NODE, nodeId);
                                }
                            }
                        
                            #endif
                            if (failedNodes >= (CO->HBcons->numberOfMonitoredNodes/2)) {//la meitat dels nodes han fallat
                                printf("[MASTER] Error greu detectat en monitoratge, passant a MASTER_ERROR");
                                masterState = MASTER_ERROR;
                            }else{
                                printf("[MASTER] Monitoratge OK, tornant a MASTER_OPERATIONAL");
                            }
                        
                    }else{
                        printf("[MASTER] EMCY DETECTAT!");
                        
                    }

                    break;

                case MASTER_ERROR:
                    /*canviar el estat a reset */
                    printf("[MASTER] ERROR: reiniciant nodes i la pila CANopen");
                    CO_NMT_sendCommand(CO->NMT, CO_NMT_RESET_COMMUNICATION, 0);
                    CO_NMT_sendCommand(CO->NMT, CO_NMT_RESET_NODE, 0); 
                    
                    reset = CO_RESET_COMM;// forçar reiniciar la pila del master, sortim del bucle 
                  
                    
                    break;
            }

            usleep(1000);
        }
    }
    return 0; 
}