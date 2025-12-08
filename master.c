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
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_efuse.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
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
CO_LSS_address_t adresses[MAX_SLAVES_TO_CONFIGURE];
CO_LSSmaster_fastscan_t fastScan;


/* --------------------- Assignar NodeID --------------------------*/
uint32_t calcularNodeID(CO_LSS_address_t identity) {
    return (identity.identity.serialNumber % 127) + 1;
    /*redueixo en un rand de 0-126. Sumi 1 per ajustar de 1-127 qye es el rang valid per node ID. */
    /*accedeixo així perquè tinc identity, que es pensa que es el union complet, no el struct intern. 
    Serial number, si mireu definició, no està dins de union directament sinó dins de struct identity. */
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
    printf("EMCY received from node 0x%02X\n", ident & 0x7F);
    printf("  Error Code: 0x%04X, Error Register: 0x%02X, Error Bit: %d, Info: 0x%08X\n",
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

typedef enum{
    LSS_START_SCAN, 
    LSS_WAIT_ADRESS, 
    LSS_SELECT_NODE, 
    LSS_ASSIGN_ID, 
    LSS_CONFIG_STORE,
    LSS_DESELECT_NODE, 
    LSS_END_DISCOVERY
}LSS_SubState_t; 

/*-------------- Funcions Complementàries ---------------*/

uint64_t get_time_ESP(){
    return esp_timer_get_time();
}

void imprimirDatosDeSlaves() {
    //imprimir dades
}

/*-------------- Init Master -------------------------- */
void init_master(){
    //Aquí hauriem de posar tot el que queda a dins del while(reset != CO_RESET_APP) per fer-ho modular
}

void app_main(void){

    CO_ReturnError_t err;
    CO_LSSmaster_return_t  err_master; 
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT; //tipus de reinicialització donada
    void* CANptr = NULL; 
    uint64_t lastTime_us = 0; 
    MasterState_t masterState = MASTER_INIT;
    LSS_SubState_t lssSubState = LSS_START_SCAN;
    uint8_t idSlave = 0; 

    config_ptr = NULL;
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL){
        ESP_LOGE(TAG, "Can't create stack open");    
        return; 
    }else{
        ESP_LOGE(TAG, "Allocated bytes for CANopen objects");
    }

    while (reset != CO_RESET_APP) {
        
        ESP_LOGE(TAG, "Reboot MASTER has started.... Config mode");
        
        CO -> CANmodule -> CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        err = CO_CANinit(CO, NULL, MASTER_BITRATE);
        if (err != CO_ERROR_NO) {
            printf("Error al inicializar CAN\n");
            break;
        }

        // Inicializar CANopen
        err = CO_CANopenInit(CO, NULL, NULL, OD, NULL, NMT_CONTROL,
                             FIRST_HB_TIME, 1000, 1000, false, MASTER_NODE_ID, NULL);
        if (err != CO_ERROR_NO) {
            printf("Error al inicializar CANopen\n");
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
            printf("Error al inicializar CANopen\n");
            break;
            //return -4 > Revisar tipus errors
        }

        uint32_t errInfo = 0;

        // Inicializar SYNC
        err = CO_SYNC_init(CO->SYNC, CO->em, OD_ENTRY_H1005, OD_ENTRY_H1006, OD_ENTRY_H1007, OD_ENTRY_H1019,
            CO->CANmodule, 0, 
            #if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_PRODUCER
                CO->CANmodule, 1,
            #endif
            &errInfo);
        if (err != CO_ERROR_NO) {
            printf("Error al inicializar SYNC\n");
            return;
        }

        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        lastTime_us = get_time_ESP();

        printf("Master node initiated\n");
       
        /* -------------------- Inicializar callback EMCY ------------------- */
        #if (CO_CONFIG_EM) & CO_CONFIG_EM_CONSUMER
            CO_EM_initCallbackRx(&CO->em, emergencyCallback);
        #endif
        
        // Bucle principal
        while (reset == CO_RESET_NOT) {
            uint64_t nowTime_us = get_time_ESP(); 
            uint32_t timeDifference_us = (uint32_t)(nowTime_us - lastTime_us);
            lastTime_us = nowTime_us; 

            reset = CO_process(CO, false, timeDifference_us, NULL);

            /* -------------------- Procesar EMCY ----------------------------- */
            #if (CO_CONFIG_EM) & CO_CONFIG_EM_CONSUMER
            CO_EM_process(&CO->em, true, timeDifference_us, NULL);
            #endif
            
            //Em sembla que aquest switch podria anar dins una funció a part. 
            switch (masterState){
                case MASTER_INIT:
                    masterState = MASTER_LSS_DISCOVERY;
                    break; 
                case MASTER_LSS_DISCOVERY:
                    /*no em cal posar enter pre-operational perquè els nodes, si no estan configurats, passen automaticamernt a LSS_WAIT
                    i a'aquí a NMT_PreOp sense que jo hagi de fer res.*/
                    
                    CO_LSSmaster_return_t err_master; 

                    switch(lssSubState){
                        case LSS_START_SCAN: 
                            err_master = CO_LSSmaster_IdentifyFastScan(CO->LSSmaster,timeDifference_us, &fastScan);
                            if(err_master == CO_LSSmaster_OK){
                                ESP_LOGE("S'ha trobat un node per configurar")
                                lssSubState = LSS_WAIT_ADRESS; 
                            }else if(err_master == CO_LSSmaster_NO_RESPONSE_TIMEOUT){
                                ESP_LOGE("No s'ha trobat cap node per configurar. Acabem descobriment...")
                                lss_SubState = LSS_END_DISCOVERY;
                            } 
                        break;
                        case LSS_SELECT_NODE:
                            err_master = CO_LSSmaster_swStateSelect(CO->LSSmaster, timeDifference_us, &adresses[idSlave]); 
                            if (err_master == CO_LSSmaster_OK) {
                                ESP_LOGE("S'ha seleccionat el node a configurar")
                                lssSubState = LSS_ASSIGN_ID;
                            } else if (err_master == CO_LSSmaster_NO_RESPONSE_TIMEOUT) {
                                ESP_LOGE("No s'ha pogut seleccionar el node trobat. Deseleccionem...")
                                lssSubState = LSS_DESELECT_NODE; 
                            } 
                        break;
                        case LSS_ASSIGN_ID: 
                            if (adresses[idSlave].identity.vendorID != 0) {
                                uint8_t newNodeID = calcularNodeID(adresses[idSlave]);
                                err_master = CO_LSSmaster_configureNodeId(CO->LSSmaster, timeDifference_us, newNodeID);
                                if (err_master == CO_LSSmaster_OK) {
                                    ESP_LOGE("Hem pogut configurar el node ID")
                                    lssSubState = LSS_CONFIG_STORE;
                                }
                            } else{
                                // El esclavo no tiene un ID válido, avanzar al deseleccionar
                                ESP_LOGE("No hem pogut configurar node ID, ID slave no vàlid. Deselecionem....")
                                lssSubState = LSS_DESELECT_NODE;
                            }
                        break;
                        case LSS_CONFIG_STORE:
                            err_master = CO_LSSmaster_configureStore(CO->LSSmaster, timeDifference_us);
                            if(err_master == CO_LSSmaster_OK){
                                ESP_LOGE("Hem porgut guardar el node ID. Deseleccionem node...")
                                lssSubState = LSS_DESELECT_NODE;
                            }else{
                                ESP_LOGE("No hem pogut guardar el node ID. Deseleccionem ...")
                            }
                        break;
                        case LSS_DESELECT_NODE: 
                            err_master = CO_LSSmaster_switchStateGlobal(CO->LSSmaster, timeDifference_us, CO_LSS_SWITCH_GLOBAL_STATE_WAIT);
                            if(err_master == CO_LSSmaster_OK){
                                ESP_LOGE("Podem canviar estat LSS... Comencem nou descobriment")
                                idSlave ++; 
                                lssSubState = LSS_START_SCAN; 
                            }else if(err_master == CO_LSSmaster_NO_RESPONSE_TIMEOUT){
                                ESP_LOGE("No podem canviar estat LSS... Repetim descobriment...")
                                lssSubState = LSS_START_SCAN; 
                            }else if(err_master != CO_LSSmaster_SCAN_BUSY){
                                ESP_LOGE("Alguna cosa està fallant... ERROR MÀSTER!!!")
                                masterState = MASTER_ERROR; 
                                lssSubState = LSS_END_DISCOVERY;
                            }
                        break;
                        case LSS_END_DISCOVERY: 
                            ESP_LOGE("No queden més nodes a descobrir. Acabem estat MASTER_LSS_DISCOVERY")
                            masterState = MASTER_OPERATIONAL; 
                            lssSubState = LSS_START_SCAN; 
                        break;                            
                    }  
                case MASTER_OPERATIONAL:
                    static bool nmtSent = false;

                    if (!nmtSent) {
                        ESP_LOGE(TAG, "Enviant NMT_ENTER_OPERATIONAL a tots els nodes");
                        CO_NMT_sendCommand(CO->NMT, CO_NMT_ENTER_OPERATIONAL, 0);
                        nmtSent = true;
                    }
                   
                        /*OJO!!! El 0 es el broadcast de tots els NMT. Quan envio a "0" realment estic enviant a tots els nodes.*/
                    uint32_t nextSync_us = UINT32_MAX;
                    CO_SYNC_status_t syncStatus;

                    if(!emergencyDetected){ //no hi ha emergencia

                        syncStatus = CO_SYNC_process(CO->SYNC, true, timeDifference_us, &nextSync_us);
                        if (syncStatus == CO_SYNC_RX_TX) {
                            imprimirDatosDeSlaves();
                        }       
                
                        /*He tingut una idea. Aquí podriem posar una cosa tipo que quan algo va malament intentar aplicar un heartbeat i 
                        sinó el que hauriem de fer és passar a l'estat de MASTER_ERROR i reiniciar a saco*/
                        bool errorGrave = false;

                        #if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE
                            uint32_t hbNext_us = UINT32_MAX;

                            // Procesar el consumidor de heartbeat
                            CO_HBconsumer_process(CO->HBcons, true, timeDifference_us, &hbNext_us);

                            // Recorrer los nodos monitorizados directamente desde CO->HBcons
                            for (uint8_t i = 0; i < CO->HBcons->numberOfMonitoredNodes; i++) {

                                // trobem quin node no ha respost dins el temps establert
                                if (CO->HBcons->monitoredNodes[i].HBstate == CO_HBconsumer_TIMEOUT) {

                                    uint8_t nodeId = CO->HBcons->monitoredNodes[i].nodeId;
                                    ESP_LOGE(TAG, "Heartbeat perdut del node %d", nodeId);
                                    // Intento de recuperación suave
                                    CO_NMT_sendCommand(CO->NMT, CO_NMT_RESET_NODE, nodeId);
                                    errorGrave = true;
                                }
                            }
                        
                       
                            if (errorGrave) {
                                ESP_LOGE(TAG, "Error greu detectat en monitoratge, passant a MASTER_ERROR");
                                masterState = MASTER_ERROR;
                            } else {
                                ESP_LOGE(TAG, "Monitoratge OK, tornant a MASTER_OPERATIONAL");
                            }
                    }else{
                        ESP_LOGE(TAG, "EMCY DETECTAT!");
                    } #endif
                break;

                case MASTER_ERROR:
                    /*canviar el estat a reset */
                    ESP_LOGE(TAG, "MASTER_ERROR: reiniciant nodes i la pila CANopen");
                    CO_NMT_sendCommand(CO->NMT, CO_NMT_RESET_COMMUNICATION, 0);
                    CO_NMT_sendCommand(CO->NMT, CO_NMT_RESET_NODE, 0); 
                    
                    reset = CO_RESET_COMM;// forçar reiniciar la pila del master, sortim del bucle 
                break;
            }
        }
    }

}
