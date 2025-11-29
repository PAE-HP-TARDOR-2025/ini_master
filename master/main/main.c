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
#define MASTER_BITRATE         250     // Bitrate CAN
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

/* --------------------- Estats del máster -----------------------*/
typedef enum {
    MASTER_INIT,
    MASTER_LSS_DISCOVERY,
    MASTER_OPERATIONAL,
    MASTER_MONITORING,
    MASTER_ERROR
} MasterState_t;

/*-------------- Funcions Complementàries ---------------*/

uint64_t get_time_ESP(){
    return esp_timer_get_time();
}

void app_main(void){

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
        ESP_LOGE(TAG, "Can't create stack open");    
        return; 
    } else {
        ESP_LOGE(TAG, "Allocated bytes for CANopen objects");
    }

    //aquí hauria de configurar la memòria????? > aL node ho vai fer

    while (reset != CO_RESET_APP) {
        
        ESP_LOGE(TAG, "Reboot MASTER has started.... Config mode");
        
        CO -> CANmodule -> CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        err = CO_CANinit(CO, NULL, MASTER_BITRATE);
        if (err != CO_ERROR_NO) {
            printf("Error al inicializar CAN\n");
            break;
            //return -2 > Revisar tipus errors
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

        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        lastTime_us = get_time_ESP();

        
        printf("Master node initiated\n");
       
        
        // Bucle principal
        while (reset == CO_RESET_NOT) {
            uint64_t nowTime_us = get_time_ESP(); 
            uint32_t timeDifference_us = (uint32_t)(nowTime_us - lastTime_us);
            lastTime_us = nowTime_us; 

            reset = CO_process(CO, false, timeDifference_us, NULL);
            
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
                        printf("Error al buscar adressa LSS del node\n");
                        break;
                    }
                    while(fastScanErr == CO_LSSmaster_OK){
                        //Encara tinc algun node pendent d'escanejar. 
                        idSlave ++; 
                        //Aquí em sembla que hauria de preguntar per l'estat de l'esclau, no del master. 
                        err_master = CO_LSSmaster_InquireLssAddress(CO->LSSmaster, timeDifference_us, &adresses[idSlave]);
                        //Em guardo la adressa LSS a adresses -> De cadascun dels esclaus, es un llistat d'adresses
                        if (err_master != CO_LSSmaster_OK) {
                            ESP_LOGE(TAG, "Error al buscar adressa LSS del node");
                            break;
                        }

                        err_master = CO_LSSmaster_swStateSelect(CO->LSSmaster, timeDifference_us, &adresses[idSlave]);
                        if (err_master != CO_LSSmaster_OK) {
                            ESP_LOGE(TAG, "Error al buscar adressa LSS del node");
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
                        ESP_LOGE(TAG, "S'han configurat tots els nodes, en total %d", idSlave);
                        masterState = MASTER_LSS_DISCOVERY;
                    }else{
                        ESP_LOGE(TAG, "Hi ha alguna cosa que ha funcionat malament.");
                    }
                break; 

                case MASTER_OPERATIONAL:
                    CO_NMT_sendCommand(&CO, CO_NMT_ENTER_OPERATIONAL, 0); /*Això si que cal, sorry*/
                    /*OJO!!! El 0 es el broadcast de tots els NMT. Quan envio a "0" realment estic enviant a tots els nodes.*/
                    
                    /*Logica del Codi*/
                    
                    break;
                case MASTER_MONITORING:
                    /*He tingut una idea. Aquí podriem posar una cosa tipo que quan algo va malament intentar aplicar un heartbeat i 
                    sinó el que hauriem de fer és passar a l'estat de MASTER_ERROR i reiniciar a saco*/
                    break; 
                case MASTER_ERROR:
                    /*Aquí es podria fer una cosa així com canviar el estat a reset  o algo por el estilo. 
                    De moment ho deixo comentat perquè hauriem de pensar en quins casos pot succeir i gestionar.*/
                    CO_NMT_sendCommand(CO->NMT, CO_NMT_RESET_COMMUNICATION, 0);
                    CO_NMT_sendCommand(CO->NMT, CO_NMT_RESET_NODE, 0); 
                    break;
            }
        }
    }
}


