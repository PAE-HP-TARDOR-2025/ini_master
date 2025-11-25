#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "CANopen.h"
#include "CO_driver.h"
#include "CO_LSSmaster.h"
#include "CO_NMT_Heartbeat.h"
#include "OD.h"
/*app_main > ESP-IDF redefineix el punt d'entrada del programa
Embeded Systems amb freeRTOS cal inicialitzar el sistema i cridar app main
Framework > Controla l'arranque del sistema. */

/* ---------------- Configuració Macros ------------------------- */
#define CAN_INTERFACE          "can0"
/*Linux -> Les interfícies CAN apareixen així
can0: indica la interfície física sobre la que reballarà el màster
    Generalment Rasberry Pi té interfície can0 activada
can1
vcan0 (CAN virtual)
*/
#define MASTER_NODE_ID         0x01        // Node ID del master
#define MASTER_BITRATE         250000      // Bitrate CAN
#define FIRST_ASSIGNED_ID      0x10        // IDs començaran als 0x10 -> Això potser ho podem canviar
#define MAX_NODES_TO_CONFIGURE 10          //nombre màxim de nodes per configurar > Això ho he posat de cara als ACKs
#define ASSIGNED_BITRATE       250000      //Bit rate que assignem als esclaus > Vull assignar-ho des d'aquí. 
#define NMT_CONTROL CO_NMT_STARTUP_TO_OPERATIONAL
#define FIRST_HB_TIME          1000        //temps del primer heartbeat. 1000 -> ms


/* -------------------- Globals ----------------------------------*/
static CO_CANmodule_t CANmodule_instance; 
static CO_t *CO = NULL;
static CO_config_t *config_ptr = 0; 
static uint32_t heapMemoryUsed = 0; 

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
    MASTER_LSS_CONFIG,
    MASTER_OPERATIONAL,
    MASTER_MONITORING,
    MASTER_ERROR
} MasterState_t;


void app_main(void){

    CO_CANmodule_t *CANmodule = &CANmodule_instance; 
    CO_ReturnError_t err; 
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT; //tipus de reinicialització donada
    void* CANptr = NULL; 
    uint64_t lastTime_us = 0; 
    MasterState_t masterState = MASTER_INIT;

    config_ptr = NULL;
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf("Error: can't create stack open\n");
        return -1;
        //Aquí fa un return -1 > Revisar els errors
    } else {
        log_printf("Allocated %u bytes for CANopen objects\n", heapMemoryUsed);
    }

    //aquí hauria de configurar la memòria????? > aL node ho vai fer

    while (reset != CO_RESET_APP) {
        
        log_printf("Reboot MASTER has started.... Config mode\n");
        
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
        err = CO_LSSmaster_init(CO->LSSmaster, CO->CANmodule, 0x7E5, 0x7E4);
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
            CO_LSSmaster_process(CO->LSSmaster, timeDifference_us);

            //Hem sembla que aquest switch podria anar dins una funció a part. 
            switch (masterState){
                case MASTER_INIT:
                    masterState = MASTER_LSS_DISCOVERY;
                    break; 
                case MASTER_LSS_DISCOVERY:
                    if (CO->LSSmaster->state == CO_LSSmaster_waiting) {
                    CO_LSSmaster_inquireIdentity(CO->LSSmaster);
                    masterState = MASTER_LSS_CONFIG;
                    }
                    break; 
                //Aquí hauria de poder configurar la velocitat des d'aquí
                case MASTER_LSS_CONFIG:
                    if (CO->LSSmaster->identity.vendorID != 0) {
                        uint8_t newNodeID = calcularNodeID(CO->LSSmaster->identity);
                        CO_LSSmaster_configureNodeID(CO->LSSmaster, newNodeID);
                        CO_LSSmaster_storeConfiguration(CO->LSSmaster);
                        masterState = MASTER_OPERATIONAL;
                    }
                    break;
                case MASTER_OPERATIONAL:
                    //Lògica del màster
                    //Enviar PDOs i SDOs
                    break;
                case MASTER_MONITORING:
                    //supervisió. Hearbeats, estats dels nodes, etc
                    //Hauria d'accedir-hi des del operational
                    break; 
                case MASTER_ERROR:
                    //gestió d'errors.
                    break;
            }
        }
    }
}


