/**
 * @file lorawan.c
 * @author Jack
 * @date 2025-08-15
 * @brief This file contains implementations related to LoRaWAN functionality
 * for the AcousticsLab components.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"

#include "lorawan.h"
#include "sx126x_bsp.h"

/* LoRaMac */
#include "utilities.h"
#include "RegionCommon.h"
#include "Commissioning.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "LmHandlerMsgDisplay.h"
#include "githubVersion.h"
#include "timer.h"
#include "nvs_flash.h"

static const char* TAG = "lorawan";       // Log tag for this module


#define LORAWAN_TASK_STACK_SIZE     (1024 * 5)
#define LORAWAN_TASK_PRIORITY       3
#define LORAWAN_TASK_NAME           "lorawan_task"
#define LORAWAN_TASK_PIN_TO_CORE    0

#define FIRMWARE_VERSION            0x01000000 // 1.0.0.0
#define DEFAULT_REGION              LORAMAC_REGION_EU868

/*!
 * LoRaWAN default end-device class
 */
#ifndef LORAWAN_DEFAULT_CLASS
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            30000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

/*!
 * LoRaWAN application port
 * @remark The allowed port range is from 1 up to 223. Other values are reserved.
 */
#define LORAWAN_APP_PORT                            2

/*!
 * LoRaWAN region
 */
LoRaMacRegion_t region = DEFAULT_REGION;

/*!
 * LoRaWAN Device EUI
 */
static uint8_t device_eui[8] = { 0x0 } ;

/*!
 * LoRaWAN Join EUI / App EUI
 */
static uint8_t app_eui[8] = { 0x0 } ;

/*!
 * LoRaWAN APP KEY
 */
static uint8_t app_key[16] = { 0x0 } ;

/*!
 *
 */
typedef enum
{
    LORAMAC_HANDLER_TX_ON_TIMER,
    LORAMAC_HANDLER_TX_ON_EVENT,
}LmHandlerTxEvents_t;

/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/*!
 * User application data structure
 */
static LmHandlerAppData_t AppData =
{
    .Buffer = AppDataBuffer,
    .BufferSize = 0,
    .Port = 0,
};


static bool g_abnormal_event = false;
static bool g_joined = false;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;

static void OnMacProcessNotify( void );
static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size );
static void OnNetworkParametersChange( CommissioningParams_t* params );
static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn );
static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn );
static void OnJoinRequest( LmHandlerJoinParams_t* params );
static void OnTxData( LmHandlerTxParams_t* params );
static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params );
static void OnClassChange( DeviceClass_t deviceClass );
static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params );
#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection );
#else
static void OnSysTimeUpdate( void );
#endif
static void PrepareTxFrame( void );
static void StartTxProcess( LmHandlerTxEvents_t txEvent );
static void UplinkProcess( void );

static void OnComplianceTxPeriodicityChanged( uint32_t periodicity );
static void OnComplianceTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed );
static void OnCompliancePingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity );

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context );

static LmHandlerCallbacks_t LmHandlerCallbacks =
{
    .GetBatteryLevel = NULL,
    .GetTemperature = NULL,
    .GetRandomSeed = NULL,
    .OnMacProcess = OnMacProcessNotify,
    .OnNvmDataChange = OnNvmDataChange,
    .OnNetworkParametersChange = OnNetworkParametersChange,
    .OnMacMcpsRequest = OnMacMcpsRequest,
    .OnMacMlmeRequest = OnMacMlmeRequest,
    .OnJoinRequest = OnJoinRequest,
    .OnTxData = OnTxData,
    .OnRxData = OnRxData,
    .OnClassChange= OnClassChange,
    .OnBeaconStatusChange = OnBeaconStatusChange,
    .OnSysTimeUpdate = OnSysTimeUpdate,
};

static LmHandlerParams_t LmHandlerParams =
{
    .Region = DEFAULT_REGION,
    .AdrEnable = LORAWAN_ADR_STATE,
    .IsTxConfirmed = LORAMAC_HANDLER_UNCONFIRMED_MSG,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
    .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
    .DataBuffer = AppDataBuffer,
    .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
};

static LmhpComplianceParams_t LmhpComplianceParams =
{
    .FwVersion.Value = FIRMWARE_VERSION,
    .OnTxPeriodicityChanged = OnComplianceTxPeriodicityChanged,
    .OnTxFrameCtrlChanged = OnComplianceTxFrameCtrlChanged,
    .OnPingSlotPeriodicityChanged = OnCompliancePingSlotPeriodicityChanged,
};

/*!
 * Indicates if LoRaMacProcess call is pending.
 * 
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;

static volatile atomic_bool IsTxFramePending = ATOMIC_VAR_INIT(false);

static volatile uint32_t TxPeriodicity = 0;



static void OnMacProcessNotify( void )
{
    IsMacProcessPending = 1;
}

static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size )
{
    DisplayNvmDataChange( state, size );
}

static void OnNetworkParametersChange( CommissioningParams_t* params )
{
    DisplayNetworkParametersUpdate( params );
}

static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn )
{
    DisplayMacMcpsRequestUpdate( status, mcpsReq, nextTxIn );
}

static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn )
{
    DisplayMacMlmeRequestUpdate( status, mlmeReq, nextTxIn );
}

static void OnJoinRequest( LmHandlerJoinParams_t* params )
{
    DisplayJoinRequestUpdate( params );
    if( params->Status == LORAMAC_HANDLER_ERROR )
    {
        LmHandlerJoin( );
    }
    else
    {
        ESP_LOGI(TAG, "Join successful");
        g_joined = true;
        LmHandlerRequestClass( LORAWAN_DEFAULT_CLASS );
    }
}

static void OnTxData( LmHandlerTxParams_t* params )
{
    DisplayTxUpdate( params );
}

static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params )
{
    DisplayRxUpdate( appData, params );
}

static void OnClassChange( DeviceClass_t deviceClass )
{
    DisplayClassUpdate( deviceClass );

    // Inform the server as soon as possible that the end-device has switched to ClassB
    LmHandlerAppData_t appData =
    {
        .Buffer = NULL,
        .BufferSize = 0,
        .Port = 0,
    };
    LmHandlerSend( &appData, LORAMAC_HANDLER_UNCONFIRMED_MSG );
}

static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params )
{

    DisplayBeaconUpdate( params );
}

#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection )
{

}
#else
static void OnSysTimeUpdate( void )
{

}
#endif

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context )
{
    TimerStop( &TxTimer );

    atomic_store(&IsTxFramePending, true);

    // Schedule next transmission
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

static void StartTxProcess( LmHandlerTxEvents_t txEvent )
{
    switch( txEvent )
    {
    default:
        // Intentional fall through
    case LORAMAC_HANDLER_TX_ON_TIMER:
        {
            // Schedule 1st packet transmission
            TimerInit( &TxTimer, OnTxTimerEvent );
            TimerSetValue( &TxTimer, TxPeriodicity );
            OnTxTimerEvent( NULL );
        }
        break;
    case LORAMAC_HANDLER_TX_ON_EVENT:
        {
        }
        break;
    }
}

void lorawan_enqueue(bool abnormal)
{
    static uint32_t last_event_time = 0;
    static bool first_enqueue = true;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (!g_joined) {
        ESP_LOGW(TAG, "LoRaWAN not joined yet, cannot enqueue event.");
        return;
    }

    // Throttle mechanism to ensure that the anomaly event is sent at a controlled rate
    if (abnormal && (current_time - last_event_time < 30000) && !first_enqueue) {
        ESP_LOGW(TAG, "Anomaly event already sent in the last 30s, skipping this one.");
        return;
    }
    if (abnormal) {
        first_enqueue = false;
        last_event_time = current_time;
    }

    if (abnormal == false) {
        if (g_abnormal_event == false) {
            ESP_LOGW(TAG, "Normal state already closed, skipping this one.");
            return;
        }
        if (atomic_load(&IsTxFramePending) == true && g_abnormal_event == true) {
            ESP_LOGW(TAG, "The abnormal event is pending and critical, skipping this normal event.");
            return;
        } 
    }

    g_abnormal_event = abnormal;

    atomic_store(&IsTxFramePending, true);

    ESP_LOGI(TAG, "Enqueued event: %s", abnormal ? "abnormal" : "normal");
}

/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame( void )
{
    if( LmHandlerIsBusy( ) == true )
    {
        return;
    }

    LmHandlerMsgTypes_t needConfirm = LORAMAC_HANDLER_UNCONFIRMED_MSG;

    AppData.Buffer[0] = g_abnormal_event ? 1 : 0;
    AppData.Buffer[1] = 0;
    AppData.Buffer[2] = 0;
    AppData.Buffer[3] = 0;
    AppData.BufferSize = 4;

    AppData.Port = LORAWAN_APP_PORT;

    if (g_abnormal_event)
    {
        needConfirm = LORAMAC_HANDLER_CONFIRMED_MSG;
    }

    if( LmHandlerSend( &AppData, needConfirm ) == LORAMAC_HANDLER_SUCCESS )
    {
        ESP_LOGI(TAG, "LoRaWAN frame was sent successfully!");
        atomic_store(&IsTxFramePending, false);
    } 
    else
    {
        ESP_LOGE(TAG, "LoRaWAN frame failed to send!");
    }
}

static void UplinkProcess( void )
{
    if( atomic_load(&IsTxFramePending) == true )
    {
        PrepareTxFrame( );
    }
}

/*!
 * LoRaWAN Compliance package callbacks
 * These callbacks are used to handle the Compliance package events.
 * This demo doesn't support Compliance test.
 */
static void OnComplianceTxPeriodicityChanged( uint32_t periodicity )
{
}

static void OnComplianceTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed )
{
}

static void OnCompliancePingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity )
{
}

/*!
 * Initializes the NVM flash storage.
 */
int nvm_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    return ret;
}

/*!
 * Task to handle LoRaWAN
 */
static void lorawan_task(void * arg)
{
    const Version_t gitHubVersion = { .Value = GITHUB_VERSION };
    printf( "\n###### ===================================== ######\n" );
    printf( "LoRaMacNode version: %d.%d.%d\n", gitHubVersion.Fields.Major, gitHubVersion.Fields.Minor, 
        gitHubVersion.Fields.Patch );
    printf( "\n###### ===================================== ######\n" );

    if ( LmHandlerInit( &LmHandlerCallbacks, &LmHandlerParams ) != LORAMAC_HANDLER_SUCCESS )
    {
        // Fatal error
        ESP_LOGE(TAG, "LoRaMac wasn't properly initialized, this is critical error!\n" );
        return;
    }

    ESP_LOGI(TAG, "Set Device EUI & APP KEY...");
    printf("DevEui: ");
    for( int i =0; i < 8; i++ ) {
        printf("%02X", device_eui[i]);
    }
    printf("\r\n");
    printf( "AppEUI/JoinEUI: " );
    for( int i =0; i < 8; i++ ) {
        printf("%02X",app_eui[i]);
    }
    printf("\r\n");
    printf( "AppKEY: " );
    for( int i =0; i < 16; i++ ) {
        printf("%02X",app_key[i]);
    }
    printf("\r\n");

    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_DEV_EUI;
    mibReq.Param.DevEui = device_eui;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_JOIN_EUI;
    mibReq.Param.JoinEui = app_eui;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_APP_KEY;
    mibReq.Param.AppKey = app_key;
    LoRaMacMibSetRequestConfirm( &mibReq );
    mibReq.Type = MIB_NWK_KEY;
    mibReq.Param.NwkKey = app_key;
    LoRaMacMibSetRequestConfirm( &mibReq );

    ESP_LOGI(TAG, "LoRaMac was initialized");
    
    // Set system maximum tolerated rx error in milliseconds
    LmHandlerSetSystemMaxRxError( 20 );

    // The LoRa-Alliance Compliance protocol package should always be
    // initialized and activated.
    LmHandlerPackageRegister( PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams );

    LmHandlerJoin( );

    StartTxProcess( LORAMAC_HANDLER_TX_ON_EVENT );

    while( 1 )
    {
        // Processes the LoRaMac events
        LmHandlerProcess( );

        // Process application uplinks management
        UplinkProcess( );

        if( IsMacProcessPending == 1 )
        {
            // Clear flag and prevent MCU to go into low power modes.
            IsMacProcessPending = 0;
        }
        else
        {
            // The MCU wakes up through events
            //BoardLowPowerHandler( );
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*!
 * Main application entry point.
 */
void lorawan_init(const char* eui, const char* app_eui_, const char* app_key_, const char* freq)
{
    // ESP_ERROR_CHECK(bsp_board_init());

    if (nvm_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "NVM initialization failed");
        return;
    }

    //NvmDataMgmtFactoryReset(); // If you modify the built-in region, you must perform a restore of nvs

    sx126x_bsp_init();

    if (hexStringToBytes(eui, device_eui, sizeof(device_eui)) != 8)
    {
        ESP_LOGE(TAG, "Invalid Device EUI format");
        return;
    }
    if (hexStringToBytes(app_eui_, app_eui, sizeof(app_eui)) != 8)
    {
        ESP_LOGE(TAG, "Invalid App EUI/Join EUI format");
        return;
    }
    if (hexStringToBytes(app_key_, app_key, sizeof(app_key)) != 16)
    {
        ESP_LOGE(TAG, "Invalid App Key format");
        return;
    }

    // Set region based on freq parameter
    if (freq != NULL) {
        ESP_LOGI(TAG, "Setting LoRaWAN region to %s", freq);
        if (strcmp(freq, "AS923") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_AS923;
        } else if (strcmp(freq, "AU915") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_AU915;
        } else if (strcmp(freq, "CN470") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_CN470;
        } else if (strcmp(freq, "CN779") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_CN779;
        } else if (strcmp(freq, "EU433") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_EU433;
        } else if (strcmp(freq, "EU868") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_EU868;
        } else if (strcmp(freq, "KR920") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_KR920;
        } else if (strcmp(freq, "IN865") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_IN865;
        } else if (strcmp(freq, "US915") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_US915;
        } else if (strcmp(freq, "RU864") == 0) {
            LmHandlerParams.Region = LORAMAC_REGION_RU864;
        } else {
            ESP_LOGW(TAG, "Invalid region %s, using default region EU868", freq);
        }
    }

    // Initialize transmission periodicity variable
    TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );

    int ret = xTaskCreatePinnedToCore(lorawan_task, LORAWAN_TASK_NAME, LORAWAN_TASK_STACK_SIZE, NULL, 
        LORAWAN_TASK_PRIORITY, NULL, LORAWAN_TASK_PIN_TO_CORE);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create lorawan task: %d", ret);
    }
}
