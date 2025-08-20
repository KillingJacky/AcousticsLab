#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"

#include "radio.h"
#include "sx126x_bsp.h"
#include "timer.h"
//#define SX126X_SPI_DBUG 


static const char *TAG = "sx126x_bsp";

#define HOST_ID                 SPI3_HOST

static const int SPI_Frequency = 2000000;
static spi_device_handle_t SpiHandle;
static SemaphoreHandle_t   radio_mutex;

static DioIrqHandler * g_dioIrqHandler = NULL;

static RadioOperatingModes_t OperatingMode;

TimerTime_t g_lora_irq_time = 0;
bool  g_have_tcxo = false;

// SPI transaction functions
static bool spi_write_byte(uint8_t* data_in, size_t DataLength )
{
#ifdef SX126X_SPI_DBUG
    printf("spi write: ");
    for(int i =0; i < DataLength ; i++) {
        printf("%x ",data_in[i]);
    }
    printf("\r\n");
#endif

	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = data_in;
		SPITransaction.rx_buffer = NULL;
		spi_device_transmit( SpiHandle, &SPITransaction );
	}

	return true;
}

static bool spi_read_byte(uint8_t* data_out, size_t DataLength )
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = NULL;
		SPITransaction.rx_buffer = data_out;
		spi_device_transmit( SpiHandle, &SPITransaction );
	}

#ifdef SX126X_SPI_DBUG
    printf("spi read: ");
    for(int i =0; i < DataLength ; i++) {
        printf("%x ",data_out[i]);
    }
    printf("\r\n");
#endif
	return true;
}

static uint8_t spi_transfer(uint8_t* data_in, uint8_t* data_out, size_t DataLength)
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = data_in;
		SPITransaction.rx_buffer = data_out;
		spi_device_transmit( SpiHandle, &SPITransaction );
	}

#ifdef SX126X_SPI_DBUG
    printf("spi write: ");
    for(int i =0; i < DataLength ; i++) {
        printf("%x ",data_in[i]);
    }
    printf(",read: ");
    for(int i =0; i < DataLength ; i++) {
        printf("%x ",data_out[i]);
    }
    printf("\r\n");
#endif
	return true;
}

// GPIO interrupt handler
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    g_lora_irq_time = TimerGetCurrentTime();
    g_dioIrqHandler(0);
}

void SX126xIoInit( void )
{
    gpio_reset_pin(SX1262_NSS_PIN);
    gpio_reset_pin(SX1262_RST_PIN);
    gpio_reset_pin(SX1262_BUSY_PIN);
    gpio_reset_pin(SX1262_DIO1_PIN);
    gpio_reset_pin(SX1262_RF_SW_PIN);

    gpio_set_direction(SX1262_NSS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SX1262_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SX1262_BUSY_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(SX1262_DIO1_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(SX1262_RF_SW_PIN, GPIO_MODE_INPUT);  //not used, use DIO2 as RF switch control

    gpio_set_level(SX1262_NSS_PIN, 1);
    gpio_set_level(SX1262_RST_PIN, 1);
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    g_dioIrqHandler = dioIrq;

    static bool Ioirq_init_flag = false;
    if( Ioirq_init_flag ) {
        return;
    }
    Ioirq_init_flag = true;

    gpio_set_intr_type(SX1262_DIO1_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);  //default interrupt flag
    gpio_isr_handler_add(SX1262_DIO1_PIN, gpio_isr_handler, (void*)SX1262_DIO1_PIN);
}

void SX126xIoDeInit( void )
{
}

void SX126xIoDbgInit( void )
{
}

void SX126xIoTcxoInit( void )
{
    SX126xSetDio3AsTcxoCtrl(SX126X_TCXO_CTRL_VOLTAGE,  SX126xGetBoardTcxoWakeupTime()<<6);

    CalibrationParams_t calibParam;
    calibParam.Value = 0x7F;
    SX126xCalibrate( calibParam );
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
#define BOARD_TCXO_WAKEUP_TIME                      5
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX126xIoRfSwitchInit( void )
{
    SX126xSetDio2AsRfSwitchCtrl( true );
}

RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}

void SX126xSetOperatingMode( RadioOperatingModes_t mode )
{
    OperatingMode = mode;
}

void SX126xReset( void )
{
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(SX1262_RST_PIN, 0);
    vTaskDelay(30 / portTICK_PERIOD_MS);
    gpio_set_level(SX1262_RST_PIN, 1);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

void SX126xWaitOnBusy( void )
{
    while(1) {
        if( !gpio_get_level(SX1262_BUSY_PIN) ) {
            return;
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void SX126xWakeup( void )
{
    xSemaphoreTake(radio_mutex, portMAX_DELAY);
    gpio_set_level(SX1262_NSS_PIN, 0);
    uint8_t tx_buf[2];
    tx_buf[0] = RADIO_GET_STATUS;
    tx_buf[1] = 0x00;
    spi_write_byte(( uint8_t *)tx_buf, sizeof(tx_buf));
    gpio_set_level(SX1262_NSS_PIN, 1);
    xSemaphoreGive(radio_mutex);

    SX126xWaitOnBusy( );
    SX126xSetOperatingMode( MODE_STDBY_RC );

}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    xSemaphoreTake(radio_mutex, portMAX_DELAY);
    if(  command == RADIO_SET_SLEEP) {
        // Update mode in advance to prevent interrupts from being triggered when the device is sleeping
        SX126xSetOperatingMode( MODE_SLEEP ); 
    }
    gpio_set_level(SX1262_NSS_PIN, 0);
    spi_write_byte(( uint8_t *)&command, 1);
    spi_write_byte( buffer, size);
    gpio_set_level(SX1262_NSS_PIN, 1);
    xSemaphoreGive(radio_mutex);

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

uint8_t SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint8_t status = 0;

    SX126xCheckDeviceReady( );

    xSemaphoreTake(radio_mutex, portMAX_DELAY);
    gpio_set_level(SX1262_NSS_PIN, 0);
    spi_write_byte(( uint8_t *)&command, 1);
    uint8_t data = 0x00;
    spi_transfer(&data, &status, 1);
    spi_read_byte(buffer, size);
    gpio_set_level(SX1262_NSS_PIN, 1);
    xSemaphoreGive(radio_mutex);

    SX126xWaitOnBusy( );

    return status;
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    xSemaphoreTake(radio_mutex, portMAX_DELAY);
    gpio_set_level(SX1262_NSS_PIN, 0);
    uint8_t tx_buf[3];
    tx_buf[0] = RADIO_WRITE_REGISTER;
    tx_buf[1] = ( address & 0xFF00 ) >> 8;
    tx_buf[2] = address & 0x00FF;
    spi_write_byte(( uint8_t *)tx_buf, 3);
    spi_write_byte( buffer, size);
    gpio_set_level(SX1262_NSS_PIN, 1);
    xSemaphoreGive(radio_mutex);

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    xSemaphoreTake(radio_mutex, portMAX_DELAY);
    gpio_set_level(SX1262_NSS_PIN, 0);
    uint8_t tx_buf[4];
    tx_buf[0] = RADIO_READ_REGISTER;
    tx_buf[1] = ( address & 0xFF00 ) >> 8;
    tx_buf[2] = address & 0x00FF;
    tx_buf[3] = 0;
    spi_write_byte(( uint8_t *)tx_buf, 4);
    spi_read_byte(buffer, size);
    gpio_set_level(SX1262_NSS_PIN, 1);
    xSemaphoreGive(radio_mutex);

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    xSemaphoreTake(radio_mutex, portMAX_DELAY);
    gpio_set_level(SX1262_NSS_PIN, 0);
    uint8_t tx_buf[2];
    tx_buf[0] = RADIO_WRITE_BUFFER;
    tx_buf[1] = offset;
    spi_write_byte(( uint8_t *)tx_buf, sizeof(tx_buf));
    spi_write_byte( buffer, size);
    gpio_set_level(SX1262_NSS_PIN, 1);
    xSemaphoreGive(radio_mutex);

    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    xSemaphoreTake(radio_mutex, portMAX_DELAY);
    gpio_set_level(SX1262_NSS_PIN, 0);
    uint8_t tx_buf[3];
    tx_buf[0] = RADIO_READ_BUFFER;
    tx_buf[1] = offset;
    tx_buf[2] = 0;
    spi_write_byte(( uint8_t *)tx_buf, sizeof(tx_buf));
    spi_read_byte(buffer, size);

    gpio_set_level(SX1262_NSS_PIN, 1);
    xSemaphoreGive(radio_mutex);
    
    SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetDeviceId( void )
{
    return SX1262;
}

void SX126xAntSwOn( void )
{
}

void SX126xAntSwOff( void )
{
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

uint32_t SX126xGetDio1PinState( void )
{
    return gpio_get_level(SX1262_DIO1_PIN);
}


void sx126x_bsp_init(void)
{
    static bool bsp_init_flag = false;

    if(bsp_init_flag){
        return;
    }
    bsp_init_flag = true;

    ESP_LOGI(TAG, "sx126x bsp init...");

    radio_mutex =xSemaphoreCreateMutex();

    //Initialize the SPI bus
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num = ESP32_RADIO_MISO,
        .mosi_io_num = ESP32_RADIO_MOSI,
        .sclk_io_num = ESP32_RADIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    ret = spi_bus_initialize(HOST_ID, &buscfg, SPI_DMA_CH_AUTO);
	ESP_LOGI(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg;
	memset( &devcfg, 0, sizeof( spi_device_interface_config_t ) );
	devcfg.clock_speed_hz = SPI_Frequency;
	// It does not work with hardware CS control.
	//devcfg.spics_io_num = SX126x_SPI_SELECT;
	// It does work with software CS control.
	devcfg.spics_io_num = -1;
	devcfg.queue_size = 7;
	devcfg.mode = 0;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;

	ret = spi_bus_add_device( HOST_ID, &devcfg, &SpiHandle);
	ESP_LOGI(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);

    SX126xIoInit();
}
