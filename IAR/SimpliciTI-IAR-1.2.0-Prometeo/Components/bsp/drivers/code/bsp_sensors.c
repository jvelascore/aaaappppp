/**************************************************************************************************
  Revised:        $Date: 03-07-2015 $
  Revision:       $Revision: 1 $

  XXX-albarc      Este archivo contiene las funciones encargadas de hacer las medidas con
                  los diferentes sensores de la placa Prometeo. Portado y modificado
                  partiendo del archivo medir.c de los proyectos originales de prometeo
                  con la HAL.
**************************************************************************************************/

#include "bsp_board_defs.h"
#include "bsp_digio.h"
#include "bsp_sensors.h"

//----------------------------------------------------------------------------------
//  Variables usadas en este archivo
//----------------------------------------------------------------------------------
static volatile uint8_t SDA_fall;
uint8_t *receive_field;
static uint8_t storeTemp[2] = { 0xff, 0xff };
static uint8_t storeHum[2] = { 0xff, 0xff };
static uint32_t numVueltasAnemometro = 0;

const uint16_t direccionesVeleta[16] = {DIR_1, DIR_2, DIR_3, DIR_4, DIR_5, DIR_6, DIR_7, DIR_8, DIR_9, DIR_10, DIR_11, DIR_12, DIR_13, DIR_14, DIR_15, DIR_16};
uint16_t repeticionesDireccionesVeleta[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//----------------------------------------------------------------------------------
//  void start_transmission_SHT75 (void)
//
//  17_12 Alba
//
//  DESCRIPTION:
//    Enviar una secuencia de transmission_start.
//----------------------------------------------------------------------------------
void start_transmission_SHT75 (void)
{
  SHT75_PORT_DIR |= SHT75_SCK + SHT75_SDA;
  
  SHT75_PORT_OUT &= ~SHT75_SCK;  // t=0, SCK=0, SDA=1
  SHT75_PORT_OUT |= SHT75_SDA; 
  BSP_DELAY_USECS(5);
  
  SHT75_PORT_OUT |= SHT75_SCK; // t=5, SCK=1, SDA=1
  BSP_DELAY_USECS(5);
  
  SHT75_PORT_OUT &= ~SHT75_SDA;  //t=10, SCK=1, SDA=0
  BSP_DELAY_USECS(5);
  
  SHT75_PORT_OUT &= ~SHT75_SCK;  // t=15, SCK=0, SDA=0
  BSP_DELAY_USECS(5);
  
  SHT75_PORT_OUT |= SHT75_SCK; // t=20, SCK=1, SDA=0
  BSP_DELAY_USECS(5);
  
  SHT75_PORT_OUT |= SHT75_SDA; // t=25, SCK=1, SDA=1
  BSP_DELAY_USECS(5);
  
  SHT75_PORT_OUT &= ~SHT75_SCK;  // t=30, SCK=0, SDA=1
}

//----------------------------------------------------------------------------------
//  void reset_connection_SHT75 (void)
//
//  17_12 Alba
//
//  DESCRIPTION:
//    Enviar una secuencia de transmission_start.
//----------------------------------------------------------------------------------
void reset_connection_SHT75 (void)
{
  // Toggle SCK 9 or more times while DATA is high
  SHT75_PORT_DIR |= SHT75_SCK + SHT75_SDA;
  
  SHT75_PORT_OUT &= ~SHT75_SCK;
  SHT75_PORT_OUT |= SHT75_SDA;
  BSP_DELAY_USECS(5);

  // A for loop would introduce an extra delay per toggle.
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
}

//----------------------------------------------------------------------------------
//  void read_temperature_SHT75 (void)
//
//  17_12 Alba
//
//  DESCRIPTION:
//    Enviar una secuencia de leer temperatura.
//----------------------------------------------------------------------------------
void read_temperature_SHT75 (void)
{
  bspIState_t intState;
  // 000-00011 while toggling the SCK, and a 9th SCK to wait for the ACK
  SHT75_PORT_DIR |= SHT75_SCK + SHT75_SDA;
  SHT75_PORT_IE &= ~(SHT75_SCK + SHT75_SDA); // disable interrupts
  
  SHT75_PORT_OUT &= ~SHT75_SCK;
  SHT75_PORT_OUT &= ~SHT75_SDA;
  BSP_DELAY_USECS(5);
  
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);  // first three SCK pulses (DATA=000, address)
  
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK; // 000
  
  BSP_DELAY_USECS(4);
  SHT75_PORT_OUT |= SHT75_SDA; // 1, data has to be stable before the SCK goes high
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT |= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT &= ~SHT75_SCK;
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT &= ~SHT75_SDA;  // and last after the SCK returns to zero.
  
  BSP_DELAY_USECS(3);
  SHT75_PORT_OUT |= SHT75_SDA;
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT |= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT &= ~SHT75_SCK;
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT &= ~SHT75_SDA;
  
  BSP_DELAY_USECS(4);
  SHT75_PORT_DIR &= ~SHT75_SDA;
  SHT75_PORT_OUT |= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT &= ~SHT75_SCK;
  
  // The sensor has control over the DATA line
  // enable interruption on SDA port and wait for it to be zero
  BSP_DigioIntSetEdge(&pinSHT75_SDA, BSP_DIGIO_INT_FALLING_EDGE);
  BSP_DigioIntConnect(&pinSHT75_SDA, &SHT75_SDA_line_ISR);
  BSP_DigioIntEnable(&pinSHT75_SDA);
  SDA_fall = FALSE;
  
  BSP_ENTER_CRITICAL_SECTION(intState);
  while(!SDA_fall)
  {
      __low_power_mode_3();
      BSP_ENTER_CRITICAL_SECTION(intState);
  }
  BSP_EXIT_CRITICAL_SECTION(intState);
  
  read_2bytes_SHT75(storeTemp);
}

//----------------------------------------------------------------------------------
//  void read_humidity_SHT75 (void)
//
//  17_12 Alba
//
//  DESCRIPTION:
//    Enviar una secuencia de leer humedad.
//----------------------------------------------------------------------------------
void read_humidity_SHT75 (void)
{  
  bspIState_t intState;
  // 000-00101 while toggling the SHT75_SCK, and a 9th SCK to wait for the ACK
  SHT75_PORT_DIR |= SHT75_SCK + SHT75_SDA;
  SHT75_PORT_IE &= ~(SHT75_SCK + SHT75_SDA); // disable interrupts
  
  SHT75_PORT_OUT &= ~SHT75_SCK;
  SHT75_PORT_OUT &= ~SHT75_SDA;
  BSP_DELAY_USECS(5);
  
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);  // first three SCK pulses (DATA=000, address)
  
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK; // 00
  
  BSP_DELAY_USECS(4);
  SHT75_PORT_OUT |= SHT75_SDA; // 1, data has to be stable before the SCK goes high
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT |= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT &= ~SHT75_SCK;
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT &= ~SHT75_SDA;  // and last after the SCK returns to zero.
  
  BSP_DELAY_USECS(4);  // 0
  SHT75_PORT_OUT |= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT &= ~SHT75_SCK;
  
  BSP_DELAY_USECS(4);
  SHT75_PORT_OUT |= SHT75_SDA; // 1, data has to be stable before the SCK goes high
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT |= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT &= ~SHT75_SCK;
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT &= ~SHT75_SDA;  // and last after the SCK returns to zero.
  
  BSP_DELAY_USECS(4);
  SHT75_PORT_DIR &= ~SHT75_SDA;
  SHT75_PORT_OUT |= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT &= ~SHT75_SCK;
  
  // The sensor has control over the DATA line
  // enable interruption on SDA port and wait for it to be zero
  BSP_DigioIntSetEdge(&pinSHT75_SDA, BSP_DIGIO_INT_FALLING_EDGE);
  BSP_DigioIntConnect(&pinSHT75_SDA, &SHT75_SDA_line_ISR);
  BSP_DigioIntEnable(&pinSHT75_SDA);
  SDA_fall = FALSE;
  
  BSP_ENTER_CRITICAL_SECTION(intState);
  while(!SDA_fall)
  {
      __low_power_mode_3();
      BSP_ENTER_CRITICAL_SECTION(intState);
  }
  BSP_EXIT_CRITICAL_SECTION(intState);
  
  read_2bytes_SHT75(storeHum);
}

//----------------------------------------------------------------------------------
//  void read_2bytes_SHT75 (uint8_t *field)
//
//  17_12 Alba
//
//  DESCRIPTION:
//    Leer 2 bytes del spi.
//----------------------------------------------------------------------------------
void read_2bytes_SHT75 (uint8_t *field)
{
  receive_field = field;
  uint8_t aux = 0x00;
  uint8_t data_in1 = 0x00;
  uint8_t data_in2 = 0x00;
  
  SHT75_PORT_IE &= ~SHT75_SDA;
  SHT75_PORT_DIR |= SHT75_SCK; // controlled by micro
  SHT75_PORT_OUT &= ~SHT75_SCK;
  SHT75_PORT_DIR &= ~SHT75_SDA;  // controlled by sensor
  BSP_DELAY_USECS(4);
  
  //read data immediately before or after pulling SCK up
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in1 = (data_in1 | aux);
  data_in1 = (data_in1 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK; // 8 SCK pulses
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in1 = (data_in1 | aux);
  data_in1 = (data_in1 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in1 = (data_in1 | aux);
  data_in1 = (data_in1 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in1 = (data_in1 | aux);
  data_in1 = (data_in1 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in1 = (data_in1 | aux);
  data_in1 = (data_in1 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in1 = (data_in1 | aux);
  data_in1 = (data_in1 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in1 = (data_in1 | aux);
  data_in1 = (data_in1 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in1 = (data_in1 | aux);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(4);
  
  SHT75_PORT_DIR |= SHT75_SDA;
  BSP_DELAY_USECS(3);
  SHT75_PORT_OUT &= ~SHT75_SDA;  // ACK - low
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(1);
  SHT75_PORT_DIR &= ~SHT75_SDA;  // DATA back to sensor.
  
  BSP_DELAY_USECS(4);
  
  // start reading second byte
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in2 = (data_in2 | aux);
  data_in2 = (data_in2 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK; // 8 SCK pulses
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in2 = (data_in2 | aux);
  data_in2 = (data_in2 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in2 = (data_in2 | aux);
  data_in2 = (data_in2 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in2 = (data_in2 | aux);
  data_in2 = (data_in2 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in2 = (data_in2 | aux);
  data_in2 = (data_in2 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in2 = (data_in2 | aux);
  data_in2 = (data_in2 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in2 = (data_in2 | aux);
  data_in2 = (data_in2 << 1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  
  if (SHT75_PORT_IN & SHT75_SDA) {
    aux = 0x01;  // 1 in the last position
  } else {
    aux = 0x00;  // 0 in the last position
  }
  data_in2 = (data_in2 | aux);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(4);
  
  SHT75_PORT_OUT |= SHT75_SDA;  // skip ACK, no CRC
  SHT75_PORT_DIR |= SHT75_SDA;
  BSP_DELAY_USECS(1);
  SHT75_PORT_OUT ^= SHT75_SCK;
  BSP_DELAY_USECS(5);
  SHT75_PORT_OUT ^= SHT75_SCK;
  
  *receive_field = data_in1;
  receive_field++;
  *receive_field = data_in2;
}    

//----------------------------------------------------------------------------------
//  int SHT75_medirTemperatura (void)
//
//  17_12 Alba
//
//  DESCRIPTION:
//    Medir la temperatura y devolverla. En int y multiplicada por 10 (27.6ºC = 276)
//----------------------------------------------------------------------------------
int SHT75_medirTemperatura (void)
{
  int temp;
  uint16_t temp16;  
  int d1 = -3965; // from datasheet. Multiplied by 100 to keep them integer
  int d2 = 1;
  
  start_transmission_SHT75();
  BSP_DELAY_USECS(500);
  read_temperature_SHT75();
  
  temp16 = ((((uint16_t)storeTemp[0]) << 8) | storeTemp[1]) & 0x3fff; //14 bits
  temp = ((((int)temp16) * d2) + d1) / 10;  // Solo divido por 10 (en vez de por 100), para darlo en decigrados
  //temp = temp + 2732; // Pasar a Kelvin
  return temp;
}

//----------------------------------------------------------------------------------
//  uint16_t SHT75_medirHumedad (void)
//
//  17_12 Alba
//
//  DESCRIPTION:
//    Medir la humedad y devolverla. En uint y multiplicada por 10 (24% = 240)
//----------------------------------------------------------------------------------
uint16_t SHT75_medirHumedad (void)
{
  uint16_t hum;
  uint16_t hum16;
  double hum16Double;
  
  double c1 = -2.0468;
  double c2 = 0.0367;
  double c3 = -0.0000015955;
  
  start_transmission_SHT75();
  BSP_DELAY_USECS(500);
  read_humidity_SHT75();
  
  hum16 = ((((uint16_t)storeHum[0]) << 8) | storeHum[1]) & 0x0fff; //12 bits
  hum16Double = (double)hum16;
  
  hum = (uint16_t)((c1 + (c2 * hum16Double) + (c3 * (hum16Double * hum16Double))) * 10);  // Multiplicado por 10 
  
  return hum;
}

//----------------------------------------------------------------------------------
//  void initAnemometro (void)
//
//  06_07_15 Alba
//
//  DESCRIPTION:
//    Habilitar la interrupcion y conectar la ISR correspondiente al anemómetro
//----------------------------------------------------------------------------------
void initAnemometro (void)
{
  BSP_DigioIntSetEdge(&pinAnemometro, BSP_DIGIO_INT_FALLING_EDGE);
  BSP_DigioIntConnect(&pinAnemometro, &anemometro_ISR);
  BSP_DigioIntEnable(&pinAnemometro);
}

//----------------------------------------------------------------------------------
//  uint16_t calcularVelocidadAnemometro (uint32_t segundos)
//
//  11_02 Alba
//
//  DESCRIPTION:
//    Calcular la velocidad del viento y devolverla en km/h
//----------------------------------------------------------------------------------
uint16_t calcularVelocidadAnemometro (uint32_t segundos)
{
  uint16_t vel;
  double velDouble;
  
  double constant = 3.621;
    
  velDouble = (constant * (double)numVueltasAnemometro)/((double)segundos);
  
  numVueltasAnemometro = 0; // Resetear para que empiece a contar desde cero hasta la siguiente
  
  vel = (uint16_t)(velDouble + 0.5);  // El 0.5 es para que el redondeo sea al entero + cercano por arriba o abajo
   
  return vel;
}

//----------------------------------------------------------------------------------
//  uint16_t calcularDireccionVeleta (void)
//
//  11_02 Alba
//
//  DESCRIPTION:
//    Calcular la direccion leyendo el adc
//----------------------------------------------------------------------------------
uint16_t calcularDireccionVeleta (void)
{
  uint16_t grados = 0;
  double gradosDouble;
  uint16_t valorADC;
  bspIState_t intState;
  
  ADC10CTL0 = ADC10SHT_2 + ADC10ON + ADC10IE; // ADC10ON, interrupt enabled
  ADC10CTL1 = PIN_VELETA_ADC_CH;              // Canal A7
  ADC10AE0 |= PIN_VELETA_BITMASK;             // Canal A7 habilitado como entrada analógica
  
  BSP_ENTER_CRITICAL_SECTION(intState);       // Deshabilitar interrupciones generales
  ADC10CTL0 |= ENC + ADC10SC;                 // Empezar conversión
  __low_power_mode_1();                       // Solo vale este modo para poder despertarse por el ADC
  valorADC = ADC10MEM;
  BSP_EXIT_CRITICAL_SECTION(intState); 
  
  ADC10CTL0 &= ~ADC10ON;                      // Apagar el ADC
  
  // Regla de tres, el máximo valor del ADC de 10 bits es 1024
  gradosDouble = ((double)360 * (double)valorADC)/(double)1024;
  
  if ((gradosDouble > LIM_SECTOR_16) || (gradosDouble <= LIM_SECTOR_1)) {
    grados = DIR_1; // (N)
    repeticionesDireccionesVeleta[0]++;
  } else if ((gradosDouble > LIM_SECTOR_1) && (gradosDouble <= LIM_SECTOR_2)) {
    grados = DIR_2;  // (NNE)
    repeticionesDireccionesVeleta[1]++;
  } else if ((gradosDouble > LIM_SECTOR_2) && (gradosDouble <= LIM_SECTOR_3)) {
    grados = DIR_3;  // (NE)
    repeticionesDireccionesVeleta[2]++;
  } else if ((gradosDouble > LIM_SECTOR_3) && (gradosDouble <= LIM_SECTOR_4)) {
    grados = DIR_4;  // (ENE)
    repeticionesDireccionesVeleta[3]++;
  } else if ((gradosDouble > LIM_SECTOR_4) && (gradosDouble <= LIM_SECTOR_5)) {
    grados = DIR_5;  // (E)
    repeticionesDireccionesVeleta[4]++;
  } else if ((gradosDouble > LIM_SECTOR_5) && (gradosDouble <= LIM_SECTOR_6)) {
    grados = DIR_6; // (ESE)
    repeticionesDireccionesVeleta[5]++;
  } else if ((gradosDouble > LIM_SECTOR_6) && (gradosDouble <= LIM_SECTOR_7)) {
    grados = DIR_7; // (SE)
    repeticionesDireccionesVeleta[6]++;
  } else if ((gradosDouble > LIM_SECTOR_7) && (gradosDouble <= LIM_SECTOR_8)) {
    grados = DIR_8; // (SSE)
    repeticionesDireccionesVeleta[7]++;
  } else if ((gradosDouble > LIM_SECTOR_8) && (gradosDouble <= LIM_SECTOR_9)) {
    grados = DIR_9; // (S)
    repeticionesDireccionesVeleta[8]++;
  } else if ((gradosDouble > LIM_SECTOR_9) && (gradosDouble <= LIM_SECTOR_10)) {
    grados = DIR_10; // (SSO)
    repeticionesDireccionesVeleta[9]++;
  } else if ((gradosDouble > LIM_SECTOR_10) && (gradosDouble <= LIM_SECTOR_11)) {
    grados = DIR_11; // (SO)
    repeticionesDireccionesVeleta[10]++;
  } else if ((gradosDouble > LIM_SECTOR_11) && (gradosDouble <= LIM_SECTOR_12)) {
    grados = DIR_12; // (OSO)
    repeticionesDireccionesVeleta[11]++;
  } else if ((gradosDouble > LIM_SECTOR_12) && (gradosDouble <= LIM_SECTOR_13)) {
    grados = DIR_13; // (O)
    repeticionesDireccionesVeleta[12]++;
  } else if ((gradosDouble > LIM_SECTOR_13) && (gradosDouble <= LIM_SECTOR_14)) {
    grados = DIR_14; // (ONO)
    repeticionesDireccionesVeleta[13]++;
  } else if ((gradosDouble > LIM_SECTOR_14) && (gradosDouble <= LIM_SECTOR_15)) {
    grados = DIR_15; // (NO)
    repeticionesDireccionesVeleta[14]++;
  } else if ((gradosDouble > LIM_SECTOR_15) && (gradosDouble <= LIM_SECTOR_16)) {
    grados = DIR_16; // (NNO)
    repeticionesDireccionesVeleta[15]++;
  }
  
  return grados;
}

//----------------------------------------------------------------------------------
//  void resetearRepeticionesVeleta (void) {
//
//  12_02 Alba
//
//  DESCRIPTION:
//    Poner a 0 todo el array de direcciones de la veleta
//----------------------------------------------------------------------------------
void resetearRepeticionesVeleta (void) {
  for(int i = 0; i < NUM_DIRECCIONES_VELETA; i++) {
    repeticionesDireccionesVeleta[i] = 0;
  }
}

//----------------------------------------------------------------------------------
//  uint16_t calcularDireccionMasRepetida (void) {
//
//  12_02 Alba
//
//  DESCRIPTION:
//    Calcular la direccion con más repeticiones. Cuidado de resetear el array
//    tras pedirla si es necesario con resetearRepeticionesVeleta.
//----------------------------------------------------------------------------------
uint16_t calcularDireccionMasRepetida (void) {
  uint16_t direccionMasRepetida = 0;
  uint16_t indexMasRepetido = 0;
  uint16_t maxCuenta = 0;
  
  for(int i = 0; i < NUM_DIRECCIONES_VELETA; i++) {
    if (repeticionesDireccionesVeleta[i] > maxCuenta) {
      maxCuenta = repeticionesDireccionesVeleta[i];
      indexMasRepetido = i;
    }
  }
  
  direccionMasRepetida = direccionesVeleta[indexMasRepetido];
  
  return direccionMasRepetida;
}

//----------------------------------------------------------------------------------
//  void SHT75_SDA_line_ISR (void)
//
//  17_12 Alba
//
//  DESCRIPTION:
//    ISR de interrupcion por la linea SDA.
//----------------------------------------------------------------------------------
static void SHT75_SDA_line_ISR (void)
{  
    SDA_fall = TRUE;
}

//----------------------------------------------------------------------------------
//  void anemometro_ISR (void)
//
//  06_07_15 Alba
//
//  DESCRIPTION:
//    ISR de interrupcion del anemómetro.
//----------------------------------------------------------------------------------
static void anemometro_ISR (void)
{  
    numVueltasAnemometro++;
}

//----------------------------------------------------------------------------------
//  void BSP_ADC10_ISR(void)
//
//  11_02 Alba
//
//  DESCRIPTION:
//    ISR del ADC10. Simplemente sale del modo de bajo consumo.
//----------------------------------------------------------------------------------
BSP_ISR_FUNCTION( BSP_ADC10_ISR, ADC10_VECTOR )
{
  __low_power_mode_off_on_exit();
}