/**************************************************************************************************
  Revised:        $Date: 05-02-2015 $
  Revision:       $Revision: 1 $
  
  XXX-albarc
**************************************************************************************************/

/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Digio include file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_DIGIO_H
#define BSP_DIGIO_H

enum {
    BSP_DIGIO_INPUT,
    BSP_DIGIO_OUTPUT
};

enum {
    BSP_DIGIO_INT_FALLING_EDGE,
    BSP_DIGIO_INT_RISING_EDGE
};


typedef struct {
    uint8_t port;     // port number
    uint8_t pin;      // pin number
    uint8_t pin_bm;   // pin bitmask
    uint8_t dir;      // direction (input or output)
    uint8_t initval;  // initial value
} digioConfig;

typedef void (*BSP_ISR_FUNC_PTR)(void);


#define BSP_DIGIO_OK       0
#define BSP_DIGIO_ERROR  (~0)

/* ------------------------------------------------------------------------------------------------
 *                                        Defines para los GPIO
 * ------------------------------------------------------------------------------------------------
 */
#define PIN_CONO_PORT       2
#define PIN_CONO_PIN        5
#define PIN_CONO_BITMASK    BIT5
#define PIN_CONO_DIR        BSP_DIGIO_INPUT
#define PIN_CONO_INIT_VAL   0

#define PIN_SHT75_SCK_PORT       2
#define PIN_SHT75_SCK_PIN        3
#define PIN_SHT75_SCK_BITMASK    BIT3
#define PIN_SHT75_SCK_DIR        BSP_DIGIO_OUTPUT
#define PIN_SHT75_SCK_INIT_VAL   0

#define PIN_SHT75_SDA_PORT       2
#define PIN_SHT75_SDA_PIN        4
#define PIN_SHT75_SDA_BITMASK    BIT4
#define PIN_SHT75_SDA_DIR        BSP_DIGIO_OUTPUT
#define PIN_SHT75_SDA_INIT_VAL   1

#define PIN_ANEMOMETRO_PORT       2
#define PIN_ANEMOMETRO_PIN        2
#define PIN_ANEMOMETRO_BITMASK    BIT2
#define PIN_ANEMOMETRO_DIR        BSP_DIGIO_INPUT
#define PIN_ANEMOMETRO_INIT_VAL   0

#define PIN_VELETA_PORT       3
#define PIN_VELETA_PIN        7
#define PIN_VELETA_BITMASK    BIT7
#define PIN_VELETA_DIR        BSP_DIGIO_INPUT
#define PIN_VELETA_INIT_VAL   0
#define PIN_VELETA_PxSEL      P3SEL
#define PIN_VELETA_ADC_CH     INCH_7

/* ------------------------------------------------------------------------------------------------
 *                                        Variables externas
 * ------------------------------------------------------------------------------------------------
 */
extern const digioConfig pinCono;
extern const digioConfig pinSHT75_SCK;
extern const digioConfig pinSHT75_SDA;
extern const digioConfig pinAnemometro;
extern const digioConfig pinVeleta;

extern BSP_ISR_FUNC_PTR port1_isr_tbl[8];
extern BSP_ISR_FUNC_PTR port2_isr_tbl[8];

/* ------------------------------------------------------------------------------------------------
 *                                        Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void BSP_InitDigio(void);
uint8_t BSP_ConfigDigio(const digioConfig* p);
uint8_t BSP_DigioClear(const digioConfig* p);
uint8_t BSP_DigioToggle(const digioConfig* p);
uint8_t BSP_DigioSet(const digioConfig* p);
uint8_t BSP_DigioGet(const digioConfig* p);

uint8_t BSP_DigioIntConnect(const digioConfig *p, BSP_ISR_FUNC_PTR func);
uint8_t BSP_DigioIntEnable(const digioConfig *p);
uint8_t BSP_DigioIntDisable(const digioConfig *p);
uint8_t BSP_DigioIntClear(const digioConfig *p);
uint8_t BSP_DigioIntSetEdge(const digioConfig *p, uint8_t edge);

/**********************************************************************************/
#endif