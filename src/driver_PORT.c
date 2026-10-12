#include "driver_PORT.h"
#include "emonTH_saml.h"

void portPinCfg(unsigned int pin, unsigned int cfg, PINCFG_t cs) {
  if (PIN_CFG_SET == cs) {
    PORT->Group[0].PINCFG[pin].reg |= cfg;
  } else {
    PORT->Group[0].PINCFG[pin].reg &= ~cfg;
  }
}

void portPinDir(unsigned int pin, PINDIR_t mode) {
  if (PIN_DIR_IN == mode) {
    PORT->Group[0].DIRCLR.reg = (1u << pin);
    PORT->Group[0].PINCFG[pin].reg |= PORT_PINCFG_INEN;
  } else {
    PORT->Group[0].DIRSET.reg = (1u << pin);
  }
}

void portPinDrv(size_t pin, PINDRV_t drv) {
  switch (drv) {
  case PIN_DRV_CLR:
    PORT->Group[0].OUTCLR.reg = (1u << pin);
    break;
  case PIN_DRV_SET:
    PORT->Group[0].OUTSET.reg = (1u << pin);
    break;
  case PIN_DRV_TGL:
    PORT->Group[0].OUTTGL.reg = (1u << pin);
    break;
  }
}

void portPinMux(unsigned int pin, unsigned int mux) {
  PORT->Group[0].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;
  if (pin & 1u) {
    PORT->Group[0].PMUX[pin >> 1].bit.PMUXO = mux;
  } else {
    PORT->Group[0].PMUX[pin >> 1].bit.PMUXE = mux;
  }
}

void portPinMuxClear(unsigned int pin) {
  PORT->Group[0].PINCFG[pin].reg &= ~PORT_PINCFG_PMUXEN;
}

unsigned int portPinValue(unsigned int pin) {
  unsigned int ret;
  ret = (0u == (PORT->Group[0].IN.reg & (1u << pin))) ? 0u : 1u;
  return ret;
}

void portSetup(void) {
  extern const uint8_t pinsGPIO_Out[];
  extern const uint8_t pinsGPIO_In[];
  extern const uint8_t pinsUnused[];
  extern const uint8_t pinExt5VSense;
  extern const uint8_t pinUartRx;

  /* GPIO outputs - also enable read buffer */
  for (unsigned int i = 0; pinsGPIO_Out[i] != 0xFF; i++) {
    portPinDir(pinsGPIO_Out[i], PIN_DIR_OUT);
    portPinCfg(pinsGPIO_Out[i], PORT_PINCFG_INEN, PIN_CFG_SET);
  }

  /* GPIO inputs  - most inputs currently need pull ups, so default enable */
  for (unsigned int i = 0; pinsGPIO_In[i] != 0xFF; i++) {
    portPinDir(pinsGPIO_In[i], PIN_DIR_IN);
    portPinCfg(pinsGPIO_In[i], PORT_PINCFG_PULLEN, PIN_CFG_SET);
    portPinCfg(pinsGPIO_In[i], PORT_PINCFG_INEN, PIN_CFG_SET);
    portPinDrv(pinsGPIO_In[i], PIN_DRV_SET);
  }

  /* 5V external sense has an external pull down */
  portPinDir(pinExt5VSense, PIN_DIR_IN);
  portPinCfg(pinExt5VSense, PORT_PINCFG_INEN, PIN_CFG_SET);

  /* Pull up the UART Rx line to avoid spurious read event at power on */
  portPinCfg(pinUartRx, PORT_PINCFG_PULLEN, PIN_CFG_SET);
  portPinDrv(pinUartRx, PIN_DRV_SET);

  /* Unused pins: input, pull down (Table 23-2) */
  for (unsigned int i = 0; pinsUnused[i] != 0xFF; i++) {
    portPinDir(pinsUnused[i], PIN_DIR_IN);
    portPinCfg(pinsUnused[i], PORT_PINCFG_PULLEN, PIN_CFG_SET);
  }
}
