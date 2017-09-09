#include "sysfs.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include <pio/gpio.h>
#include <pio/peripheral_manager_client.h>

const int TDIPin = 22;
const int TMSPin = 4;
const int TCKPin = 17;
const int TDOPin = 27;
const int LEDPin = 21;

IOSysFsGPIO::IOSysFsGPIO()
    : tdi_gpio(NULL), tms_gpio(NULL), tck_gpio(NULL), tdo_gpio(NULL), client(NULL), one(1), zero(0) {

  client = APeripheralManagerClient_new();

  tdi_gpio = setup_gpio(TDIPin, 0);
  tms_gpio = setup_gpio(TMSPin, 0);
  tck_gpio = setup_gpio(TCKPin, 0);
  tdo_gpio = setup_gpio(TDOPin, 1);

  //led_gpio = setup_gpio(LEDPin, 0);
}

IOSysFsGPIO::~IOSysFsGPIO() {

  AGpio_delete(tck_gpio);
  AGpio_delete(tms_gpio);
  AGpio_delete(tdi_gpio);
  AGpio_delete(tdo_gpio);

  //AGpio_delete(led_gpio);

  APeripheralManagerClient_delete(client);
}

int IOSysFsGPIO::setupGPIOs(int tck, int tms, int tdi, int tdo) { return 1; }

void IOSysFsGPIO::txrx_block(const unsigned char *tdi, unsigned char *tdo,
                             int length, bool last) {
  int i = 0;
  int j = 0;
  unsigned char tdo_byte = 0;
  unsigned char tdi_byte;

  if (tdi) tdi_byte = tdi[j];

  while (i < length - 1) {
    tdo_byte = tdo_byte + (txrx(false, (tdi_byte & 1) == 1) << (i % 8));
    if (tdi) tdi_byte = tdi_byte >> 1;
    i++;
    if ((i % 8) == 0) {            // Next byte
      if (tdo) tdo[j] = tdo_byte;  // Save the TDO byte
      tdo_byte = 0;
      j++;
      if (tdi) tdi_byte = tdi[j];  // Get the next TDI byte
    }
  };
  tdo_byte = tdo_byte + (txrx(last, (tdi_byte & 1) == 1) << (i % 8));
  if (tdo) tdo[j] = tdo_byte;

  //write(tck_fd, zero, 1);
  AGpio_setValue(tck_gpio, zero);

  return;
}

void IOSysFsGPIO::tx_tms(unsigned char *pat, int length, int force) {
  int i;
  unsigned char tms;
  for (i = 0; i < length; i++) {
    if ((i & 0x7) == 0) tms = pat[i >> 3];
    tx((tms & 0x01), true);
    tms = tms >> 1;
  }

  //write(tck_fd, zero, 1);
  AGpio_setValue(tck_gpio, zero);
}

void IOSysFsGPIO::tx(bool tms, bool tdi) {
  //write(tck_fd, zero, 1);
  AGpio_setValue(tck_gpio, zero);

  //write(tdi_fd, tdi ? one : zero, 1);
  AGpio_setValue(tdi_gpio, tdi ? one : zero);

  //write(tms_fd, tms ? one : zero, 1);
  AGpio_setValue(tms_gpio, tms ? one : zero);

  //write(tck_fd, one, 1);
  AGpio_setValue(tck_gpio, one);

  //AGpio_setValue(led_gpio, tms ? one : zero);
}

bool IOSysFsGPIO::txrx(bool tms, bool tdi) {
  //static char buf[1];

  tx(tms, tdi);

  /*lseek(tdo_fd, 0, SEEK_SET);

  if (read(tdo_fd, &buf, sizeof(buf)) < 0) {
    // reading tdo failed
    return false;
  }
  return buf[0] != '0';
  */
  int gpioValue;
  if(AGpio_getValue(tdo_gpio, &gpioValue) != 0){
    // reading tdo failed
    std::cerr << "ERROR: Couldn't read from tdo pin " << std::endl;
    return false;
  }
  return gpioValue != 0;
}

AGpio* IOSysFsGPIO::setup_gpio(int gpio, int is_input) {

  AGpio* gpioPin;
  char gpiostr[6];
  snprintf(gpiostr, sizeof(gpiostr), "BCM%d", gpio);

  int openResult = APeripheralManagerClient_openGpio(client, gpiostr, &gpioPin);

  if (openResult != 0) {
    std::cerr << "ERROR: Couldn't export gpio " << gpio << std::endl;
    return 0;
  }

  AGpioDirection direction = is_input ? AGPIO_DIRECTION_IN : AGPIO_DIRECTION_OUT_INITIALLY_LOW;
  int setDirectionResult = AGpio_setDirection(gpioPin, direction);

  if(setDirectionResult != 0){
    std::cerr << "ERROR: Couldn't set direction for gpio " << gpio << std::endl;
    AGpio_delete(gpioPin);
    return 0;
  }

  int setActiveTypeResult = AGpio_setActiveType(gpioPin, AGPIO_ACTIVE_HIGH);
  if(setActiveTypeResult != 0){
    std::cerr << "ERROR: Couldn't set active type for gpio " << gpio << std::endl;
    AGpio_delete(gpioPin);
    return 0;
  }

  return gpioPin;
}
