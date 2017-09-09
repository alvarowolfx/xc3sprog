#include "iobase.h"

#include <pio/gpio.h>
#include <pio/peripheral_manager_client.h>

class IOSysFsGPIO : public IOBase {
 public:
  IOSysFsGPIO();
  int setupGPIOs(int tck, int tms, int tdi, int tdo);
  virtual ~IOSysFsGPIO();

 private:
  void tx(bool tms, bool tdi);
  bool txrx(bool tms, bool tdi);

  void txrx_block(const unsigned char *tdi, unsigned char *tdo, int length,
                  bool last);
  void tx_tms(unsigned char *pat, int length, int force);

  AGpio* setup_gpio(int gpio, int is_input);
  bool is_gpio_valid(int gpio) { return gpio >= 0 && gpio < 1000; }

 private:
  APeripheralManagerClient* client;
  AGpio* tck_gpio;
  AGpio* tms_gpio;
  AGpio* tdi_gpio;
  AGpio* tdo_gpio;
  AGpio* led_gpio;

  const int one;
  const int zero;
};
