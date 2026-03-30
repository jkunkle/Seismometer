#include <avr/io.h>
#include <stdint.h>
#include <stddef.h>

// Include SdFat types (SdSpiConfig, etc.)
#include "ArduinoCompat.h"
#include "SdFat.h"
#include "SdCard/SdSpiCard/SpiDriver/SdSpiBaseClass.h"


// --- Choose your CS pin here (PB2 is Arduino D10 / SS) ---
static constexpr uint8_t CS_BIT  = PB2;
static inline void cs_high() { PORTB |=  (1 << CS_BIT); }
static inline void cs_low()  { PORTB &= ~(1 << CS_BIT); }

// Very small helper: SPI transfer one byte
static inline uint8_t spi_xfer(uint8_t v) {
  SPDR = v;
  while (!(SPSR & (1 << SPIF))) {}
  return SPDR;
}

// External driver required by SPI_DRIVER_SELECT=3
class AvrSdSpiDriver : public SdSpiBaseClass {
 public:
  void begin(SdSpiConfig config) override {
    (void)config;  // we hardwire CS pin below; you can map config.csPin if desired

    // Set MOSI (PB3), SCK (PB5), CS (PB2) as outputs. MISO (PB4) as input.
    DDRB |= (1 << DDB2) | (1 << DDB3) | (1 << DDB5);
    DDRB &= ~(1 << DDB4);

    // Deassert CS
    cs_high();

    // Enable SPI, Master, mode 0, MSB first.
    // Start slow for SD init: f_SCK = F_CPU/128 (safe)
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
    SPSR &= ~(1 << SPI2X);
  }

  void activate() override {}
  void deactivate() override {}

  uint8_t receive() override {
    return spi_xfer(0xFF);
  }

  uint8_t receive(uint8_t* buf, size_t count) override {
    for (size_t i = 0; i < count; i++) {
      buf[i] = spi_xfer(0xFF);
    }
    return 0;  // 0 = no error
  }

  void send(uint8_t data) override {
    (void)spi_xfer(data);
  }

  void send(const uint8_t* buf, size_t count) override {
    for (size_t i = 0; i < count; i++) {
      (void)spi_xfer(buf[i]);
    }
  }

  // After init, SdFat may call this to raise SCK speed.
  void setSckSpeed(uint32_t maxSck) override {
    // Simple mapping: choose best divider <= maxSck.
    // F_SCK = F_CPU / div, div in {2,4,8,16,32,64,128}
    // You can improve this, but this is a solid start.
    uint32_t f = F_CPU;

    uint8_t spcr = (1 << SPE) | (1 << MSTR);
    uint8_t spsr = 0;

    auto set_div = [&](uint16_t div) {
      // Set SPR1:SPR0 and SPI2X for divider
      // div:2 => SPI2X=1, SPR=00
      // div:4 => SPI2X=0, SPR=00
      // div:8 => SPI2X=1, SPR=01
      // div:16=> SPI2X=0, SPR=01
      // div:32=> SPI2X=1, SPR=10
      // div:64=> SPI2X=0, SPR=10
      // div:128=>SPI2X=0, SPR=11
      switch (div) {
        case 2:   spsr = (1 << SPI2X); spcr |= 0; break;
        case 4:   spsr = 0;            spcr |= 0; break;
        case 8:   spsr = (1 << SPI2X); spcr |= (1 << SPR0); break;
        case 16:  spsr = 0;            spcr |= (1 << SPR0); break;
        case 32:  spsr = (1 << SPI2X); spcr |= (1 << SPR1); break;
        case 64:  spsr = 0;            spcr |= (1 << SPR1); break;
        default:  spsr = 0;            spcr |= (1 << SPR1) | (1 << SPR0); break; // 128
      }
    };

    uint16_t div = 128;
    // Pick fastest divider that does not exceed maxSck
    static const uint16_t divs[] = {2,4,8,16,32,64,128};
    for (uint8_t i = 0; i < 7; i++) {
      uint16_t d = divs[i];
      if ((f / d) <= maxSck) { div = d; break; }
    }
    set_div(div);

    SPCR = spcr;
    SPSR = (SPSR & ~(1 << SPI2X)) | spsr;
  }
};

// Provide a global instance SdFat can use (your SdFat usage will reference this)
AvrSdSpiDriver sdSpi;
