#pragma once
#include "SdFat.h"

class AvrSdSpiDriver : public SdSpiBaseClass {
public:
  void begin(SdSpiConfig config) override;
  uint8_t receive() override;
  uint8_t receive(uint8_t* buf, size_t count) override;
  void send(uint8_t data) override;
  void send(const uint8_t* buf, size_t count) override;
  void setSckSpeed(uint32_t maxSck) override;
  void activate() override;
  void deactivate() override;
};

extern AvrSdSpiDriver sdSpi;
