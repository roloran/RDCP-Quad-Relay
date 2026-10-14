#ifndef _LORA_868_ROUTE
#define _LORA_868_ROUTE

#include <stdint.h>

enum class Radio868 : uint8_t {
  A,
  B,
  C,
  Count
};

class Lora868Route {
public:
  Lora868Route(uint8_t da_channel, uint8_t mg_channel, uint8_t lw_channel);

  uint8_t channel(Radio868 radio) const;
  Radio868 radio(uint8_t channel) const;
  Radio868 da_radio() const;
  Radio868 next_da_radio() const;
  bool has_channel(uint8_t channel) const;
  void rotate_da();

private:
  static constexpr uint8_t RADIO_COUNT = static_cast<uint8_t>(Radio868::Count);

  uint8_t da_channel;
  uint8_t channels[RADIO_COUNT];
};

#endif
