#include "lora-868-route.h"

Lora868Route::Lora868Route(uint8_t da_channel, uint8_t mg_channel, uint8_t lw_channel)
  : da_channel(da_channel), channels{da_channel, mg_channel, lw_channel}
{
}

uint8_t Lora868Route::channel(Radio868 radio) const
{
  return channels[static_cast<uint8_t>(radio)];
}

Radio868 Lora868Route::radio(uint8_t channel) const
{
  for (uint8_t i = 0; i < RADIO_COUNT; i++)
  {
    if (channels[i] == channel)
    {
      return static_cast<Radio868>(i);
    }
  }

  return Radio868::Count;
}

Radio868 Lora868Route::da_radio() const
{
  return radio(da_channel);
}

Radio868 Lora868Route::next_da_radio() const
{
  uint8_t current = static_cast<uint8_t>(da_radio());
  uint8_t next = (current + 1) % RADIO_COUNT;

  return static_cast<Radio868>(next);
}

bool Lora868Route::has_channel(uint8_t channel) const
{
  return radio(channel) != Radio868::Count;
}

void Lora868Route::rotate_da()
{
  Radio868 current = da_radio();
  Radio868 next = next_da_radio();
  uint8_t current_idx = static_cast<uint8_t>(current);
  uint8_t next_idx = static_cast<uint8_t>(next);
  uint8_t displaced = channels[next_idx];

  channels[current_idx] = displaced;
  channels[next_idx] = da_channel;
}
