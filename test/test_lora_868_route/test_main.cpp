#include <unity.h>

#include "lora-868-route.h"

namespace {

constexpr uint8_t CHANNEL_DA = 1;
constexpr uint8_t CHANNEL_MG = 2;
constexpr uint8_t CHANNEL_LW = 3;

Lora868Route make_route()
{
  return Lora868Route(CHANNEL_DA, CHANNEL_MG, CHANNEL_LW);
}

void test_initial_mapping()
{
  Lora868Route route = make_route();

  TEST_ASSERT_EQUAL_UINT8(CHANNEL_DA, route.channel(Radio868::A));
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_MG, route.channel(Radio868::B));
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_LW, route.channel(Radio868::C));
}

void test_da_radio_round_robins()
{
  Lora868Route route = make_route();

  TEST_ASSERT_EQUAL_INT(static_cast<int>(Radio868::A), static_cast<int>(route.da_radio()));
  route.rotate_da();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Radio868::B), static_cast<int>(route.da_radio()));
  route.rotate_da();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Radio868::C), static_cast<int>(route.da_radio()));
  route.rotate_da();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Radio868::A), static_cast<int>(route.da_radio()));
}

void test_rotation_keeps_all_channels()
{
  Lora868Route route = make_route();

  for (uint8_t i = 0; i < 12; i++)
  {
    TEST_ASSERT_TRUE(route.has_channel(CHANNEL_DA));
    TEST_ASSERT_TRUE(route.has_channel(CHANNEL_MG));
    TEST_ASSERT_TRUE(route.has_channel(CHANNEL_LW));

    route.rotate_da();
  }
}

void test_rotation_swaps_da_only()
{
  Lora868Route route = make_route();

  route.rotate_da();
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_MG, route.channel(Radio868::A));
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_DA, route.channel(Radio868::B));
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_LW, route.channel(Radio868::C));

  route.rotate_da();
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_MG, route.channel(Radio868::A));
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_LW, route.channel(Radio868::B));
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_DA, route.channel(Radio868::C));

  route.rotate_da();
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_DA, route.channel(Radio868::A));
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_LW, route.channel(Radio868::B));
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_MG, route.channel(Radio868::C));
}

}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_initial_mapping);
  RUN_TEST(test_da_radio_round_robins);
  RUN_TEST(test_rotation_keeps_all_channels);
  RUN_TEST(test_rotation_swaps_da_only);
  return UNITY_END();
}
