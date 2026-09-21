#include <gtest/gtest.h>

#include "micras/sim/core/scenario.hpp"

namespace micras::sim {
namespace {
constexpr ButtonDelays firmware_delays{.long_press = 500, .extra_long_press = 2000};

TEST(Scenario, ParsesTheNamesTheCommandLineUses) {
    EXPECT_EQ(parse_command("none"), Command::NONE);
    EXPECT_EQ(parse_command("explore"), Command::EXPLORE);
    EXPECT_EQ(parse_command("solve"), Command::SOLVE);
    EXPECT_EQ(parse_command("calibrate"), Command::CALIBRATE);
    EXPECT_EQ(parse_command("Explore"), std::nullopt);
    EXPECT_EQ(parse_command(""), std::nullopt);

    EXPECT_EQ(parse_button_press("none"), ButtonPress::NONE);
    EXPECT_EQ(parse_button_press("short"), ButtonPress::SHORT);
    EXPECT_EQ(parse_button_press("long"), ButtonPress::LONG);
    EXPECT_EQ(parse_button_press("extra_long"), ButtonPress::EXTRA_LONG);
    EXPECT_EQ(parse_button_press("extralong"), std::nullopt);
}

TEST(Scenario, AddressesEachCommandThroughItsPoolVariable) {
    EXPECT_EQ(command_variable(Command::EXPLORE), "Explore");
    EXPECT_EQ(command_variable(Command::SOLVE), "Solve");
    EXPECT_EQ(command_variable(Command::CALIBRATE), "Calibrate");
    EXPECT_TRUE(command_variable(Command::NONE).empty());
}

TEST(Scenario, HoldsTheButtonPastEachClassificationThreshold) {
    EXPECT_EQ(Scenario::hold_time_ms(ButtonPress::NONE, firmware_delays), 0U);
    EXPECT_EQ(Scenario::hold_time_ms(ButtonPress::SHORT, firmware_delays), 250U);
    EXPECT_EQ(Scenario::hold_time_ms(ButtonPress::LONG, firmware_delays), 501U);
    EXPECT_EQ(Scenario::hold_time_ms(ButtonPress::EXTRA_LONG, firmware_delays), 2001U);
}
}  // namespace
}  // namespace micras::sim
