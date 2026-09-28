/**
 * @file
 */

#include <array>
#include <cstdint>

#include <doctest/doctest.h>

#include "micras/models/as5047u_model.hpp"

namespace micras::models {
namespace {
using Frame = std::array<uint8_t, As5047uModel::frame_size>;

constexpr uint32_t read_flag{1U << 22U};
constexpr uint32_t error_flag{1U << 22U};

Frame frame_of(uint32_t upper) {
    const auto high = static_cast<uint8_t>(upper >> 16U);
    const auto low = static_cast<uint8_t>(upper >> 8U);
    return {high, low, As5047uModel::crc(high, low)};
}

Frame read_command(uint16_t address) {
    return frame_of(read_flag | static_cast<uint32_t>(address) << 8U);
}

Frame write_command(uint16_t address) {
    return frame_of(static_cast<uint32_t>(address) << 8U);
}

Frame data_frame(uint16_t data) {
    return frame_of(static_cast<uint32_t>(data) << 8U);
}

uint32_t transfer(As5047uModel& chip, const Frame& frame) {
    Frame received{};
    chip.select();
    chip.exchange(frame, received);
    chip.deselect();
    return static_cast<uint32_t>(std::get<0>(received)) << 16U | static_cast<uint32_t>(std::get<1>(received)) << 8U |
           std::get<2>(received);
}

uint16_t data_of(uint32_t answer) {
    return static_cast<uint16_t>((answer >> 8U) & 0x3FFFU);
}

bool valid(uint32_t answer) {
    return As5047uModel::crc(static_cast<uint8_t>(answer >> 16U), static_cast<uint8_t>(answer >> 8U)) ==
           static_cast<uint8_t>(answer);
}

uint16_t read_register(As5047uModel& chip, uint16_t address) {
    transfer(chip, read_command(address));
    return data_of(transfer(chip, read_command(As5047uModel::nop_address)));
}

void write_register(As5047uModel& chip, uint16_t address, uint16_t data) {
    transfer(chip, write_command(address));
    transfer(chip, data_frame(data));
}

TEST_CASE("As5047uModel.AnswersInSpiMode1") {
    const As5047uModel chip;

    CHECK_EQ(chip.mode(), hal::host::SpiDevice::Mode::MODE_1);
}

TEST_CASE("As5047uModel.ComputesTheFrameCrc") {
    CHECK_EQ(As5047uModel::crc(0x00, 0x00), 0xF1);
    CHECK_EQ(As5047uModel::crc(0x40, 0x01), 0x06);
    CHECK_EQ(As5047uModel::crc(0x40, 0x1A), 0x04);
    CHECK_EQ(As5047uModel::crc(0x00, 0x80), 0xD7);
}

TEST_CASE("As5047uModel.PowersOnWithZeroedRegistersAndAnEmptyAnswer") {
    As5047uModel chip;

    for (const uint16_t address :
         {As5047uModel::errfl_address, As5047uModel::disable_address, As5047uModel::zposm_address,
          As5047uModel::zposl_address, As5047uModel::settings1_address, As5047uModel::settings2_address,
          As5047uModel::settings3_address, As5047uModel::ecc_address}) {
        CHECK_MESSAGE(chip.peek(address) == 0, "register " << address);
    }

    CHECK_EQ(transfer(chip, read_command(As5047uModel::nop_address)), 0x0000F1U);
}

TEST_CASE("As5047uModel.AnswersEachFrameInTheNextOne") {
    As5047uModel chip;
    write_register(chip, As5047uModel::settings3_address, 0x80);
    write_register(chip, As5047uModel::zposm_address, 0x12);

    transfer(chip, read_command(As5047uModel::settings3_address));
    const uint32_t settings3 = transfer(chip, read_command(As5047uModel::zposm_address));
    const uint32_t zposm = transfer(chip, read_command(As5047uModel::nop_address));

    CHECK_EQ(data_of(settings3), 0x80);
    CHECK_EQ(data_of(zposm), 0x12);
    CHECK(valid(settings3));
    CHECK(valid(zposm));
}

TEST_CASE("As5047uModel.WritesWithACommandAndADataFrame") {
    As5047uModel chip;
    write_register(chip, As5047uModel::settings2_address, 0x04);

    transfer(chip, write_command(As5047uModel::settings2_address));
    const uint32_t old_content = transfer(chip, data_frame(0x21));
    const uint32_t new_content = transfer(chip, read_command(As5047uModel::nop_address));

    CHECK_EQ(data_of(old_content), 0x04);
    CHECK_EQ(data_of(new_content), 0x21);
    CHECK_EQ(chip.peek(As5047uModel::settings2_address), 0x21);
}

TEST_CASE("As5047uModel.HoldsEveryVolatileRegister") {
    As5047uModel chip;
    uint16_t     value = 0x11;

    for (const uint16_t address :
         {As5047uModel::disable_address, As5047uModel::zposm_address, As5047uModel::zposl_address,
          As5047uModel::settings1_address, As5047uModel::settings2_address, As5047uModel::settings3_address,
          As5047uModel::ecc_address}) {
        write_register(chip, address, value);
        CHECK_MESSAGE(read_register(chip, address) == value, "register " << address);
        value += 0x11;
    }
}

TEST_CASE("As5047uModel.IgnoresWritesToOtherAddresses") {
    As5047uModel chip;

    write_register(chip, As5047uModel::errfl_address, 0x7F);
    write_register(chip, 0x3FFF, 0x7F);

    CHECK_EQ(chip.peek(As5047uModel::errfl_address), 0);
    CHECK_EQ(read_register(chip, 0x3FFF), 0);
}

TEST_CASE("As5047uModel.RejectsAFrameWithAWrongCrc") {
    As5047uModel chip;
    Frame        command = write_command(As5047uModel::settings3_address);
    std::get<2>(command) ^= 0x01;

    transfer(chip, command);
    const uint32_t after_error = transfer(chip, data_frame(0x80));

    CHECK_EQ(chip.peek(As5047uModel::settings3_address), 0);
    CHECK_EQ(chip.peek(As5047uModel::errfl_address), As5047uModel::crc_error);
    CHECK_NE(after_error & error_flag, 0U);
    CHECK(valid(after_error));
}

TEST_CASE("As5047uModel.DropsAWriteWhoseDataFrameHasAWrongCrc") {
    As5047uModel chip;
    Frame        data = data_frame(0x80);
    std::get<2>(data) ^= 0x80;

    transfer(chip, write_command(As5047uModel::settings3_address));
    transfer(chip, data);
    transfer(chip, data_frame(0x80));

    CHECK_EQ(chip.peek(As5047uModel::settings3_address), 0);
    CHECK_EQ(chip.peek(As5047uModel::errfl_address), As5047uModel::crc_error);
}

TEST_CASE("As5047uModel.ClearsTheErrorFlagsWhenErrflIsRead") {
    As5047uModel chip;
    Frame        command = read_command(As5047uModel::settings3_address);
    std::get<0>(command) ^= 0x01;
    transfer(chip, command);

    transfer(chip, read_command(As5047uModel::errfl_address));
    const uint32_t errfl = transfer(chip, read_command(As5047uModel::nop_address));
    const uint32_t after = transfer(chip, read_command(As5047uModel::nop_address));

    CHECK_EQ(data_of(errfl), As5047uModel::crc_error);
    CHECK_NE(errfl & error_flag, 0U);
    CHECK_EQ(after & error_flag, 0U);
    CHECK_EQ(chip.peek(As5047uModel::errfl_address), 0);
}

TEST_CASE("As5047uModel.FlagsAFrameCutShortByTheChipSelect") {
    As5047uModel                 chip;
    const Frame                  command = write_command(As5047uModel::settings3_address);
    const std::array<uint8_t, 2> partial{std::get<0>(command), std::get<1>(command)};
    std::array<uint8_t, 2>       received{};

    chip.select();
    chip.exchange(partial, received);
    chip.deselect();
    transfer(chip, data_frame(0x80));

    CHECK_EQ(chip.peek(As5047uModel::settings3_address), 0);
    CHECK_EQ(chip.peek(As5047uModel::errfl_address), As5047uModel::framing_error);
}

TEST_CASE("As5047uModel.TakesAFrameSplitAcrossExchanges") {
    As5047uModel                 chip;
    const Frame                  command = write_command(As5047uModel::zposl_address);
    const std::array<uint8_t, 1> first{std::get<0>(command)};
    const std::array<uint8_t, 2> rest{std::get<1>(command), std::get<2>(command)};
    std::array<uint8_t, 1>       first_received{};
    std::array<uint8_t, 2>       rest_received{};

    chip.select();
    chip.exchange(first, first_received);
    chip.exchange(rest, rest_received);
    chip.deselect();
    transfer(chip, data_frame(0x3F));

    CHECK_EQ(chip.peek(As5047uModel::zposl_address), 0x3F);
    CHECK_EQ(chip.peek(As5047uModel::errfl_address), 0);
}
}  // namespace
}  // namespace micras::models
