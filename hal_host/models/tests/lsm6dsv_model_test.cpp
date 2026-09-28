/**
 * @file
 */

#include <array>
#include <cstdint>
#include <numbers>
#include <vector>

#include <doctest/doctest.h>

#include "lsm6dsv_reg.h"
#include "micras/models/lsm6dsv_model.hpp"

namespace micras::models {
namespace {
constexpr uint8_t read_flag{0x80};

std::vector<uint8_t> transact(Lsm6dsvModel& chip, std::vector<uint8_t> transmitted) {
    std::vector<uint8_t> received(transmitted.size());
    chip.select();
    chip.exchange(transmitted, received);
    chip.deselect();
    return received;
}

void write(Lsm6dsvModel& chip, uint8_t address, uint8_t value) {
    transact(chip, {address, value});
}

uint8_t read(Lsm6dsvModel& chip, uint8_t address) {
    return transact(chip, {static_cast<uint8_t>(address | read_flag), 0}).at(1);
}

int16_t word(const std::vector<uint8_t>& bytes, std::size_t low) {
    return static_cast<int16_t>(bytes.at(low) | (bytes.at(low + 1) << 8U));
}

void configure(Lsm6dsvModel& chip, uint8_t gyroscope_scale, uint8_t accelerometer_scale) {
    write(chip, LSM6DSV_HAODR_CFG, 0x01);
    write(chip, LSM6DSV_CTRL1, 0x1C);
    write(chip, LSM6DSV_CTRL2, 0x1C);
    write(chip, LSM6DSV_CTRL6, gyroscope_scale);
    write(chip, LSM6DSV_CTRL8, accelerometer_scale);
}

std::vector<uint8_t> read_burst(Lsm6dsvModel& chip) {
    std::vector<uint8_t> command(17);
    command.at(0) = LSM6DSV_STATUS_REG | read_flag;
    return transact(chip, command);
}

TEST_CASE("Lsm6dsvModel.AnswersInSpiMode3") {
    const Lsm6dsvModel chip;

    CHECK_EQ(chip.mode(), hal::host::SpiDevice::Mode::MODE_3);
}

TEST_CASE("Lsm6dsvModel.PowersOnWithTheDatasheetDefaults") {
    const Lsm6dsvModel chip;

    for (uint8_t address = 0; address < Lsm6dsvModel::register_count; address++) {
        uint8_t expected = 0x00;

        if (address == LSM6DSV_WHO_AM_I) {
            expected = LSM6DSV_ID;
        } else if (address == LSM6DSV_PIN_CTRL) {
            expected = 0x23;
        } else if (address == LSM6DSV_CTRL3) {
            expected = 0x44;
        }

        CHECK_MESSAGE(chip.peek(address) == expected, "register " << static_cast<int>(address));
    }
}

TEST_CASE("Lsm6dsvModel.AnswersWhoAmIWithTheDeviceId") {
    Lsm6dsvModel chip;

    const std::vector<uint8_t> received = transact(chip, {LSM6DSV_WHO_AM_I | read_flag, 0xFF});

    CHECK_EQ(received.at(0), 0x00);
    CHECK_EQ(received.at(1), LSM6DSV_ID);
}

TEST_CASE("Lsm6dsvModel.AdvancesTheAddressAfterEveryByte") {
    Lsm6dsvModel chip;

    transact(chip, {LSM6DSV_CTRL1, 0x16, 0x1C});
    const std::vector<uint8_t> received = transact(chip, {LSM6DSV_CTRL1 | read_flag, 0, 0});

    CHECK_EQ(received.at(1), 0x16);
    CHECK_EQ(received.at(2), 0x1C);
}

TEST_CASE("Lsm6dsvModel.KeepsOneTransactionAcrossSeveralExchanges") {
    Lsm6dsvModel chip;
    write(chip, LSM6DSV_CTRL2, 0x1C);

    const std::array<uint8_t, 1> command{LSM6DSV_CTRL1 | read_flag};
    std::array<uint8_t, 1>       ignored{};
    const std::array<uint8_t, 2> dummy{};
    std::array<uint8_t, 2>       received{};

    chip.select();
    chip.exchange(command, ignored);
    chip.exchange(dummy, received);
    chip.deselect();

    CHECK_EQ(std::get<0>(received), 0x00);
    CHECK_EQ(std::get<1>(received), 0x1C);
}

TEST_CASE("Lsm6dsvModel.StartsEveryTransactionWithACommand") {
    Lsm6dsvModel chip;

    transact(chip, {LSM6DSV_CTRL1, 0x16});
    transact(chip, {LSM6DSV_CTRL2, 0x1C});

    CHECK_EQ(chip.peek(LSM6DSV_CTRL1), 0x16);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL2), 0x1C);
}

TEST_CASE("Lsm6dsvModel.StaysOnOneRegisterWithoutAutoIncrement") {
    Lsm6dsvModel chip;
    write(chip, LSM6DSV_CTRL3, 0x40);

    transact(chip, {LSM6DSV_CTRL1, 0x16, 0x1C});
    const std::vector<uint8_t> received = transact(chip, {LSM6DSV_CTRL1 | read_flag, 0, 0});

    CHECK_EQ(received.at(1), 0x1C);
    CHECK_EQ(received.at(2), 0x1C);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL2), 0x00);
}

TEST_CASE("Lsm6dsvModel.IgnoresWritesToReadOnlyAndReservedRegisters") {
    Lsm6dsvModel chip;

    write(chip, LSM6DSV_WHO_AM_I, 0x00);
    write(chip, LSM6DSV_STATUS_REG, 0xFF);
    write(chip, LSM6DSV_OUTX_L_G, 0x12);
    write(chip, 0x04, 0x34);

    CHECK_EQ(chip.peek(LSM6DSV_WHO_AM_I), LSM6DSV_ID);
    CHECK_EQ(chip.peek(LSM6DSV_STATUS_REG), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_OUTX_L_G), 0x00);
    CHECK_EQ(chip.peek(0x04), 0x00);
}

TEST_CASE("Lsm6dsvModel.RestoresTheDefaultsOnASoftwarePowerOnReset") {
    Lsm6dsvModel chip;
    configure(chip, LSM6DSV_4000dps, LSM6DSV_8g);
    chip.push_sample({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

    transact(chip, {LSM6DSV_FUNC_CFG_ACCESS, 0x04, 0x55});

    CHECK_EQ(chip.peek(LSM6DSV_FUNC_CFG_ACCESS), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_PIN_CTRL), 0x23);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL1), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL2), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL3), 0x44);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL6), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_STATUS_REG), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_OUTX_L_G), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_HAODR_CFG), 0x00);
    CHECK_EQ(read(chip, LSM6DSV_WHO_AM_I), LSM6DSV_ID);
}

TEST_CASE("Lsm6dsvModel.RestoresTheDefaultsOnASoftwareReset") {
    Lsm6dsvModel chip;
    configure(chip, LSM6DSV_4000dps, LSM6DSV_8g);

    write(chip, LSM6DSV_CTRL3, 0x45);

    CHECK_EQ(chip.peek(LSM6DSV_CTRL3), 0x44);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL2), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL6), 0x00);
}

TEST_CASE("Lsm6dsvModel.HidesTheMainPageWhileAnotherIsSelected") {
    Lsm6dsvModel chip;

    write(chip, LSM6DSV_FUNC_CFG_ACCESS, 0x80);
    CHECK_EQ(read(chip, LSM6DSV_WHO_AM_I), 0x00);
    write(chip, LSM6DSV_CTRL1, 0x1C);

    write(chip, LSM6DSV_FUNC_CFG_ACCESS, 0x00);
    CHECK_EQ(read(chip, LSM6DSV_WHO_AM_I), LSM6DSV_ID);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL1), 0x00);
}

TEST_CASE("Lsm6dsvModel.IgnoresSamplesWhileTheSensorsAreOff") {
    Lsm6dsvModel chip;

    chip.push_sample({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

    CHECK_EQ(chip.peek(LSM6DSV_STATUS_REG), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_OUTX_L_G), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_OUTX_L_A), 0x00);
}

TEST_CASE("Lsm6dsvModel.SetsTheDataReadyBitOfEachSensorThatIsOn") {
    Lsm6dsvModel chip;
    write(chip, LSM6DSV_CTRL2, 0x1C);

    chip.push_sample({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

    CHECK_EQ(chip.peek(LSM6DSV_STATUS_REG), 0x02);
    CHECK_EQ(chip.peek(LSM6DSV_OUTX_L_A), 0x00);

    write(chip, LSM6DSV_CTRL1, 0x1C);
    chip.push_sample({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

    CHECK_EQ(chip.peek(LSM6DSV_STATUS_REG), 0x03);
}

TEST_CASE("Lsm6dsvModel.ClearsTheDataReadyBitsWithTheBurstThatReadsTheSample") {
    Lsm6dsvModel chip;
    configure(chip, LSM6DSV_4000dps, LSM6DSV_8g);
    chip.push_sample({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

    const std::vector<uint8_t> first = read_burst(chip);
    const std::vector<uint8_t> second = read_burst(chip);

    CHECK_EQ(first.at(1), 0x03);
    CHECK_EQ(second.at(1), 0x00);
    CHECK_EQ(chip.peek(LSM6DSV_STATUS_REG), 0x00);
    CHECK_EQ(word(second, 5), word(first, 5));
}

TEST_CASE("Lsm6dsvModel.ClearsEachDataReadyBitWithAHighByteOfItsSensor") {
    Lsm6dsvModel chip;
    configure(chip, LSM6DSV_4000dps, LSM6DSV_8g);
    chip.push_sample({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

    read(chip, LSM6DSV_OUTX_L_G);
    CHECK_EQ(chip.peek(LSM6DSV_STATUS_REG), 0x03);

    read(chip, LSM6DSV_OUTY_H_G);
    CHECK_EQ(chip.peek(LSM6DSV_STATUS_REG), 0x01);

    read(chip, LSM6DSV_OUTZ_H_A);
    CHECK_EQ(chip.peek(LSM6DSV_STATUS_REG), 0x00);
}

TEST_CASE("Lsm6dsvModel.EncodesTheSampleWithTheFullScaleInTheRegisters") {
    Lsm6dsvModel chip;
    configure(chip, LSM6DSV_4000dps, LSM6DSV_8g);
    chip.push_sample({1.0, -1.0, 0.0}, {9.80665, -9.80665, 0.0});

    const std::vector<uint8_t> wide = read_burst(chip);

    CHECK_EQ(word(wide, 5), 409);
    CHECK_EQ(word(wide, 7), -409);
    CHECK_EQ(word(wide, 9), 0);
    CHECK_EQ(word(wide, 11), 4098);
    CHECK_EQ(word(wide, 13), -4098);
    CHECK_EQ(word(wide, 15), 0);

    write(chip, LSM6DSV_CTRL6, LSM6DSV_125dps);
    write(chip, LSM6DSV_CTRL8, LSM6DSV_2g);
    chip.push_sample({1.0, 100.0, -100.0}, {9.80665, 100.0, -100.0});

    const std::vector<uint8_t> narrow = read_burst(chip);

    CHECK_EQ(word(narrow, 5), 13096);
    CHECK_EQ(word(narrow, 7), 32767);
    CHECK_EQ(word(narrow, 9), -32768);
    CHECK_EQ(word(narrow, 11), 16393);
    CHECK_EQ(word(narrow, 13), 32767);
    CHECK_EQ(word(narrow, 15), -32768);
}

TEST_CASE("Lsm6dsvModel.GivesTheSensitivityOfEveryFullScale") {
    Lsm6dsvModel     chip;
    constexpr double mdps{std::numbers::pi / 180000.0};
    constexpr double mg{0.00980665};

    const std::array<std::pair<uint8_t, double>, 6> gyroscope{{
        {LSM6DSV_125dps, 4.375},
        {LSM6DSV_250dps, 8.75},
        {LSM6DSV_500dps, 17.5},
        {LSM6DSV_1000dps, 35.0},
        {LSM6DSV_2000dps, 70.0},
        {LSM6DSV_4000dps, 140.0},
    }};
    const std::array<std::pair<uint8_t, double>, 4> accelerometer{{
        {LSM6DSV_2g, 0.061},
        {LSM6DSV_4g, 0.122},
        {LSM6DSV_8g, 0.244},
        {LSM6DSV_16g, 0.488},
    }};

    for (const auto& [code, sensitivity] : gyroscope) {
        write(chip, LSM6DSV_CTRL6, code);
        CHECK_EQ(chip.gyroscope_sensitivity(), doctest::Approx(sensitivity * mdps).epsilon(1e-12));
    }

    for (const auto& [code, sensitivity] : accelerometer) {
        write(chip, LSM6DSV_CTRL8, code);
        CHECK_EQ(chip.accelerometer_sensitivity(), doctest::Approx(sensitivity * mg).epsilon(1e-12));
    }
}

TEST_CASE("Lsm6dsvModel.DecodesTheOutputDataRates") {
    Lsm6dsvModel chip;

    CHECK_EQ(chip.gyroscope_data_rate(), 0.0);
    CHECK_EQ(chip.accelerometer_data_rate(), 0.0);

    write(chip, LSM6DSV_CTRL2, 0x10 | (LSM6DSV_ODR_HA01_AT_8000Hz & 0x0F));
    write(chip, LSM6DSV_CTRL1, 0x10 | (LSM6DSV_ODR_HA01_AT_125Hz & 0x0F));

    CHECK_EQ(chip.gyroscope_data_rate(), 7680.0);

    write(chip, LSM6DSV_HAODR_CFG, 0x01);
    CHECK_EQ(chip.gyroscope_data_rate(), 8000.0);
    CHECK_EQ(chip.accelerometer_data_rate(), 125.0);

    write(chip, LSM6DSV_HAODR_CFG, 0x02);
    CHECK_EQ(chip.gyroscope_data_rate(), 6400.0);

    write(chip, LSM6DSV_CTRL1, LSM6DSV_ODR_AT_1Hz875);
    CHECK_EQ(chip.accelerometer_data_rate(), 1.875);
}
}  // namespace
}  // namespace micras::models
