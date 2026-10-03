#pragma once

#include <cstdint>
#include <optional>

class PressureProcessor
{
public:
    bool Process(int32_t adcValue) noexcept
    {
        if (adcValue < 0 or adcValue > kAdcMaximum) {
            InvalidateMeasurement();
            return false;
        }

        const uint32_t voltageMillivolts    = AdcToMillivolts(static_cast<uint32_t>(adcValue));
        const uint32_t pressureMilliPascals = MillivoltsToMilliPascals(voltageMillivolts);

        if (not atmosphericPressureMilliPascals_.has_value()) {
            atmosphericPressureMilliPascals_ = pressureMilliPascals;
        }
        if (not atmosphericMilliVoltage_.has_value()) {
            atmosphericMilliVoltage_ = voltageMillivolts;
        }

        voltageMillivolts_    = voltageMillivolts;
        pressureMilliPascals_ = pressureMilliPascals;
        depthMillimeters_     = MilliPascalsToDepthMillimeters(pressureMilliPascals, atmosphericPressureMilliPascals_.value());
        depthMicrometers_ =
            AdcToDepthMicrometers(adcValue, atmosphericMilliVoltage_.value(), atmosphericPressureMilliPascals_.value());
        // depthMicrometers_ = MilliPascalsToDepthMicrometers(pressureMilliPascals, atmosphericPressureMilliPascals_.value());
        return true;
    }

    [[nodiscard]] std::optional<uint64_t> DepthMillimeters() const noexcept
    {
        return depthMillimeters_;
    }

    [[nodiscard]] std::optional<uint64_t> DepthMicrometers() const noexcept
    {
        return depthMicrometers_;
    }

    [[nodiscard]] std::optional<uint32_t> PressureMilliPascals() const noexcept
    {
        return pressureMilliPascals_;
    }

    [[nodiscard]] std::optional<uint32_t> VoltageMillivolts() const noexcept
    {
        return voltageMillivolts_;
    }

    [[nodiscard]] std::optional<uint32_t> AtmosphericMilliPascals() const noexcept
    {
        return atmosphericPressureMilliPascals_;
    }

    [[nodiscard]] std::optional<uint32_t> AtmosphericMilliVoltage() const noexcept
    {
        return atmosphericMilliVoltage_;
    }

private:
    // Transfer function for a 0.5...4.5 V, 0...300 PSI pressure sensor
    static constexpr uint32_t kReferenceVoltageMillivolts  = 5000;
    static constexpr int32_t kAdcMaximum                   = 8388607;
    static constexpr uint32_t kSensorMinimumMillivolts     = 500;
    static constexpr uint32_t kSensorMaximumMillivolts     = 4500;
    static constexpr uint32_t kSensorSpanMillivolts        = kSensorMaximumMillivolts - kSensorMinimumMillivolts;
    static constexpr uint32_t kMaximumPressurePsi          = 300;
    static constexpr uint32_t kMilliPascalsPerPsi          = 6894760;
    static constexpr uint32_t kMaximumPressureMilliPascals = kMaximumPressurePsi * kMilliPascalsPerPsi;
    static constexpr uint32_t kWaterDensityTimesGravity    = 9800;

    std::optional<uint32_t> atmosphericPressureMilliPascals_;
    std::optional<uint32_t> atmosphericMilliVoltage_;
    std::optional<uint32_t> voltageMillivolts_;
    std::optional<uint32_t> pressureMilliPascals_;
    std::optional<uint32_t> depthMillimeters_;
    std::optional<uint64_t> depthMicrometers_;

    void InvalidateMeasurement() noexcept
    {
        voltageMillivolts_.reset();
        pressureMilliPascals_.reset();
        depthMillimeters_.reset();
        depthMicrometers_.reset();
    }

    static uint32_t AdcToMillivolts(uint32_t adcValue) noexcept
    {
        const uint64_t voltageNumerator = static_cast<uint64_t>(adcValue) * kReferenceVoltageMillivolts;
        return static_cast<uint32_t>((voltageNumerator + (kAdcMaximum / 2)) / kAdcMaximum);
    }

    static uint32_t MillivoltsToMilliPascals(uint32_t voltageMillivolts) noexcept
    {
        if (voltageMillivolts <= kSensorMinimumMillivolts) {
            return 0;
        }

        if (voltageMillivolts >= kSensorMaximumMillivolts) {
            return kMaximumPressureMilliPascals;
        }

        const uint32_t activeVoltage  = voltageMillivolts - kSensorMinimumMillivolts;
        const uint64_t scaledPressure = static_cast<uint64_t>(activeVoltage) * kMaximumPressureMilliPascals;
        return static_cast<uint32_t>((scaledPressure + (kSensorSpanMillivolts / 2)) / kSensorSpanMillivolts);
    }

    static uint32_t MilliPascalsToDepthMillimeters(uint32_t pressureMilliPascals,
                                                   uint32_t atmosphericPressureMilliPascals) noexcept
    {
        if (pressureMilliPascals <= atmosphericPressureMilliPascals) {
            return 0;
        }

        const uint32_t waterPressureMilliPascals = pressureMilliPascals - atmosphericPressureMilliPascals;
        return static_cast<uint32_t>(static_cast<uint64_t>(waterPressureMilliPascals) / kWaterDensityTimesGravity);
    }

    static uint32_t MilliPascalsToDepthMicrometers(uint32_t pressureMilliPascals,
                                                   uint32_t atmosphericPressureMilliPascals) noexcept
    {
        if (pressureMilliPascals <= atmosphericPressureMilliPascals) {
            return 0;
        }

        const uint32_t waterPressureMilliPascals = pressureMilliPascals - atmosphericPressureMilliPascals;
        return static_cast<uint32_t>(static_cast<uint64_t>(waterPressureMilliPascals * 1000) / kWaterDensityTimesGravity);
    }

    static uint64_t AdcToDepthMicrometers(uint32_t adcValue,
                                          uint32_t atmosphericMilliVoltage,
                                          uint32_t atmosphericPressureMilliPascals) noexcept
    {
        const uint64_t voltageNumerator  = static_cast<uint64_t>(adcValue) * kReferenceVoltageMillivolts;
        const uint32_t voltageMillivolts = static_cast<uint32_t>((voltageNumerator + kAdcMaximum / 2) / kAdcMaximum);

        if ((voltageMillivolts <= atmosphericMilliVoltage) || (voltageMillivolts <= kSensorMinimumMillivolts)) {
            return 0;
        }

        uint32_t activeVoltageMillivolts = voltageMillivolts - kSensorMinimumMillivolts;
        if (activeVoltageMillivolts > kSensorSpanMillivolts) {
            activeVoltageMillivolts = kSensorSpanMillivolts;
        }
        const uint64_t totalPressureNumerator = static_cast<uint64_t>(activeVoltageMillivolts) * kMaximumPressureMilliPascals;
        const uint64_t atmosNumerator         = static_cast<uint64_t>(atmosphericPressureMilliPascals) * kSensorSpanMillivolts;

        if (totalPressureNumerator <= atmosNumerator) {
            return 0;
        }

        const uint64_t waterPressureNumerator = totalPressureNumerator - atmosNumerator;
        const uint64_t finalNumerator         = waterPressureNumerator * 5ULL;
        constexpr uint64_t finalDenominator   = 196000ULL;

        return (finalNumerator + finalDenominator / 2) / finalDenominator;
    }
};
