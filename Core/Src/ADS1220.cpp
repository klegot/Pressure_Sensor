#include "ADS1220.hpp"

ADS1220::ADS1220(SPI_HandleTypeDef& spi, GPIO_TypeDef* chipSelectPort, uint16_t chipSelectPin) noexcept
    : spi_{ spi }
    , chipSelectPort_{ chipSelectPort }
    , chipSelectPin_{ chipSelectPin }
{}

bool ADS1220::Init() noexcept
{
    measurementInProgress_ = false;
    dataReady_             = false;

    if (not SendCommand(kResetCommand)) {
        return false;
    }

    HAL_Delay(10);

    const std::array<uint8_t, kRegisterCount> configuration{
        kPressureRegister0, // pressure channel
        0b11010100, // Turbo mode, continuous conversion, 2000 sps = 2kHz
        0x40, // External reference on REFP0/REFN0
        0x00, // Dedicated active-low DRDY output enabled
    };

    return WriteRegisters(0, configuration.data(), configuration.size());
}

bool ADS1220::StartMeasurement(Channel channel) noexcept
{
    if (measurementInProgress_) {
        return false;
    }

    const uint8_t register0 = Register0For(channel);
    if (not WriteRegisters(0, &register0, 1)) {
        return false;
    }

    dataReady_             = false;
    activeChannel_         = channel;
    measurementInProgress_ = true;

    if (not SendCommand(kStartCommand)) {
        measurementInProgress_ = false;
        return false;
    }

    return true;
}

std::optional<ADS1220::Measurement> ADS1220::ReadMeasurement() noexcept
{
    if (not measurementInProgress_ or not dataReady_) {
        return std::nullopt;
    }

    dataReady_             = false;

    const auto adcValue = ReadConversion();
    if (not adcValue.has_value()) {
        return std::nullopt;
    }

    return Measurement{ activeChannel_, adcValue.value() };
}

bool ADS1220::IsMeasurementInProgress() const noexcept
{
    return measurementInProgress_;
}

void ADS1220::OnDataReadyInterrupt() noexcept
{
    dataReady_ = true;
}

bool ADS1220::SendCommand(uint8_t command) noexcept
{
    Select();
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(&spi_, &command, 1, HAL_MAX_DELAY);
    Deselect();
    return status == HAL_OK;
}

bool ADS1220::WriteRegisters(uint8_t firstRegister, const uint8_t* data, uint8_t count) noexcept
{
    if (data == nullptr or count == 0 or firstRegister >= kRegisterCount or count > kRegisterCount - firstRegister) {
        return false;
    }

    uint8_t command = static_cast<uint8_t>(kWriteRegisterCommand | (firstRegister << 2) | (count - 1));

    Select();
    const HAL_StatusTypeDef commandStatus = HAL_SPI_Transmit(&spi_, &command, 1, HAL_MAX_DELAY);
    const HAL_StatusTypeDef dataStatus =
        commandStatus == HAL_OK ? HAL_SPI_Transmit(&spi_, const_cast<uint8_t*>(data), count, HAL_MAX_DELAY) : HAL_ERROR;
    Deselect();

    return commandStatus == HAL_OK and dataStatus == HAL_OK;
}

std::optional<int32_t> ADS1220::ReadConversion() noexcept
{
    std::array<uint8_t, kConversionByteCount> data{};

    Select();
    const HAL_StatusTypeDef dataStatus = HAL_OK ? HAL_SPI_Receive(&spi_, data.data(), data.size(), HAL_MAX_DELAY) : HAL_ERROR;
    Deselect();

    if (dataStatus != HAL_OK) {
        return std::nullopt;
    }

    int32_t value = (static_cast<int32_t>(data[0]) << 16) | (static_cast<int32_t>(data[1]) << 8) | static_cast<int32_t>(data[2]);

    if ((value & 0x00800000U) != 0U) {
        return std::nullopt;
    }

    return value;
}

void ADS1220::Select() noexcept
{
    HAL_GPIO_WritePin(chipSelectPort_, chipSelectPin_, GPIO_PIN_RESET);
}

void ADS1220::Deselect() noexcept
{
    HAL_GPIO_WritePin(chipSelectPort_, chipSelectPin_, GPIO_PIN_SET);
}

uint8_t ADS1220::Register0For(Channel channel) noexcept
{
    switch (channel) {
        case Channel::Pressure:
            return kPressureRegister0;
        case Channel::Temperature:
            return kTemperatureRegister0;
    }
    return kPressureRegister0;
}
