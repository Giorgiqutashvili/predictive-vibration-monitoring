// app_main.cpp
// C++ application entry point for the Vibration Diagnostic / TinyML PdM Node.
// Sensor: STEVAL-MKI208V1K (IIS3DWB ultra-wideband 3-axis vibration sensor), SPI mode.
// Called from main.c's USER CODE BEGIN 2 block via alt_main().

#include <cstdint>
#include <cstring>

extern "C" {
#include "main.h"   // HAL handles: hspi1, GPIO defs, HAL_GetTick(), etc.

extern SPI_HandleTypeDef hspi1;

}



// ---------------------------------------------------------------------------
// Adjust these to match your actual .ioc pin/peripheral configuration
// ---------------------------------------------------------------------------
#define IIS3DWB_SPI              (&hspi1)
#define IIS3DWB_CS_GPIO_Port     GPIOB          // TODO: set to your actual CS GPIO port

// ---------------------------------------------------------------------------
// IIS3DWB register map (subset needed for basic 3-axis polling read)
// ---------------------------------------------------------------------------
namespace IIS3DWB {
    constexpr uint8_t WHO_AM_I    = 0x0F;
    constexpr uint8_t WHO_AM_I_VAL = 0x7B;

    constexpr uint8_t CTRL1_XL    = 0x10;  // XL_EN[7:5], FS_XL[3:2]
    constexpr uint8_t CTRL6_C     = 0x15;  // XL_AXIS_SEL[1:0]
    constexpr uint8_t STATUS_REG  = 0x1E;  // XLDA = bit0

    constexpr uint8_t OUTX_L_A    = 0x28;
    constexpr uint8_t OUTX_H_A    = 0x29;
    constexpr uint8_t OUTY_L_A    = 0x2A;
    constexpr uint8_t OUTY_H_A    = 0x2B;
    constexpr uint8_t OUTZ_L_A    = 0x2C;
    constexpr uint8_t OUTZ_H_A    = 0x2D;

    // Init values per ST AN5444 recommended sequence
    constexpr uint8_t CTRL6_C_3AXIS_MODE     = 0x00;
    constexpr uint8_t CTRL1_XL_ODR26667_FS2G = 0xA0; // XL_EN=101 (26.667kHz), FS_XL=00 (+-2g)

    constexpr uint8_t SPI_READ_BIT  = 0x80;
    constexpr uint8_t SPI_WRITE_BIT = 0x00;
}

// ---------------------------------------------------------------------------
// Low-level SPI register access
// ---------------------------------------------------------------------------
static void cs_low()  { HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_RESET); }
static void cs_high() { HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_SET); }

static uint8_t iis3dwb_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg | IIS3DWB::SPI_READ_BIT), 0x00 };
    uint8_t rx[2] = { 0, 0 };

    cs_low();
    HAL_SPI_TransmitReceive(IIS3DWB_SPI, tx, rx, 2, HAL_MAX_DELAY);
    cs_high();

    return rx[1];
}

static void iis3dwb_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg | IIS3DWB::SPI_WRITE_BIT), value };

    cs_low();
    HAL_SPI_Transmit(IIS3DWB_SPI, tx, 2, HAL_MAX_DELAY);
    cs_high();
}

static bool iis3dwb_read_multi(uint8_t start_reg, uint8_t* dst, uint8_t len)
{
    uint8_t tx = static_cast<uint8_t>(start_reg | IIS3DWB::SPI_READ_BIT);

    cs_low();
    HAL_StatusTypeDef s1 = HAL_SPI_Transmit(IIS3DWB_SPI, &tx, 1, HAL_MAX_DELAY);
    HAL_StatusTypeDef s2 = HAL_SPI_Receive(IIS3DWB_SPI, dst, len, HAL_MAX_DELAY);
    cs_high();

    return (s1 == HAL_OK) && (s2 == HAL_OK);
}

// ---------------------------------------------------------------------------
// Sensor sample struct + window buffer
// ---------------------------------------------------------------------------
struct AccelSample {
    int16_t x, y, z;
};

constexpr uint32_t WINDOW_SIZE = 256;

class SampleWindow {
public:
    void push(const AccelSample& s) {
        buffer_[head_] = s;
        head_ = (head_ + 1) % WINDOW_SIZE;
        if (count_ < WINDOW_SIZE) count_++;
    }
    bool isFull() const { return count_ == WINDOW_SIZE; }
    const AccelSample* data() const { return buffer_; }
    uint32_t size() const { return count_; }
    void reset() { head_ = 0; count_ = 0; }

private:
    AccelSample buffer_[WINDOW_SIZE] = {};
    uint32_t head_ = 0;
    uint32_t count_ = 0;
};

// ---------------------------------------------------------------------------
// IIS3DWB driver
// ---------------------------------------------------------------------------
static bool sensor_init()
{
    cs_high(); // idle high before first transaction

    uint8_t who_am_i = iis3dwb_read_reg(IIS3DWB::WHO_AM_I);
    if (who_am_i != IIS3DWB::WHO_AM_I_VAL) {
        return false; // wrong chip / wiring problem
    }

    // AN5444 recommended init sequence
    iis3dwb_write_reg(IIS3DWB::CTRL6_C, IIS3DWB::CTRL6_C_3AXIS_MODE);
    iis3dwb_write_reg(IIS3DWB::CTRL1_XL, IIS3DWB::CTRL1_XL_ODR26667_FS2G);

    return true;
}

static bool sensor_data_ready()
{
    uint8_t status = iis3dwb_read_reg(IIS3DWB::STATUS_REG);
    return (status & 0x01) != 0; // XLDA bit
}

static bool sensor_read(AccelSample& out)
{
    uint8_t raw[6] = {0};
    if (!iis3dwb_read_multi(IIS3DWB::OUTX_L_A, raw, 6)) {
        return false;
    }

    out.x = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    out.y = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    out.z = static_cast<int16_t>((raw[5] << 8) | raw[4]);
    return true;
}

// ---------------------------------------------------------------------------
// Inference placeholder - wire in TFLite Micro / Edge Impulse SDK later
// ---------------------------------------------------------------------------
static void run_inference(const SampleWindow& window)
{
    // TODO:
    //  1. Feature extraction (FFT magnitude, RMS, kurtosis, crest factor...) from window.data()
    //  2. Feed features into TFLite Micro interpreter->Invoke()
    //  3. Read output tensor -> classify (normal / imbalance / bearing fault / misalignment)
    (void)window;
}

// ---------------------------------------------------------------------------
// Entry point, called from main.c
// ---------------------------------------------------------------------------
extern "C" void alt_main(void)
{
    static SampleWindow window;

    if (!sensor_init()) {
        // WHO_AM_I mismatch or SPI wiring problem — blink an error LED here if you have one
        while (1) { /* halt / error indicator */ }
    }

    while (1)
    {
        if (sensor_data_ready())
        {
            AccelSample s;
            if (sensor_read(s)) {
                window.push(s);
            }

            if (window.isFull()) {
                run_inference(window);
                window.reset();
            }
        }
        // Note: IIS3DWB's native ODR is 26.667kHz - polling like this in the main loop
        // will bottleneck well below that. For real vibration analysis bandwidth,
        // switch to the FIFO + SPI DMA once this basic path is verified working.
    }
}
