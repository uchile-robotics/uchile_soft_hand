#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_NONE
// #define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_DEBUG
// https://github.com/rorromr/serial_dxl/archive/v0.1.tar.gz
#include <SerialDXL.h>

// SERVO DXL basic config
#define HAND_MODEL 100
#define HAND_FIRMWARE 100

/**
 * Has MODEL_L, MODEL_H, FIRMWARE, ID, BAUDRATE and RETURN DELAY
 * This variables are inside DeviceDXL class
 */ 

// We add 3 UInt8 (3 bytes)
#define HAND_MMAP_SIZE 3

// MMap position for commands (mem Addrs)

#define TACTIL1_ADDR_LB      6
#define TACTIL1_ADDR_HB      7
#define TACTIL2_ADDR_LB      8
#define TACTIL2_ADDR_HB      9

//commands (not addr)
#define CMD1 1
#define CMD2 2

// Number of sensors
#define NUM_SENSORS 2

// Pin numbers on Arduino MiniPro to control the devices
#define PIN_TACTIL1 A0
#define PIN_TACTIL2 A1

// Com pins
#define PIN_DIR 4
#define PIN_RST 8

/**
 * @brief Tactil sensor Data Interface using DXL communication protocol over RS485.
 * This implementation uses 2 uint8_t variables to read 2 analog pins, tactil sensor in particular.
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device.
 */
class HandDXL: public DeviceDXL<HAND_MODEL, HAND_FIRMWARE, HAND_MMAP_SIZE>
{
  public:
    HandDXL(uint8_t dir_pin, uint8_t reset_pin):
    DeviceDXL(), // Call parent constructor
    dir_pin_(dir_pin),    // Direction pin for RS485
    reset_pin_(reset_pin), // Reset pin
    tactil1_(MMap::Access::RW, MMap::Storage::RAM),
    tactil2_(MMap::Access::RW, MMap::Storage::RAM),
    cmd1_(MMap::Access::RW, MMap::Storage::RAM)
    {
      // Config pins
      pinMode(dir_pin_, OUTPUT);
      pinMode(reset_pin_, INPUT);
      pinMode(PIN_TACTIL1, INPUT);
      pinMode(PIN_TACTIL2, INPUT);

      // Get mask and port for data control pin
      dataControlPinMask_ = digitalPinToBitMask(dir_pin);
      dataControlPinReg_ = portOutputRegister(digitalPinToPort(dir_pin));
    }

    void init()
    {
        DEBUG_PRINTLN("INIT");
        /*
        * Register variables
        */
        DeviceDXL::init();

        mmap_.registerVariable(&tactil1_);
        mmap_.registerVariable(&tactil2_);
        mmap_.registerVariable(&cmd1_);
        mmap_.init();

        /*
        * Load default values
        */
        DEBUG_PRINTLN("Load default");
        mmap_.load(); // Load values from EEPROM
        DEBUG_PRINT("tactil1: ");DEBUG_PRINTLN(tactil1_.data);
        DEBUG_PRINT("tactil2: ");DEBUG_PRINTLN(tactil1_.data);
        INFO_PRINT("id: ");INFO_PRINTLN_RAW(id_.data);

        /*
        * Read sensor data
        * e.g. Use ADC, check buttons, etc.
        */
    }

    void read_tactil1(void)
    {
        tactil1_.data = analogRead(PIN_TACTIL1);
    }

    void read_tactil2(void)
    {
        tactil2_.data = analogRead(PIN_TACTIL2);
    }

    void update()
    {
      read_tactil1();
      delayMicroseconds(5);
      read_tactil2();
      Serial.print(tactil1_.data);Serial.print("\t\t");Serial.println(tactil2_.data);
    }

    inline bool onReset()
    {
      DEBUG_PRINTLN("ON RESET");
      return digitalRead(reset_pin_) == HIGH ? true : false;
    }

    inline void setTX()
    {
      *dataControlPinReg_ |= dataControlPinMask_;
    }

    inline void setRX()
    {
      *dataControlPinReg_ &= ~dataControlPinMask_;
    }

  private:
    // Communication direction pin
    uint8_t dataControlPinMask_;
    volatile uint8_t *dataControlPinReg_;

    const uint8_t dir_pin_;     // Toggle communication direction pin
    const uint8_t reset_pin_;   // Reset pin
    
    // sensors variables
    MMap::Integer<UInt16, 0U, 1023U, 0U>::type tactil1_;
    MMap::Integer<UInt16, 0U, 1023U, 0U>::type tactil2_;
    MMap::Integer<UInt8, 0U, 255U, 0U>::type cmd1_;
};

//General device
HandDXL hand_dxl(PIN_DIR, PIN_RST);
SerialDXL<HandDXL> serialDxl;

void setup()
{
     //Serial port for debug
    Serial.begin(115200);
    DEBUG_PRINTLN("INIT SETUP");

    hand_dxl.init();
    hand_dxl.reset();
    hand_dxl.mmap_.serialize();

    // Init serial communication using Dynamixel format
    serialDxl.init(&Serial1 , &hand_dxl);
    DEBUG_PRINTLN("INIT MAIN PROGRAM");
}

void loop()
{
    // Update msg buffer
    while (Serial1.available())
    {
        serialDxl.process(Serial1.read());
    }

    hand_dxl.mmap_.deserialize();
    hand_dxl.update();
    hand_dxl.mmap_.serialize();
}