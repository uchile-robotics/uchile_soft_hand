// #define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_NONE
#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_DEBUG
// https://github.com/rorromr/serial_dxl/archive/v0.1.tar.gz
#include <SerialDXL.h>

// SERVO DXL basic config
#define SERVO_MODEL 100
#define SERVO_FIRMWARE 100

/**
 * DEVICEDXL_MIN_BUFFER = 6
 * Is the overhead for dynamixel like devices 
 * Has MODEL_L, MODEL_H, FIRMWARE, ID, BAUDRATE and RETURN DELAY
 * This variables are inside DeviceDXL class
 */ 

// We add 13 UInt8 (13 bytes)
#define SERVO_MMAP_SIZE 2

// MMap position for commands (mem Addrs)

#define TACTIL1_ADDR      6
#define TACTIL2_ADDR      7

//commands (not addr)
#define CMD1 1
#define CMD2 2

// Number of sensors
#define NUM_SENSORS 2

// Pin numbers on Arduino MiniPro to control the devices
#define PIN_TACTIL1 A0
#define PIN_TACTIL2 A1

// Com pins
#define PIN_DIR 3
#define PIN_RST 2

/**
 * @brief Tactil sensor Data Interface using DXL communication protocol over RS485.
 * This implementation uses 2 uint8_t variables to read 2 analog pins, tactil sensor in particular.
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device.
 */
class HeadDXL: public DeviceDXL<SERVO_MODEL, SERVO_FIRMWARE, SERVO_MMAP_SIZE>
{
  public:
    HeadDXL(uint8_t dir_pin, uint8_t reset_pin, uint8_t num_sensors, uint8_t sensors_pins[]):
    DeviceDXL(), // Call parent constructor
    dir_pin_(dir_pin),    // Direction pin for RS485
    reset_pin_(reset_pin), // Reset pin
    num_sensors_(num_sensors), // numero de servos
    tactil1_(MMap::Access::RW, MMap::Storage::RAM),
    tactil2_(MMap::Access::RW, MMap::Storage::RAM),
    cmd1_(MMap::Access::RW, MMap::Storage::RAM)
    {
      // Config pins
      pinMode(dir_pin_, OUTPUT);
      pinMode(reset_pin_, OUTPUT);
      for(uint8_t i=0;i<num_sensors_;i++)
      {
        pinMode(sensors_pins[i], INPUT);
      }
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
      read_tactil2();
    }

    inline bool onReset()
    {
      DEBUG_PRINTLN("ON RESET");
      return digitalRead(reset_pin_) == HIGH ? true : false;
    }

    inline void setTX()
    {
      digitalWrite(dir_pin_,HIGH);
    }

    inline void setRX()
    {
      digitalWrite(dir_pin_,LOW);
    }

  private:
    const uint8_t dir_pin_;     // Toggle communication direction pin
    const uint8_t reset_pin_;   // Reset pin
    const uint8_t num_sensors_;
    
    // sensors variables
    MMap::Integer<UInt8, 0U, 255U, 0U>::type tactil1_;
    MMap::Integer<UInt8, 0U, 255U, 0U>::type tactil2_;
    MMap::Integer<UInt8, 0U, 255U, 0U>::type cmd1_;
};

uint8_t sensors_pins[] = {PIN_TACTIL1, PIN_TACTIL2};

//General device
HeadDXL head_dxl(PIN_DIR, PIN_RST, NUM_SENSORS, sensors_pins);
SerialDXL<HeadDXL> serialDxl;

void setup()
{
    // Init serial communication using Dynamixel format
    serialDxl.init(&Serial3 , &head_dxl);
}

void loop()
{
    // Update msg buffer
    while (Serial3.available())
    {
        serialDxl.process(Serial3.read());
    }

    head_dxl.mmap_.deserialize();
    head_dxl.update();
    head_dxl.mmap_.serialize();
}