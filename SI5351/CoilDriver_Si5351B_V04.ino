/*
 * Teensy LC Coil Driver controller via SI5351B clock generator
 * April 4, 2018
 * Ben Avants
 * 
 * Makes use of a modified version of the Etherkit Si5351 library 
 * 
 * Communication and monitoring of the Apex SA57A chips are handled directly by the Teensy LC.
 * Frequency generation is handled by the Silicon Labs SI5351B.
 * The SI5351B is programmed via I2C (SDA1 & SCL1 - the secondary pins).
 * Frequencies are accurate to 0.01 Hz but may contain phase noise and some jitter.
 * This is unlikely to cause a problem in this application, but it is worth noting.
 * Frequency values are handled as ULL (unsigned long long) with 1 = 0.01 Hz,
 * This requires conversion from float or int values in regular Hz... managed by this program.
 */

// CHANGED FROM PREVIOUS VERSIONS
// Frequency is now set as the target BIPAHSIC frequency, not the fundamental frequency.  The uC sets the clocks to TWICE
// the desired frequnecy to get the target output.

// SI5351B breakout mounts directly to GND-3V-23-22--17, 17 is optional, but all
// intermediate pins including 17 are covered with PCB - may be usable but tight space.

#include <EEPROM.h>
#include <i2c_t3.h>
#include <si5351.h>

class MCP4
{
  // The MCP4 family digital potentiometers and rheostats are useful for digitally controlled low power resistors in tunable circuits.
  // This library is specifically designed for the MCP4662, which is a dual rheostat version, with each channel @ 100 kOhms.
  // When used with a 30 pF capacitor for a 74LVC1G123 multivibrator, pulse time vs resistance is linearized to:
  // Pt = 0.04203 * R + 62.75
  
  // I2C Address for the MCP4662.  Base address is '0101 1**' and the rest is set by pins on the chip, currently '10'.
  // This is a 7-bit addres, so the compiler is given an address with an extra 0 at the beginning, to properly interpret the binary.
  const uint8_t MCP4662_BASE_ADDR = B00101100;
  uint8_t MCP4662_BITS = B00000010;
  uint8_t MCP4662_ADDR = B00101110;

  uint8_t d_bit;
  uint8_t command_byte;
  uint8_t data_byte;

  float resistance_2_wiper;
  float wiper_2_resistance;
  float half_step;
  uint16_t wiper0;
  uint16_t wiper1;

  i2c_t3 * mcp4_wire;
  uint8_t wire_error;

  public:
  bool is_connected;
  float max_resistance;
  float resistance0;
  float resistance1;
  float total_resistance;
  float pulse_time;
  
  MCP4()    {
    #ifndef MCP4_H_
    #define MCP4_H_
  	#define R_OFFSET	4700
  	#define T_SLOPE		0.04203
  	#define T_I_SLOPE	23.7925291458482
  	#define T_B			62.75
  	#define MIN_NS		260
  	#define MAX_NS		8667
	  // These pulse lengths enable a 0.9 duty cycle for stim frequencies between ~50 kHz and 1.73 MHz
    // Bits 3 & 2 are the command bits - command sets as follows:
    #define WRITE_BITS  B00000000
    #define INC_BITS    B00000100
    #define DEC_BITS    B00001000
    #define READ_BITS   B00001100
    // Bits 7, 6, 5, & 4 make up the register address
    #define WIP_0_VOL   B00000000
    #define WIP_1_VOL   B00010000
    #define WIP_0_NOV   B00100000
    #define WIP_1_NOV   B00110000
    #define TCON_NOV    B01000000
    #define STATUS_R    B01010000
    #define EEPROM_0    B01100000
    #define EEPROM_1    B01110000
    #define EEPROM_2    B10000000
    #define EEPROM_3    B10010000
    #define EEPROM_4    B10100000
    #define EEPROM_5    B10110000
    #define EEPROM_6    B11000000
    #define EEPROM_7    B11010000
    #define EEPROM_8    B11100000
    #define EEPROM_9    B11110000
    // General Call address for the MCP4*** family
    #define MCP4_GEN_WIP0_W_ADDR  B01000000
    #define MCP4_GEN_WIP1_W_ADDR  B01001000
    #define MCP4_GEN_TCON_ADDR    B01100000
    #define MCP4_GEN_WIP0_I_ADDR  B01000010
    #define MCP4_GEN_WIP1_I_ADDR  B01001010
    #define MCP4_GEN_WIP0_D_ADDR  B01000100
    #define MCP4_GEN_WIP1_D_ADDR  B01001100
    #endif
  }

  bool init(uint16_t resistance0_init, uint16_t resistance1_init, unsigned int maxResistance = 100000, uint8_t addr_bits = B00000010)    {
    // Assumes that I2C is already setup (by the si5351 library in this case)
    mcp4_wire = &Wire1;
    MCP4662_BITS = addr_bits;
    MCP4662_ADDR = MCP4662_BASE_ADDR | addr_bits;
    mcp4_wire->beginTransmission(MCP4662_ADDR);
	  //mcp4_wire->beginTransmission(B00101110);
    wire_error = mcp4_wire->endTransmission();
    if (wire_error == 0) {
      max_resistance = maxResistance;
      resistance_2_wiper = 256 / max_resistance;
      wiper_2_resistance = max_resistance / 256;
      half_step = wiper_2_resistance * 0.5;
      wiper0 = round(resistance0_init * resistance_2_wiper);
      resistance0 = wiper0 * wiper_2_resistance;
      wiper1 = round(resistance1_init * resistance_2_wiper);
      resistance1 = wiper1 * wiper_2_resistance;
  	  total_resistance = resistance0 + resistance1 + R_OFFSET;
  	  pulse_time = (total_resistance * T_SLOPE) + T_B;
      if (!write(WIP_0_VOL, wiper0))  {
        return is_connected = false;
      }
      if (!write(WIP_1_VOL, wiper1))  {
        return is_connected = false;
      }
      return is_connected = true;
    }
    else  {
      return is_connected = false;
    }
  }

  bool write(uint8_t reg, uint16_t data)  {
    if (data > 255) {
      d_bit = 1;
    }
    else  {
      d_bit = 0;
    }
    command_byte = reg | WRITE_BITS | d_bit;
    mcp4_wire->beginTransmission(MCP4662_ADDR);
    mcp4_wire->write(command_byte);
    mcp4_wire->write((uint8_t)data);
    wire_error = mcp4_wire->endTransmission();
    return  wire_error == 0;
  }

  uint16_t read(uint8_t reg) {
    command_byte = reg | READ_BITS;
    mcp4_wire->beginTransmission(MCP4662_ADDR);
    mcp4_wire->write(command_byte);
    uint8_t bytes_recieved;
    bytes_recieved = mcp4_wire->requestFrom(MCP4662_ADDR, (uint8_t)2);
    if (bytes_recieved == 0)  {
      return 0;
    }
    uint16_t data = (uint16_t)mcp4_wire->readByte()<<8;
    data |= (uint16_t)mcp4_wire->readByte();
    wire_error = mcp4_wire->endTransmission();
    return data;
  }

  bool write_both(uint16_t setpoint0, uint16_t setpoint1) {
    // Writes resistances setpoint0 and setpoint1 to the volatile wiper0 and wiper1 registers respectively
    mcp4_wire->beginTransmission(MCP4662_ADDR);
    if (setpoint0 > 255) {
      d_bit = 1;
    }
    else  {
      d_bit = 0;
    }
    command_byte = WIP_0_VOL | WRITE_BITS | d_bit;
    mcp4_wire->write(command_byte);
    mcp4_wire->write((uint8_t)setpoint0);
    if (setpoint1 > 255) {
      d_bit = 1;
    }
    else  {
      d_bit = 0;
    }
    command_byte = WIP_1_VOL | WRITE_BITS | d_bit;
    mcp4_wire->write(command_byte);
    mcp4_wire->write((uint8_t)setpoint1);
    wire_error = mcp4_wire->endTransmission();
    return wire_error == 0;
  }

  bool set_symetrical(float res) {
    // Sets both wipers in one command to half of the value of res.  Wipers are set to give the closest value possible to res while
    // being the same or nearly the same in value.
    float half_res = (res - R_OFFSET) / 2;
    uint16_t setpoint0 = round(half_res * resistance_2_wiper);
    uint16_t setpoint1;
    // Check if both wipers having the same setting is closest to the desired total resistance
    float res_offset = (setpoint0 * 2 * wiper_2_resistance) - res; // Symetric setting offset from res
    if (abs(res_offset) <= half_step)  {
      setpoint1 = setpoint0;
    }
    else if (res_offset < 0)  {
      setpoint1 = setpoint0 + 1;
    }
    else  {
      setpoint1 = setpoint0 - 1;
    }
    // Set values
    if (write_both(setpoint0, setpoint1)) {
      resistance0 = setpoint0 * wiper_2_resistance;
      resistance1 = setpoint1 * wiper_2_resistance;
	  total_resistance = resistance0 + resistance1 + R_OFFSET;
	  pulse_time = (total_resistance * T_SLOPE) + T_B;
      return true;
    }
    else  {
      Serial.println("DigiPot symetrical set failed");
      return false;
    }
  }
  
  bool set_pulse_time(float pulse_ns)	{
	  // Uses set_symetrical to get as close as possible to a target multivibrator pulse time
	  float R = ((pulse_ns - T_B) * T_I_SLOPE);	// Find the resistance for the desired time from linear fit equation
	  if (R < R_OFFSET)	{
		  R = R_OFFSET;
	  }
	  else if(R > (max_resistance * 2) + R_OFFSET)	{
		  R = max_resistance * 2;
	  }
	  return set_symetrical(R);
  }
  
  bool set_duty_cycle(float frequency, float duty)	{
    // IMPORTANT - function assumes 'frequency' is actually half the fundamental frequency because other parts of the
    // program work in output frequency, not fundamental frequency.  Must be modified if passed frequency is the fundamental frequency.
	  float period = duty * 500000000 / frequency;
    // float period = duty * 1000000000 / frequency;
	  return set_pulse_time(period);
  }
};

const char freq_mux_0 = 21;
const char freq_mux_1 = 20;

// For gen1 large-parts:
// CLK0:	HIGH, HIGH
// CLK1:	LOW,  HIGH
// CLK2:	LOW,  LOW
#define	CLK0_0	HIGH
#define	CLK0_1	HIGH
#define	CLK1_0	LOW
#define	CLK1_1	HIGH
#define	CLK2_0	LOW
#define	CLK2_1	LOW

const uint8_t SCLPin = 22; //19
const uint8_t SDAPin = 23; //18

// This union makes EEPROM to int32_t conversion easier
union {
  int32_t value;
  byte bytes[4];
} calibrationOffset;

// Blink time count
boolean doBlink = true;
unsigned long lastBlink = 0ul;
unsigned long loopMicros = 0ul;

// Controls for Bi-Frequency pulse mode: ie. each pulse is split into equal parts of the primary and secondary frequencies
boolean doBifreq = false;
volatile uint8_t pulseState = 0;
volatile boolean freqChange = false;
uint8_t pulseStateCopy = 0;

// Pulse IntervalTimer
IntervalTimer pulseTimer;

// Target pulse values specified by user
boolean doPulse = true;
float targetPulseRate = 100; // in Hz
float targetPulsePeriod = 10000.0; // in microseconds
float targetPulseDuty = 0.012; // Percent of pulse period when stimulation occurs
float PulseStimMicroseconds = 120.0; // Stimulate period in microseconds
float PulseOffMicroseconds = 9880.0; // pulse period - stim
float HalfPulseStimeMicros = 60.0; // pulse stim / 2

// Variables for auto-stopping after a certain number of pulses
boolean doTerminate = false;
uint16_t terminateCount = 0;
unsigned long terminateDuration = 0;
unsigned long terminateMicros = 0ul;

// Sweep IntervalTimer
IntervalTimer sweepTimer;

// Controls for sweep mode: stimulate from a start frequerncy to a stop frequency in defined steps that last a certain time
volatile boolean sweepTakeStep = false;
uint64_t startSweepFreq = 10000000ull; // 100 kHz
uint64_t nextSweepFreq = 10000000ull; // 100 kHz
uint64_t stopSweepFreq = 12000000ull; // 120 kHz
uint64_t sweepStepFreq = 50000ull; // 500 Hz
unsigned long sweepStepMicros = 500000ul;

// Programming IntervalTimer
IntervalTimer programTimer;

// Variables for programming / running the stimulation IC
volatile boolean programChange = false;
uint8_t programState   = 0; // 0 - Power, 1 - '1' bit, 2 - '0' bit
float bitMicros1       = 426.67; // Duration in microseconds of each '1' bit
float bitMicros0       = 320.00; // Duration in microseconds of each '1' bit
const uint8_t numBits  = 20; // Number of bits per program cycle
boolean progBits[20];
const uint8_t numHead  = 14; // Number of bits in the header
const uint8_t numAmp   = 5;
boolean headerBits[]   = {1,0,1,0,1,0,1,0,1,0,1,1,1,0,0,0,0,0,1,1,1,1,0,0};
const float progMicros = 5000.0; // Total microseconds for programming
float stimFreq         = 50.0; // IC stimulation frequency in Hz
float stimMicros       = 20000.0; // Stimulation period microseconds minus programming microseconds
uint8_t stimAmp        = 0; // Can be an integer from 0 to 31
boolean stimWidth      = false;
uint8_t progCounter    = 0;
uint8_t bit_counter    = 0;


// Store current frequency
uint64_t freq_set_1 = 10000000ull; // 100.00 kHz - units are in hundredths of Hertz
float frequency_1 = 100000.0;
uint64_t freq_set_2 = 15000000ull;
float frequency_2 = 150000.0;
uint64_t freq_set_3 = 20000000ull;
float frequency_3 = 200000.0;
float target_duty = 0.75;
uint8_t auto_duty = 1;

// Global SI5351 detected flag
bool SI5351Detected = false;

// Global stimulation enabled flag
bool Enable = false;

// Serial input holder
String message = "";

Si5351 si5351;
MCP4 mcp4;

void setup() {
  
  //pinMode(DisablePin,OUTPUT);
  //digitalWriteFast(DisablePin,HIGH);
  pinMode(13,OUTPUT);
  digitalWriteFast(13, HIGH);
  pinMode(freq_mux_0, OUTPUT);
  digitalWriteFast(freq_mux_0, HIGH);
  pinMode(freq_mux_1, OUTPUT);
  digitalWriteFast(freq_mux_1, HIGH);
  message.reserve(33);
  assembleWord();
  Serial.begin(250000);
  while	(!Serial);	// This prevents moving forward until a Serial connection is made.  Remove or limit by time to run headless
  //delay(1500);
  Serial.println("Starting up...");
  Serial.println("Checking EEPROM");
  delay(250);
  // check EEPROM first two bytes for 0x14 & 0xE7, or 5351 in DEC
  if (EEPROM.read(0) == 0x14 && EEPROM.read(1) == 0xE7) {
    calibrationOffset.bytes[0] = EEPROM.read(2);
    calibrationOffset.bytes[1] = EEPROM.read(3);
    calibrationOffset.bytes[2] = EEPROM.read(4);
    calibrationOffset.bytes[3] = EEPROM.read(5);
    Serial.println("EEPROM loaded");
  }
  else {
    calibrationOffset.value = 0;
    Serial.println("No EEPROM data found");
  }
  Serial.println("Initializing MCP4***...");
  delay(250);
  Wire1.begin();
  Wire1.setClock(I2C_RATE_400);
  if (mcp4.init(5000, 5000))  {
    Serial.println("MCP4*** initialized");
  }
  else  {
    Serial.println("MCP4*** Failed");
  }
  Serial.println("Initializing SI5351");
  delay(250);
  // Initialize SI5351 - XRCGB25M000F1H00R0 25 MHz xtal has 6pF load C, using Wire1, 400 kHz bus
  SI5351Detected = si5351.init2(SI5351_CRYSTAL_LOAD_6PF, 0, calibrationOffset.value, 1);
  if (SI5351Detected) {
    si5351.set_freq(freq_set_1, SI5351_CLK0); // Set output
	  si5351.set_freq(freq_set_2, SI5351_CLK1); // Set output
	  si5351.set_freq(freq_set_3, SI5351_CLK2); // Set output
    disableAll(); // Disable all clocks after first frequency set
    si5351.update_status();
  	if (mcp4.is_connected)	{
  	  mcp4.set_duty_cycle(frequency_1, target_duty);
  	}
    Serial.println("SI5351 initialized");
  }
  else  {
    Serial.println("SI5351 Failed");
  }
  delay(250);
  Serial.println("Setup Complete\n");
}

void loop() {
  loopMicros = micros();
  if (freqChange) {
    if (pulseState == 2)  {
	  set_clock(1);
    }
    else  {
	  set_clock(0);
    }
  }
  if (programChange)  {
    programChange = false;
    checkProgramState();
    set_clock(programState);
    if (programState == 0)  {
      programTimer.begin(programmer,stimMicros);
    }
    else if (programState == 1)  {
      programTimer.begin(programmer,bitMicros1);
    }
    else  {
      programTimer.begin(programmer,bitMicros0);
    }
  }
  if (Enable && doTerminate && loopMicros - terminateMicros >= terminateDuration) {
    pulseTimer.end();
    disableAll();
    Enable = false;
    Serial.println("Stimulation auto-Terminated");
  }
  if (sweepTakeStep)  {
    if (nextSweepFreq > stopSweepFreq)  {
      sweepTimer.end();
      disableAll();
      si5351.set_freq(freq_set_1, SI5351_CLK0);
      Serial.println("Sweep Completed");
    }
    else  {
      si5351.set_freq(nextSweepFreq, SI5351_CLK0);
      nextSweepFreq += sweepStepFreq;
    }
  }
  if (doBlink && loopMicros - lastBlink >= 500000) {
    digitalWriteFast(13,!digitalReadFast(13));
    lastBlink = loopMicros;
  }
  if (Serial.available()) {
    message = "";
    for (char ii = 0; ii < 65; ii++)  {
      char c = Serial.read();
      if (c != 10)  {
        message += c;
        if (message.length() == 64) {
          break;
        }
      }
      else {
        break;
      }
    }
    parseMessage();
    message = "";
  }
}


inline void parseMessage() {
  if (message.startsWith("stop")) {
    stopStim();
    Serial.println("Stopped");
  }
  else if (message.startsWith("start")) {
    if (!Enable)  {
      Enable = true;
      enableAll();
      si5351.set_freq(freq_set_1, SI5351_CLK0);
      freqChange = false;
      pulseState = 1;
      if (doPulse)  {
        if (doBifreq) {
          pulseTimer.begin(pulser1,HalfPulseStimeMicros);
        }
        else  {
          pulseTimer.begin(pulser2,PulseStimMicroseconds);
        }
      }
      //digitalWriteFast(DisablePin,LOW);
      terminateMicros = micros();
      Serial.println("Stimulating");
    }
    else  {
      Serial.println("start disregarded - stimulating");
    }
  }
  else if (message.startsWith("sweep")) {
    if (!Enable)  {
      Enable = true;
      enableAll();
      si5351.set_freq(startSweepFreq, SI5351_CLK0);
      sweepTakeStep = false;
      nextSweepFreq = startSweepFreq + sweepStepFreq;
      //digitalWriteFast(DisablePin,LOW);
      sweepTimer.begin(sweeper,sweepStepMicros);
      Serial.println("Sweeping");
    }
    else  {
      Serial.println("sweep disregarded - stimulating");
    }
  }
  else if (message.startsWith("ICset")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      ind += 1;
      int c1 = message.indexOf(',',ind);
      if (c1 > 0) {
        c1 += 1;
        int c2 = message.indexOf(',',c1);
        if (c2 > 0) {
        c2 += 1;
        float val1 = atof(message.substring(ind).c_str());
        float val2 = atof(message.substring(c1).c_str());
        float val3 = atof(message.substring(c2).c_str());
        if (val1 >= 5000 && val1 <= 1000000 && val2 >= 5000 && val2 <= 1000000 && val3 >= 5000 && val3 <= 1000000) {
          frequency_1 = val1;
          frequency_2 = val2;
          bitMicros1 = 32000000 / val2;  // Calculate '1' bit microseconds
          frequency_3 = val3;
          bitMicros0 = 32000000 / val3;  // Calculate '0' bit microseconds
          freq_set_1 = (uint64_t)((val1 * SI5351_FREQ_MULT) + 0.5) << 1;
          freq_set_2 = (uint64_t)((val2 * SI5351_FREQ_MULT) + 0.5) << 1;
          freq_set_3 = (uint64_t)((val3 * SI5351_FREQ_MULT) + 0.5) << 1;
          if (auto_duty == 1)	{
            mcp4.set_duty_cycle(frequency_1, target_duty);
          }
          else if (auto_duty == 2)	{
            mcp4.set_duty_cycle(frequency_2, target_duty);
          }
          else if (auto_duty == 3)	{
            mcp4.set_duty_cycle(frequency_3, target_duty);
          }
          si5351.set_freq(freq_set_1,SI5351_CLK0);
          si5351.set_freq(freq_set_2,SI5351_CLK1);
          si5351.set_freq(freq_set_3,SI5351_CLK2);
          si5351.update_status();
          if (!si5351.dev_status.LOL_A) {
            printStatus();
          }
          else  {
            Serial.print("LossOfLock_A: ");
            Serial.println(si5351.dev_status.LOL_A);
          }
        }
        else{
        Serial.print("At least one value out of range: ");
        Serial.print(val1);
        Serial.print("\t");
        Serial.print(val2);
        Serial.print("\t");
        Serial.println(val3);
        }
      }
      else{
      Serial.println("Comma 2 not found");
      }
      }
      else{
      Serial.println("Comma 1 not found");
      }
    }
    else{
    Serial.println("Colon not found");
    }
  }
  else if (message.startsWith("freq1")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      float val = atof(message.substring(ind+1).c_str());
      if (val >= 5000 && val <= 1000000) {
        frequency_1 = val;
        freq_set_1 = (uint64_t)((val1 * SI5351_FREQ_MULT) + 0.5) << 1;
    		if (auto_duty == 1)	{
    		  mcp4.set_duty_cycle(frequency_1, target_duty);
    		}
        si5351.set_freq(freq_set_1,SI5351_CLK0);
        si5351.update_status();
        if (!si5351.dev_status.LOL_A) {
          Serial.print("Frequency set to: ");
          Serial.println(frequency_1,2);
        }
        else  {
          Serial.print("LossOfLock_A: ");
          Serial.println(si5351.dev_status.LOL_A);
        }
      }
      else  {
        Serial.println("Invalid frequency");
      }
    }
    else  {
      Serial.println("Invalid frequency");
    }
  }
  else if (message.startsWith("freq2")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      float val = atof(message.substring(ind+1).c_str());
      if (val >= 2500 && val <= 2000000) {
        frequency_2 = val;
        bitMicros1 = 32000000 / val;  // Calculate '1' bit microseconds
        freq_set_2 = (uint64_t)((val2 * SI5351_FREQ_MULT) + 0.5) << 1;
    		if (auto_duty == 2)	{
    		  mcp4.set_duty_cycle(frequency_2, target_duty);
    		}
        si5351.set_freq(freq_set_2,SI5351_CLK1);
        si5351.update_status();
        if (!si5351.dev_status.LOL_A) {
          Serial.print("Frequency set to: ");
          Serial.println(frequency_2,2);
        }
        else  {
          Serial.print("LossOfLock_A: ");
          Serial.println(si5351.dev_status.LOL_A);
        }
      }
      else  {
        Serial.println("Invalid frequency");
      }
    }
    else  {
      Serial.println("Invalid frequency");
    }
  }
  else if (message.startsWith("freq3")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      float val = atof(message.substring(ind+1).c_str());
      if (val >= 2500 && val <= 2000000) {
        frequency_3 = val;
        bitMicros0 = 32000000 / val;  // Calculate '0' bit microseconds
        freq_set_3 = (uint64_t)((val3 * SI5351_FREQ_MULT) + 0.5) << 1;
    		if (auto_duty == 3)	{
    		  mcp4.set_duty_cycle(frequency_3, target_duty);
    		}
        si5351.set_freq(freq_set_3,SI5351_CLK2);
        si5351.update_status();
        if (!si5351.dev_status.LOL_A) {
          Serial.print("Frequency set to: ");
          Serial.println(frequency_3,2);
        }
        else  {
          Serial.print("LossOfLock_A: ");
          Serial.println(si5351.dev_status.LOL_A);
        }
      }
      else  {
        Serial.println("Invalid frequency");
      }
    }
    else  {
      Serial.println("Invalid frequency");
    }
  }
  else if (message.startsWith("duty")) {
    int ind = message.indexOf(':',4);
    if (ind > 0) {
      float val = atof(message.substring(ind+1).c_str());
	  float freq;
      if (val > 0.05 && val <= 0.95) {
		target_duty = val;
		if (digitalReadFast(freq_mux_0) == HIGH)	{
		  mcp4.set_duty_cycle(frequency_1, val);
		  auto_duty = 1;
		  freq = frequency_1;
		}
		else if (digitalReadFast(freq_mux_1) == HIGH)	{
		  mcp4.set_duty_cycle(frequency_2, val);
		  auto_duty = 2;
		  freq = frequency_2;
		}
		else{
		  mcp4.set_duty_cycle(frequency_3, val);
		  auto_duty = 3;
		  freq = frequency_3;
		}
		// Calculate actual Duty Cycle for output!!!!!!!!!!!!
		Serial.print("Duty cycle set to ~");
		Serial.print(mcp4.pulse_time * 0.000000001 * freq);
		Serial.print(" of frequency ");
		Serial.print(freq);
		Serial.print(" for a pulse time of ");
		Serial.print(mcp4.pulse_time);
		Serial.println(" ns");
	  }
	}
  }
  else if (message.startsWith("res")) {
    int ind = message.indexOf(':',3);
    if (ind > 0) {
      float val = atof(message.substring(ind+1).c_str());
	  Serial.print("Resistance Setpoint: ");
	  Serial.println(val);
      if (val >= R_OFFSET && val <= (200000 + R_OFFSET)) {
		auto_duty = 0;
        if (mcp4.set_symetrical(val))	{
			Serial.print("DigiPot set to:\t");
			Serial.print(mcp4.resistance0);
			Serial.print("\t");
			Serial.print(mcp4.resistance1);
			Serial.print("\tTotaling: ");
			Serial.println(mcp4.resistance0 + mcp4.resistance1 + R_OFFSET);
		}
		else{
			Serial.println("res FAILED");
		}
      }
    }
  }
  else if (message.startsWith("ns"))	{
	  int ind = message.indexOf(':',2);
	  if (ind > 0) {
		  float val = atof(message.substring(ind+1).c_str());
		  if (MIN_NS <= val && val <= MAX_NS)	{
			  auto_duty = 0;
			  Serial.print("MCP4 Pulse ns Setpoint:\t");
			  Serial.println(val);
			  if (mcp4.set_pulse_time(val))	{
				  Serial.print("Resistance:\t");
				  Serial.print(mcp4.total_resistance);
				  Serial.print("\tMV Pulse Time (ns):\t");
				  Serial.println(mcp4.pulse_time);
			  }
			  else{
				  Serial.println("Setting MCP4 Failed");
			  }
		  }
		  else{
			  Serial.println("MCP4 Pulse Time Out of Range");
		  }
	  }
  }
  else if (message.startsWith("out"))	{
	int ind = message.indexOf(':',3);
    if (ind > 0) {
		int val = message.substring(ind+1).toInt();
		if (0 <= val && val <=2)	{
		  set_clock(val);
		  Serial.print("Clock ");
		  Serial.print(val);
		  Serial.println(" Selected");
		}
		else	{
			Serial.println("Invalid Clock ID (0, 1, & 2 are valid)");
		}
	}
  }
  else if (message.startsWith("stimf")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      float val = atof(message.substring(ind+1).c_str());
      if (val > 0 && val <= 150) {
        stimFreq = val;
        stimMicros = 1000000 / val;
      }
    }
  }
  else if (message.startsWith("stima")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val >= 0 && val <= 31) {
        stimAmp = (uint8_t)val;
      }
    }
  }
  else if (message.startsWith("stimw")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val == 0 || val == 1) {
        stimWidth = (boolean)val;
      }
    }
  }
  else if (message.startsWith("stims")) {
    if (!Enable)  {
      Enable = true;
      enableAll();
      // si5351.set_freq(freq_set_1, SI5351_CLK0);
      // si5351.set_freq(freq_set_2, SI5351_CLK1);
      // si5351.set_freq(freq_set_3, SI5351_CLK2);
      freqChange = false;
      pulseState = 1;
      terminateMicros = loopMicros;
      programChange = false;
      programState = 0;
      progCounter = 0;
      programTimer.begin(programmer,stimMicros);
      Serial.println("Powering Implant");
    }
    else  {
      Serial.println("start disregarded - stimulating");
    }
  }
  else if (message.startsWith("bifreq")) {
    if (doBifreq) {
      doBifreq = false;
      Serial.println("Single frequency pulse mode");
    }
    else  {
      doBifreq = true;
      Serial.println("Bi-Frequency pulse mode");
    }
  }
  else if (message.startsWith("swstart")) {
    int ind = message.indexOf(':',7);
    if (ind > 0) {
      float val = atof(message.substring(ind+1).c_str());
      if (val >= 2500 && val <= 2000000) {
        val *= SI5351_FREQ_MULT;
        val += 0.5;
        startSweepFreq = (uint64_t)val;
      }
      else  {
        Serial.println("Invalid frequency");
      }
    }
    else  {
      Serial.println("Invalid frequency");
    }
  }
  else if (message.startsWith("swstop")) {
    int ind = message.indexOf(':',6);
    if (ind > 0) {
      float val = atof(message.substring(ind+1).c_str());
      if (val >= 2500 && val <= 2000000) {
        val *= SI5351_FREQ_MULT;
        val += 0.5;
        stopSweepFreq = (uint64_t)val;
      }
      else  {
        Serial.println("Invalid frequency");
      }
    }
    else  {
      Serial.println("Invalid frequency");
    }
  }
  else if (message.startsWith("swstep")) {
    int ind = message.indexOf(':',6);
    if (ind > 0) {
      float val = atof(message.substring(ind+1).c_str());
      if (val > 0 && val <= 2000000) {
        val *= SI5351_FREQ_MULT;
        val += 0.5;
        sweepStepFreq = (uint64_t)val;
      }
      else  {
        Serial.println("Invalid frequency");
      }
    }
    else  {
      Serial.println("Invalid frequency");
    }
  }
  else if (message.startsWith("swrate")) {
    int ind = message.indexOf(':',6);
    if (ind > 0) {
      float val = message.substring(ind+1).toFloat();
      if (val > 0 && val <= 100)  {
        sweepStepMicros = (unsigned long)((val * 1000000.0) + 0.5);
        Serial.print("Sweep step rate set to ");
        Serial.print(val);
        Serial.println(" Hz.");
      }
      else  {
        Serial.println("Invalid Sweep Step Rate");
      }
    }
    else  {
      Serial.println("Invalid Sweep Step Rate");
    }
  }
  else if (message.startsWith("count")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val >= 0 && val <= 1000000) {
        if (val > 0)  {
          doTerminate = true;
          terminateCount = val;
          terminateDuration = (unsigned long)((terminateCount*1000000)/targetPulseRate);
          if (Enable) {
            terminateMicros = loopMicros;
          }
          Serial.print(val);
          if (val > 1)  {
            Serial.println(" pulses per start");
          }
          else  {
            Serial.println(" pulse per start");
          }
        }
        else  {
          doTerminate = false;
          Serial.println("No pulse limit");
        }
      }
    }
  }
  else if (message.startsWith("init")) {
    SI5351Detected = si5351.init2(SI5351_CRYSTAL_LOAD_6PF, 0, calibrationOffset.value, 1);
    if (SI5351Detected) {
      //Wire.setClock(400000L); // Set I2C to fast mode (400 kHz)
      // Set clocks 0, 1, & 2 to use the same MultiSynth source
      // si5351.set_clock_source(SI5351_CLK0, SI5351_CLK_SRC_MS0);
      // si5351.set_clock_source(SI5351_CLK1, SI5351_CLK_SRC_MS0);
      // si5351.set_clock_source(SI5351_CLK2, SI5351_CLK_SRC_MS0);
      si5351.set_freq(freq_set_1, SI5351_CLK0); // Set output
	  si5351.set_freq(freq_set_2, SI5351_CLK1); // Set output
	  si5351.set_freq(freq_set_3, SI5351_CLK2); // Set output
      disableAll(); // Disable all clocks after first frequency set
      si5351.update_status();
    }
  }
  else if (message.startsWith("LED")) {
    int ind = message.indexOf(':',3);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val > 0)  {
        if (!doBlink) {
          doBlink = true;
          digitalWriteFast(13,HIGH);
          lastBlink = loopMicros;
        }
      }
      else  {
        doBlink = false;
        digitalWriteFast(13,LOW);
      }
    }
  }
  else if (message.startsWith("check")) {
    printStatus();
  }
  else if (message.startsWith("ID")) {
    Serial.println("Teensy LC - Coil Driver SI5351 BiF SI5351 PWM");
  }
 // else if (message.startsWith("")) {
   
 // }
  else  {
    Serial.print("Command Unrecognized: <");
    Serial.print(message);
    Serial.println(">");
  }
}

// The following 3 functions manage the Pulse functionality, including Bi-Frequency switching
void pulser1()  {
  pulseState = 2;
  freqChange = true;
  pulseTimer.begin(pulser2,HalfPulseStimeMicros);
}

void pulser2()  {
  pulseTimer.begin(pulser3,PulseOffMicroseconds);
  if (pulseState != 1)  {
    pulseState = 1;
    freqChange = true;
  }
}

void pulser3()  {
  if (doBifreq) {
    pulseTimer.begin(pulser1,HalfPulseStimeMicros);
  }
  else  {
    pulseTimer.begin(pulser2,PulseStimMicroseconds);
  }
}

// Callback function for sweep intervaltimer
void sweeper()  {
  sweepTakeStep = true;
  sweepTimer.begin(sweeper,sweepStepMicros);
}

// Callback function for programming and running the ASIC
void programmer() {
  programChange = true;
}

inline void printStatus() {
  Serial.print("SI5351: ");
  if (SI5351Detected) {
    Serial.print("Detected  - ");
    Serial.print("Enable: ");
    Serial.println(Enable);
    Serial.print("Frequency 1: ");
    Serial.print(frequency_1,2);
    Serial.write(9);
	Serial.print("Frequency 2: ");
    Serial.print(frequency_2,2);
    Serial.write(9);
    Serial.print("Frequency 3: ");
    Serial.println(frequency_3,2);
  }
  else  {
    Serial.println("Not Found");
  }
  
  Serial.print("MCP4***: ");
  if (mcp4.is_connected)  {
    Serial.println("Detected");
    Serial.print("Resistance 0: ");
    Serial.print(mcp4.resistance0);
    Serial.print("\tResistance 1: ");
    Serial.print(mcp4.resistance1);
    Serial.print("\tTotal: ");
    Serial.println(mcp4.resistance0 + mcp4.resistance1 + R_OFFSET);
	Serial.print("Pulse Time (ns): ");
	Serial.println(mcp4.pulse_time);
  }
  else  {
    Serial.println("Not Found");
  }
  
  if (SI5351Detected && mcp4.is_connected)	{
	float freq;
	if (digitalReadFast(freq_mux_0) == HIGH)	{
	  freq = frequency_1;
	}
	else if (digitalReadFast(freq_mux_1) == HIGH)	{
	  freq = frequency_2;
	}
	else{
	  freq = frequency_3;
	}
	Serial.print("Duty Cycle Setpoint: ");
	Serial.print(target_duty);
	Serial.print("\tCurrent Duty Cycle: ");
	Serial.println(mcp4.pulse_time * 0.000000001 * freq);
  }
  
  Serial.print("Pulse: ");
  if (doPulse)  {
    Serial.print("Enabled\t");
  }
  else  {
    Serial.print("Disabled\t");
  }
  Serial.print("Pulse frequency: ");
  Serial.print(targetPulseRate,2);
  Serial.write(9);
  Serial.print("Pulse Duty: ");
  Serial.println(targetPulseDuty,3);
  Serial.print("Pulses per start: ");
  if (terminateCount == 0) {
    Serial.println("Unlimited");
  }
  else  {
    Serial.println(terminateCount);
  }
}

inline void stopStim() {
  if (Enable) {
    pulseTimer.end();
    sweepTimer.end();
    programTimer.end();
    disableAll();
    Enable = false;
    pulseState = 1;
    freqChange = true;
    programChange = false;
    programState = 0;
    progCounter = 0;
  }
}

inline void enableAll()  {
  if (SI5351Detected) {
    si5351.si5351_write(SI5351_OUTPUT_ENABLE_CTRL, 0x00);
  }
}

inline void disableAll() {
  if (SI5351Detected) {
    si5351.si5351_write(SI5351_OUTPUT_ENABLE_CTRL, 0xFF);
  }
}

inline void set_clock(uint8_t clock)	{
  if (clock == 0)	{
	digitalWriteFast(freq_mux_0, CLK0_0);
	digitalWriteFast(freq_mux_1, CLK0_1);
  }
  else if (clock == 1)	{
	digitalWriteFast(freq_mux_0, CLK1_0);
	digitalWriteFast(freq_mux_1, CLK1_1);
  }
  else if (clock == 2)	{
	digitalWriteFast(freq_mux_0, CLK2_0);
	digitalWriteFast(freq_mux_1, CLK2_1);
  }
}

inline void assembleWord()  {
  bit_counter = 0;
  for (uint8_t ii = 0; ii < numHead; ii++) {
    progBits[bit_counter] = headerBits[ii];
    bit_counter += 1;
  }
  for (uint8_t ii = 0; ii < numAmp; ii++) {
    progBits[bit_counter] = stimAmp >> ii;
    bit_counter += 1;
  }
  progBits[19] = stimWidth;
}

inline void checkProgramState() {
  if (progCounter >= numBits) {
    programState = 0;
    progCounter = 0;
  }
  else  {
    if (progBits[progCounter])  {
      programState = 1;
    }
    else  {
      programState = 2;
    }
    progCounter += 1;
  }
}
