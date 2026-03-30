#define F_CPU 16000000
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2c_master.h"
#include "controller.h"

// ATMEGA 328p pins
//
// PB0 : SHIFT RESET
// PB1 : NC
// PB2 : SPI SS
// PB3 : SPI MOSI
// PB4 : SPI MISO
// PB5 : SPI SCK
// PB6 : NC 
// PB7 : NC 
// PC0 : PCINT8 : rotary encoder A
// PC1 : PCINT9 : rotary encoder B
// PC2 : PCINT10 : rotarty encoder button?
// PC3 : I2C SCL
// PC4 : I2C SDA
// PC5 : NC 
// PC6 : NC 
// PD0 : NC 
// PD1 : External Button
// PD2 : INT0 RTC
// PD3 : OC2B
// PD4 : NC
// PD5 : NC
// PD6 : NC
// PD7 : NC

#define BUTTON_PIN PD1 // external button input
#define SCOPE_PIN PD6 // scope power controller

#include <stdio.h>
#include "ArduinoCompat.h"
#include "AvrSdSpiDriver.h"
#include "SdFat.h"
#include "SdFatConfig.h"


static const uint8_t rtc_device_address = 0xD0;
static const uint8_t rtc_address_seconds = 0x00;
static const uint8_t rtc_address_minutes = 0x01;
static const uint8_t rtc_address_hours = 0x02;
static const uint8_t rtc_address_date = 0x04;
static const uint8_t rtc_address_month = 0x05;
static const uint8_t rtc_address_year = 0x06;
static const uint8_t rtc_address_am1 = 0x07;
static const uint8_t rtc_address_am2 = 0x08;
static const uint8_t rtc_address_am3 = 0x09;
static const uint8_t rtc_address_am4 = 0x0a;
static const uint8_t rtc_address_conrol = 0x0e;
static const uint8_t rtc_address_status = 0x0f;

static const uint8_t mag_device_addr = 0x44;
static const uint8_t mag_addr_config1 = 0x00;
static const uint8_t mag_addr_config2 = 0x01;
static const uint8_t mag_addr_conf_sensor  = 0x02;
static const uint8_t mag_config_rd_mode = 0x01;
static const uint8_t mag_config_samp_rate_4k = 0x0c; // 4.4k sample rate (8x average)
static const uint8_t mag_config_samp_rate_2k = 0x10; // 2.4k sample rate (16x average)
static const uint8_t mag_config_samp_rate_1k = 0x14; // 1.2k sample rate (32x average)
static const uint8_t mag_config_cont_samp = 0x02; // continuously sample
static const uint8_t mag_config_chan = 0x40; // enable Z axis only
                                
uint16_t mag_range_min = 3000; // FIXME 
uint16_t mag_range_max = 15000; // FIXME expect values between 1000 and 3000

// default data rate to 40 Hz
uint8_t data_rate_hz = 40;

uint16_t fast_timer_rate = 4000;
uint16_t read_rate = 400;
// must be updated if data rate changes

uint8_t mode=0;
//uint8_t cur_day=0;
                         
// count data writes as a timer for
// oscilloscope timeout
volatile uint16_t scope_timeout_count=0;
volatile uint16_t fast_read_count=0;
volatile uint16_t write_count=0;

volatile uint8_t fast_read_flag=0;
volatile uint8_t write_flag=0;
volatile uint8_t update_date_flag=0;
volatile uint8_t sync_flag=0;

uint16_t scope_timeout_sec=120; // 2 minutes timeout
uint8_t scope_enabled=1; 
                                
// must be updated if data rate changes
//
uint16_t fast_timer_count_thresh=0;
uint16_t read_count_thresh=0;
uint16_t scope_timeout_thresh=0;
uint8_t write_count_thresh=0;
uint16_t ms_increment=0;
                              
// Holds the current date/time
struct DateTime cur_dt;
volatile uint32_t cur_second=0;
volatile uint16_t cur_ms=0;

SdFat sd;
FatFile ofile;

// *************************************
// millis function replaces a function
// in sdfat that otherwise relies
// on the Arduino.h library which we 
// do not use.
// g_millis should be updated in the 
// fast timer
// *************************************
static volatile uint32_t g_millis = 0;  // counts
static volatile uint8_t  sub_ms = 0;   // counts 0.25ms ticks
extern "C" uint32_t millis(void)
{
    uint32_t m;
    uint8_t sreg = SREG;
    cli();
    m = g_millis;
    SREG = sreg;
    return m;
}


// *************************************
// convert bits as read from DTC module
// into the DateTime structure
void convert_bits_to_datetime( struct DateTime *dt )
{

    uint8_t secs_ones = (dt->second_bits & 0x0f);
    uint8_t secs_tens = (dt->second_bits & 0x70) >> 4;

    dt->second = secs_tens*10 + secs_ones;

    uint8_t mins_ones = (dt->minute_bits & 0x0f);
    uint8_t mins_tens = (dt->minute_bits & 0x70) >> 4;

    dt->minute = mins_tens*10 + mins_ones;

    uint8_t hour_ones = (dt->hour_bits & 0x0f);
    uint8_t hour_tens = (dt->hour_bits & 0x30) >> 4;

    dt->hour = hour_tens*10 + hour_ones;

    uint8_t date_ones = (dt->date_bits & 0x0f);
    uint8_t date_tens = (dt->date_bits & 0x30) >> 4;

    dt->date = date_tens*10 + date_ones;

    uint8_t month_ones = (dt->month_bits & 0x0f);
    uint8_t month_tens = (dt->month_bits & 0x10) >> 4;

    dt->month = month_tens*10 + month_ones;

    uint8_t year_ones = (dt->year_bits & 0x0f);
    uint8_t year_tens = (dt->year_bits & 0xf0) >> 4;

    uint8_t century = (dt->month_bits & 0x80);

    dt->year = 2000 + (uint16_t)(century*100 + year_tens*10 + year_ones);

}

// *************************************
// convert the DateTime structure
// into bits to write to DTC module
// Only used if writing time
void convert_datetime_to_bits(struct DateTime *dt)
{

    // seconds (00–59): bits 0–3 ones, 4–6 tens
    dt->second_bits =
        ((dt->second / 10) << 4) |
        (dt->second % 10);

    // minutes (00–59)
    dt->minute_bits =
        ((dt->minute / 10) << 4) |
        (dt->minute % 10);

    // hours (00–23): bits 0–3 ones, 4–5 tens
    dt->hour_bits =
        ((dt->hour / 10) << 4) |
        (dt->hour % 10);

    // date (01–31)
    dt->date_bits =
        ((dt->date / 10) << 4) |
        (dt->date % 10);

    // month (01–12): bits 0–3 ones, bit 4 tens
    dt->month_bits =
        ((dt->month / 10) << 4) |
        (dt->month % 10);

    // year stored as offset from 2000
    uint16_t y = dt->year - 2000;
    dt->month_bits = dt->month_bits | ((y/100) << 7);
    dt->year_bits =
        ((y / 10) << 4) |
        (y % 10);

}

// *************************************
// convert the hour, minute, and second
// into a count of seconds in the day
uint32_t get_cur_second_from_dt(struct DateTime dt) 
{
    return (uint32_t)dt.second+60*(uint32_t)dt.minute+3600*(uint32_t)dt.hour;
} 

// *************************************
// After the DTC alarm sends an interrupt
// reset it so that it will send the next
// interrupt
uint8_t reset_dtc_alarm(void)
{
    // Read status
    uint8_t res = 0;
    uint8_t status;
    res |= i2c_start(rtc_device_address);      // 0xD0 write
    res |= i2c_write(rtc_address_status);
    res |= i2c_start(rtc_device_address | 0x01); // 0xD1 read
    status = i2c_read_nack();
    i2c_stop();
    
    // Clear A1F (bit0)
    status &= ~(1 << 0);
    
    // Write status back
    res |= i2c_start(rtc_device_address);
    res |= i2c_write(rtc_address_status);
    res |= i2c_write(status);
    i2c_stop();

    return res;

}

// *************************************
// setup the DTC alarm to send an 
// interrupt every second
uint8_t configure_dtc_alarm(void)
{

    uint8_t res = 0;
    res |= i2c_start(rtc_device_address);
    // set alarm 1 to interrupt every second
    res |= i2c_write(rtc_address_am1);
    res |= i2c_write(0x80);
    res |= i2c_write(0x80);
    res |= i2c_write(0x80);
    res |= i2c_write(0x80);
    i2c_stop();

    i2c_init();
    res |= i2c_start(rtc_device_address);
    res |= i2c_write(rtc_address_conrol);
    // enable interrupts on alarm1
    res |= i2c_write(0x05);
    i2c_stop();

    res |= reset_dtc_alarm();

    return res;

}

// *************************************
// Read current time from DTC module
void read_time(struct DateTime *dt)
{
    // make sure the register pointer 
    // points to the 0th register
    i2c_start(rtc_device_address);
    i2c_write(rtc_address_seconds);
    i2c_stop();

    // R/W bit to 1 to enable reading
    i2c_start(rtc_device_address | 0x01);
    dt->second_bits = i2c_read_ack();
    dt->minute_bits = i2c_read_ack();
    dt->hour_bits = i2c_read_ack();
    (void) i2c_read_ack(); // do not use DoW
    dt->date_bits = i2c_read_ack();
    dt->month_bits = i2c_read_ack();
    dt->year_bits = i2c_read_nack();

    convert_bits_to_datetime(dt);

}
                                  

// *************************************
// Write time to DTC module
// Only use when setting the time
uint8_t write_time(struct DateTime dt)
{

    uint8_t res = i2c_start(rtc_device_address);
    // start writing at the first address
    // The DS3231 automatically 
    // advnaces to the next address
    // at each write
    i2c_write(rtc_address_seconds);
    
    i2c_write(dt.second_bits);
    i2c_write(dt.minute_bits);
    i2c_write(dt.hour_bits);
    // we don't care about the DoW
    // set it to 0x01 so that 
    // we can set bytes sequentially
    i2c_write(0x01);
    i2c_write(dt.date_bits);
    i2c_write(dt.month_bits);
    i2c_write(dt.year_bits);
    i2c_stop();

    return res;
}


// *************************************
// configure the hall sensor 
// it should read only the Z channel
// and should internally update the
// value at about 1.1 kHz
uint8_t configure_mag_sensor(void)
{

    uint8_t res = 0;
    res |= i2c_start(mag_device_addr);
    res |= i2c_write(mag_addr_config1); //1st device config register
    i2c_write(mag_config_rd_mode | mag_config_samp_rate_1k); 
    i2c_write(mag_config_cont_samp); //2nd device config register
    i2c_write(mag_config_chan); // configure to read only Z channel
    i2c_stop(); // no further config needed
                
    return res;
}


// *************************************
// read the 15-bit sensor value
// (highest bit is sign which is not used)
uint16_t read_mag_sensor(void)
{
    i2c_start(mag_device_addr | 0x01); // set the read bit
    uint8_t mag_high = i2c_read_ack();
    uint8_t mag_low = i2c_read_ack();
    // conversion status, can be read 
    // if needed
    (void)i2c_read_nack();
    i2c_stop();

    uint16_t mag_val = (uint16_t)(mag_high << 8) | mag_low;

    return mag_val;
}

// *************************************
// disable the ADC as it is not needed
// this just saves some power
void disable_adc(void) {

    ACSR = 0x80;
}

// *************************************
// Enable output port for the 
// scope controller
void setup_scope(void){

    DDRD |= (0x1 << SCOPE_PIN);
}

// *************************************
// Enable scope by updating pin signal
// and updating the scope count
void enable_scope(void) {
    
    PORTD |= (0x1 << SCOPE_PIN); // enable PWM
    scope_timeout_count = 0;
    scope_enabled = 1;

}

// *************************************
// Disable scope by updating the pin signal
// and disabling the PWM
void disable_scope(void) {
    
    PORTD &= ~(0x1 << SCOPE_PIN);
    // no output on PWM pin
    OCR2B = 0;
    scope_enabled = 0;

}

// *************************************
// enable the timer with an upper value
// of comp_val and prescale of 1
// w.r.t system clock
void enable_fast_timer(uint16_t comp_val) 
{

    // disable timers
    TCCR1B = 0;
    TCCR1A = 0;

    // enable timer with prescale of 1
    TCCR1B = (0x1 << CS10) | (0x1 << WGM12);
    // enable output compare match A interrupt
    // Note: Interrupts need to be enabled
    TIMSK1 = (0x1 << OCIE1A);

    OCR1A = comp_val;

}

// *************************************
// update the timer based on the input
// value. Not Used
void update_read_timer(uint16_t comp_val) 
{

    // disable interrupts while setting to avoid
    // conflicts 
    cli();
    OCR1A = comp_val;
    sei();

}

// *************************************
// enable the timer for PWM
void enable_pwm_timer() 
{

    // disable timers and interrupts
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = 0; 

    // enable output port
    DDRD |= (1 << DDD3);

    // enable PWM mode, flip output on compare match
    TCCR2A = (0x1 << COM2B1) | (0x1 << WGM21) | (0x1 << WGM20);
    //TCCR2B = (0x1 << CS20) | (0x1 << CS21) | (0x1 << CS22);
    TCCR2B = (0x1 << CS20);

    // start with no output
    OCR2B = 0;

}

// *************************************
// scale the value to PWM range and set
void update_pwm(uint16_t mag_val)
{
      if (mag_val <= mag_range_min) {
        OCR2B = 0;
        return;
    }
    if (mag_val >= mag_range_max) {
        OCR2B = 255;
        return;
    }

    uint16_t x    = (uint16_t)(mag_val - mag_range_min);
    uint16_t span = (uint16_t)(mag_range_max - mag_range_min);

    OCR2B = (uint8_t)(((uint32_t)x * 255u) / span);
}

// *************************************
// Initialize the SD card communication
bool sd_init()
{
  // Use SS just as a default cs “id”. If your driver controls CS itself, this can be ignored.
  SdSpiConfig cfg(SS, DEDICATED_SPI, SD_SCK_MHZ(4), &sdSpi);
  return sd.begin(cfg);
}

// *************************************
// for data output, the sub-second timing
// is based on counting millisecond increments
// Nominally it is 1000/40 = 25
uint16_t update_ms_increment(uint8_t read_rate)
{
    return (uint16_t) (1000/read_rate);
}

// *************************************
// Calculate the upper threshold 
// based on the CPU rate and fast 
// timer rate
// Nominally it is 3999 (4000 - 1)
uint16_t update_fast_timer_thresh(uint16_t timer_rate)
{
    return (uint16_t) ((F_CPU/timer_rate) - 1);
}

// *************************************
// Calculate the upper threshold for 
// reading based on the fast timer rate
// and the and the read rate
// Nominally it is 9 (10 - 1)
uint16_t update_read_thresh(uint16_t fast_rate, uint16_t read_rate)
{
    return (uint16_t) ((fast_rate/read_rate) - 1);
}

// *************************************
// Calculate the write count threshold
// for writing to SD card
// Nominally it is 9 (10 - 1)
uint8_t update_write_count_thresh(uint16_t read_rate, uint8_t data_rate)
{
    return (uint8_t)((read_rate/data_rate) -1);
}

// *************************************
// Calculate the scope timeout threshold
// based on the write rate
// Nominally it is 4799 (4800 - 1)
uint16_t update_scope_timeout(uint16_t timeout_sec, uint8_t data_rate)
{
    return timeout_sec*(uint16_t)data_rate;
}

// *************************************
// Write one line to SD card
void write_line(uint32_t second, uint16_t ms, uint16_t value)
{
    char buf[20];
    int len = snprintf(buf, sizeof(buf), "%lu.%u,%u\r\n", second, ms, value);
    ofile.write(buf, len);
}

// *************************************
// Create file name baed on year, month, day
// Name length is limited due to use of
// short file names
void generate_filename(char * out, uint16_t year, uint8_t month, uint8_t day) {

  snprintf(out, 13, "%04u%02u%02u.CSV", year, month, day);
}

// *************************************
// Enable external interrupts
// to receive signal from DTC module
void setup_external_interrupts(void)
{
    // enable INT0 on PD2
    // activated on falling edge
    DDRD &= ~(0x1 << DDD2);
    PORTD |=  (1 << PORTD2);
    EICRA = (EICRA & ~((0x1<<ISC01) | (0x1<<ISC00))) | (0x1<<ISC01);
    EIFR  = (0x1 << INTF0);
    EIMSK |= (0x1 << INT0 );

}

// *************************************
// Internal interupt routine
// to recieve interrupt from RTC module
ISR(INT0_vect){

    // rather than reading the RTC
    // every second, just update the 
    // internal time until seconds count
    // reaches 86400.  Only in this case, 
    // do a full read and update
    cur_second += 1;
    cur_ms = 0;
    sync_flag = 1;
       
    if( cur_second >= 86400 )
    {
        update_date_flag = 1;
        cur_second = 0;
    }
}

// *************************************
// Fast timer interrupt routine
ISR( TIMER1_COMPA_vect ){

    fast_read_count += 1;

    // check all counts that are based
    // on the fast read timer
    // if a count matches/exceeds the threshold, 
    // raise the flag, which will be 
    // utilized in the main loop
    
    // for each fast read, update the
    // write count
    // nominally the read rate should be
    // 400 Hz, with read_count_thresh = 9 (10 - 1)
    if(fast_read_count  > read_count_thresh) {
        write_count += 1;
        fast_read_flag = 1;
        fast_read_count = 0;
    }

    // Nominally the write rate should be
    // 40 Hz, with write_count_thresh = 99 (100 - 1)
    // For each write, increment the scope
    // timeout count 
    if( write_count > write_count_thresh) {
        write_flag = 1;
        write_count = 0;
        scope_timeout_count += 1;
    }

    // 4 ticks = 1ms
    // implement the g_millis 
    if (++sub_ms >= 4) {
        sub_ms = 0;
        g_millis++;
    }

}

int main(void)
{

  disable_adc();

  // Enable output port for
  // exernal button
  DDRD &= ~(1 << DDD1);      // input
  PORTD |= (1 << PORTD1);    // enable internal pull-up

  // calculate timing thresholds
  fast_timer_count_thresh = update_fast_timer_thresh(fast_timer_rate);
  read_count_thresh = update_read_thresh(fast_timer_rate, read_rate);
  write_count_thresh = update_write_count_thresh(read_rate, data_rate_hz);
  scope_timeout_thresh = update_scope_timeout(scope_timeout_sec, data_rate_hz);
  ms_increment = update_ms_increment(data_rate_hz);

  i2c_init();

  configure_dtc_alarm();
  configure_mag_sensor();

  // fast timer at nominally 4000 Hz
  enable_fast_timer(fast_timer_count_thresh);

  //// enable the pwm
  enable_pwm_timer();

  setup_external_interrupts();

  // read the time before enabling the interrupt
  // which can change the time
  read_time(&cur_dt);
  cur_second = get_cur_second_from_dt(cur_dt);

  // enable interrupts
  sei();

  // init SD communication
  sd_init();

  // *************************************
  // Only use when setting the time
  //struct DateTime dt;
  //dt.year = 2026;
  //dt.month=3;
  //dt.date=7;
  //dt.hour=23;
  //dt.minute=48;
  //dt.second=0;
  //convert_datetime_to_bits(&dt);
  //write_time(dt);
  // *************************************

  setup_scope();
  enable_scope();
  update_pwm(0);

  _delay_us(500);

  uint16_t mag_val = 0;

  while(1) {

      if(fast_read_flag)
      {
        fast_read_flag=0;
        mag_val = read_mag_sensor();
        update_pwm(mag_val);
        if( !(PIND & (0x1 << BUTTON_PIN))){
            enable_scope();
        }
      }
      if( write_flag ){
          write_flag = 0;

          if(!ofile.isOpen())
          {
              char filename[13];

              generate_filename(filename, cur_dt.year, cur_dt.month, cur_dt.date);

              ofile.open(filename,O_WRONLY | O_CREAT | O_TRUNC);
          }

          uint32_t write_second=0;
          uint16_t write_ms=0;
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            write_second = cur_second;
            write_ms = cur_ms;
          }
   

          if( write_ms < 1000 )
          {
            write_line(write_second, write_ms, mag_val);
          }
          cur_ms += ms_increment;
         
      }
      if( update_date_flag ) {
          update_date_flag = 0;
          read_time(&cur_dt);
          ofile.close();
      }
      if (sync_flag) {
          sync_flag = 0;
          reset_dtc_alarm();
          ofile.sync();
      }
      if( scope_enabled ) 
      {
          if(scope_timeout_count > scope_timeout_thresh)
          {
              cli();
              scope_timeout_count = 0;
              sei();
              disable_scope();
          }
      }

  }
}

