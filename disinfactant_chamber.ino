
#include <avr/wdt.h>

#define SERIAL_LOG

#define LED_OFF     (0)
#define LED_ON      (1)

#define RELAY_1     (1)
#define RELAY_2     (2)

const uint8_t relay_1_ctrl_pin = 5;
const uint8_t relay_2_ctrl_pin = 6;
const uint8_t heart_beat_led = 13;

const uint8_t sonar_trigger_pin = 2;
const uint8_t sonar_echo_pin = 3;

const uint8_t sonar_no_samples = 4;

uint8_t obj_precense_samples[sonar_no_samples] = {0};

unsigned long s_time = 0;
unsigned long f_time = 0;

typedef enum {
  STATE_IDLE = 0,
  STATE_WAIT_FOR_OBJECT,
  STATE_SPRAY_DISINFACTANT_START,
  STATE_SPRAY_DISINFACTANT_TIMEOUT,
  STATE_SPRAY_DISINFACTANT_END,
  STATE_RELAY_BREAK_MAKE_DELAY,
  STATE_HEART_BEAT
} tSystemStates;

volatile tSystemStates sys_cur_state = STATE_IDLE;
volatile tSystemStates next_sys_state = STATE_IDLE;
volatile tSystemStates prev_sys_state = STATE_IDLE;

static uint8_t next_relay_to_energize;
static uint8_t object_presence_flag = 0;

volatile uint32_t loop_count = 0;
volatile uint32_t spray_timer_counter = 0;
volatile uint32_t relay_break_make_counter = 0;
volatile uint8_t sonar_sample_time_counter = 0;

static uint8_t sonar_sample_counter = 0;
static uint8_t heart_beat_led_state = LED_OFF; 

void sonar_trigger_init(void);
uint16_t sonar_distance_inch(void);
uint8_t object_presence(void);

void sonar_trigger_init(void)
{
  digitalWrite(sonar_trigger_pin, HIGH);
  delayMicroseconds(20);

  digitalWrite(sonar_trigger_pin, LOW);
}

uint16_t sonar_distance_inch(void)
{
  unsigned long pulse_duration = 0;
  uint16_t distance_inch = 0;

  sonar_trigger_init();
  
  pulse_duration = pulseIn(sonar_echo_pin, HIGH, 5000);

  distance_inch = (uint16_t)(pulse_duration / 148);
  
  return (distance_inch);
}

uint8_t object_presence(void) 
{
  uint8_t obj_presence = 0;
  uint16_t distance_inch = 0;

  for (uint8_t i = 0; i < sonar_no_samples; i++)
  {
    sonar_trigger_init();
    distance_inch += sonar_distance_inch();
  }

  distance_inch = (distance_inch / sonar_no_samples);

  if ( (distance_inch >= 5) && (distance_inch <= 30) )
  {
    obj_presence = 1;
  }
  
  return (obj_presence);
}

void setup() 
{
  pinMode(relay_1_ctrl_pin, OUTPUT);
  pinMode(relay_2_ctrl_pin, OUTPUT);
  pinMode(heart_beat_led, OUTPUT);
  pinMode(sonar_trigger_pin, OUTPUT);
  pinMode(sonar_echo_pin, INPUT);
  
  digitalWrite(relay_1_ctrl_pin, HIGH);
  digitalWrite(relay_2_ctrl_pin, HIGH);
  
  digitalWrite(heart_beat_led, LOW);
  digitalWrite(sonar_trigger_pin, LOW);

  #ifndef SERIAL_LOG
    Serial.begin(9600);
  #endif

  wdt_enable(WDTO_4S);
}

void loop() 
{
  switch (sys_cur_state)
  {
    case STATE_IDLE:
    // Turn Relay-1 OFF
    digitalWrite(relay_1_ctrl_pin, HIGH);

    // Turn Relay-2 ON after break-make timeout
    relay_break_make_counter = 0;
    next_relay_to_energize = RELAY_2;

    // Save next system state after break-make delay
    next_sys_state = STATE_WAIT_FOR_OBJECT;
    
    // Update current system state
    sys_cur_state = STATE_RELAY_BREAK_MAKE_DELAY;

    #ifndef SERIAL_LOG
      Serial.println(" State - Idle ");
    #endif
    
    break;

    case STATE_WAIT_FOR_OBJECT:
      //object_presence_flag = object_presence();
      
      sonar_sample_time_counter++;
      
      // Wait at least 60ms to start another iteration of sonar sample
      if (sonar_sample_time_counter >= 60) 
      {
        uint16_t dist = sonar_distance_inch();
        
        // Check if object is within range between 2-inch to 24-inch
        if ( (dist >= 2) && (dist <= 24) )
        {
          // Object within limit, put '1' for this sample in the storage buffer
          obj_precense_samples[sonar_sample_counter] = 1;
        }
        else
        {
          // Object off limit, put '0' for this sample in the storage buffer
          obj_precense_samples[sonar_sample_counter] = 0;
        }

        // increase sample counter for next iteration
        sonar_sample_counter++;

        // Check if all samples are taken to perform a calculation
        if (sonar_sample_counter == sonar_no_samples)
        {
          uint8_t sonar_valid_sample = 0;
          for (uint8_t i = 0; i < sonar_no_samples; i++)
          {
            if (obj_precense_samples[i] == 1)
            {
              sonar_valid_sample++;
            }

            // We will consider an object presence if (total sample - 1) 
            // samples are within range
            if ( sonar_valid_sample >= (sonar_no_samples-1) )
            {
              object_presence_flag = 1;
            }
          }

          sonar_sample_counter = 0;
        }

        sonar_sample_time_counter = 0;
      }
      
      if (object_presence_flag) 
      {
        object_presence_flag = 0;
        
        sys_cur_state = STATE_SPRAY_DISINFACTANT_START;
      }
      else
      {
        prev_sys_state = STATE_WAIT_FOR_OBJECT;
        
        sys_cur_state = STATE_HEART_BEAT;
      }
      
      #ifndef SERIAL_LOG
        Serial.println(" State - Object Presence ");
      #endif
      
    break;

    case STATE_SPRAY_DISINFACTANT_START:
    // Reset Spray Timer Counter
    spray_timer_counter = 0;

    // Turn Relay-2 OFF
    digitalWrite(relay_2_ctrl_pin, HIGH);
    
    // Turn Relay-1 ON after break-make timeout
    relay_break_make_counter = 0;
    next_relay_to_energize = RELAY_1;

    // Save next system state
    next_sys_state = STATE_SPRAY_DISINFACTANT_TIMEOUT;

    // Update current system state
    sys_cur_state = STATE_RELAY_BREAK_MAKE_DELAY;

    #ifndef SERIAL_LOG
      Serial.println(" State - Spray Start ");
    #endif
    
    break;

    case STATE_SPRAY_DISINFACTANT_TIMEOUT:
    // Chcek Spray Timer Counter for timeout of 30 seconds
      if (spray_timer_counter >= 30000) // 3333 ---> 5 sec apprx.
      {
        spray_timer_counter = 0;

        sys_cur_state = STATE_SPRAY_DISINFACTANT_END;
      }
      else
      {
        spray_timer_counter++;

        // Save previous state
        prev_sys_state = STATE_SPRAY_DISINFACTANT_TIMEOUT;

        // Update current systen state
        sys_cur_state = STATE_HEART_BEAT;
      }

      #ifndef SERIAL_LOG
        Serial.println(" State - Wait for 30 Sec ");
      #endif
      
    break;

    case STATE_SPRAY_DISINFACTANT_END:
      // Same as idle state
      sys_cur_state = STATE_IDLE;

      #ifndef SERIAL_LOG
        Serial.println(" State - Spary Finished ");
      #endif
      
    break;
    
    case STATE_RELAY_BREAK_MAKE_DELAY:
      if (relay_break_make_counter >= 500) // 500 ms apprx.
      {
        relay_break_make_counter = 0;
        
        // Relay break-make timeout, so energize the next relay
        if (RELAY_1 == next_relay_to_energize)
        {
          digitalWrite(relay_1_ctrl_pin, LOW);
        }
        else
        {
          digitalWrite(relay_2_ctrl_pin, LOW);
        }
        
        // Update current system state
        sys_cur_state = next_sys_state;
      }
      else
      {
        relay_break_make_counter++;

        sys_cur_state = STATE_HEART_BEAT;

        prev_sys_state = STATE_RELAY_BREAK_MAKE_DELAY;
      }
      
      #ifndef SERIAL_LOG
        Serial.println(" State - Break-Make Delay ");
      #endif
      
    break;

    case STATE_HEART_BEAT:
    
      if (LED_ON == heart_beat_led_state)
      {
        digitalWrite(heart_beat_led, HIGH);

        if (loop_count >= 100)
        {
          heart_beat_led_state = LED_OFF;

          loop_count = 0;
        }
      }
      else
      {
        digitalWrite(heart_beat_led, LOW);

        if (loop_count >= 900)
        {
          heart_beat_led_state = LED_ON;

          loop_count = 0;
        }
      }
      
      loop_count++;
      
      // Update current system state
      sys_cur_state = prev_sys_state;

      delay(1);
      
      #ifndef SERIAL_LOG
        Serial.println(" State - Heart Beat ");
      #endif

      wdt_reset();
      
    break;

    default:
    break;
  }
}
