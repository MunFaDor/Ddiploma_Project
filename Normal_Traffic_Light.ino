#include <SPI.h>
#include <stdint.h>

/* Sampling time in milliseconds */
#define SAMPLE_TIME_MS 20UL

#define BUTTON_PIN  8
#define LATCH_PIN  10
#define DATA_PIN   11
#define CLOCK_PIN  13

#define TRAFFIC_DIRECTIONS_TOTAL          4
#define TRAFFIC_ROTATION_COUNT           16
#define TRAFFIC_ROTATION_COUNT_SPECIAL    3

/* Possible normal states of a singular traffic direction */
typedef enum
{
  ALL_GO_SOON,   /* Adjacent pedestrian off */
  ALL_GO,        /* Adjacent pedestrian light on */
  ALL_STOP_SOON, /* Adjacent pedestrian off */
  ALL_STOPPED,   /* Adjacent pedestrian off */
  TRAFFIC_DIR_STATE_COUNT
} TrafficDirectionState;

/* Represents the current state of the whole traffic junction */
typedef struct
{
  TrafficDirectionState directions[TRAFFIC_DIRECTIONS_TOTAL];
  uint32_t StateDuration_ms;
} TrafficJunctionState;

/* TrafficDirectionState represented as uint8_t */
/* Format is:
   - 1x3 bits for straight/right traffic light (red, yellow, green)
   - 1x3 bits for left traffic right (red, yellow, green)
   - 1x2 bits for pedestrian traffic light (red, green)
   Active state is 0, inactive state is 1
*/
const static uint8_t TrafficDirectionStatesInBits[TRAFFIC_DIR_STATE_COUNT] = {
  0b00100101, /* ALL_GO_SOON */
  0b11011010, /* ALL_GO -- Adjacent pedestrian light is also on */
  0b10110101, /* ALL_STOP_SOON */
  0b01101101  /* ALL_STOPPED */
};

/* Keeps the expected traffic junction rotation */
/* Order is Direction1, Direction2, Direction3, Direction4, time on in milliseconds */
const static TrafficJunctionState NormalTrafficRotation[TRAFFIC_ROTATION_COUNT] = {
  {ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , 1000}, /* State 00: all off        */
  {ALL_GO_SOON   , ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , 1000}, /* State 01: dir1 go soon   */
  {ALL_GO        , ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , 3000}, /* State 02: dir1 go        */
  {ALL_STOP_SOON , ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , 1000}, /* State 03: dir1 stop soon */
  {ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , 1000}, /* State 04: all off        */
  {ALL_STOPPED   , ALL_GO_SOON   , ALL_STOPPED   , ALL_STOPPED   , 1000}, /* State 05: dir2 go soon   */
  {ALL_STOPPED   , ALL_GO        , ALL_STOPPED   , ALL_STOPPED   , 3000}, /* State 06: dir2 go        */
  {ALL_STOPPED   , ALL_STOP_SOON , ALL_STOPPED   , ALL_STOPPED   , 1000}, /* State 07: dir2 stop soon */
  {ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , 1000}, /* State 08: all off        */
  {ALL_STOPPED   , ALL_STOPPED   , ALL_GO_SOON   , ALL_STOPPED   , 1000}, /* State 09: dir3 go soon   */
  {ALL_STOPPED   , ALL_STOPPED   , ALL_GO        , ALL_STOPPED   , 3000}, /* State 10: dir3 go        */
  {ALL_STOPPED   , ALL_STOPPED   , ALL_STOP_SOON , ALL_STOPPED   , 1000}, /* State 11: dir3 stop soon */
  {ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , 1000}, /* State 12: all off        */
  {ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , ALL_GO_SOON   , 1000}, /* State 13: dir4 go soon   */
  {ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , ALL_GO        , 3000}, /* State 14: dir4 go        */
  {ALL_STOPPED   , ALL_STOPPED   , ALL_STOPPED   , ALL_STOP_SOON , 1000}  /* State 15: dir4 stop soon */
};

/* Each array member is a direction from the traffic junction
   - 1x3 bits for straight/right traffic light (red, yellow, green)
   - 1x3 bits for left traffic right (red, yellow, green)
   - 1x2 bits for pedestrian traffic light (red, green)
   Active state is 0, inactive state is 1
*/
static uint8_t traffic_direction_array[TRAFFIC_DIRECTIONS_TOTAL];

/* Used as indexes to access rotation specific information (time, light order, etc) */
static uint8_t normal_operation_counter         = 0;
static uint8_t normal_operation_counter_prev    = 0;

/* Sampling rate variable to keep track of time passed in current loop */
static unsigned long _timer_loop                = 0;
static unsigned long timer_loop                 = 0;
/* Execution time measurement */
static unsigned long _loop_time                 = 0;
static unsigned long loop_time                  = 0;
/* Time stamp for each consecutive traffic light switch */
static unsigned long TIME_STAMP_SWITCH          = 0;
/* Keeps the expected time between traffic switching - is expected to be changing constantly */
static unsigned long TIME_PERIOD_TRAFFIC_SWITCH = 0;

void setup () {
  /* Set types for pins */
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LATCH_PIN ,       OUTPUT);
  pinMode(DATA_PIN  ,       OUTPUT);
  pinMode(CLOCK_PIN ,       OUTPUT);

  /* Setup SPI communication for shift register control */
  SPI.begin();
  /* Begin SPI communication */
  SPI.beginTransaction(SPISettings(8000000, LSBFIRST, SPI_MODE0));

  /* Make sure everything is turned off on power on */
  init_traffic_fill();
  /* SPI Send - output to actual LEDs */
  sender();

  delay(1000);

  /* Start serial communication for logging purposes */
  Serial.begin(115200);
}  /* end of setup */

void loop ()
{
  /* 50 Hz loop rate desired */
  if ((unsigned long)(millis() - timer_loop) >= SAMPLE_TIME_MS)
  {
    /* Get loop start time */
    _loop_time = micros();
    /* Logging only variable to make sure we have the desired SAMPLE_TIME_US */
    _timer_loop = millis() - timer_loop;
    /* Main sample time variable */
    timer_loop = millis();

    /* Check if it's time to change the traffic state */
    if ((unsigned long)(millis() - TIME_STAMP_SWITCH) >= TIME_PERIOD_TRAFFIC_SWITCH)
    {
      /* Remember the time needed to be in this state */
      TIME_STAMP_SWITCH = millis();

      /* Parse the predefined array with traffic states */
      parseTrafficLights(normal_operation_counter);

      /* Assign a value to the wait period until next traffic switch */
      TIME_PERIOD_TRAFFIC_SWITCH = NormalTrafficRotation[normal_operation_counter].StateDuration_ms;

      /* Progress the normal traffic sequence counter */
      normal_operation_counter_prev = normal_operation_counter;
      normal_operation_counter++;
      if (normal_operation_counter > TRAFFIC_ROTATION_COUNT - 1)
      {
        normal_operation_counter = 0; /* Once the end of the rotation is reached, start again */
      }
        
      /* SPI Send - output to actual LEDs */
      sender();
    }

    /* Output some variables of interest to the console */
    printToConsole();

    /* Get loop end time to calculate loop speed */
    loop_time = micros() - _loop_time;
  }
} /* end of loop */

/* Fill the active LED array with predefined values from the normal traffic sequence */
void parseTrafficLights(uint8_t sequenceStep)
{
  for (uint8_t i = 0; i < TRAFFIC_DIRECTIONS_TOTAL; i++)
  {
    /* This is one hefty one-liner */
    traffic_direction_array[i] = TrafficDirectionStatesInBits[NormalTrafficRotation[sequenceStep].directions[i]];
  }
}

void init_traffic_fill()
{
  for (uint8_t i = 0; i < TRAFFIC_DIRECTIONS_TOTAL; i++)
  {
    traffic_direction_array[i] = 255; /* All lights off */
  }
}

void sender()
{
  /* Slave Select low */
  digitalWrite(LATCH_PIN, LOW);

  for (int8_t i = 0; i < TRAFFIC_DIRECTIONS_TOTAL; i++)
  {
    /* This should send 32 bits down the drain for all 32 lights */
    SPI.transfer(traffic_direction_array[i]);
  }
  /* Slave Select high -> rising edge transfers data to outputs */
  digitalWrite(LATCH_PIN, HIGH);
}

void printToConsole()
{
  /* Log to COM Port */
  Serial.print("Desired Rate: ")    ; Serial.print(1000.0f / SAMPLE_TIME_MS);
  Serial.print(" | Actual Rate: ")  ; Serial.print(1000.0f / _timer_loop);
  Serial.print(" | Loop Time: ")    ; Serial.print((100.0f * (loop_time / 1000.0f)) / SAMPLE_TIME_MS); Serial.print("% of available time ");
  Serial.print(" | Traffic State: "); Serial.println(normal_operation_counter_prev);
}
