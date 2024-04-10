#include <SPI.h>
#include <stdint.h>

/* Sampling time in milliseconds */
#define SAMPLE_TIME_MS 20UL

#define DEBUG_PIN   5
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
  STRAIGHT_RIGHT_GO_SOON,
  STRAIGHT_RIGHT_GO, /* Adjoining pedestrian light is also on */
  STRAIGHT_RIGHT_STOP_SOON,
  LEFT_GO_SOON,
  LEFT_GO,
  LEFT_STOP_SOON,
  ALL_STOPPED,
  TRAFFIC_DIR_STATE_COUNT
} TrafficDirectionState;

/* Represents the current state of the whole traffic junction */
typedef struct
{
  TrafficDirectionState directions[TRAFFIC_DIRECTIONS_TOTAL];
  uint32_t StateDuration_ms;
} TrafficJunctionState;

/* TrafficDirectionState represented as uint8_t */
const static uint8_t TrafficDirectionStatesInBits[TRAFFIC_DIR_STATE_COUNT] = {
  0b00101101, /* STRAIGHT_RIGHT_GO_SOON */
  0b11001110, /* STRAIGHT_RIGHT_GO -- adjoining pedestrian light is also on */
  0b10101101, /* STRAIGHT_RIGHT_STOP_SOON */
  0b01100101, /* LEFT_GO_SOON */
  0b01111001, /* LEFT_GO */
  0b01110101, /* LEFT_STOP_SOON */
  0b01101101  /* ALL_STOPPED */
};

/* TrafficDirectionState represented as uint8_t FOR TRANSITION TO SPECIAL PEDESTRIAN MODE */
const static uint8_t TrafficDirectionStatesInBits_Special[TRAFFIC_DIR_STATE_COUNT] = {
  0b10101101, /* STRAIGHT_RIGHT_GO_SOON */
  0b10101110, /* STRAIGHT_RIGHT_GO */
  0b10101101, /* STRAIGHT_RIGHT_STOP_SOON */
  0b01110101, /* LEFT_GO_SOON */
  0b01110101, /* LEFT_GO */
  0b01110101, /* LEFT_STOP_SOON */
  0b01101101  /* ALL_STOPPED */
};

/* TrafficDirectionState represented as uint8_t FOR ACTIVE SPECIAL PEDESTRIAN MODE */
/* All pedestrian lights are on */
const static uint8_t TrafficDirectionStatesInBits_SpecialMode = 0b01101110;

/* Time periods in milliseconds for the special mode states */
const static uint32_t SpecialModeTimes_ms[TRAFFIC_ROTATION_COUNT_SPECIAL] = {
  1000, /* Time in transition period */
  1000, /* Time with everything off - cars and pedestrians (except pedestrians that are already green) */
  5000  /* Time in actual special pedestrian mode */
};

/* Keeps the expected traffic junction rotation */
/* Order is Direction1, Direction2, Direction3, Direction4, time on in milliseconds */
const static TrafficJunctionState NormalTrafficRotation[TRAFFIC_ROTATION_COUNT] = {
  {ALL_STOPPED             , ALL_STOPPED             , ALL_STOPPED             , ALL_STOPPED             , 1000}, /* State 00: all off */
  {STRAIGHT_RIGHT_GO_SOON  , ALL_STOPPED             , ALL_STOPPED             , STRAIGHT_RIGHT_GO_SOON  , 1000}, /* State 01: dir1,dir4 preparing for straight/right and pedestrian */
  {STRAIGHT_RIGHT_GO       , ALL_STOPPED             , ALL_STOPPED             , STRAIGHT_RIGHT_GO       , 3000}, /* State 02: dir1,dir4 straight/right and pedestrian on */
  {STRAIGHT_RIGHT_STOP_SOON, ALL_STOPPED             , ALL_STOPPED             , STRAIGHT_RIGHT_STOP_SOON, 1000}, /* State 03: dir1,dir4 straight/right going off and pedestrian hard off */
  {ALL_STOPPED             , ALL_STOPPED             , ALL_STOPPED             , ALL_STOPPED             , 1000}, /* State 04: all off */
  {LEFT_GO_SOON            , ALL_STOPPED             , ALL_STOPPED             , LEFT_GO_SOON            , 1000}, /* State 05: dir1,dir4 preparing for left turns */
  {LEFT_GO                 , ALL_STOPPED             , ALL_STOPPED             , LEFT_GO                 , 3000}, /* State 06: dir1,dir4 left turn on */
  {LEFT_STOP_SOON          , ALL_STOPPED             , ALL_STOPPED             , LEFT_STOP_SOON          , 1000}, /* State 07: dir1,dir4 left turn going off */
  {ALL_STOPPED             , ALL_STOPPED             , ALL_STOPPED             , ALL_STOPPED             , 1000}, /* State 08: all off */
  {ALL_STOPPED             , STRAIGHT_RIGHT_GO_SOON  , STRAIGHT_RIGHT_GO_SOON  , ALL_STOPPED             , 1000}, /* State 09: dir2,dir3 preparing for straight/right and pedestrian */
  {ALL_STOPPED             , STRAIGHT_RIGHT_GO       , STRAIGHT_RIGHT_GO       , ALL_STOPPED             , 3000}, /* State 10: dir2,dir3 straight/right and pedestrian on */
  {ALL_STOPPED             , STRAIGHT_RIGHT_STOP_SOON, STRAIGHT_RIGHT_STOP_SOON, ALL_STOPPED             , 1000}, /* State 11: dir2,dir3 straight/right going off and pedestrian hard off */
  {ALL_STOPPED             , ALL_STOPPED             , ALL_STOPPED             , ALL_STOPPED             , 1000}, /* State 12: all off */
  {ALL_STOPPED             , LEFT_GO_SOON            , LEFT_GO_SOON            , ALL_STOPPED             , 1000}, /* State 13: dir2,dir3 preparing for left turns */
  {ALL_STOPPED             , LEFT_GO                 , LEFT_GO                 , ALL_STOPPED             , 3000}, /* State 14: dir2,dir3 left turn on */
  {ALL_STOPPED             , LEFT_STOP_SOON          , LEFT_STOP_SOON          , ALL_STOPPED             , 1000}  /* State 15: dir2,dir3 left turn going off */
};

/*
   Each array member is a direction from the traffic junction
   - 1x3 traffic light for straight/right turn
   - 1x3 traffic light for left turn
   - 1x2 traffic light for pedestrians
   individual bits will be turned on/off with masking
*/
static uint8_t traffic_direction_array[TRAFFIC_DIRECTIONS_TOTAL];

/* Input buttons for pedestrian special mode */
static uint8_t input_button = 0;
/* Special pedestrian mode flag */
static uint8_t special_mode = 0;
/* Special pedestrian mode button flag to prevent overlapping triggers */
static uint8_t buttonFreeze = 0;

/* Used as indexes to access rotation specific information (time, light order, etc) */
static uint8_t normal_operation_counter      = 0;
static uint8_t normal_operation_counter_prev = 0;
static uint8_t special_operation_counter     = 0;

/* Sampling rate variable to keep track of time passed in current loop */
static unsigned long _timer_loop = 0;
static unsigned long timer_loop  = 0;
/* Execution time measurement */
static unsigned long _loop_time = 0;
static unsigned long loop_time  = 0;
/* Time stamp for each consecutive traffic light switch */
static unsigned long TIME_STAMP_SWITCH          = 0;
/* Keeps the expected time between traffic switching - is expected to be changing constantly */
static unsigned long TIME_PERIOD_TRAFFIC_SWITCH = 0;

void setup () {
  /* Set types for pins */
  pinMode(DEBUG_PIN ,       OUTPUT);
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

    /* Check for new button presses for the special pedestrian mode, but only if we are not currently in it */
    if ( (0 == special_mode) && (0 == buttonFreeze) )
    {
      input_button = digitalRead(BUTTON_PIN);
      if (!input_button)
      {
        /* Turn on special mode */
        special_mode = 1;
        /* Reset special mode sequence counter */
        special_operation_counter = 0;
      }
    }

    /* Check if it's time to change the traffic state */
    if ((unsigned long)(millis() - TIME_STAMP_SWITCH) >= TIME_PERIOD_TRAFFIC_SWITCH)
    {
      /* Remember the time needed to be in this state */
      TIME_STAMP_SWITCH = millis();

      digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));

      /* If the special mode button was previously frozen to enable no overlap of special modes, enable it again */
      if (1 == buttonFreeze)
      {
        buttonFreeze = 0;
      }
      
      /* Normal operation */
      if (0 == special_mode)
      {
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
      }
      /* Special mode */
      else
      {
        /* Parse a special sequence with traffic states */
        parseTrafficLightsSpecial(normal_operation_counter_prev);

        /* Assign a value to the wait period until next traffic switch IN SPECIAL MODE */
        TIME_PERIOD_TRAFFIC_SWITCH = SpecialModeTimes_ms[special_operation_counter - 1];

        /* Must wait out the last interval of special pedestrian mode before the button becomes active again */
        if (0 == special_mode)
        {
          buttonFreeze = 1;
        }
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

/* Fill the active LED array with predefined values from the special mode traffic sequence */
void parseTrafficLightsSpecial(uint8_t sequenceStep)
{
  /* Keep track at which step of the special mode we are in */
  special_operation_counter++;

  if (1 == special_operation_counter)
  {
    for (uint8_t i = 0; i < TRAFFIC_DIRECTIONS_TOTAL; i++)
    {
      /* This is yet another hefty one-liner */
      traffic_direction_array[i] = TrafficDirectionStatesInBits_Special[NormalTrafficRotation[sequenceStep].directions[i]];
    }
  }

  if (2 == special_operation_counter)
  {
    for (uint8_t i = 0; i < TRAFFIC_DIRECTIONS_TOTAL; i++)
    {
      /* Full stop everything */
      if (traffic_direction_array[i] == TrafficDirectionStatesInBits_Special[STRAIGHT_RIGHT_GO])
      {
        traffic_direction_array[i] = TrafficDirectionStatesInBits_SpecialMode;
      }
      else
      {
        traffic_direction_array[i] = TrafficDirectionStatesInBits_Special[ALL_STOPPED];
      }
    }
  }

  if (3 == special_operation_counter)
  {
    for (uint8_t i = 0; i < TRAFFIC_DIRECTIONS_TOTAL; i++)
    {
      /* All pedestrian lights are on */
      traffic_direction_array[i] = TrafficDirectionStatesInBits_SpecialMode;
    }
    /* Turn off special mode after this interval */
    special_mode = 0;

    /* Reset to normal operations */
    normal_operation_counter = 0;
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
  Serial.print(" | Button: ")       ; Serial.print(!input_button);
  Serial.print(" | Special Mode: ") ; Serial.print(special_mode);
  Serial.print(" | Special State: "); Serial.print(special_operation_counter);
  Serial.print(" | Traffic State: "); Serial.println(normal_operation_counter_prev);
}
