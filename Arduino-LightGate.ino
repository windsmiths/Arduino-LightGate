#include <RingBuf.h>
#include <limits.h>
#include <Adafruit_SH1106.h>
Adafruit_SH1106 display(0);

// Constants/defines...
#define NO_OF_TIMERS 2
#define EVENT_BUFFER_LENGTH 4
#define IR_KHZ 38
#define IR_DUTY_CYCLE 0.05 // 0.05 seems to work well...
#define MODE_TIMER 0
#define MODE_HZ 1
#define MODE_RPM 2 
#define SECONDS "s"
#define HZ "Hz"
#define RPM "rpm"
#define LOG_DECIMALS 6
#define DISP_DECIMALS 3
#define DEBOUNCE_SECS 50e-6

// Pin Mappings for Maker Nano
const byte interruptPins[NO_OF_TIMERS] = {2, 3};
const byte indicatorLEDs[NO_OF_TIMERS] = {4, 5};
const byte gateLEDs[NO_OF_TIMERS] = {9, 10};
#define RESET_PIN 6


// Structures...
struct TimerState {
  bool invert;
  byte mask;
  bool last_value;
  unsigned long time_start;
  unsigned long time_prev_start;
};

struct TimerEvent {
  int timer;
  unsigned long micros_prev_start;
  unsigned long micros_timer0_start;
  unsigned long micros_start;
  unsigned long micros_end;
};

// Globals...
volatile TimerState timers[NO_OF_TIMERS];
unsigned long startup_micros;
RingBuf<TimerEvent, EVENT_BUFFER_LENGTH> my_buffers[NO_OF_TIMERS];
bool log_to_serial = false;
byte mode = MODE_RPM;
bool clear_display = false;
byte no_of_readings[NO_OF_TIMERS];
float period_max[NO_OF_TIMERS];
float period_min[NO_OF_TIMERS];
float period_sum[NO_OF_TIMERS];
unsigned int period_count[NO_OF_TIMERS];

// Utility functions...
float delta_from_micros(unsigned long start_micros, unsigned long end_micros) {
  if (end_micros >= start_micros) {
    return ((float)(end_micros - start_micros)) / 1e6;
  } else {
    unsigned long delta1 = ULONG_MAX - start_micros;
    return ((float)(delta1 + 1 + end_micros)) / 1e6;
  }
}

// PWM Setup
void set_PWM1_kHz(float kHz, float duty_cycle, bool enableA, bool enableB) {
  // disable interrupts while we setup...
  noInterrupts();
  // Calculate top value for required frequency assuming no prescaling
  float count = 16000 / kHz - 1;
  // Work out if we need to set the prescaler
  byte cs12 = 0;
  if (count > 65535) {
    count = count / 1024;
    cs12 = 1;
  }
  unsigned int top = (unsigned int)count;
  // Work out 'on' count for ocr
  unsigned int ocr = top * duty_cycle;
  // Initialise control registers (always do this first)
  TCCR1A = 0;
  TCCR1B = 0;
  // set TOP
  ICR1 = top;
  // Set On time for output A
  OCR1A = ocr;
  // Set On time for output B
  OCR1B = ocr;
  // set output controls A nd B with non-inverting mode
  TCCR1A |= _BV(COM1A1) | _BV(COM1B1);
  // set Fast PWM mode using ICR1 as TOP
  TCCR1A |= _BV(WGM11);
  TCCR1B |= _BV(WGM12) | _BV(WGM13);
  // START the timer with no prescaler or 1024 prescaler
  TCCR1B |= _BV(CS10) | (cs12 << CS12);
  // enable all interrupts
  interrupts();
  // Enables outputs
  if (enableA)
    DDRB |= _BV(DDB1);
  if (enableB)
    DDRB |= _BV(DDB2);
  // output debug info if required
  // Serial.println(top);
  // Serial.println(ocr);
}

// Interrupt routines...

void state_change(int timer) {
  // Read pin state first in case it changes
  byte pind = PIND;
  // Then read the micros() timer first to minimise dither
  unsigned long microsecs = micros();
  // Get the index for the timers
  int i = timer - 1;
  // Read the state and mimic on the indicators LED
  bool value = (pind & timers[i].mask) !=0 ;
  if(timers[i].invert){
    value = !value;
  }
  digitalWrite(indicatorLEDs[i], value);  
  // Drop out if value hasn't changed (we may have missed an interrupt, 
  // or value may have changed before we read it)
  if (value != timers[i].last_value) { ;
    // Deal with interrupt... 
    if (!value) {
      // End of event - send results to RingBuffer
      struct TimerEvent event = {timer, timers[i].time_prev_start, timers[0].time_start, 
        timers[i].time_start, microsecs};
      my_buffers[i].push(event); 
    } else {
      // Start of event
      timers[i].time_prev_start = timers[i].time_start;
      timers[i].time_start = microsecs;
    }
  }
  // Store value
  timers[i].last_value = value;
}

void timer1() { state_change(1); }

void timer2() { state_change(2); }

// Initialise...
void setup() {
  // Initialise serial port
  Serial.begin(115200);
  pinMode(RESET_PIN, INPUT_PULLUP);
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);    
  display.setTextWrap(false);
  display.clearDisplay();
  display.display();
  set_mode();
  // Note start time
  startup_micros = micros();    
  // setup pins and variables...
  timers[0].invert = true;
  timers[1].invert = false;
  for (int i = 0; i < NO_OF_TIMERS; i++) {
    pinMode(indicatorLEDs[i], OUTPUT);
    pinMode(gateLEDs[i], OUTPUT);
    pinMode(interruptPins[i], INPUT_PULLUP);
    int value = digitalRead(interruptPins[i]);
    digitalWrite(indicatorLEDs[i], value);
    timers[i].invert = true;
    timers[i].mask = _BV(interruptPins[i]);
    if (timers[i].invert){
      timers[i].last_value = !value;
    } else {
      timers[i].last_value = value;
    }
    timers[i].time_prev_start = startup_micros;
    timers[i].time_start = startup_micros; 
    my_buffers[i].clear();      
  }
  // Set the IR output frequency and PWM
  set_PWM1_kHz(IR_KHZ, IR_DUTY_CYCLE, true, true);
  // We have to map the interrupt routines 'manually'
  attachInterrupt(digitalPinToInterrupt(interruptPins[0]), timer1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPins[1]), timer2, CHANGE);
  reset_counters();
}

void reset_counters() {
  for (int i = 0; i < NO_OF_TIMERS; i++) {
    period_max[i] = 0;
    period_min[i] = INFINITY;
    period_sum[i] = 0;
    period_count[i] = 0;
    no_of_readings[i] = 0;
  }
}

void update_counter(int timer, float period) {
  int i = timer - 1;  
  if (period > 0) {
    if (no_of_readings[i] > 0) {
      period_max[i] = fmaxf(period_max[i], period);
      period_min[i] = fminf(period_min[i], period);
      period_sum[i] += period;
      period_count[i]++;
    } else {
      no_of_readings[i]++;
    }      
  }
}

void set_mode() {
  display.clearDisplay();
  display.setCursor(0, 0);
  switch (mode) {
    case MODE_TIMER:
      display.print("Timer Mode...");
      break;
    case MODE_HZ:
      display.print("Frequency Mode (Hz)...");
      break;              
    case MODE_RPM:
      display.print("Frequency Mode (rpm)...");
      break;               
  }
  display.display();
  delay(500);
  clear_display = true;
}

void check_for_commands() {
    if (Serial.available() > 0) {
        char receivedChar = Serial.read();
        switch(tolower(receivedChar)){
          case 's':
            log_to_serial = !log_to_serial;
            break;
          case 'm':
            mode++;
            if (mode > MODE_RPM) mode = 0;
            set_mode();
            break;
          case 'r':
            reset_counters();
            set_mode();
            break;          
        };
    }
}

void display_units(){
  switch (mode) {
    case MODE_TIMER:
      display.print(SECONDS);
      break;
    case MODE_HZ:
      display.print(HZ);
      break;              
    case MODE_RPM:
      display.print(RPM);
      break;               
  }  
}

// Run...
void loop() {
  struct TimerEvent event;
  float gap, start, duration,period, delta, hz;
  float value, mean, range;
  int i, no_of_events, e;

  // check for commands
  check_for_commands();
  if (!digitalRead(RESET_PIN)){
    reset_counters();
    set_mode();
  }
  // check ring buffers
  for (i=0; i < NO_OF_TIMERS; i++) {
    // see if we have any events for this timer...
    no_of_events = my_buffers[i].size();
    if (no_of_events == 0) continue;
    // See if debounce time has elapsed since last event...
    gap = delta_from_micros(micros(), my_buffers[i][0].micros_end);
    if (gap < DEBOUNCE_SECS) continue;
    // if so, loop through events for this timer
    for (e = 0; e < no_of_events; e++) {
      // get event, bailing if get fails
      if (!my_buffers[i].pop(event)) continue;
      // check duration (off time), bail if less than debounce time
      duration = delta_from_micros(event.micros_start, event.micros_end);
      if (duration < DEBOUNCE_SECS) continue;
      // Check On time
      period = delta_from_micros(event.micros_prev_start, event.micros_start);      
      if (period - duration < DEBOUNCE_SECS) continue;
      // calculate remaining values
      start = delta_from_micros(startup_micros, event.micros_start);

      delta = delta_from_micros(event.micros_timer0_start, event.micros_start);
      hz = 1.0 / period;
      update_counter(event.timer, period);
      if (log_to_serial){
        Serial.println();
        Serial.print(", "); Serial.print(event.timer);
        Serial.print(", "); Serial.print(start, LOG_DECIMALS);
        Serial.print(", "); Serial.print(period, LOG_DECIMALS);
        Serial.print(", "); Serial.print(delta, LOG_DECIMALS);        
        Serial.print(", "); Serial.print(duration, LOG_DECIMALS);
        Serial.print(", "); Serial.print(hz, LOG_DECIMALS);
        Serial.print(", "); Serial.print(60.0 * hz, LOG_DECIMALS);
      }
      // Update display
      // output to LCD - 1:Period,Duration
      //                 2:Delta,Duration
      int delta_x = 8;
      int delta_y = 12;
      if (clear_display) {
        display.clearDisplay();
        clear_display = false;
      }
      display.setCursor(0, delta_y * i );
      display.print(event.timer); display.print(":");
      switch(mode){
        case MODE_TIMER:
          if(event.timer == 1){
            display.print(period, DISP_DECIMALS);
          } else {
            display.print(delta, DISP_DECIMALS);
          }
          display.print(",");display.print(duration, DISP_DECIMALS);
          display.print(SECONDS);  display.print("   ");
          break;  
        case MODE_HZ:
        case MODE_RPM:
          value = hz;
          mean = period_count[i] / period_sum[i];      
          range = 1.0 / period_min[i] - 1.0 / period_max[i];     
          if (mode == MODE_RPM) {
            value = 60 * value;
            mean = 60 * mean;      
            range = 60 * range;
          }
          display.print(value, DISP_DECIMALS); 
          display.print(" "); display_units(); 
          display.print("        ");
          display.setCursor(0, delta_y * (1 + event.timer) );
          display.print(event.timer); display.print(":");
          display.print(mean, DISP_DECIMALS);
          display.print(","); display.print(range, DISP_DECIMALS);
          display.print(" "); display_units(); 
          display.print("        ");      
          break;
      }
      display.display();
    }
  }
}
