#define DISPLAY_TYPE 'OLEDI2C'

#include <RingBuf.h>
#include <limits.h>

#if DISPLAY_TYPE == 'LCD'
  #include <LiquidCrystal.h>
  // initialize the LiquidCrystal library by associating any needed LCD interface pins
  // with the arduino pin number it is connected to
  const int rs = 6, en = 7, d4 = 8, d5 = 11, d6 = 12, d7 = 13;
  LiquidCrystal display(rs, en, d4, d5, d6, d7);
#endif
#if DISPLAY_TYPE == 'OLEDI2C'
  #include <Adafruit_SH1106.h>
  Adafruit_SH1106 display(0);
#endif

// Constants/defines...
#define no_of_timers 2
#define event_buffer_length 5
#define ir_kHz 38
#define ir_duty_cycle 0.05 // 0.05 seems to work well...
#define MODE_TIMER 0
#define MODE_HZ 1
#define MODE_RPM 2 
#define SECONDS "s"
#define HZ "Hz"
#define RPM "rpm"
#define LOG_DECIMALS 6
#define DISP_DECIMALS 3

// Pin Mappings for Maker Nano
const byte interruptPins[no_of_timers] = {2, 3};
const byte indicatorLEDs[no_of_timers] = {4, 5};
const byte gateLEDs[no_of_timers] = {9, 10};
#define RESET_PIN 6


// Structures...
struct TimerEvent {
  int timer_;
  unsigned long micros_prev_start;
  unsigned long micros_timer0_start;
  unsigned long micros_start;
  unsigned long micros_end;
};

// Globals...
volatile unsigned long timer_time_start[no_of_timers];
volatile unsigned long timer_time_prev_start[no_of_timers];
volatile bool timer_invert[no_of_timers] = {false, true};
unsigned long startup_micros;
RingBuf<TimerEvent, event_buffer_length> my_buffer;
bool log_to_serial = false;
byte mode = MODE_RPM;
bool clear_display = false;
byte no_of_readings[no_of_timers];
float period_max[no_of_timers];
float period_min[no_of_timers];
float period_sum[no_of_timers];
unsigned int period_count[no_of_timers];

// Utility functions...
float delta_micros(unsigned long start_micros, unsigned long end_micros) {
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
  // Read the micros() timer first to minimise dither
  unsigned long microsecs = micros();
  // Get the index for the timers
  int i = timer - 1;
  // Read the state and mimic on the indicators LED
  int state = digitalRead(interruptPins[i]);
  if(timer_invert[i]){
    state = !state;
  }
  digitalWrite(indicatorLEDs[i], state);
  // Deal with interrupt...
  if (state == HIGH) {
    // End of event - send results to RingBuffer
    struct TimerEvent event = {timer, timer_time_prev_start[i], timer_time_start[0], 
      timer_time_start[i], microsecs};
    my_buffer.push(event); 
  } else {
    // Start of event
    timer_time_prev_start[i] = timer_time_start[i];
    timer_time_start[i] = microsecs;
  }
}

void timer1() { state_change(1); }

void timer2() { state_change(2); }

// Initialise...
void setup() {
  // Initialise serial port
  Serial.begin(115200);
  pinMode(RESET_PIN, INPUT_PULLUP);
  #if DISPLAY_TYPE == 'LCD'
    // set up the LCD's number of columns and rows:
    display.begin(16, 2);
  #endif
  #if DISPLAY_TYPE == 'OLEDI2C'  
    display.begin(SH1106_SWITCHCAPVCC, 0x3C);
    display.setTextSize(1);
    display.setTextColor(WHITE, BLACK);    
    display.setTextWrap(false);
    display.clearDisplay();
    display.display();
  #endif  
  set_mode();
  // setup pins and variables...
  for (int i = 0; i < no_of_timers; i++) {
    pinMode(indicatorLEDs[i], OUTPUT);
    pinMode(gateLEDs[i], OUTPUT);
    pinMode(interruptPins[i], INPUT_PULLUP);
    int state = digitalRead(interruptPins[i]);
    digitalWrite(indicatorLEDs[i], state);
    timer_time_prev_start[i] = startup_micros;
    timer_time_start[i] = startup_micros;
  }
  // Set the IR output frequency and PWM
  set_PWM1_kHz(ir_kHz, ir_duty_cycle, true, true);
  // We have to map the interrupt routines 'manually'
  attachInterrupt(digitalPinToInterrupt(interruptPins[0]), timer1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPins[1]), timer2, CHANGE);
  // Note start time
  startup_micros = micros();  
  my_buffer.clear();
  reset_counters();
}

void reset_counters() {
  for (int i = 0; i < no_of_timers; i++) {
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
    if (no_of_readings[i] > 1) {
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
  float start, duration,period, delta, hz;
  float value, mean, range;
  int i;

  // check for commands
  check_for_commands();
  if (!digitalRead(RESET_PIN)){
    reset_counters();
    set_mode();
  }
  // check ring buffer
  if (my_buffer.pop(event)) {
    i = event.timer_ - 1;
    start = delta_micros(startup_micros, event.micros_start);
    duration = delta_micros(event.micros_start, event.micros_end);
    period = delta_micros(event.micros_prev_start, event.micros_start);
    delta = delta_micros(event.micros_timer0_start, event.micros_start);
    hz = 1.0 / period;
    update_counter(event.timer_, period);
    if (log_to_serial){
      Serial.println();
      Serial.print(", "); Serial.print(event.timer_);
      Serial.print(", "); Serial.print(start, LOG_DECIMALS);
      Serial.print(", "); Serial.print(period, LOG_DECIMALS);
      Serial.print(", "); Serial.print(delta, LOG_DECIMALS);        
      Serial.print(", "); Serial.print(duration, LOG_DECIMALS);
      Serial.print(", "); Serial.print(hz, LOG_DECIMALS);
      Serial.print(", "); Serial.print(60.0 * hz, LOG_DECIMALS);
    }
    // output to LCD - 1:Period,Duration
    //                 2:Delta,Duration
    int delta_x = 1;
    int delta_y = 1;
    #if DISPLAY_TYPE == 'OLEDI2C'
      delta_x = 8;
      delta_y = 12;
    #endif
    if (clear_display) {
      display.clearDisplay();
      clear_display = false;
    }
    display.setCursor(0, delta_y * i );
    display.print(event.timer_); display.print(":");
    switch(mode){
      case MODE_TIMER:
        if(event.timer_ == 1){
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
        #if DISPLAY_TYPE == 'OLEDI2C'
          display.setCursor(0, delta_y * (1 + event.timer_) );
          display.print(event.timer_); display.print(":");
          display.print(mean, DISP_DECIMALS);
          display.print(","); display.print(range, DISP_DECIMALS);
          display.print(" "); display_units(); 
          display.print("        ");
        #endif         
        break;
        
    }
    display.display();
  }
}
