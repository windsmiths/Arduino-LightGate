#include <RingBuf.h>
#include <limits.h>
#include <LiquidCrystal.h>

// Constants...
const byte no_of_timers = 2;
const byte event_buffer_length = 10;
const float ir_kHz = 38;
const float ir_duty_cycle = 0.1; // 0.1 seems to work well...

// Pin Mappings for Maker Nano
const byte interruptPins[no_of_timers] = {2, 3};
const byte indicatorLEDs[no_of_timers] = {4, 5};
const byte gateLEDs[no_of_timers] = {9, 10};

// initialize the LiquidCrystal library by associating any needed LCD interface pins
// with the arduino pin number it is connected to
const int rs = 6, en = 7, d4 = 8, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

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
  Serial.begin(9600);
  // Note start time
  startup_micros = micros();
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
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

}

// Run...
void loop() {
  struct TimerEvent event;
  float start, duration,period, delta;
  if (my_buffer.pop(event)) {
    start = delta_micros(startup_micros, event.micros_start);
    duration = delta_micros(event.micros_start, event.micros_end);
    period = delta_micros(event.micros_prev_start, event.micros_start);
    delta = delta_micros(event.micros_timer0_start, event.micros_start);
    Serial.println();
    Serial.print(", "); Serial.print(event.timer_);
    Serial.print(", "); Serial.print(start, 6);
    Serial.print(", "); Serial.print(period, 6);
    Serial.print(", "); Serial.print(delta, 6);        
    Serial.print(", "); Serial.print(duration, 6);
    // output to LCD - 1:Period,Duration
    //                 2:Delta,Duration
    lcd.setCursor(0, event.timer_ - 1);
    lcd.print(event.timer_); lcd.print(":");
    if(event.timer_ == 1){
      lcd.print(period,3);
    } else {
      lcd.print(delta,3);
    }
    lcd.print(",");lcd.print(duration,3);lcd.print("   ");
  }
}
