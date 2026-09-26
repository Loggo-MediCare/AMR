/*
  Differential-drive wheel encoder firmware.

  Milestone 3 placeholder firmware for Arduino-class boards.

  Responsibility:
    quadrature encoder pulses -> signed cumulative tick counts -> Serial USB

  This sketch intentionally contains no ROS logic. ROS 2 reads the serial
  stream on the host side.

  PLACEHOLDER hardware assumptions:
    - Pin assignments below must be changed after the real motor/encoder
      hardware and Arduino board are selected.
    - Encoders are assumed to expose quadrature channel A and B signals.
    - CHANGE interrupts are used on channel A pins; direction is inferred from
      the A/B state.

  Counter overflow:
    Encoder counters are explicit signed 32-bit values (int32_t,
    -2,147,483,648 to 2,147,483,647). Long-running cumulative counters can
    roll over. Host software must compute tick deltas with signed 32-bit
    rollover handling instead of assuming monotonic integer growth forever.
*/

#include <stdint.h>

const uint8_t LEFT_ENCODER_A_PIN = 2;   // PLACEHOLDER: interrupt-capable pin
const uint8_t LEFT_ENCODER_B_PIN = 4;   // PLACEHOLDER
const uint8_t RIGHT_ENCODER_A_PIN = 3;  // PLACEHOLDER: interrupt-capable pin
const uint8_t RIGHT_ENCODER_B_PIN = 5;  // PLACEHOLDER

const unsigned long BAUD_RATE = 115200;
const unsigned long REPORT_INTERVAL_MS = 25;  // ~40 Hz, within 20-50 Hz target

volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;

unsigned long last_report_ms = 0;

void handleLeftEncoderA()
{
  const bool a = digitalRead(LEFT_ENCODER_A_PIN);
  const bool b = digitalRead(LEFT_ENCODER_B_PIN);

  // PLACEHOLDER convention: if direction is inverted on real hardware, swap
  // the increment/decrement or swap encoder channel wiring.
  if (a == b) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

void handleRightEncoderA()
{
  const bool a = digitalRead(RIGHT_ENCODER_A_PIN);
  const bool b = digitalRead(RIGHT_ENCODER_B_PIN);

  // PLACEHOLDER convention: keep signed direction consistent with the left
  // wheel so forward motion increases both counters.
  if (a == b) {
    right_ticks++;
  } else {
    right_ticks--;
  }
}

void setup()
{
  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);

  Serial.begin(BAUD_RATE);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN),
                  handleLeftEncoderA,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A_PIN),
                  handleRightEncoderA,
                  CHANGE);
}

void loop()
{
  const unsigned long now_ms = millis();
  if (now_ms - last_report_ms < REPORT_INTERVAL_MS) {
    return;
  }
  last_report_ms = now_ms;

  int32_t left_snapshot = 0;
  int32_t right_snapshot = 0;

  // Copy volatile multi-byte counters atomically so the interrupt handlers
  // cannot update them halfway through a read.
  noInterrupts();
  left_snapshot = left_ticks;
  right_snapshot = right_ticks;
  interrupts();

  // Machine-readable packet. Host parser expects:
  //   L:<signed_left_ticks>,R:<signed_right_ticks>
  Serial.print("L:");
  Serial.print(left_snapshot);
  Serial.print(",R:");
  Serial.println(right_snapshot);
}
