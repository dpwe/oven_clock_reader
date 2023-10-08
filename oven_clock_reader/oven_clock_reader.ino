// oven_clock_reader.ino
// 2023-10-07 dan.ellis@gmail.com
//
// This code drives a substitute for a Vacuum Fluorescent Display (VFD)
// on my kitchen range that had become difficult to read with age (these
// displays typically begin to dim after 10 years or so).  My solution is
// to mirror the intended VFD display on a separate 128 x 64 pixel
// backlit LCD display. 
//
// Passive external circuitry takes the grid and anode voltages from the
// VFB and drop/limit them to 0 and 3.3v, suitable for input to a 3.3v
// MCU. I'm using an RP2040 Pico because I need 18 input pins to read the 
// 7 grids and 11 anodes that drive the VFD, then another 4 output pins
// to drive the ST7290 LCD display module that I'm using.
//
// The VFD drive cycles between the 7 grid "regions" in the display, 
// synchronously setting the 11 segment voltages.  This code attaches an
// interrupt to each grid voltage; on the interrupt, it sets up a brief 
// timer to wait for the anode voltages to stabilize.  When that timer 
// expires, the 11 anode (segment) pins are read in and stored, 
// independently for each grid region.  After reading the 7th grid, all
// the stored segment bits are copied to an output buffer.  
//
// The board  cycles through the grids in about 4ms in 10 slots of ~400us
// (with ~380us positive pulses and ~20us gaps), so the sampling of the
// segment anodes is timed to be 100us after the grid interrupt.
//
// These captured values are monitored by the foreground task.  When a
// change is detected, the segment bits are interpreted and re-rendered
// on the LCD display.  Each of the 4 clock digits and 3 temperature
// digits, which are 7-segment numeric displays on the original display
// are re-interpreted into the underlying digit (or "F", "-", and " ")
// and displayed as large glyphs roughly copying the original display.
// The remaining segments correspond to entire words in the original 
// display ("BAKE", "TIME", etc.) and these are displayed in a small
// font, again roughly matching the original display.

// References:
// Layout and pinout of the VFD:
//   https://www.alibaba.com/product-detail/Futaba-MWO-Display-7-LT-91G_62527234028.html
// Video revealing the display model number:
//   https://www.youtube.com/watch?v=bSUa4f8KvhM
// .. from the service (Nick's TV Repair) that will replace the dim displays:
//   https://nickselectronics.com/products/318010100?variant=44698119340308

// External circuit:
// With an oscilloscope, I found that that the oven control board had a 12v
// supply for its logic, but the VFD had a -19V bias on its 3VAC cathode
// (filament) emitter.  Then, the anode drives were approx +6V for "on"
// (same for grid and segment), and about -24V for "off".  To map these
// down into 0 to 3.3V, I used 19 repetitions of this circuit:
//
//  VFD Anode Drive  o----|>|----^v^v^v-----+-------o  RP2040 input pin
//   -24 .. +6V         1N4148   12 Kohm    |            0 .. +3.3V
//                       diode              >
//                                  22 Kohm <
//                                          >
//                                          |
//                                         ///

#include <Arduino.h>
#include "pico.h"


// 7 seg layout
//     --P7--
//    P5     P6
//     --P4--
//    P2     P3
//     --P1--
// P8:
// P9: 3G: colon

typedef struct seg_char {
  uint16_t segs;
  char c;
} seg_char_t;

const seg_char_t segs_char_tab[] = {
  {0b1110111, '0'},
  {0b0100100, '1'},
  {0b1101011, '2'},
  {0b1101101, '3'},
  {0b0111100, '4'},
  {0b1011101, '5'},
  {0b1011111, '6'},
  {0b1100100, '7'},
  {0b1111111, '8'},
  {0b1111101, '9'},
  {0b0000000, ' '},
  {0b1011011, 'E'},
  {0b1011010, 'F'},
  {0b0001000, '-'},
};

const int NUM_GRID_PINS = 7;

char segs_to_char(uint16_t segs) {
  segs = segs & 0x7f;  // Bottom 7 bits.
  if (segs == 0) { return ' '; }
  for (int i=0; i < sizeof(segs_char_tab) / sizeof(seg_char_t); ++i) {
    if (segs_char_tab[i].segs == segs) {
      return segs_char_tab[i].c;
    }
  }
  return '?';
}

// ----- 128x64 LCD display ------
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// 128x64 ST7920 LCD display
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 21, /* data=*/ 22, /* CS=*/ 20, /* reset=*/ 26);

void setup_display(void) {
  u8g2.begin();

  //u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  u8g2.setFontMode(1);  // Don't draw white bits.
}

void render_display(uint16_t *seg_vals) {
  u8g2.clearBuffer();

  // Main clock
  //u8g2.setFont(u8g2_font_spleen16x32_mn);
  u8g2.setFont(u8g2_font_inr27_mr);
  char s[2] = " ";
  uint8_t x, y, xbase = 0, xbase2 = 80, xw = 17, lxw=12, ybase = 10, ybase2 = 44, ybase3 = 32;
  for (int i = 0; i < 4; ++i) {
    x = xbase + xw * i + 8 * (i > 1);
    s[0] = segs_to_char(seg_vals[i]);
    u8g2.drawStr(x, ybase, s);
  }
  // Colon
  if (seg_vals[2] & 0x100) {
    u8g2.drawStr(xbase + xw * 2 -3 , ybase - 4, ":");
  }

  // Oven temp
  u8g2.setFont(u8g2_font_inr19_mr);
  for (int i = 0; i < 3; ++i) {
    x = xbase2 + lxw * i;
    s[0] = segs_to_char(seg_vals[4 + i]);
    u8g2.drawStr(x, ybase, s);
  }
  
  // Other annotations.
  //u8g2.setFont(u8g2_font_6x13_mr);
  //const int cw = 6;
  u8g2.setFont(u8g2_font_tom_thumb_4x6_mr);
  const int cw = 4, ch = 7;
  if (seg_vals[0] & 0x80)  u8g2.drawStr(xbase, ybase2, "DELAY");
  if (seg_vals[1] & 0x80)  u8g2.drawStr(xbase + xw + 8, ybase + 7, "HR");
  if (seg_vals[1] & 0x100)  u8g2.drawStr(xbase + xw, ybase2, "BAKE");
  if (seg_vals[2] & 0x80)  u8g2.drawStr(xbase + 2 * xw, ybase2, "CLN");
  if (seg_vals[2] & 0x200)  u8g2.drawStr(xbase + 2 * xw + 8, ybase2, "STOP");
  if (seg_vals[3] & 0x80)  u8g2.drawStr(xbase + 3 * xw + 8 + 10, ybase + 6, "o");
  if (seg_vals[3] & 0x100)  u8g2.drawStr(xbase + 3 * xw + 8, ybase2, "TIME");
  if (seg_vals[3] & 0x200)  u8g2.drawStr(xbase + 3 * xw + 8 + 4 * cw, ybase2, "R");
  if (seg_vals[4] & 0x200)  u8g2.drawStr(xbase2, ybase3, "BROIL");
  if (seg_vals[4] & 0x200)  u8g2.drawStr(xbase2, ybase3 + ch, "DELAY");
  if (seg_vals[4] & 0x200)  u8g2.drawStr(xbase2, ybase3 + 2 * ch, "CLEAN");
  if (seg_vals[5] & 0x80)  u8g2.drawStr(xbase2 + 6 * cw, ybase3, "CONV");
  if (seg_vals[5] & 0x100)  u8g2.drawStr(xbase2 + 6 * cw, ybase3 + ch, "BAKE");
  if (seg_vals[5] & 0x200)  u8g2.drawStr(xbase2 + 6 * cw, ybase3 + 2 * ch, "LOCK");
  if (seg_vals[5] & 0x400)  u8g2.drawStr(xbase2 + 6 * cw + 4 * cw, ybase3 + 2 * ch, "ED");
  if (seg_vals[6] & 0x80)  u8g2.drawStr(xbase2 + 39, ybase + 1, "o");
  if (seg_vals[6] & 0x100)  u8g2.drawStr(xbase2 + 40, ybase + 10, "ON");

  u8g2.sendBuffer();
}

// ---- Oven Clock Reader Interface ----

#include "hardware/structs/timer.h"

#define ALARM_NUM0 0
#define ALARM_IRQ0 TIMER_IRQ_0

typedef void (*isr_type)(void);

// Grid interrupt semaphores

volatile int grid_num = 0;
volatile uint32_t grid_time_micros = 0;

// Set after grid 7 read, indicating a complete set ready.  Cleared by grid1.
volatile bool grid_vals_ready = false;

//// Timer delay interrupts
//const int NUM_GRID_PINS = 7;
const int grid_pins[NUM_GRID_PINS] = {8, 7, 6, 5, 4, 3, 2};  // 7G is the rightmost pin.
const int NUM_SEG_PINS = 11;
const int seg_pins[NUM_SEG_PINS] = { 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9 };  // P11 is the rightmost segment pin


// output pin to indicate when sampling happens.
const int diag_pin = 28;


// grid_seg_vals methods -------------------------

// Temporary buffer
volatile uint16_t grid_seg_values_buf[NUM_GRID_PINS] = { 0, 0, 0, 0, 0, 0, 0 };
// Where the most recent complete set of grid vals go.
volatile uint16_t grid_seg_values[NUM_GRID_PINS] = { 0, 0, 0, 0, 0, 0, 0 };

bool grid_seg_vals_equal(volatile uint16_t *vals1, volatile uint16_t *vals2) {
  for (int i = 0; i < NUM_GRID_PINS; ++i) {
    if (vals1[i] != vals2[i]) return false;
  }
  return true;
}

// (Can't make src const because I'm using volatile.)
void grid_seg_vals_copy(/* const */ volatile uint16_t *src, volatile uint16_t *dst) {
  for (int i = 0; i < NUM_GRID_PINS; ++i) {
    dst[i] = src[i];
  }
}

void print_binary_fixed(int val, int bits) {
  int bit = 1 << (bits - 1);
  for (int i = 0; i < bits; ++i) {
    if (val & bit) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
    bit >>= 1;
  }
}

void grid_seg_vals_print(const uint16_t *grid_seg_values) {
    //Serial.println("grid vals:");
    for (int i = 0; i < NUM_GRID_PINS; ++i) {
      Serial.print("  ");
      // Print 10 bits.
      //print_binary_fixed(grid_seg_values[i], 10);
      //Serial.print("  0x");
      //Serial.print(grid_seg_values[i], HEX);
      Serial.print(segs_to_char(grid_seg_values[i]));
      Serial.print(" ");
    }
    Serial.println();
}

// Read seg pins --------------------------------

volatile bool timer_pending = false;

static void read_seg_pins(unsigned int unused) {
  // Read the seg pins shortly after a grid pin interrupt.
  // grid_num is assumed to describe the latest grid pin.
  uint16_t seg_val = 0;
  uint16_t seg_bit = 1;
  for (int i = 0; i < NUM_SEG_PINS; ++i) {
    if (digitalRead(seg_pins[i])) { seg_val |= seg_bit; }
    seg_bit <<= 1;
  }
  if (grid_num > 0) {
    // Valid grid_num values are 1..7.
    grid_seg_values_buf[grid_num - 1] = seg_val;
    // After reading the final vals of a set, copy to output buffer.
    if (grid_num == NUM_GRID_PINS) {
      grid_seg_vals_copy(grid_seg_values_buf, grid_seg_values);
    }
  }
  // Clear grid-set semaphore
  grid_num = 0;

  // Set diag high to show when pins are read.
  digitalWrite(diag_pin, HIGH);

  // Mark timer used.
  timer_pending = false;
}

void setup_seg_pins(void) {
  for (int i = 0; i < NUM_SEG_PINS; ++i) {
    pinMode(seg_pins[i], INPUT);
  }
  pinMode(diag_pin, OUTPUT);
}

// Hardware timer -------------------------------

void setup_timer(void) {
    hardware_alarm_claim(ALARM_NUM0);
    //hardware_alarm_set_callback	(ALARM_NUM0, alarm_isr);
}

static void call_after_delay(uint32_t delay_us, hardware_alarm_callback_t alarm_isr) {
  if (timer_pending == false) {
    // Set the requested time in the future.
    uint64_t current_time = time_us_64();
    absolute_time_t t = {current_time + delay_us};
    hardware_alarm_set_target(ALARM_NUM0, t);
    // Enable callback on alarm.  alarm_isr must clear timer_pending.
    hardware_alarm_set_callback	(ALARM_NUM0, alarm_isr);
    timer_pending = true;
  }
}

//// Grid pin interrupts -------------------------

const int post_grid_delay_us = 100;

void isr_g1() {
  grid_time_micros = micros();
  grid_num = 1;
  call_after_delay(post_grid_delay_us, read_seg_pins);
  // Pull diagnostic output low to show when G1 detected.
  digitalWrite(diag_pin, LOW);
}
void isr_g2() {
  grid_time_micros = micros();
  grid_num = 2;
  call_after_delay(post_grid_delay_us, read_seg_pins);
}
void isr_g3() {
  grid_time_micros = micros();
  grid_num = 3;
  call_after_delay(post_grid_delay_us, read_seg_pins);
}
void isr_g4() {
  grid_time_micros = micros();
  grid_num = 4;
  call_after_delay(post_grid_delay_us, read_seg_pins);
}

void isr_g5() {
  grid_time_micros = micros();
  grid_num = 5;
  call_after_delay(post_grid_delay_us, read_seg_pins);
}

void isr_g6() {
  grid_time_micros = micros();
  grid_num = 6;
  call_after_delay(post_grid_delay_us, read_seg_pins);
}

void isr_g7() {
  grid_time_micros = micros();
  grid_num = 7;
  call_after_delay(post_grid_delay_us, read_seg_pins);
}

const isr_type isr_array[NUM_GRID_PINS] = { &isr_g1, &isr_g2, &isr_g3, &isr_g4, &isr_g5, &isr_g6, &isr_g7 };

void setup_grid_pin_interrupts() {
  for (int i = 0; i < NUM_GRID_PINS; ++i) {
    int grid_pin = grid_pins[i];
    pinMode(grid_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(grid_pin), isr_array[i], RISING);
  }
}

// ------------------------------------------------------

void setup() {
  Serial.begin(9600);  // initialize serial communication
  setup_seg_pins();
  setup_timer();
  setup_grid_pin_interrupts();

  setup_display();

  Serial.println("started ");
}

uint16_t last_grid_seg_values[NUM_GRID_PINS] = { 0, 0, 0, 0, 0, 0, 0 };

uint32_t last_micros = 0;

void loop() {
  if (!grid_seg_vals_equal(last_grid_seg_values, grid_seg_values)) {
    delay(50);  // Let the display settle.
    grid_seg_vals_copy(grid_seg_values, last_grid_seg_values);

    // Check all segments.
    //for (int i = 0; i < 7; ++i)  last_grid_seg_values[i] = 0xFFF;

    grid_seg_vals_print(last_grid_seg_values);

    render_display(last_grid_seg_values);
  }
  // Tick every 10s, just to know we're alive.
  if (false && micros() > last_micros + 10000000) {
    last_micros = micros();
    Serial.println("tick");
    Serial.print("last grid=");
    Serial.println(grid_num);
    Serial.print("timer_pending=");
    Serial.println(timer_pending);
  }
}
