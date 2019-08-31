#include <avr/sleep.h>
#include <avr/interrupt.h>


#define OFF_LEVEL (int16_t)255
#define MAX_BRIGHTNESS 251
#define MIN_BRIGHTNESS 3
#define MICROS_PER_BRIGHTNESS_STEP 1000
#define INVERTED_BRIGHTNESS 1

// min brightness = 3
#ifdef __AVR_ATtiny85__
#define LED_PIN 4
#define BUTTON_PIN 3
#define INTERRUPT_BUTTON_PIN PCINT3
#define DELAY_MULTIPLIER (uint32_t)64

#else
#define LED_PIN 3
#define BUTTON_PIN 2
#define DELAY_MULTIPLIER (uint32_t)1
#endif

#include "helpers.h"
#include "ClickChain.h"

ClickChain mainButton(BUTTON_PIN, 1, 1, 30 * DELAY_MULTIPLIER);

enum class OPERATING_MODE {
  OFF,
  ON,
  BRIGHTNESS_ADJUST,
  MOMENTARY,
  STROBE,
  VOLTAGE,
  MAX
};

int8_t adjust_direction = 0;
int32_t adjust_brightness = 0;

OPERATING_MODE operating_mode = OPERATING_MODE::OFF, previous_operating_mode = OPERATING_MODE::ON, debug_print = OPERATING_MODE::ON;
uint8_t brightness = 23, current_brightness = 23, sleep_mode = 1, beacon_mode = 0, operations_done = 0;
uint32_t mode_timer = 0, time_to_sleep = 0, brightness_timer = 0;
uint16_t voltage = 0;
uint8_t voltage_temp = 0, voltage_digit = 0;
const uint16_t beacon_duration[4][3] = {{500, 1000, 0}, {100, 900, 0}, {50, 150, 0}, {0}}; //zero terminated arrays, both ways

#ifdef __AVR_ATtiny85__
ISR(PCINT0_vect) {}
#else
void ISR_ROUTINE() {
}
#endif

void sleep() {
  nanoDebug("SPIJAM");
#ifdef __AVR_ATtiny85__
  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(INTERRUPT_BUTTON_PIN);     // Use PB3 as interrupt pin
#else
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ISR_ROUTINE, CHANGE);
#endif
  ADCSRA &= ~_BV(ADEN);                   // ADC off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                  // Enable interrupts
  sleep_cpu();                            // sleep

  cli();                                  // Disable interrupts
#ifdef __AVR_ATtiny85__
  PCMSK &= ~_BV(INTERRUPT_BUTTON_PIN);    // Turn off PB3 as interrupt pin
#else
  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
#endif
  sleep_disable();                        // Clear SE bit
  ADCSRA |= _BV(ADEN);                    // ADC on
  sei();                                  // Enable interrupts

  //  click_chain_counter = 0;
  sleep_mode = 2;
  time_to_sleep = x_millis();
  //  x_delay(20);
} // sleep


void button_down(uint8_t clickCount) {
  nanoDebug("button_down");
  if (operating_mode == OPERATING_MODE::VOLTAGE) {
    operating_mode = OPERATING_MODE::ON;
    previous_operating_mode = OPERATING_MODE::OFF;
    mainButton.endChain();
    return;
  }
}

void button_up(uint8_t clickCount) {
  nanoDebug("button_up");

  if (sleep_mode == 2) {
    operating_mode = previous_operating_mode;
    previous_operating_mode = OPERATING_MODE::OFF;
    sleep_mode = 3;
  }

  if (operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
    adjust_direction = 0;
    return;
  }

}

void button_hold(uint8_t clickCount) {
  nanoDebug("button_hold");

  if (operating_mode == OPERATING_MODE::MOMENTARY) { //momentary mode can't be bothered by long clicks
    mainButton.endChain();
    return;
  }

  if (operating_mode == OPERATING_MODE::OFF && previous_operating_mode != OPERATING_MODE::STROBE) { //if we were at sleep, now we are definitely not
    operating_mode = OPERATING_MODE::ON;
    sleep_mode = 0;
  }

  if ((clickCount == 1 || clickCount == 2) && operating_mode == OPERATING_MODE::ON) {
    operating_mode = OPERATING_MODE::BRIGHTNESS_ADJUST;
    adjust_direction = (clickCount % 2) ? 1 : -1;
    return;
  }
  if (clickCount > 1 && operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
    adjust_direction = (clickCount % 2) ? 1 : -1;
    brightness = brightness + adjust_brightness;
  }

  if (clickCount == 1 && operating_mode == OPERATING_MODE::STROBE) {
    if (sleep_mode == 3) {
      sleep_mode = 0;

    } else {
      beacon_mode++;
      mode_timer = x_millis();
      if (beacon_duration[beacon_mode][0] == 0) beacon_mode = 0;
    }
    mainButton.endChain();
    return;
  }

}

void button_at_rest(uint8_t clickCount) {
  nanoDebug("button_at_rest");
  //  nanoDebug(String(clickCount).c_str());

  if (operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
    operating_mode = OPERATING_MODE::ON;
    previous_operating_mode = OPERATING_MODE::OFF;
    brightness = brightness + adjust_brightness;
    return;
  }

  if (operating_mode != OPERATING_MODE::MOMENTARY && clickCount == 1) {
    nanoDebug("sleep_try");
    if (sleep_mode == 0) {
      voltage = 0;
      voltage_temp = 0;
      voltage_digit = 0;
      nanoDebug("sleeping initiated");

      previous_operating_mode = operating_mode;
      operating_mode = OPERATING_MODE::OFF;
      sleep_mode = 1;
    } else {
      nanoDebug("sleeping stopped");
      sleep_mode = 0;
    }
  }

  if (clickCount == 2 && operating_mode == OPERATING_MODE::ON) {
    sleep_mode = 0;
    brightness = 255; //turbo
  }

  if (clickCount == 5) {
    if (operating_mode == OPERATING_MODE::OFF || operating_mode == OPERATING_MODE::ON) {
      operating_mode = OPERATING_MODE::MOMENTARY;
      sleep_mode = 0;
    } else if (operating_mode == OPERATING_MODE::MOMENTARY) {
      operating_mode = OPERATING_MODE::OFF;
      sleep_mode = 1;
      previous_operating_mode = OPERATING_MODE::ON;
    }
  }

  if (clickCount == 6) {
    if (operating_mode == OPERATING_MODE::OFF || operating_mode == OPERATING_MODE::ON) {
      operating_mode = OPERATING_MODE::STROBE;
      sleep_mode = 0;
    } else if (operating_mode == OPERATING_MODE::STROBE) {
      operating_mode = OPERATING_MODE::OFF;
      sleep_mode = 1;
      previous_operating_mode = OPERATING_MODE::ON;
    }
  }

  if (clickCount == 7) {
    if (operating_mode == OPERATING_MODE::OFF || operating_mode == OPERATING_MODE::ON) {
      operating_mode = OPERATING_MODE::VOLTAGE;
      sleep_mode = 0;
      mode_timer = x_millis();
      voltage = 0;
      voltage_temp = 0;
      voltage_digit = 0;
      nanoDebug("Voltage mode ON");
    }
    mainButton.endChain();
  }
}

void led_write(uint8_t force = 0) {
  if (force) {
    current_brightness = brightness;
  } else if (current_brightness < brightness) {
    if (x_micros() - brightness_timer >= MICROS_PER_BRIGHTNESS_STEP) {
      current_brightness += min(brightness - current_brightness, (x_micros() - brightness_timer) / MICROS_PER_BRIGHTNESS_STEP);
      brightness_timer = x_micros();
    }
  } else if (current_brightness > brightness) {
    if (x_micros() - brightness_timer >= MICROS_PER_BRIGHTNESS_STEP) {
      current_brightness -= min(current_brightness - brightness, (x_micros() - brightness_timer) / MICROS_PER_BRIGHTNESS_STEP);
      brightness_timer = x_micros();
    }
  }
  //  analogWrite(LED_PIN, (operating_mode == OPERATING_MODE::OFF) ? 0 : cie[current_brightness]);
}

void setup() {
#ifdef __AVR_ATtiny85__
  //fast PWM
  TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
  TCCR0B = 0 << WGM02 | 1 << CS00;
  TCCR1 = 0 << PWM1A | 0 << COM1A0 | 1 << CS10;
  GTCCR = 1 << PWM1B | 2 << COM1B0;
#else
  Serial.begin(115200);
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  //  pinMode(BUTTON_PIN, INPUT_PULLUP);

  mainButton.setFunctions(button_down, button_up, button_hold, button_at_rest);
  mainButton.begin(300 * DELAY_MULTIPLIER);

  x_delay(50);
  nanoDebug("START");
}

void loop() {
  //  if (sleep_mode == 1 && (x_millis() - time_to_sleep) > 1000) sleep();
  if (sleep_mode == 1) sleep();

  mainButton.run();
  /*  if (operating_mode != debug_print) {
      debug_print = operating_mode;
      if (operating_mode == OPERATING_MODE::OFF) {
        nanoDebug("OPERATING_MODE::OFF");
      } else if (operating_mode == OPERATING_MODE::ON) {
        nanoDebug("OPERATING_MODE::ON");
      } else if (operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
        nanoDebug("OPERATING_MODE::BRIGHTNESS_ADJUST");
      } else if (operating_mode == OPERATING_MODE::MOMENTARY) {
        nanoDebug("OPERATING_MODE::MOMENTARY");
      } else if (operating_mode == OPERATING_MODE::STROBE) {
        nanoDebug("OPERATING_MODE::STROBE");
      } else if (operating_mode == OPERATING_MODE::VOLTAGE) {
        nanoDebug("OPERATING_MODE::VOLTAGE");
      }
    }*/


  if (operating_mode == OPERATING_MODE::OFF) {
    analogWrite(LED_PIN, 0);

  } else if (operating_mode == OPERATING_MODE::ON) {
    analogWrite(LED_PIN, cie[brightness]);

  } else if (operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
    if (adjust_direction) {
      uint32_t duration = (mainButton.sinceLastEvent() / DELAY_MULTIPLIER );
      adjust_brightness = (duration / 2000.0) * 254;
      adjust_brightness *= adjust_direction;
      if ((int32_t)brightness + adjust_brightness < 0) adjust_brightness = (int32_t) - brightness;
      if ((int32_t)brightness + adjust_brightness > 254) adjust_brightness = (int32_t)254 - brightness;
      adjust_brightness += 1;
    }

    analogWrite(LED_PIN, cie[brightness + adjust_brightness]);

  } else if (operating_mode == OPERATING_MODE::MOMENTARY) {
    analogWrite(LED_PIN, digitalRead(BUTTON_PIN) ? 0 : cie[brightness]);

  } else if (operating_mode == OPERATING_MODE::STROBE) {
    uint8_t i = 0;
    while (beacon_duration[beacon_mode][i] && (x_millis() - mode_timer) > beacon_duration[beacon_mode][i]) {
      i++;
    }
    if (beacon_duration[beacon_mode][i] == 0) {
      mode_timer = x_millis();
      i = 0;
    }
    analogWrite(LED_PIN, ((i + 1) % 2) ? cie[brightness] : 0);

  } else if (operating_mode == OPERATING_MODE::VOLTAGE) {
    //    nanoDebug(String(  String((int32_t)x_millis() - mode_timer) + String(" | ") + String(voltage) + String(", ") + String(voltage_digit) + String(", ") + String(voltage_temp)  ).c_str());

    if (voltage_digit == 0 && voltage_temp == 0) {
      analogWrite(LED_PIN, 0);
      voltage = read_Vcc() / 10;
      voltage_digit = 3;
    }

    if (voltage_temp == 0) {
      nanoDebug("voltage_temp == 0");
      voltage_temp = (voltage / (((voltage_digit >= 3) ? 10 : 1 ) * ((voltage_digit >= 2) ? 10 : 1 )) ) % 10;
      if (voltage_temp == 0) voltage_temp = 10;
      voltage_digit--;
      mode_timer = x_millis() + 2000;
    } else if ((int32_t)x_millis() - (int32_t)mode_timer >= 1000) {
      nanoDebug("voltage_temp && ((int32_t)x_millis() - mode_timer >= 1000)");
      voltage_temp--;
      mode_timer = x_millis();
    }

    analogWrite(LED_PIN, ((int32_t)x_millis() - (int32_t)mode_timer >= 500) ? cie[brightness] : 0);

    if (voltage_digit == 0 && voltage_temp == 0 && voltage != 0) {
      nanoDebug("voltage_digit == 0 && voltage_temp == 0 && voltage != 0");
      voltage = 0;
      operating_mode = OPERATING_MODE::OFF;
      sleep_mode = 1;
      previous_operating_mode = OPERATING_MODE::ON;
    }
  }


}
