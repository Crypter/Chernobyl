#include <avr/sleep.h>
#include <avr/interrupt.h>


#define OFF_LEVEL (int16_t)255
#define MAX_BRIGHTNESS 251
#define MIN_BRIGHTNESS 3
#define MICROS_PER_BRIGHTNESS_STEP 1000
#define BUTTON_DELAY 400


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

ClickChain mainButton(BUTTON_PIN, 1, 1, 30*DELAY_MULTIPLIER);

enum class OPERATING_MODE {
  OFF,
  ON,
  MOMENTARY,
  BEACON,
  VOLTAGE,
  MAX
};

OPERATING_MODE operating_mode = OPERATING_MODE::OFF, previous_operating_mode = OPERATING_MODE::ON;
uint8_t brightness = 23, current_brightness = 23, sleep_mode = 1, beacon_mode = 0, operations_done = 0;
uint32_t mode_timer = 0, time_to_sleep = 0, brightness_timer = 0;
uint16_t voltage = 0;
uint8_t voltage_temp = 0, voltage_digit = 0;
const uint16_t beacon_duration[4][3] = {{500, 1000, 0}, {100, 900, 0}, {50, 150, 0}, {0}}; //zero terminated arrays, both ways
//const uint8_t beacon_state[4][3] = {{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {0}};


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


void button_down(uint8_t clickCount){
  nanoDebug(String(String("button_down: ") + String(clickCount)).c_str());
}

void button_up(uint8_t clickCount){
  nanoDebug(String(String("button_up: ") + String(clickCount)).c_str());
}

void button_hold(uint8_t clickCount){
  nanoDebug(String(String("button_hold: ") + String(clickCount)).c_str());
}

void button_at_rest(uint8_t clickCount){
  nanoDebug(String(String("button_at_rest: ") + String(clickCount)).c_str());
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
  mainButton.begin(300*DELAY_MULTIPLIER);

  x_delay(50);
  nanoDebug("START");
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


void loop() {
  mainButton.run();

  //  if (sleep_mode == 1 && (x_millis() - time_to_sleep) > 1000) sleep();
  if (sleep_mode == 1) sleep();

//  if (click_chain_counter) nanoDebug("STATUS");
//  if (click_chain_counter) nanoDebug(String(x_millis() - click_chain[click_chain_counter - 1]).c_str());
//  if (click_chain_counter) nanoDebug(String(click_chain_counter).c_str());

//  check_pin_change();
  //  led_write();
/*
  if (click_chain_counter > 0) {
    if (click_chain_counter % 2) {
      //button down, button hold

      nanoDebug("button down, button hold");

      if ((x_millis() - click_chain[click_chain_counter - 1]) > BUTTON_DELAY && (operating_mode == OPERATING_MODE::OFF || previous_operating_mode == OPERATING_MODE::VOLTAGE)) {
        operating_mode = OPERATING_MODE::ON;
        sleep_mode = 0;
      }

      if (operating_mode != OPERATING_MODE::MOMENTARY) {
        if (click_chain_counter == 1 && (x_millis() - click_chain[0]) > BUTTON_DELAY) {
          if (operating_mode == OPERATING_MODE::ON || operating_mode == OPERATING_MODE::OFF) {

            uint32_t duration = ((x_millis() - click_chain[click_chain_counter - 1]) - 300 + 2500);
            brightness = abs((int32_t)((duration) % 5000) - 2500) / 2500.0 * 251 + 1;

            nanoDebug("brightness adjust");
          }
          if (operating_mode == OPERATING_MODE::BEACON && operations_done == 0 && sleep_mode != 2) {
            beacon_mode++;
            mode_timer = x_millis();
            if (beacon_duration[beacon_mode][0] == 0) beacon_mode = 0;
            operations_done = 1;
          }
        }

        if (click_chain_counter == 3 && (x_millis() - click_chain[2]) > BUTTON_DELAY && operations_done == 0 ) {
          brightness = 255;
          operations_done = 1;
        }

      }
    } else {
      //button up, button at rest

      if (x_millis() - click_chain[click_chain_counter - 1] > BUTTON_DELAY) {
        if (click_chain_counter == 10 && (click_chain[9] - click_chain[0]) < BUTTON_DELAY * 10 && (operating_mode == OPERATING_MODE::ON || operating_mode == OPERATING_MODE::OFF || operating_mode == OPERATING_MODE::MOMENTARY)) {
          if (operating_mode != OPERATING_MODE::MOMENTARY) {
            operating_mode = OPERATING_MODE::MOMENTARY;
            sleep_mode = 0;
            nanoDebug("Momentary mode ON");
          } else {
            operating_mode = OPERATING_MODE::OFF;
            sleep_mode = 1;
            previous_operating_mode = OPERATING_MODE::ON;
            nanoDebug("Momentary mode OFF");
          }
          operations_done = 1;
        }

        if (operating_mode != OPERATING_MODE::MOMENTARY && click_chain_counter == 2 && (click_chain[1] - click_chain[0]) <= BUTTON_DELAY) {
          if (sleep_mode == 0) {
            voltage = 0;
            voltage_temp = 0;
            voltage_digit = 0;
            nanoDebug("sleeping initiated");
            if (operating_mode == OPERATING_MODE::VOLTAGE) operating_mode = OPERATING_MODE::ON;

            previous_operating_mode = operating_mode;
            operating_mode = OPERATING_MODE::OFF;
            sleep_mode = 1;
          } else {
            nanoDebug("sleeping stopped");
            if (sleep_mode != 3) {
              operating_mode = previous_operating_mode;
            previous_operating_mode = OPERATING_MODE::OFF;
            }
            sleep_mode = 0;
          }
          operations_done = 1;
        }

        else if (click_chain_counter == 12 && (click_chain[11] - click_chain[0]) < BUTTON_DELAY * 12 && (operating_mode == OPERATING_MODE::ON || operating_mode == OPERATING_MODE::OFF || operating_mode == OPERATING_MODE::BEACON)) {
          if (operating_mode != OPERATING_MODE::BEACON) {
            operating_mode = OPERATING_MODE::BEACON;
            sleep_mode = 0;
            mode_timer = x_millis();
            nanoDebug("Beacon mode ON");
          } else {
            operating_mode = OPERATING_MODE::OFF;
            sleep_mode = 1;
            previous_operating_mode = OPERATING_MODE::ON;
            nanoDebug("Beacon mode OFF");
          }
          operations_done = 1;
        }

        else if (click_chain_counter == 14 && (click_chain[13] - click_chain[0]) < BUTTON_DELAY * 14 && (operating_mode == OPERATING_MODE::ON || operating_mode == OPERATING_MODE::OFF || operating_mode == OPERATING_MODE::VOLTAGE)) {
          if (operating_mode != OPERATING_MODE::VOLTAGE) {
            operating_mode = OPERATING_MODE::VOLTAGE;
            sleep_mode = 0;
            mode_timer = x_millis();
            nanoDebug("Voltage mode ON");
          } else {
            operating_mode = OPERATING_MODE::OFF;
            sleep_mode = 1;
            previous_operating_mode = OPERATING_MODE::ON;
            voltage = 0;
            voltage_temp = 0;
            voltage_digit = 0;
            nanoDebug("Voltage mode OFF");
          }
          operations_done = 1;
        }
        click_chain_counter = 0;
      } else if (sleep_mode == 2 && previous_operating_mode == OPERATING_MODE::ON) {
        sleep_mode = 3;
        operating_mode = previous_operating_mode;
        previous_operating_mode = OPERATING_MODE::OFF;
      }

      if (x_millis() - click_chain[click_chain_counter - 1] > BUTTON_DELAY * 2) {
        click_chain_counter = 0;
      }

      if (operations_done) {
        operations_done = 0;
        if (sleep_mode == 2) {
          operating_mode = OPERATING_MODE::ON;
          sleep_mode = 0;
        }
      }
    }
  }
*/
  if (operating_mode == OPERATING_MODE::OFF) {
    analogWrite(LED_PIN, 0);
  } else if (operating_mode == OPERATING_MODE::ON) {
    analogWrite(LED_PIN, cie[brightness]);
  } else if (operating_mode == OPERATING_MODE::MOMENTARY) {
    analogWrite(LED_PIN, digitalRead(BUTTON_PIN) ? 0 : cie[brightness]);
  } else if (operating_mode == OPERATING_MODE::BEACON) {
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
