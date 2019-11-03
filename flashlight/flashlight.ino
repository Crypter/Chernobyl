#include <avr/sleep.h>
#include <avr/interrupt.h>


#define MAX_BRIGHTNESS 255  //turbo will always be 255, fatman safe is 250
#define MIN_BRIGHTNESS 1    //the step where there's officially no visible light
#define MICROS_PER_BRIGHTNESS_STEP 350 //simulating a normal bulb
#define INVERTED_BRIGHTNESS 1 //1 for p-channel mosfet driver
#define VCC_OFFSET 210 //sometimes a diode is place, which drops the BATT voltage roughly by 300 or 700 millivolts, depending on type
//#define VCC_OFFSET 0 //sometimes a diode is place, which drops the BATT voltage roughly by 300 or 700 millivolts, depending on type

#ifdef __AVR_ATtiny85__
#define LED_PIN 4
#define BUTTON_PIN 3
#define DELAY_MULTIPLIER (uint32_t)64

 //tankata
#define RED_LED_PIN 0
#define GREEN_LED_PIN 1
#define USB_POWER_PIN 5 //RESET!!!
#define CHARGING_STATUS_PIN 2

#else
#define LED_PIN 3
#define BUTTON_PIN 2
#define DELAY_MULTIPLIER (uint32_t)1
#endif

#include "helpers.h"
#include "ClickChain.h"

ClickChain mainButton(BUTTON_PIN, 1, 1, 30 * DELAY_MULTIPLIER);

enum class OPERATING_MODE {
  ON,
  BRIGHTNESS_ADJUST,
  MOMENTARY,
  STROBE,
  VOLTAGE,
  CHARGING,
  MAX
};

uint8_t light_enabled = 0;

int8_t adjust_direction = 0;
int32_t adjust_brightness = 0;
uint8_t start_brightness = 0;

OPERATING_MODE operating_mode = OPERATING_MODE::ON; /*, debug_print = OPERATING_MODE::ON;*/
uint8_t brightness = 1, current_brightness = 1, sleep_mode = 1, beacon_mode = 0;
uint32_t mode_timer = 0, brightness_timer = 0, bat_check_timer = 0;
uint16_t voltage = 0;
uint8_t voltage_temp = 0, voltage_digit = 0;
const uint16_t beacon_duration[5][5] = {{500, 1000, 0}, {100, 1000, 0}, {75, 150, 225, 1000, 0}, {50, 150, 0}, {0}}; //zero terminated arrays, both ways

#ifdef __AVR_ATtiny85__
ISR(PCINT0_vect) {}
#else
void ISR_ROUTINE() {
}
#endif

void sleep() {
  //nanoDebug("SPIJAM");
#ifdef __AVR_ATtiny85__
  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  
  #if defined(USB_POWER_PIN)
  PCMSK = _BV(BUTTON_PIN) | _BV(USB_POWER_PIN);     // Use PB3 as and power pin as interrupt pin
  #else
  PCMSK = _BV(BUTTON_PIN);     // Use PB3 as interrupt pin
  #endif
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
  PCMSK = 0;    // Turn off PB3 as interrupt pin
#else
  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
#endif
  sleep_disable();                        // Clear SE bit
  ADCSRA |= _BV(ADEN);                    // ADC on
  sei();                                  // Enable interrupts

  sleep_mode = 2;
} // sleep



void led_write(uint8_t force = 0) {
  uint8_t target_brightness = (light_enabled) ? brightness : 0;
  if (force) {
    current_brightness = target_brightness;
  }

  if (current_brightness == target_brightness) {
    brightness_timer = x_micros();

  } else if (current_brightness < target_brightness) {
    if (x_micros() - brightness_timer >= MICROS_PER_BRIGHTNESS_STEP) {
      current_brightness += min(target_brightness - current_brightness, (x_micros() - brightness_timer) / MICROS_PER_BRIGHTNESS_STEP);
      brightness_timer = x_micros();
    }

  } else if (current_brightness > target_brightness) {
    if (x_micros() - brightness_timer >= MICROS_PER_BRIGHTNESS_STEP) {
      current_brightness -= min(current_brightness - target_brightness, (x_micros() - brightness_timer) / MICROS_PER_BRIGHTNESS_STEP);
      brightness_timer = x_micros();
    }
  }

  analogWrite(LED_PIN, pwm_to_cie(brightness_to_pwm(current_brightness)) );
}


void button_down(uint8_t clickCount) {
  //nanoDebug("button_down");

  if (operating_mode == OPERATING_MODE::CHARGING) {
    mainButton.endChain();
    return;
  }

  if (operating_mode == OPERATING_MODE::MOMENTARY) {
    light_enabled = 1;
    return;
  }

  if (operating_mode == OPERATING_MODE::VOLTAGE) {
    operating_mode = OPERATING_MODE::ON;
    light_enabled = 0;
    sleep_mode = 1;
    mainButton.endChain();
    return;
  }

}

void button_hold(uint8_t clickCount) {
  //nanoDebug("button_hold");

  if (operating_mode == OPERATING_MODE::MOMENTARY) { //momentary mode can't be bothered by long clicks
    mainButton.endChain();
    return;
  }

  if (sleep_mode == 2) {
    sleep_mode = 3;
    mode_timer = x_millis();
  }

  if ((clickCount == 1 || clickCount == 2) && operating_mode == OPERATING_MODE::ON) {
    operating_mode = OPERATING_MODE::BRIGHTNESS_ADJUST;
    adjust_direction = (clickCount % 2) ? 1 : -1;
    if (!light_enabled) {
      brightness = 1;
      light_enabled = 1;
      led_write(1);
    }
    start_brightness = brightness;
    return;
  }

  if (clickCount > 1 && operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
    adjust_direction = (clickCount % 2) ? 1 : -1;
    start_brightness = brightness;
  }

  if (clickCount == 1 && operating_mode == OPERATING_MODE::STROBE) {
    if (sleep_mode != 3) {
      beacon_mode++;
      if (beacon_duration[beacon_mode][0] == 0) beacon_mode = 0;
    }
    mode_timer = x_millis();
    mainButton.endChain();
    return;
  }

}

void button_up(uint8_t clickCount) {
  //nanoDebug("button_up");

  if (sleep_mode == 2) {
    sleep_mode = 3;
    mode_timer = x_millis();
  }

  if (operating_mode == OPERATING_MODE::MOMENTARY) {
    light_enabled = 0;
  }

  if (operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
    adjust_direction = 0;
  }

  if (operating_mode == OPERATING_MODE::ON && !light_enabled) {
    light_enabled = 1;
  }

}

void button_at_rest(uint8_t clickCount) {
  //nanoDebug("button_at_rest");

  if (operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
    operating_mode = OPERATING_MODE::ON;
    sleep_mode = 0;
    return;
  }

  if (operating_mode != OPERATING_MODE::MOMENTARY && clickCount == 1 && sleep_mode == 0) { //prepare to sleep
    voltage_temp = 0;
    voltage_digit = 0;

    light_enabled = 0;
    sleep_mode = 1;
  } else {
    sleep_mode = 0;
  }

  if (clickCount == 2 && operating_mode == OPERATING_MODE::ON) {
    if (brightness != 255) brightness = 255; //turbo
    else brightness = 1; //moonlight
    light_enabled = 1;
    return;
  }

  if (clickCount == 5) {
    if (operating_mode == OPERATING_MODE::ON) {
      operating_mode = OPERATING_MODE::MOMENTARY;
      sleep_mode = 0;
    } else if (operating_mode == OPERATING_MODE::MOMENTARY) {
      operating_mode = OPERATING_MODE::ON;
      light_enabled = 0;
      sleep_mode = 1;
    }
    return;
  }

  if (clickCount == 6) {
    if (operating_mode == OPERATING_MODE::ON) {
      operating_mode = OPERATING_MODE::STROBE;
      mode_timer = x_millis();
      sleep_mode = 0;
    } else if (operating_mode == OPERATING_MODE::STROBE) {
      operating_mode = OPERATING_MODE::ON;
      sleep_mode = 1;
      light_enabled = 0;
    }
    return;
  }

  if (clickCount == 7) {
    if (operating_mode == OPERATING_MODE::ON) {
      operating_mode = OPERATING_MODE::VOLTAGE;
      sleep_mode = 0;
      mode_timer = x_millis();
      voltage_temp = 0;
      voltage_digit = 0;
    }
    return;
  }
}


void setup() {
#ifdef __AVR_ATtiny85__
  //fast PWM
  TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
  TCCR0B = 0 << WGM02 | 1 << CS00;
  TCCR1 = 0 << PWM1A | 0 << COM1A0 | 1 << CS10;
  GTCCR = 1 << PWM1B | 2 << COM1B0;

  #if defined(RED_LED_PIN) && defined(GREEN_LED_PIN)
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, LOW);
  
    pinMode(GREEN_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, LOW);
  #endif

  #if defined(USB_POWER_PIN) && defined(CHARGING_STATUS_PIN)
    pinMode(USB_POWER_PIN, INPUT);
    pinMode(CHARGING_STATUS_PIN, INPUT);
  #endif



#else
  Serial.begin(115200);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, INVERTED_BRIGHTNESS ? HIGH : LOW);
  //  pinMode(BUTTON_PIN, INPUT_PULLUP);

  mainButton.setFunctions(button_down, button_up, button_hold, button_at_rest);
  mainButton.begin(350 * DELAY_MULTIPLIER);

  x_delay(50);
  //nanoDebug("START");
}

void loop() {

  #if defined(USB_POWER_PIN)
  if (sleep_mode == 1 && !current_brightness && !digitalRead(USB_POWER_PIN) && !mainButton.lastEvent()) sleep();
  #else
  if (sleep_mode == 1 && !current_brightness && !mainButton.lastEvent()) sleep();
  #endif
  
  mainButton.run();
  /*
    if (operating_mode != debug_print) {
      debug_print = operating_mode;
      if (operating_mode == OPERATING_MODE::ON) {
        //nanoDebug("OPERATING_MODE::ON");
      } else if (operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
        //nanoDebug("OPERATING_MODE::BRIGHTNESS_ADJUST");
      } else if (operating_mode == OPERATING_MODE::MOMENTARY) {
        //nanoDebug("OPERATING_MODE::MOMENTARY");
      } else if (operating_mode == OPERATING_MODE::STROBE) {
        //nanoDebug("OPERATING_MODE::STROBE");
      } else if (operating_mode == OPERATING_MODE::VOLTAGE) {
        //nanoDebug("OPERATING_MODE::VOLTAGE");
      }
    }
  */

  if (operating_mode == OPERATING_MODE::BRIGHTNESS_ADJUST) {
    if (adjust_direction) {
      uint32_t duration = (mainButton.sinceLastEvent() / DELAY_MULTIPLIER );
      adjust_brightness = duration * 3 / 25 ; // approximate overflow limit reduction from: 254/2000, 3/25 = 240/2000
      adjust_brightness *= adjust_direction;
      if ((int32_t)start_brightness + adjust_brightness < 0) adjust_brightness = (int32_t) - start_brightness;
      else if ((int32_t)start_brightness + adjust_brightness > 253) adjust_brightness = (int32_t)253 - start_brightness; //for max of 254, 255 is turbo
      adjust_brightness += 1;
    }

    brightness = start_brightness + adjust_brightness;
    led_write(1);

  } else if (operating_mode == OPERATING_MODE::MOMENTARY) {
    light_enabled = !digitalRead(BUTTON_PIN);
    led_write();

  } else if (operating_mode == OPERATING_MODE::STROBE) {
    uint8_t i = 0;
    while (beacon_duration[beacon_mode][i] && (x_millis() - mode_timer) > beacon_duration[beacon_mode][i]) {
      i++;
    }
    if (beacon_duration[beacon_mode][i] == 0) {
      mode_timer = x_millis();
      i = 0;
    }
    light_enabled = ((i + 1) % 2);
    if (beacon_mode > 0) led_write(1);

  } else if (operating_mode == OPERATING_MODE::VOLTAGE) {
    //        nanoDebug(String(  String((int32_t)x_millis() - mode_timer) + String(" | ") + String(voltage) + String(", ") + String(voltage_digit) + String(", ") + String(voltage_temp)  ).c_str());

    if (voltage_digit == 0 && voltage_temp == 0) {
      light_enabled = 0;
      led_write(1);
      voltage = (read_Vcc() + VCC_OFFSET) / 10;
      voltage_digit = 3;
    }

    if (voltage_temp == 0) {
      voltage_temp = (voltage / (((voltage_digit >= 3) ? 10 : 1 ) * ((voltage_digit >= 2) ? 10 : 1 )) ) % 10;
      if (voltage_temp == 0) voltage_temp = 10;
      voltage_digit--;
      mode_timer = x_millis() + 2000;
    } else if ((int32_t)x_millis() - (int32_t)mode_timer >= 1000) {
      voltage_temp--;
      mode_timer = x_millis();
    }

    light_enabled = ((int32_t)x_millis() - (int32_t)mode_timer >= 500);
    led_write(1);

    if (voltage_digit == 0 && voltage_temp == 0 && voltage != 0) {
      voltage = 0;
      operating_mode = OPERATING_MODE::ON;
      sleep_mode = 1;
    }
  }


#ifdef __AVR_ATtiny85__

  if (voltage == 0 || x_millis() - bat_check_timer > 5000) {
    voltage = (read_Vcc() + VCC_OFFSET) / 10;
    bat_check_timer = x_millis();
  }

  #if defined(USB_POWER_PIN) && defined(CHARGING_STATUS_PIN)
  uint8_t power_pin = digitalRead(USB_POWER_PIN);
  uint8_t charging_pin = digitalRead(CHARGING_STATUS_PIN);

  if (power_pin) {
    operating_mode = OPERATING_MODE::CHARGING;
    sleep_mode = 0;
    light_enabled = 0;
    led_write(1);
  } else if (operating_mode == OPERATING_MODE::CHARGING) {
    operating_mode = OPERATING_MODE::ON;
    sleep_mode = 1;
    light_enabled = 0;
  }


  if (power_pin) {
    analogWrite(GREEN_LED_PIN, (charging_pin) ? 255-8 : 255-0); //8 brightness, more than enough
    analogWrite(RED_LED_PIN, (charging_pin) ? 255-0 : 255-255); //red is super easy to burn out. fuck.
  } else {
    #endif
    #if defined(RED_LED_PIN) && defined(GREEN_LED_PIN)
    if (voltage > 370) {
      analogWrite(GREEN_LED_PIN, (sleep_mode == 1) ? 255-0 : 255-8);
      digitalWrite(RED_LED_PIN, 1);
      
    } else if (voltage > 320) {
      analogWrite(GREEN_LED_PIN, (sleep_mode == 1) ? 255-0 : 255-8);
      analogWrite(RED_LED_PIN,  (sleep_mode == 1) ? 255-0 : 255-255);
      
    } else {
      digitalWrite(GREEN_LED_PIN, 1);
      analogWrite(RED_LED_PIN,  (sleep_mode == 1 || (x_millis() / 500) % 2) ? 255-0 : 255-255);
    }
    #endif
    #if defined(USB_POWER_PIN) && defined(CHARGING_STATUS_PIN)
  }
    #endif

#endif


  led_write();

}
