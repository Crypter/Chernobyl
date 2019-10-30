// CIE1931 correction table
// See excel file for generation
const uint8_t cie[256] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 29, 29, 30, 30, 31, 32, 32, 33, 34, 34, 35, 36, 36, 37, 38, 38, 39, 40, 41, 41, 42, 43, 44, 45, 45, 46, 47, 48, 49, 50, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 88, 89, 90, 91, 93, 94, 95, 97, 98, 100, 101, 102, 104, 105, 107, 108, 109, 111, 112, 114, 115, 117, 119, 120, 122, 123, 125, 126, 128, 130, 131, 133, 135, 136, 138, 140, 142, 143, 145, 147, 149, 150, 152, 154, 156, 158, 160, 162, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 186, 188, 190, 192, 194, 196, 198, 201, 203, 205, 207, 209, 212, 214, 216, 219, 221, 223, 226, 228, 231, 233, 235, 238, 240, 243, 245, 248, 250, 253, 255};

//void nanoDebug(const char *data){
//#ifndef __AVR_ATtiny85__
////  Serial.println(data);
////  Serial.flush();
//#endif
//}

uint8_t brightness_to_pwm(uint8_t brightness){
  if (INVERTED_BRIGHTNESS){
    if (brightness == 255) return 0;
    else if (brightness == 0) return 255;
    else return map(brightness, 1, 254, MAX_BRIGHTNESS, MIN_BRIGHTNESS);
  } else {
    if (brightness == 255) return 255;
    else if (brightness == 0) return 0;
    else return map(brightness, 1, 254, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
  }
}

uint16_t read_Vcc() {
//  return 4200;
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(5); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  uint16_t result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  //calculated reference is ~1.21V, hence devidant is 1240700

  return result; // Vcc in millivolts
}

int16_t read_temperature() {
#ifdef __AVR_ATtiny85__
  analogReference(INTERNAL1V1);
  int16_t raw = analogRead(A0+15);
  /* Original code used a 13 deg adjustment. But based on my results, I didn't seem to need it. */
  // raw -= 13; // raw adjust = kelvin //this value is used to calibrate to your chip
  int16_t in_c = raw - 273; // celcius
//  analogReference(DEFAULT);
  return in_c;
#else
  return 25;
#endif
}


uint32_t x_millis() {
  return millis() / DELAY_MULTIPLIER;
}

uint32_t x_micros() {
  return micros() / DELAY_MULTIPLIER;
}

void x_delay(uint32_t X) {
  return delay(X * DELAY_MULTIPLIER);
}
