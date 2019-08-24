// CIE1931 correction table
const uint8_t cie[256] = {
  0, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  3, 3, 3, 3, 4, 4, 4, 4, 4, 4,
  4, 4, 4, 4, 5, 5, 5, 5, 5, 5,
  5, 6, 6, 6, 6, 6, 6, 7, 7, 7,
  7, 8, 8, 8, 8, 8, 9, 9, 9, 9,
  10, 10, 10, 10, 10, 11, 11, 11, 11, 12,
  12, 12, 13, 13, 13, 14, 14, 14, 15, 15,
  15, 16, 16, 16, 17, 17, 17, 18, 18, 18,
  19, 19, 19, 20, 20, 20, 21, 21, 21, 22,
  22, 23, 23, 24, 24, 25, 25, 26, 26, 27,
  28, 28, 29, 29, 30, 31, 31, 32, 32, 33,
  34, 34, 35, 36, 37, 37, 38, 39, 39, 40,
  41, 42, 43, 43, 44, 45, 46, 47, 47, 48,
  49, 50, 51, 52, 53, 54, 54, 55, 56, 57,
  58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
  68, 70, 71, 72, 73, 74, 75, 76, 77, 79,
  80, 81, 82, 83, 85, 86, 87, 88, 90, 91,
  92, 94, 95, 96, 98, 99, 100, 102, 103, 105,
  106, 108, 109, 110, 112, 113, 115, 116, 118, 120,
  121, 123, 124, 126, 128, 129, 131, 132, 134, 136,
  138, 139, 141, 143, 145, 146, 148, 150, 152, 154,
  155, 157, 159, 161, 163, 165, 167, 169, 171, 173,
  175, 177, 179, 181, 183, 185, 187, 189, 191, 193,
  196, 198, 200, 202, 204, 207, 209, 211, 214, 216,
  218, 220, 223, 225, 228, 230, 232, 235, 237, 240,
  242, 245, 247, 250, 252, 255
};

void nanoDebug(const char *data){
#ifndef __AVR_ATtiny85__
  Serial.println(data);
  Serial.flush();
#endif
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
#ifdef __AVR_ATtiny85__
  return millis() / DELAY_MULTIPLIER;
#else
  return millis();
#endif
}

uint32_t x_micros() {
#ifdef __AVR_ATtiny85__
  return micros() / DELAY_MULTIPLIER;
#else
  return micros();
#endif
}

void x_delay(uint32_t X) {
#ifdef __AVR_ATtiny85__
  return delay(X * DELAY_MULTIPLIER);
#else
  return delay(X);
#endif
}

