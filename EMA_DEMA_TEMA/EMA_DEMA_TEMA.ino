

class EMA {
  public:
  float coef;
  float value=0;
  float samples;

  float setCoef(float value){
    this->coef = value;
    this->samples = (2/value)-1;
  }

  float setSamples(float value){
    this->setCoef(2/(value+1));
    this->samples = value;
  }

  float put(int16_t value){
    this->value = (this->value * coef) + (value * (1-coef));
    //this->value = this->value + coef*(value - this->value);
    return this->value;
  }

  float get(){
    return this->value;
  }
};


class DEMA {
  public:
  float coef;
  float value=0;
  float samples;

  EMA first_order;
  EMA second_order;

  float setCoef(float value){
    this->coef = value;
    this->samples = (2/value)-1;
    this->first_order.setCoef(value);
    this->second_order.setCoef(value);
  }

  float setSamples(float value){
    this->setCoef(2/(value+1));
    this->samples = value;
    this->first_order.setSamples(value);
    this->second_order.setSamples(value);
  }

  float put(int16_t value){
    this->first_order.put(value);
    this->second_order.put(this->first_order.get());
    this->value = (2*this->first_order.get()) - this->second_order.get();
    return this->value;
  }

  float get(){
    return this->value;
  }
};

class TEMA {
  public:
  float coef;
  float value=0;
  float samples;

  EMA first_order;
  EMA second_order;
  EMA third_order;

  float setCoef(float value){
    this->coef = value;
    this->samples = (2/value)-1;
    this->first_order.setCoef(value);
    this->second_order.setCoef(value);
    this->third_order.setCoef(value);
  }

  float setSamples(float value){
    this->setCoef(2/(value+1));
    this->samples = value;
    this->first_order.setSamples(value);
    this->second_order.setSamples(value);
    this->third_order.setSamples(value);
  }

  float put(int16_t value){
    this->first_order.put(value);
    this->second_order.put(this->first_order.get());
    this->third_order.put(this->second_order.get());
    this->value = (3*this->first_order.get()) - (3*this->second_order.get()) + this->third_order.get();
    return this->value;
  }

  float get(){
    return this->value;
  }
};


EMA ema;
DEMA dema;
TEMA tema;

void setup() {
pinMode(A5, OUTPUT);
pinMode(A4, INPUT);
pinMode(A3, OUTPUT);

digitalWrite(A5, HIGH);
digitalWrite(A3, LOW);
Serial.begin(115200);
ema.setCoef(0.9);
dema.setCoef(0.9);
tema.setCoef(0.9);
}

void loop() {
  uint16_t result = analogRead(A4)+random(-10, +10);
  ema.put(result);
  dema.put(result);
  tema.put(result);
  
Serial.print("0,1023,");
Serial.print(result);
Serial.print(",");
Serial.print(ema.get());
Serial.print(",");
Serial.print(dema.get());
Serial.print(",");
Serial.println(tema.get());
delay(75);
}
