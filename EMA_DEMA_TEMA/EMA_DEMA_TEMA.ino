

class EMA {
  public:
  float coef;
  float value=0;
  float samples;

  float setCoef(float value){
    this->coef = value;
    this->samples = (value+1)/(1-value);
  }

  float setSamples(float value){
    this->setCoef((value-1)/(value+1)); // = (1-(2/(samples+1)))
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
    this->samples = (value+1)/(1-value);
    this->first_order.setCoef(value);
    this->second_order.setCoef(value);
  }

  float setSamples(float value){
    this->setCoef((value-1)/(value+1)); // = (1-(2/(samples+1)))
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
    this->samples = (value+1)/(1-value);
    this->first_order.setCoef(value);
    this->second_order.setCoef(value);
    this->third_order.setCoef(value);
  }

  float setSamples(float value){
    this->setCoef((value-1)/(value+1)); // = (1-(2/(samples+1)))
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

class QEMA {
  public:
  float coef;
  float value=0;
  float samples;

  EMA first_order;
  EMA second_order;
  EMA third_order;
  EMA fourth_order;
  EMA fifth_order;

  float setCoef(float value){
    this->coef = value;
    this->samples = (value+1)/(1-value);
    this->first_order.setCoef(value);
    this->second_order.setCoef(value);
    this->third_order.setCoef(value);
    this->fourth_order.setCoef(value);
    this->fifth_order.setCoef(value);
  }

  float setSamples(float value){
    this->setCoef((value-1)/(value+1)); // = (1-(2/(samples+1)))
    this->samples = value;
    this->first_order.setSamples(value);
    this->second_order.setSamples(value);
    this->third_order.setSamples(value);
    this->fourth_order.setSamples(value);
    this->fifth_order.setSamples(value);
  }

//5*MA1-10*MA2+10*MA3-5*MA4+MA5
  float put(int16_t value){
    this->first_order.put(value);
    this->second_order.put(this->first_order.get());
    this->third_order.put(this->second_order.get());
    this->fourth_order.put(this->third_order.get());
    this->fifth_order.put(this->fourth_order.get());
    this->value = (5*this->first_order.get()) - (10*this->second_order.get()) + (10*this->third_order.get()) - (5*fourth_order.get()) + fifth_order.get();
    return this->value;
  }

  float get(){
    return this->value;
  }
};



EMA ema;
DEMA dema;
TEMA tema, tema2;
QEMA qema, qema2;

void setup() {
  // put your setup code here, to run once:
pinMode(A5, OUTPUT);
pinMode(A4, INPUT);
pinMode(A3, OUTPUT);

digitalWrite(A5, HIGH);
digitalWrite(A3, LOW);
Serial.begin(115200);
ema.setSamples(100);
dema.setSamples(100);
tema.setSamples(100);
qema.setSamples(100);
tema2.setSamples(100*1.5);
qema2.setSamples(100);
}

void loop() {
  int16_t result = analogRead(A4) +random(-50, +50);
  result = constrain(result, 0, 1023);
  ema.put(result);
  dema.put(result);
  tema.put(result);
  qema.put(result);
  tema2.put(result);
  qema2.put(result);
  
Serial.print("\r\n0,1023,");
Serial.print(result);
Serial.print(",");
Serial.print(ema.get());
Serial.print(",");
Serial.print(dema.get());
Serial.print(",");
Serial.print(tema.get());
Serial.print(",");
Serial.print(qema.get());
Serial.print(",");
Serial.print(0.7*qema2.get() + 0.3*tema2.get()); //best damping with OK overshoot
delay(10);
}
