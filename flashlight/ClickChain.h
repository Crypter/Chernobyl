#ifndef BUTTONHANDLER_H
#define BUTTONHANDLER_H


class ClickChain {
  private:
    uint8_t pin;
    uint8_t resting_position;
    uint8_t pullup;
    uint32_t state_change_timeout;
    uint32_t timer;

    uint8_t last_click_event = 0;
    uint32_t click_chain_timer = 0;
    uint8_t debounce_pin_state = 1, old_pin_state = 1, click_chain_counter = 0;
    uint32_t debounce_timer = 0, time_to_sleep = 0, debounce_ms;
//    uint32_t click_chain[20];

    void (*button_down)(uint8_t), (*button_up)(uint8_t); //immediate actions
    void (*button_hold)(uint8_t), (*button_at_rest)(uint8_t); //confirmed actions

    void check_pin_change() {
      uint8_t pin_state = digitalRead(pin);

      if (pin_state != debounce_pin_state) {
        debounce_pin_state = pin_state;
        debounce_timer = millis();
      }

      if (millis() - debounce_timer > debounce_ms && pin_state != old_pin_state) {

        if (click_chain_counter == 0 && pin_state == resting_position) return; //error state, we missed the start of the click or the chain has ended, anyway nothing to do

        if (click_chain_counter < 19) {
//          click_chain[click_chain_counter] = millis();
          click_chain_timer = millis();
          click_chain_counter++;
        }
        old_pin_state = pin_state;
      }
    }

    void manage_clicks() {
      if (click_chain_counter > 0) { //click event occured

        if (click_chain_counter % 2) { //button pressed
//          if ((millis() - click_chain[click_chain_counter - 1]) > state_change_timeout ) {
          if ((millis() - click_chain_timer) > state_change_timeout ) {
            if (last_click_event == 1) {
              if (button_hold) button_hold(click_chain_counter / 2 + 1); //call button hold confirmed function
              last_click_event = 2;
            }
          }

          if (last_click_event == 0 || last_click_event == 3) {
            if (button_down) button_down(click_chain_counter / 2 + 1); //call button pressed function
            last_click_event = 1;
          }
        } else { //button not pressed
          if ((millis() - click_chain_timer) > state_change_timeout ) {
            if (last_click_event == 3) {
              if (button_at_rest) button_at_rest(click_chain_counter / 2); //call button at rest confirmed function
              last_click_event = 0;
              click_chain_counter = 0;
            }
          }
          if (last_click_event == 1 || last_click_event == 2) {
            if (button_up) button_up(click_chain_counter / 2); //call button at rest function
            last_click_event = 3;
          }
        }
      }
    }

  public:
    ClickChain(uint8_t pin, uint8_t resting_position = 1, uint8_t pullup = 1, uint16_t debounce_ms = 30) {
      this->pin = pin;
      this->resting_position = resting_position;
      this->pullup = pullup;
      this->debounce_ms = debounce_ms;
    }

    void begin(uint32_t button_up_timeout) {
      pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
      this->timer = millis();
      this->state_change_timeout = button_up_timeout;
    }
    void setFunctions(void (*button_down)(uint8_t), void (*button_up)(uint8_t), void (*button_hold)(uint8_t), void (*button_at_rest)(uint8_t)) {

      this->button_down = button_down; //instant callback
      this->button_up = button_up; //instant callback
      this->button_hold = button_hold; //callback after state_change_timeout
      this->button_at_rest = button_at_rest;  //callback after state_change_timeout
    }

    void run() {
      check_pin_change();
      manage_clicks();
    }

    uint32_t sinceLastEvent() {
//      return millis() - click_chain_timer - (last_click_event == 2 || last_click_event == 0) ? 0:state_change_timeout;


        if (last_click_event == 2 || last_click_event == 0)
          return millis() - click_chain_timer - state_change_timeout;
        else
          return millis() - click_chain_timer;

      
//      if (click_chain_counter){
//        if (last_click_event == 2 || last_click_event == 0)
//          return millis() - click_chain[click_chain_counter - 1] - state_change_timeout;
//        else
//          return millis() - click_chain[click_chain_counter - 1];
//      }
//      else return 0;
    }

    void endChain() {
      click_chain_counter = 0;
    }

};

#endif
