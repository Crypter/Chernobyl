#ifndef CLICKCHAIN_H
#define CLICKCHAIN_H


class ClickChain {
  private:
    uint8_t pin;
    uint8_t resting_position;
    uint8_t pullup;
    uint32_t state_change_timeout;
    uint32_t timer;

    uint8_t last_click_event = 0, end_chain=0;
    uint32_t click_chain_timer = 0;
    uint8_t debounce_pin_state = 1, old_pin_state = 1, click_chain_counter = 0;
    uint32_t debounce_timer = 0, debounce_ms;

    void (*button_down)(uint8_t), (*button_up)(uint8_t); //immediate actions
    void (*button_hold)(uint8_t), (*button_at_rest)(uint8_t); //confirmed actions

    void check_pin_change() {
      uint8_t pin_state = digitalRead(pin);

      if (pin_state != debounce_pin_state) {
        debounce_pin_state = pin_state;
        debounce_timer = millis();
      }

      if (millis() - debounce_timer > debounce_ms && pin_state != old_pin_state) {
        if (click_chain_counter == 0 && pin_state == resting_position) {
          last_click_event = 0;
          old_pin_state = pin_state;
          return; //error state, we missed the start of the click, probably the button was not resting during initialisation, anyway nothing to do
        }
        click_chain_timer = millis();
        click_chain_counter++;
        old_pin_state = pin_state;
      }
    }

    void manage_clicks() {
      if (click_chain_counter > 0) { //click event occured

        if (click_chain_counter % 2) { //button pressed
          if (last_click_event == 0 || last_click_event == 3) {
            last_click_event = 1;
            if (button_down && !end_chain) button_down(click_chain_counter / 2 + 1); //call button pressed function
          }

          if ((millis() - click_chain_timer) > state_change_timeout ) {
            if (last_click_event == 1) {
              last_click_event = 2;
              if (button_hold && !end_chain) button_hold(click_chain_counter / 2 + 1); //call button hold confirmed function
            }
          }
        } else { //button not pressed
          if (last_click_event == 1 || last_click_event == 2) {
            last_click_event = 3;
            if (button_up && !end_chain) button_up(click_chain_counter / 2); //call button at rest function
          }
          if ((millis() - click_chain_timer) > state_change_timeout ) {
            if (last_click_event == 3) {
              last_click_event = 0;
              if (button_at_rest && !end_chain) button_at_rest(click_chain_counter / 2); //call button at rest confirmed function
              click_chain_counter = 0;
              end_chain=0;
            }
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
      if (last_click_event == 2 || last_click_event == 0)
        return millis() - click_chain_timer - state_change_timeout;
      else
        return millis() - click_chain_timer;
    }

    uint8_t lastEvent(){
      return last_click_event;
    }

    void endChain() {
      end_chain = 1;
    }

};

#endif
