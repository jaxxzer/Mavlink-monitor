#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class Button {
public:
	Button();
	void init(void);
	void update(void);
private:
  uint32_t last_update_ms;
};
#endif
