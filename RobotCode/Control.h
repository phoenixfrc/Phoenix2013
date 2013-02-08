#ifndef CONTROL_H
#define CONTROL_H
#include "WPILib.h"
#include "Robot.h"
#include <vector>

class Log;

class Control {
	Joystick *left_, *right_;
	// joystick to get button presses
	Joystick *control_;

	Joystick *gamepad_;
	double leftScale_, rightScale_, gamepadScale_;

	std::vector<bool> isTriggered_;
	std::vector<bool> wasTriggered_;

	std::vector<bool> gamepadIsTriggered_;
	std::vector<bool> gamepadWasTriggered_;

	Log *log_;

	public:
	enum Mode { Tank, Arcade };
	Control(Joystick *left, Joystick *right, Joystick *gamepad, 
			Mode mode, Log *log);
	virtual ~Control();

	// set static scaling
	void setLeftScale(double scale);
	void setRightScale(double scale);
	void setGamepadScale(double scale);

	// axes
	double left();
	double right();
	double gamepadLeft();
	double gamepadRight();

	// scaling
	double throttle();
	bool isReversed();

	// buttons
	bool button(int num);
	bool toggleButton(int num);
	bool gamepadButton(int num);
	bool gamepadToggleButton(int num);
};

#endif
