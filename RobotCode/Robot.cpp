#include "WPILib.h"
#include "Robot.h"
#include "Control.h"
#include "Drive.h"
#include "Log.h"

class BcdSwitch {
	DigitalInput *port[4];
public:
	BcdSwitch(int port1, int port2, int port3, int port4) {
		this->port[0] = new DigitalInput(port1);
		this->port[1] = new DigitalInput(port2);
		this->port[2] = new DigitalInput(port3);
		this->port[3] = new DigitalInput(port4);
	}
	int value() {
		int ret = 0;
		for(int i = 0; i < 4; ++i) {
			if (port[i]->Get())
				ret |= 1<<i;
		}
		return ret;
	}

};

class Robot : public IterativeRobot {
	Drive* drive;
	Control* control;
	BcdSwitch* bcd;
	Log* log;

public:
	Robot() {
		
		bcd = new BcdSwitch(11, 12, 13, 14);
		
		log = new Log(this);
		
		drive = new Drive(2, this);
		drive->addMotor(Drive::Left, 2, 1);
		drive->addMotor(Drive::Left, 3, 1);
		drive->addMotor(Drive::Right, 4, -1);
		drive->addMotor(Drive::Right, 5, -1);
		
		control = new Control(
				new Joystick(1), new Joystick(2), new Joystick(3), 
				Control::Tank, log);
		control->setLeftScale(-1);
		control->setRightScale(-1);
		control->setGamepadScale(-1);
	}
	
	void AutonomousInit() {
		//int value = bcd->value();
		// TODO
	}
	
	void AutonomousPeriodic() { 
		// TODO
	}

	void AutonomousDisabled() {
		//delete autonomous;
	}
	
	void TeleopInit() {
		drive->setShiftMode(Drive::Manual);
	}

	Relay::Value gathererDirection;

	void TeleopPeriodic() {
		if (control->button(2)) {
			//robot.balance->loop();
			return;
		}

		// drive
		drive->setLeft(control->left());
		drive->setRight(control->right());
		drive->setScale(control->throttle());
		drive->setReversed(control->toggleButton(11));
		drive->setLowShift(control->gamepadToggleButton(9));

		// ball gatherer
		/*
		if (control->gamepadButton(6)) 
			gathererDirection = Relay::kForward;
		else if (control->gamepadButton(7))
			gathererDirection = Relay::kReverse;
		else if (control->gamepadButton(8))
			gathererDirection = Relay::kOff;
		gatherer->setDirection(gathererDirection);
		*/

		// Button 11 is mode control: if enabled, the joystick controls
		// the bridge mechanism. If not, it's the standard arm.
		/*
		if (!control->gamepadButton(11)) {
			if (control->gamepadButton(4))
				arm->setPosition(Arm::Up);
			else if (control->gamepadButton(3))
				arm->setPosition(Arm::Middle);
			else if (control->gamepadButton(5))
				arm->setPosition(Arm::Down);
			else 
				arm->setPower(control->gamepadLeft());
			
			rampDevice->set(0);
		} else {
			rampDevice->set(control->gamepadLeft());
			arm->setPower(0);
		}
		*/
			
		/*if (control->gamepadButton(10))
			arm->setPidFactor(arm->pidFactor() - 0.01);
		if (control->gamepadButton(11))
			arm->setPidFactor(arm->pidFactor() + 0.01);*/

		// assorted debug
		log->info("Shift %s", control->toggleButton(8) 
				? "low" : "high");
		//log->info("ArmPot: %.0f", arm->encoderValue());
		//log->info("pidf: %.2f", arm->pidFactor());
		//log->info("piden: %s", arm->isPidEnabled() ? "true" : "false");
		log->print();
		
		/*
		// Print out shape matches from camera
		imageTracker->updateImage();
		//imageTracker->writeFiles();
		std::vector<RectangleMatch> matches = imageTracker->matches();
		RectangleMatch topMatch;
		memset(&topMatch, 0, sizeof(topMatch));
		for(size_t i = 0; i < matches.size(); ++i) {
			if(matches[i].score > topMatch.score) topMatch = matches[i];
		}*/
		
		/*log->info("rect %.1f: %.1f, %.1f, %.1f, %.1f", 
				topMatch.score,
				topMatch.corner[1].x,
				topMatch.corner[1].y,
				topMatch.corner[3].x,
				topMatch.corner[3].y);*/
	}
};

START_ROBOT_CLASS(Robot);

