/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helpers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * A wrapper for the WPILib Button class, to be used with ControllerWrapper.
 * Allows Runnables to be used rather than commands.
 * 
 * @author hrl
 */
public class ButtonWrapper extends Trigger {
    private final XboxController controller;
    private final int id; // either contains the button ID or the POV number
    private final int angle; // only used if there's a POV button
    private final boolean isPOV; // determines if the button's a POV button
    private final boolean isTrigger; // determines whether trigger mapped to button

    private boolean lastState = false; // previous state of the button

    public ButtonWrapper(XboxController controller, int id) {
        this.controller = controller;
        this.id = id;
        this.angle = 0; // not a POV button
        this.isPOV = false; // still not a POV button
        this.isTrigger = false; // not a trigger
    }

    /**
     * Only use this constructor if you're setting a POV button.
     * You're *probably* working on POV 0.
     */
    public ButtonWrapper(XboxController controller, int angle, int povNumber) {
        this.isPOV = true;
        this.controller = controller;
        this.id = povNumber;
        this.angle = angle;
        this.isTrigger = false; // not a trigger
    }

    public ButtonWrapper(XboxController controller, int id, boolean trigger) {
        this.isPOV = false;
        this.controller = controller;
        this.id = id;
        this.angle = 0;
        this.isTrigger = trigger;
    }

    /**
     * Retrieves the state of the button.
     * Required to extend the Button class.
     * @return button state as a boolean
     */
    public boolean get() {
        if (isPOV) {
            return this.controller.getPOV(this.id) == this.angle;
        }
        else if (isTrigger) {
            return this.controller.getRawAxis(this.id) > 0.5 || this.controller.getRawAxis(this.id) < -0.5;
        }
        return this.controller.getRawButton(this.id);
    }

    /**
     * Runs "something" on the rising edge of a button press.
     * @param func a lambda representing the action
     */
    public void whenPressed(Runnable func) {
        // button is true, and we ran a whenReleased
        if (this.get() && !lastState) {
            func.run();
            lastState = true;
        }
    }

    /**
     * Runs "something" on the falling edge of a button press.
     * @param func a lambda representing the action
     */
    public void whenReleased(Runnable func) {
        // button is false, and a whenPressed or whileHeld happened
        if (!this.get() && lastState) {
            func.run();
            lastState = false;
        }
    }

    /**
     * Runs "something" while the button is held.
     * This shouldn't conflict with whenPressed()'s implementation, as we shouldn't
     * be binding whenPressed and whileHeld to the same button ever.
     * @param func a lambda representing the action
     */
    public void whileHeld(Runnable func) {
        if (this.get()) {
            func.run();
            lastState = true;
        }
    }

    /**
     * Manually updates the states of the button.
     */
    public void updateState() {
        if (this.get() && !lastState) { // rising edge
            lastState = true;
        } else if (!this.get() && lastState) { // falling edge
            lastState = false;
        }
    }
}
