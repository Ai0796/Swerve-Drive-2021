Rotational Kinematics
===========================

.. meta::
   :description lang=en: Introduction to Rotational Kinematics


Wheels, which you will find are used a lot in robotics, cannot be calculated with translational kinematics, instead they require rotational kinematics, which are used to calculate:

* Angular Displacement
* Rotational Velocity
* Rotational Acceleration

In most cases in FRC, rotational calculations aren't very complicated as they usually are converted to translational units anyways.

Angular Displacement
--------------------

Angular Displacement commonly uses `radians <https://en.wikipedia.org/wiki/Radian>`_ in order to calculate how much a wheel has rotated from it's starting position. This is typically used in Swerves and Auton to rotate an object the correct distance and/or orientation.

Rotational Velocity
--------------------

Rotational Velocity is the change in Angular Displacement over a unit of time. Typically in FRC you will see units such as:

* rpm (rotations per minute) <- Unit used by most encoders
* rps (rotations per second)
* radians per second
* degrees per second

Rotational Velocity is commonly converted into Translational velocity by either:

* for rpm and rps multiply the value by the circumference of the wheel (make sure to transfer the seconds or minutes)
* for radians divide the value by 2pi and multiply by circumference
* for degrees divide by 360 and multiple by circumference

Rotational Acceleration is the change in Rotational Velocity, acceleration typically does not play a massive effect in FRC since motors can accelerate to max velocity in less than a second in normal conditions

Further information
-------------------

`Wikipedia Page <https://en.wikipedia.org/wiki/Rotation_around_a_fixed_axis>`_
