Basic Math of a Swerve Drive
===========================

.. meta::
   :description lang=en: Introduction to Swerve Drives and how they work


Swerve Drives (hopefully) move on a 2D plane, which means we can apply kinematics to the drive (you learn kinematics in Physics 1)

There are two main forces are play with a Swerve Drive:

* Linear force, which you can think as a force moving it in a straight line
* Rotational force, which you can think as a force that rotates an object

We can combine these two forces using kinematics in order to move in any direction and rotation at the same time!

Kinematics
----------

Kinematics is a field of physics used to describe motion of points and objects, it is sometimes also known as the geometry of motion. Kinematics deals with how an object moves without looking at the cause

Usual terms you'll hear for kinematics are:

* Distance
* Speed
* Direction
* Velocity
* Acceleration
* etc.


Distance
--------

Distance is simply enough the amount that an object has moved over time. It's the most basic part of kinematics and you'll use it a lot if you ever plan to take any math or physics class

Usual measurements in FRC:

* in (inches) 
* ft (feet) <- most our measurements are in imperial and usually aren't big enough to use feet
* m (meters)
* mm (milimeters)

Speed
-----

Everything with Kinematics starts with speed (and direction), a speed is how fast an object moves over a given time period.

Usual endings in FRC:

* ft/s (feet per second) <- We use ft/s for pretty much everything
* mph (miles per hours)
* m/s (meters per second)

Direction
---------

The course that an object takes. We usually only use relative direction (Left, right, forwards, etc.) in FRC due to our gyrometer working relative to the robot.

Velocity
--------

The difference between speed and velocity is that velocity is directed towards a direction. For instance, speed would be saying I'm walking, while velocity would be I'm walking forwards.

Acceleration
-------------

Acceleration is one of the fun niche concepts that you basically have to know to understand a swerve drive (along with possibly jerk). Acceleration is a change in velocity over time. For example, when you drop an apple it doesn't fall at a constant speed, as it falls it speeds up, that's acceleration.

.. seealso:: Jerk is the change in acceleration over time, usually robotics never has to go into jerk, but since swerving works in such a small rotation, using jerk could save multiple seconds in a 150 second match.

Usual units of acceleration that we use are:

* ft/s^2(feet per second squared)
* m/s^2 (meter per second squared)