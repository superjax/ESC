# ESC
An arduino library for running SimonK (400 Hz refresh rate) Electronic Speed Controllers (ESC) for Brushless DC motors.  These motors are typically used in RC multirotors and cars, and require update rates faster than the standard 50 Hz in the standard Servo library.

This is a modified version of the standard Servo library written by Michael Margolis for Arduino, but it has been modified to use a 400 Hz refresh rate for use with SimonK ESCs, and some additional arming and disarming functionality.

By default, the ESCs will be sent a 1000us signal upon attaching, which will cause them to turn on, but not to spin the motors.  Upon calling arm(), the ESCs will be limited to a minimum value of 1100us, which should cause the motors to constantly spin, until disarmed.  This is implemented for safety reasons, but if your application requires something different, let me know and I'll make that a configurable option.

The library assumes your ESCs are calibrated to from 1000us to 2000us, and the write() function takes values between 0.0 and 1.0, being a "normalized throttle" value which maps output from 1000us to 2000us.  The writeMicroseconds() functions as before, but with the caveat that you must arm the ESC before any value above 1000us can be written to the ESC.

This code has only been teseted on the UNO.  It is not supported on any other arduino.  However, I tried to keep the library as similar to the original Servo library as possible, so it is likely that it will work on other arduinos as well.


