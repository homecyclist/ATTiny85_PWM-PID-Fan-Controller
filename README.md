# ATTiny85_PWM-PID-Fan-Controller
This is an implementation of PID control for a PC Fan following Intel's PWM spec.

This is my Arduino implementation of PWM Fan control using a 10K NTC and PID. I rewrote the timer implementation on the Arduino core to have 64 bit resolution for tracking milliseconds. Re-writing the timers was required to gain 25KHz PWM on PIN6 using Timer0. PWM resolution is approx 7.313 bits (0-159).
