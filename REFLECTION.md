# CarND-Controls-PID Project Reflection

---

## PID Introduction

PID (proportional–integral–derivative controller) is a algorithm widely used for continuously modulated control. A PID controller calculates the cross track error and applies a correction based on proportional, integral and derivative. [1]

* P (Propotional)
This term is propotional to the current error. When error is large and positive, the correction will be proportionally large and positive. However if the hyperparameter `Kp` is too big, over correction might be performed. As a result the vehicle might be contiously oscilating on the track.
[Example video when `Kp` is large (2.0)](./large_kp.mp4)

* I (Integral)
This term is integral of past errors. The integral term tries to eliminate the residual error (bias) by applying the correction.
The hyperparameter `Ki` is usually very small. Similary large values might cause over correction on the steering angle.
[Example video when `Ki` is large (1.0)](./large_ki.mp4)

* D (Derivative)
This term stands for the current rate of change on error. It is calculated as `current_error - last_error`.
The hyperparameter `Kd` is usually larger than `Kp` and `Ki`. Visually it smooths the possible over correction and reduces vehicle oscilation.
[Example video when `Kd` is small (0.1)](./small_kd.mp4)


## Hyperparameters

Hyperparameters `Kp`, `Ki` and `Kd` are chosen through the twiddle algorithm.
When the following global variable is set, the simulator will run with twiddle program instead of chosen hyperparameters.
```c++
static constexpr bool TWIDDLE = true;
```

Similar to the twiddle algorithm described by Sabstian, it measures the average error in two hundred frames and adjusts all three parameters accordingly.

The simulator is reset after every 400 frames to restart the tuning process.

While the algorithm is running, it prints out the best error and hyperparameters so far.
```
Best Error: 0.0216366 Best PID: (1, 0, 2.1)
Adjusting p
CTE: 2.1717 Steering Value: 0.028893
42["steer",{"steering_angle":0.0288930000000005,"throttle":0.3}]
```

Eventually the best error coverges at the following hyperprameters:
`Kp` = 0.109
`Ki` = 0.0
`Kd` = 0.901

## References
[1] https://en.wikipedia.org/wiki/PID_controller