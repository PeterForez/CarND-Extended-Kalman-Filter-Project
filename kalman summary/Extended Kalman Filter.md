- [01: Kalman Filter in C++](#01-kalman-filter-in-c)
- [02: Intro](#02-intro)
- [03: Lesson Map and Fusion Flow](#03-lesson-map-and-fusion-flow)
- [04: Lesson Variables and Equations](#04-lesson-variables-and-equations)
- [05: Estimation Problem Refresh](#05-estimation-problem-refresh)
  - [QUIZ QUESTION](#quiz-question)
- [06: Kalman Filter Intuition](#06-kalman-filter-intuition)
  - [Kalman Filter Intuition](#kalman-filter-intuition)
    - [Prediction](#prediction)
    - [Update](#update)
  - [A Note About the State Transition Function: $Bu$](#a-note-about-the-state-transition-function-bu)
- [07: Kalman Filter Equations in C++](#07-kalman-filter-equations-in-c)
  - [Prediction Step](#prediction-step)
  - [Update Step](#update-step)
  - [Summary](#summary)
  - [Kalman Filter Algorithm](#kalman-filter-algorithm)
  - [Implement a multi-dimensional Kalman Filter](#implement-a-multi-dimensional-kalman-filter)
- [09: State Prediction](#09-state-prediction)
  - [Quiz](#quiz)
    - [Quiz 1](#quiz-1)
    - [Quiz 2](#quiz-2)
- [10: Process Covaraince Noise](#10-process-covaraince-noise)
  - [Calculating Acceleration Noise Parameters](#calculating-acceleration-noise-parameters)
  - [Process Covariance Matrix $Q$ - Intuition](#process-covariance-matrix-q---intuition)
  - [Note on Notation](#note-on-notation)
- [11: Laser Measurements Part 1](#11-laser-measurements-part-1)
  - [Variable Definitions](#variable-definitions)
  - [$H$ Matrix Quiz](#h-matrix-quiz)
- [12: Laser Measurements Part 2](#12-laser-measurements-part-2)
- [13: Laser Measurements Part 3](#13-laser-measurements-part-3)
  - [Measurement Noise Covariance Matrix R continued](#measurement-noise-covariance-matrix-r-continued)
  - [Helpful Equations](#helpful-equations)
- [15: Radar Measurements](#15-radar-measurements)
  - [H versus h(x)](#h-versus-hx)
  - [Definition of Radar Variables](#definition-of-radar-variables)
- [16: Mapping with a Nonlinear Function](#16-mapping-with-a-nonlinear-function)
- [17: Extended Kalman Filter](#17-extended-kalman-filter)
  - [How to Perform a Taylor Expansion](#how-to-perform-a-taylor-expansion)
- [18: Multivariate Taylor Series Expansion](#18-multivariate-taylor-series-expansion)
- [19: Jacobian Matrix](#19-jacobian-matrix)
- [20: CalculateJacobian()](#20-calculatejacobian)
- [21: EKF Algorithm Generalization](#21-ekf-algorithm-generalization)
  - [Extended Kalman Filter Equations](#extended-kalman-filter-equations)
  - [Clarification of $u = 0$](#clarification-of-u--0)
  - [More Details About Calculations with Radar Versus Lidar](#more-details-about-calculations-with-radar-versus-lidar)
  - [QUIZ QUESTION](#quiz-question-1)
- [23: Evaluating KF Performance](#23-evaluating-kf-performance)

# 01: Kalman Filter in C++
https://www.youtube.com/watch?v=Hsvzm7zDG_A 

# 02: Intro
Andrei Vatavu: Sensior Fusion Engineer at Mercedes Benz

* We will learn how to develop a Fusion System by using Kalaman Filter.
* We will combine measurement from different sensors. 
* We will use sensor fusion to track pedestrian. 
* We will use RADAR and LIDAR sensors.
* We will estimate the location and heading and speed.

# 03: Lesson Map and Fusion Flow
![](https://video.udacity-data.com/topher/2017/February/58b4d902_screenshot-from-2017-02-27-19-56-58/screenshot-from-2017-02-27-19-56-58.png)

Imagine you are in a car equipped with sensors on the outside. The car sensors can detect objects moving around: for example, the sensors might detect a pedestrian, as described in the video, or even a bicycle. For variety, let's step through the Kalman Filter algorithm using the bicycle example.

The Kalman Filter algorithm will go through the following steps:

* **first measurement** - the filter will receive initial measurements of the bicycle's position relative to the car. These measurements will come from a radar or lidar sensor.
* **initialize state and covariance matrices** - the filter will initialize the bicycle's position based on the first measurement.
* then the car will receive another sensor measurement after a time period $\Delta{t}$.
* **predict** - the algorithm will predict where the bicycle will be after time $\Delta{t}$. One basic way to predict the bicycle location after $\Delta{t}$ is to assume the bicycle's velocity is constant; thus the bicycle will have moved velocity * $\Delta{t}$. In the extended Kalman filter lesson, we will assume the velocity is constant.
* **update** - the filter compares the "predicted" location with what the sensor measurement says. The predicted location and the measured location are combined to give an updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value.
* then the car will receive another sensor measurement after a time period $\Delta{t}$. The algorithm then does another **predict** and **update** step.

# 04: Lesson Variables and Equations
[Sensor Fusion EKF Reference.pdf](https://video.udacity-data.com/topher/2018/June/5b327c11_sensor-fusion-ekf-reference/sensor-fusion-ekf-reference.pdf)
# 05: Estimation Problem Refresh
![](https://video.udacity-data.com/topher/2017/February/58b4bc06_screen-shot-2017-02-27-at-17.52.45/screen-shot-2017-02-27-at-17.52.45.png)
Definition of Variables
* $x$ is the **mean state vector**. For an extended Kalman filter, the mean state vector contains information about the object's position and velocity that you are tracking. It is called the "mean" state vector because position and velocity are represented by a gaussian distribution with mean $x$.
* $P$ is the **state covariance matrix**, which contains information about the uncertainty of the object's position and velocity. You can think of it as containing standard deviations.
* $k$ represents time steps. So $x_k$ refers to the object's position and velocity vector at time $k$.
* The notation $k+1|k$ refers to the prediction step. At time $k+1$, you receive a sensor measurement. Before taking into account the sensor measurement to update your belief about the object's position and velocity, you predict where you think the object will be at time $k+1$. 
You can predict the position of the object at $k+1$ based on its position and velocity at time $k$. Hence $x_{k+1|k}$ means that you have predicted where the object will be at $k+1$ but have not yet taken the sensor measurement into account.
* $x_{k+1}$ means that you have now predicted where the object will be at time $k+1$ and then used the sensor measurement to update the object's position and velocity.

## QUIZ QUESTION
What should a Kalman Filter do if both the radar and laser measurements arrive at the same time, $k+3$? Hint: The Kalman filter algorithm predicts -> updates -> predicts -> updates, etc. If two sensor measurements come in simultaneously, the time step between the first measurement and the second measurement would be zero.



QUIZ QUESTION
What should a Kalman Filter do if both the radar and laser measurements arrive at the same time, k+3? Hint: The Kalman filter algorithm predicts -> updates -> predicts -> updates, etc. If two sensor measurements come in simultaneously, the time step between the first measurement and the second measurement would be zero.


- [ ] Predict the state to $k+3$, and then update the state with only one of those measurements, either laser or radar.
- [ ] Predict the state to $k+3$, and then only update with the laser measurement because it is more accurate. 
- [x] Predict the state to k+3 then use either one of the sensors to update. Then predict the state to k+3 again and update with the other sensor measurement.
- [ ] Skip the prediction step becaseu we have two measurements. Just update the prior probability distribution twice.

> As you saw, the Kalman filter is a two-step process: predict, and then update. If you receive two measurements simultaneously, you can use this process with either measurement and then repeat the process with the other measurement. The order does not matter!

![](https://video.udacity-data.com/topher/2017/February/58b4bc59_measureupdatequizpost/measureupdatequizpost.png)
> Because we have already run a prediction-update iteration with the first sensor at time k+3, the output of the second prediction at time k+3 will actually be identical to the output from the update step with the first sensor. So, in theory, you could skip the second prediction step and just run a prediction, update, update iteration.

# 06: Kalman Filter Intuition

## Kalman Filter Intuition
The Kalman equation contains many variables, so here is a high level overview to get some intuition about what the Kalman filter is doing.

### Prediction
Let's say we know an object's current position and velocity , which we keep in the $x$ variable. Now one second has passed. We can predict where the object will be one second later because we knew the object position and velocity one second ago; we'll just assume the object kept going at the same velocity.

The $x' = Fx + \nu x$ equation does these prediction calculations for us.

But maybe the object didn't maintain the exact same velocity. Maybe the object changed direction, accelerated or decelerated. So when we predict the position one second later, our uncertainty increases. $P' = FPF^T + Q$ represents this increase in uncertainty.

Process noise refers to the uncertainty in the prediction step. We assume the object travels at a constant velocity, but in reality, the object might accelerate or decelerate. The notation $\nu \sim N(0, Q)$ defines the process noise as a gaussian distribution with mean zero and covariance $Q$.

### Update
Now we get some sensor information that tells where the object is relative to the car. First we compare where we think we are with what the sensor data tells us $y = z - Hx'$.

The $K$ matrix, often called the **Kalman filter gain**, combines the uncertainty of where we think we are $P'$ with the uncertainty of our sensor measurement $R$. If our sensor measurements are very uncertain ($R$ is high relative to $P'$), then the Kalman filter will give more weight to where we think we are: $x'$. If where we think we are is uncertain ($P'$ is high relative to $R$), the Kalman filter will put more weight on the sensor measurement: $z$.

Measurement noise refers to uncertainty in sensor measurements. The notation $\omega \sim N(0, R)$ defines the measurement noise as a gaussian distribution with mean zero and covariance $R$. Measurement noise comes from uncertainty in sensor measurements.

## A Note About the State Transition Function: $Bu$
If you go back to the video, you'll notice that the state transition function was first given as $x' = Fx + Bu + \nu x$. But then $Bu$ was crossed out leaving $x' = Fx + \nu x$. $B$ is a matrix called the control input matrix and $u$ is the control vector.

As an example, let's say we were tracking a car and we knew for certain how much the car's motor was going to accelerate or decelerate over time; in other words, we had an equation to model the exact amount of acceleration at any given moment. #Bu# would represent the updated position of the car due to the internal force of the motor. We would use $\nu$ to represent any random noise that we could not precisely predict like if the car slipped on the road or a strong wind moved the car.

For the Kalman filter lessons, we will assume that there is no way to measure or know the exact acceleration of a tracked object. For example, if we were in an autonomous vehicle tracking a bicycle, pedestrian or another car, we would not be able to model the internal forces of the other object; hence, we do not know for certain what the other object's acceleration is. Instead, we will set $Bu = 0$ and represent acceleration as a random noise with mean $\nu$.

# 07: Kalman Filter Equations in C++
Now, let's do a quick refresher of the Kalman Filter for a simple 1D motion case. Let's say that your goal is to track a pedestrian with state $x$ that is described by a position and velocity.
$$
x = \begin{pmatrix} p \\ v \end{pmatrix}
$$ 
## Prediction Step
When designing the Kalman filter, we have to define the two linear functions: the state transition function and the measurement function. The state transition function is

$$x' = Fx + noise$$

where,

$$ F = \begin{pmatrix} 1 & \Delta t \\ 0 & 1 \end{pmatrix} $$ 

and $x'$ is where we predict the object to be after time $\Delta{t}$.

$F$ is a matrix that, when multiplied with $x$, predicts where the object will be after time $\Delta t$.

By using the linear motion model with a constant velocity, the new location, $p'$ is calculated as

$$p' = p + v * \Delta t$$ 

where $p$ is the old location and $v$, the velocity, will be the same as the new velocity ($v' = v$) because the velocity is constant.

We can express this in a matrix form as follows:

$$\begin{pmatrix} p' \\ v' \end{pmatrix} = \begin{pmatrix}1 & \Delta t \\ 0 & 1 \end{pmatrix} \begin{pmatrix} p \\ v \end{pmatrix}$$

Remember we are representing the object location and velocity as gaussian distributions with mean $x$. When working with the equation $x' = F*x + noise$, we are calculating the mean value of the state vector. The noise is also represented by a gaussian distribution but with mean zero; hence, noise = 0 is saying that the mean noise is zero. The equation then becomes $x' = F*x$

But the noise does have uncertainty. The uncertainty shows up in the $Q$ matrix as acceleration noise.

## Update Step
For the update step, we use the measurement function to map the state vector into the measurement space of the sensor. To give a concrete example, lidar only measures an object's position. But the extended Kalman filter models an object's position and velocity. So multiplying by the measurement function $H$ matrix will drop the velocity information from the state vector $x$. Then the lidar measurement position and our belief about the object's position can be compared.

$$z = H*x + w$$

where $w$ represents sensor measurement noise.

So for lidar, the measurement function looks like this:

$$z = p'$$
It also can be represented in a matrix form:

$$z=\begin{pmatrix} 1 & 0 \end{pmatrix}\begin{pmatrix} p' \\ v' \end{pmatrix}$$ 

As we already know, the general algorithm is composed of a prediction step where I predict the new state $x$ and covariance $P$.

And we also have a measurement update (or also called many times a correction step) where we use the latest measurements to update our estimate and our uncertainty.

## Summary
Equation | Description
-- | -- 
$p$| Postion
$v$| Veclocity
$P$|Uncertainity Covariance
$\nu \sim N(0, Q)$|Process Noise
$w \sim N(0, R)$|Measurement Noise
$Q$|Process Covariance Matrix
$R$|Measurement Covariance Matrix
$x = \begin{pmatrix} p \\ v \end{pmatrix}$ | State Vector
$F = \begin{pmatrix} 1 & \Delta t \\ 0 & 1 \end{pmatrix}$|State Trasnition matrix
$H=\begin{pmatrix} 1 & 0 \end{pmatrix}$|Measurement Functon
$x' = Fx + \nu$| State transition Function
$z = Hx' + w$| Measurement Function
$K=P'H^TS^-1$|Kalman Gain

##  Kalman Filter Algorithm
1. Prediction
$$x' = Fx + \nu$$
$$P'=FPF^T+Q$$
2. Measurement update
$$y=z-Hx'$$
$$S=HP'H^T+R$$
$$K=P'H^TS^-1$$
3. New State
$$x=x+Ky$$
$$P=(1-KH)P'$$

## Implement a multi-dimensional Kalman Filter
```CPP
/** 
 * Write a function 'filter()' that implements a multi-
 *   dimensional Kalman Filter for the example given
 */

#include <iostream>
#include <vector>
#include "Dense"

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// Kalman Filter variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);


int main() {
  /**
   * Code used as example to work with Eigen matrices
   */
  // design the KF with 1D motion
  x = VectorXd(2);
  x << 0, 0;

  P = MatrixXd(2, 2);
  P << 1000, 0, 0, 1000;

  u = VectorXd(2);
  u << 0, 0;

  F = MatrixXd(2, 2);
  F << 1, 1, 0, 1;

  H = MatrixXd(1, 2);
  H << 1, 0;

  R = MatrixXd(1, 1);
  R << 1;

  I = MatrixXd::Identity(2, 2);

  Q = MatrixXd(2, 2);
  Q << 0, 0, 0, 0;

  // create a list of measurements
  VectorXd single_meas(1);
  single_meas << 1;
  measurements.push_back(single_meas);
  single_meas << 2;
  measurements.push_back(single_meas);
  single_meas << 3;
  measurements.push_back(single_meas);

  // call Kalman filter algorithm
  filter(x, P);

  return 0;
}


void filter(VectorXd &x, MatrixXd &P) 
{
  for (unsigned int n = 0; n < measurements.size(); ++n) 
  {
    VectorXd z = measurements[n];
    // TODO: YOUR CODE HERE
    /**
     * KF Measurement update step
     */
    VectorXd y = z - H * x;
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P * Ht * Si;

    // new state
    x = x + (K * y);
    P = (I - K * H) * P;

    /**
     * KF Prediction step
     */
    x = F * x + u;
    MatrixXd Ft = F.transpose();
    P = F * P * Ft + Q;

    cout << "x=" << endl <<  x << endl;
    cout << "P=" << endl <<  P << endl;
  }
}
```
>Note that in the quiz above, in the filter() function, we actually do the measurement and then the prediction in the loop. Over time, the order of these doesn't have a huge impact, since it is just a cycle from one to the other. Here, the first thing you need is a measurement because otherwise there is no location information or even information that the object exists unless a sensor picked it up. So, you initialize location values with the measurement.


# 09: State Prediction
The State Transition Matrix
$$
\begin{pmatrix} p_x' \\\ p_y' \\\ v_x' \\\ v_y' \\\ \end{pmatrix} = \begin{pmatrix} 1 & 0 & \Delta t & 0 \\\ 0 & 1 & 0 & \Delta t \\\ 0 & 0 & 1 & 0 \\\ 0 & 0 & 0 & 1 \end{pmatrix} \begin{pmatrix} p_x \\\ p_y \\\ v_x \\\ v_y \end{pmatrix} + \begin{pmatrix} \nu_{px} \\\\ \nu_{py} \\\\ \nu_{vx} \\\\ \nu_{vy} \end{pmatrix}
$$ 
 
As a reminder, the above equation is $x' = Fx + noise$

Motion noise and process noise refer to the same case: uncertainty in the object's position when predicting location. The model assumes velocity is constant between time intervals, but in reality we know that an object's velocity can change due to acceleration. The model includes this uncertainty via the process noise.

Measurement noise refers to uncertainty in sensor measurements, which will be discussed in more detail later.

## Quiz
### Quiz 1

Suppose you have a pedestrian state X. I want you to compare two scenarios: in the first predict the state 0.1s into the future and in the second 5s into the future. Which of these two scenarios leads to a higher uncertainty? In answering this, consider whether or not random noise has an increasing effect with increasing gaps between prediction times.

- [ ] A time difference of 0.1s leads to a higher uncertainty.
- [x] A time difference of 5s leads to a higher uncertainty.
- [ ] Both time differences have the same uncertainity.
> Right! Here's another way of thinking about it: if you split the 5s time difference into several intermediate predictions with a difference of 0.1s, then compared to the first case, you will predict the same state many more times without receiving any feedback about the object's new position. Thus, the uncertainty increases.



### Quiz 2
Let's say we use our linear motion model with fixed time increments, but the pedestrian is randomly changing her velocity (accelerating), sometimes speeding up, slowing down or changing direction. However, the overall mean change is zero. This introduces a noise in the tracking process - what kind of noise is it?

- [ ] Measurement noise
- [x] Process noise
- [ ] Neither

> Correct! The prediction equation treats the pedestrian's velocity as constant. As such, a randomly accelerating pedestrian creates a process noise.




From the examples I’ve just showed you we can clearly see that the process noise depends on both: the elapsed time and the uncertainty of acceleration. So, how can we model the process noise by considering both of these factors? Keep going to find out :)



# 10: Process Covaraince Noise
## Calculating Acceleration Noise Parameters
Before we discuss the derivation of the process covariance matrix $Q$, you might be curious about how to choose values for $\sigma_{ax_{}}^2$ and $\sigma_{ay}^2$.

For the extended Kalman filter project, you will be given values for both.

## Process Covariance Matrix $Q$ - Intuition
As a reminder, here are the state covariance matrix update equation and the equation for Q.

$$P' = FPF^T + Q$$

$$ 
Q = \begin{pmatrix} \frac{\Delta t^4}{{4}}\sigma_{ax}^2 & 0 & \frac{\Delta t^3}{{2}}\sigma_{ax}^2 & 0 \\ 0 & \frac{\Delta t^4}{{4}}\sigma_{ay}^2 & 0 & \frac{\Delta t^3}{{2}}\sigma_{ay}^2 \\ \frac{\Delta t^3}{{2}}\sigma_{ax}^2 & 0 & \Delta t^2\sigma_{ax}^2 & 0 \\ 0 & \frac{\Delta t^3}{{2}}\sigma_{ay}^2 & 0 & \Delta t^2\sigma_{ay}^2 \end{pmatrix}$$ 

Because our state vector only tracks position and velocity, we are modeling acceleration as a random noise. The $Q$ matrix includes time $\Delta t$ to account for the fact that as more time passes, we become more uncertain about our position and velocity. So as $\Delta t$ increases, we add more uncertainty to the state covariance matrix $P$.

Combining both 2D position and 2D velocity equations previously deducted formulas we have:
$$
\begin{cases} p_x' = p_x + v_x \Delta t + \frac{a_x \Delta t^2}{{2}}\\ p_y' = p_y + v_y \Delta t + \frac{a_y \Delta t^2}{{2}}\\ v_x' = v_x + a_x \Delta t\\ v_y' = v_y + a_y \Delta t \end{cases} $$ 

Since the acceleration is unknown we can add it to the noise component, and this random noise would be expressed analytically as the last terms in the equation derived above. So, we have a random acceleration vector \nuν in this form:

$$\nu = \begin{pmatrix} \nu_{px} \\ \nu_{py} \\ \nu_{vx} \\ \nu_{vy} \end{pmatrix} = \begin{pmatrix} \frac{a_x \Delta t^2}{{2}} \\ \frac{a_y \Delta t^2}{{2}} \\ a_x \Delta t \\ a_y \Delta t \end{pmatrix}$$

which is described by a zero mean and a covariance matrix $Q$, so $\nu \sim N(0,Q)$.

The vector $\nu$ can be decomposed into two components a 4 by 2 matrix $G$ which does not contain random variables and a 2 by 1 matrix $a$ which contains the random acceleration components:

$$
\nu = \begin{pmatrix} \frac{a_x \Delta t^2}{{2}} \\ \frac{a_y \Delta t^2}{{2}} \\ a_x \Delta t \\ a_y \Delta t \end{pmatrix} = \begin{pmatrix} \frac{\Delta t^2}{{2}} & 0 \\ 0 & \frac{\Delta t^2}{{2}} \\ \Delta t & 0 \\ 0 & \Delta t \end{pmatrix} \begin{pmatrix} a_x\\ a_y \end{pmatrix} = Ga$$

$\Delta t$ is computed at each Kalman Filter step and the acceleration is a random vector with zero mean and standard deviations $\sigma_{ax_{}}$ and $\sigma_{ay}$.

Based on our noise vector we can define now the new covariance matrix $Q$. The covariance matrix is defined as the expectation value of the noise vector $\nu$ times the noise vector $\nu^T$. So let’s write this down:
$$
Q = E[\nu \nu^T] = E[Gaa^TG^T]
$$

As $G$ does not contain random variables, we can put it outside the expectation calculation.

$$Q = G E[aa^T] G^T = G \begin{pmatrix} \sigma_{ax}^2 & \sigma_{axy} \\ \sigma_{axy} & \sigma_{ay}^2 \end{pmatrix} G^T = G Q_{\nu} G^T
$$

This leaves us with three statistical moments:

* the expectation of ax times ax, which is the variance of ax squared: $\sigma_{ax}^2$.
* the expectation of ay times ay, which is the variance of ay squared: $\sigma_{ay}^2$.
* and the expectation of ax times ay, which is the covariance of axax and ayay: $\sigma_{axy}$.


$a_{x_{}}$ and a_{y_{}} are assumed uncorrelated noise processes. This means that the covariance $\sigma_{axy_{}}$ in $Q_{\nu}$ is zero:
$$
Q_{\nu} = \begin{pmatrix} \sigma_{ax}^2 & \sigma_{axy} \\ \sigma_{axy} & \sigma_{ay}^2 \end{pmatrix} = \begin{pmatrix} \sigma_{ax}^2 & 0 \\ 0 & \sigma_{ay}^2 \end{pmatrix}
$$

So after combining everything in one matrix we obtain our 4 by 4 $Q$ matrix:

$$
Q = G Q_{\nu} G^T = \begin{pmatrix} \frac{\Delta t^4}{{4}}\sigma_{ax}^2 & 0 & \frac{\Delta t^3}{{2}}\sigma_{ax}^2 & 0 \\ 0 & \frac{\Delta t^4}{{4}}\sigma_{ay}^2 & 0 & \frac{\Delta t^3}{{2}}\sigma_{ay}^2 \\ \frac{\Delta t^3}{{2}}\sigma_{ax}^2 & 0 & \Delta t^2\sigma_{ax}^2 & 0 \\ 0 & \frac{\Delta t^3}{{2}}\sigma_{ay}^2 & 0 & \Delta t^2\sigma_{ay}^2 \end{pmatrix}
$$ 

## Note on Notation
Some authors describe $Q$ as the complete process noise covariance matrix. And some authors describe $Q$ as the covariance matrix of the individual noise processes. In our case, the covariance matrix of the individual noise processes matrix is called $Q_\nu$, which is something to be aware of.


# 11: Laser Measurements Part 1
## Variable Definitions
To reinforce was what discussed in the video, here is an explanation of what each variable represents:

* $z$ is the measurement vector. For a lidar sensor, the $z$ vector contains the position-x and position-ymeasurements.
* $H$ is the matrix that projects your belief about the object's current state into the measurement space of the sensor. For lidar, this is a fancy way of saying that we discard velocity information from the state variable since the lidar sensor only measures position: The state vector $x$ contains information about $[p_x, p_y, v_x, v_y]$ whereas the $z$ vector will only contain $[px, py]$. Multiplying $Hx$ allows us to compare $x$, our belief, with $z$, the sensor measurement.
* What does the prime notation in the $x$ vector represent? The prime notation like $p_x'$ means you have already done the prediction step but have not done the measurement step yet. In other words, the object was at $p_x$. After time $\Delta{t}$, you calculate where you believe the object will be based on the motion model and get $p_x'$.

## $H$ Matrix Quiz
Find the right $H$ matrix to project from a 4D state to a 2D observation space, as follows:

$$
\begin{pmatrix} p_x \\ p_y \end{pmatrix} = H \begin{pmatrix} p_x' \\ p_y' \\ v_x' \\ v_y' \end{pmatrix}$$

Here are your options:

A. $H = \begin{pmatrix} 1 & 0 \\ 0 & 1 \end{pmatrix}$  
B. $H = \begin{pmatrix} 1 & 0 \\ 0 & 1 \\ 0 & 0 \\ 0 & 0 \end{pmatrix}$  
C. $H = \begin{pmatrix} 1 & 1 \end{pmatrix}$  
D. $H = \begin{pmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{pmatrix}$  
(Hint: first consider the matrix dimensions, then try to use a 0 or 1 to correctly project the components into the measurement space.)

QUIZ QUESTION
Select the correct H matrix given the projection above.  

- [ ] A
- [ ] B
- [ ] C
- [X] D

# 12: Laser Measurements Part 2
What is the dimensionality of the noise covariance matrix, $R$?    

- [ ] 1x1
- [x] 2x2
- [ ] 4x4
- [ ] 2x4

# 13: Laser Measurements Part 3
## Measurement Noise Covariance Matrix R continued
For laser sensors, we have a 2D measurement vector. Each location component px, py are affected by a random noise. So our noise vector $\omega$ has the same dimension as $z$. And it is a distribution with zero mean and a 2 x 2 covariance matrix which comes from the product of the vertical vector $\omega$ and its transpose.
$$
R = E[\omega \omega^T] = \begin{pmatrix} \sigma^2_{px} & 0 \\ 0 & \sigma^2_{py} \end{pmatrix}
$$ 

where $R$ is the measurement noise covariance matrix; in other words, the matrix $R$ represents the uncertainty in the position measurements we receive from the laser sensor.

Generally, the parameters for the random noise measurement matrix will be provided by the sensor manufacturer. For the extended Kalman filter project, we have provided $R$ matrices values for both the radar sensor and the lidar sensor.

Remember that the off-diagonal $0$s in $R$ indicate that the noise processes are uncorrelated.

You have all you need for laser-only tracking! Now, I want you to apply what you've learned in a programming assignment.

## Helpful Equations
You will be modifying these matrices in the Kalman Filter with the observed time step, dt.

$$ 
F = \begin{pmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix}
$$  

In the tracking class $\sigma_{ax}^2 = noise_{ax}$ and $\sigma_{ay}^2 = noise_{ay}$

$$
Q = \begin{pmatrix} \frac{\Delta t^4}{4} \sigma_{ax}^2 & 0 & \frac{\Delta t^3}{2} \sigma_{ax}^2 & 0\\ 0 & \frac{\Delta t^4}{4} \sigma_{ay}^2 & 0 & \frac{\Delta t^3}{2} \sigma_{ay}^2\\ \frac{\Delta t^3}{2}\sigma_{ax}^2& 0 & \Delta t^2 \sigma_{ax}^2 & 0\\ 0 & \frac{\Delta t^3}{2} \sigma_{ay}^2 & 0 & \Delta t^2 \sigma_{ay}^2 \end{pmatrix}
$$ 

# 15: Radar Measurements
## H versus h(x)
The $H$ matrix from the lidar lesson and $h(x)$ equations from the radar lesson are actually accomplishing the same thing; they are both needed to solve $y = z - Hx'$ in the update step.

But for radar, there is no $H$ matrix that will map the state vector $x$ into polar coordinates; instead, you need to calculate the mapping manually to convert from cartesian coordinates to polar coordinates.

Here is the $h$ function that specifies how the predicted position and speed get mapped to the polar coordinates of range, bearing and range rate.
$$
h(x')= \begin{pmatrix} \rho\\ \phi\\ \dot{\rho} \end{pmatrix} = \begin{pmatrix} \sqrt{ p{'}_x^2 + p{'}_y^2 }\\ \arctan(p_y' / p_x')\\ \frac{p_x' v_x' + p_y' v_y'}{\sqrt{p{'}_x^2 + p{'}_{y}^2}} \end{pmatrix}
$$
 

Hence for radar $y = z - Hx'$ becomes $y = z - h(x')$.

## Definition of Radar Variables
* The range, ($\rho$), is the distance to the pedestrian. The range is basically the magnitude of the position vector $\rho$ which can be defined as $\rho = sqrt(p_x^2 + p_y^2)$.
* $\varphi = atan(p_y / p_x)$. Note that $\varphi$ is referenced counter-clockwise from the x-axis, so $\varphi$ from the video clip above in that situation would actually be negative.
* The range rate, $\dot{\rho}$ is the projection of the velocity, $v$, onto the line, $L$.

# 16: Mapping with a Nonlinear Function
![](https://video.udacity-data.com/topher/2017/February/58b4cc34_screenshot-from-2017-02-27-19-02-34/screenshot-from-2017-02-27-19-02-34.png)

What happens if we have a nonlinear measurement function, $h(x)$. Can we apply the Kalman Filter equations to update the predicted state, $X$, with new measurements, $z$?

- [ ] Yes! We have defined both the motion model and the measurement model so we should be good to go.
- [x] No! We aren't working with Gaussian distributions after applying a nonlinear measurement function.

# 17: Extended Kalman Filter
![](https://video.udacity-data.com/topher/2017/February/58b4cda2_screenshot-from-2017-02-27-19-08-41/screenshot-from-2017-02-27-19-08-41.png)


## How to Perform a Taylor Expansion
The general form of a Taylor series expansion of an equation, $f(x)$, at point $\mu$ is as follows:
$$
f(x) \approx f(\mu) + \frac{\partial f(\mu)}{\partial x} ( x - \mu)
$$

Simply replace $f(x)$ with a given equation, find the partial derivative, and plug in the value $\mu$ to find the Taylor expansion at that value of $\mu$.

See if you can find the Taylor expansion of $arctan(x)$.

Let’s say we have a predicted state density described by

$\mu = 0$ and $\sigma = 3$.

The function that projects the predicted state, $x$, to the measurement space $z$ is

$h(x) = arctan(x)$

and its partial derivative is

$\partial h = 1/(1+ x^2)$

I want you to use the first order Taylor expansion to construct a linear approximation of $h(x)$ to find the equation of the line that linearizes the function $h(x)$ at the mean location $\mu$.

![](https://video.udacity-data.com/topher/2017/February/58b4cef1_screenshot-from-2017-02-27-19-14-14/screenshot-from-2017-02-27-19-14-14.png)

The orange line represents the first order Taylor expansion of arctan(x). What is it?

A) $h(x) \approx x$  
B) $h(x) \approx 1/(1+x^2)$  
C) $h(x) \approx x + arctan(x)$  
D) $h(x) \approx 3 + x$

Which of the above equations (↑) represents the first order Taylor expansion of arctan(x) around mu = 0?

- [x] A
- [ ] B
- [ ] C
- [ ] D

$$ h(x)≈h(μ)+ \frac{∂h(μ)}{∂x}(x−μ)=arctan(μ)+ \frac{1}{1+μ^2}(x−μ)
$$
In our example $\mu=0$, therefore:
$$
h(x) \approx \arctan(0) + \frac{1}{1+0}(x-0) = x
$$

So, the function, $h(x) = \arctan(x)$, will be approximated by a line: $h(x) \approx x$.

# 18: Multivariate Taylor Series Expansion
Now that you’ve seen how to do a Taylor series expansion with a one-dimensional equation, we’ll need to look at the Taylor series expansion for multi-dimensional equations. Recall from the Radar Measurements lecture that the h function is composed of three equations that show how the predicted state, $x' = (p_x', p_y', v_x', v_y')^T$, is mapped into the measurement space, $z = (\rho, \varphi, \dot{\rho})^T$:
$$
h(x')= \begin{pmatrix} \sqrt{ p{'}_x^2 + p{'}_y^2 }\\ \arctan(p_y' / p_x')\\ \frac{p_x' v_x' + p_y' v_y'}{\sqrt{p{'}_x^2 + p{'}_{y}^2}} \end{pmatrix}
$$

These are multi-dimensional equations, so we will need to use a multi-dimensional Taylor series expansion to make a linear approximation of the $h$ function. Here is a general formula for the multi-dimensional Taylor series expansion:
$$
T(x) = f(a) + (x - a)^TDf(a) + \frac 1{2!}(x-a)^T{D^2f(a)}(x - a) + ...
$$

where $Df(a)$ is called the Jacobian matrix and $D^2f(a)$ is called the **Hessian matrix**. They represent first order and second order derivatives of multi-dimensional equations. A full Taylor series expansion would include higher order terms as well for the third order derivatives, fourth order derivatives, and so on.

Notice the similarities between the multi-dimensional Taylor series expansion and the one-dimensional Taylor series expansion:
$$
T(x) = f(a) + \frac{f'(a)}{1!}(x - a) + \frac{f''(a)}{2!}(x-a)^2 + \frac{f'''(a)}{3!}(x - a)^3 + ...
$$

To derive a linear approximation for the h function, we will only keep the expansion up to the Jacobian matrix $Df(a)$. We will ignore the Hessian matrix $D^2f(a)$ and other higher order terms. Assuming $(x - a)$ is small, $(x - a)^2$ or the multi-dimensional equivalent $(x-a)^T (x - a)$ will be even smaller; the extended Kalman filter we'll be using assumes that higher order terms beyond the Jacobian are negligible.

Let's first calculate the Jacobian matrix for the extended Kalman filter. Then we'll show the difference between the Kalman filter equations and the extended Kalman filter equations.

# 19: Jacobian Matrix
Jacobian, $H_j$H is:
$$
\Large H_j = \begin{bmatrix} \frac{p_x}{\sqrt[]{p_x^2 + p_y^2}} & \frac{p_y}{\sqrt[]{p_x^2 + p_y^2}} & 0 & 0\\ -\frac{p_y}{p_x^2 + p_y^2} & \frac{p_x}{p_x^2 + p_y^2} & 0 & 0\\ \frac{p_y(v_x p_y - v_y p_x)}{(p_x^2 + p_y^2)^{3/2}} & \frac{p_x(v_y p_x - v_x p_y)}{(p_x^2 + p_y^2)^{3/2}} & \frac{p_x}{\sqrt[]{p_x^2 + p_y^2}} & \frac{p_y}{\sqrt[]{p_x^2 + p_y^2}}\\ \end{bmatrix}
$$

# 20: CalculateJacobian()
FIll in the missing code in the CalculateJacobian() function to return the correct Jacobian matrix.
~~~cpp
#include <iostream>
#include <vector>
#include "Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
  /**
   * Compute the Jacobian Matrix
   */

  // predicted state example
  // px = 1, py = 2, vx = 0.2, vy = 0.4
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd Hj = CalculateJacobian(x_predicted);

  cout << "Hj:" << endl << Hj << endl;

  return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
~~~

# 21: EKF Algorithm Generalization
## Extended Kalman Filter Equations
Although the mathematical proof is somewhat complex, it turns out that the Kalman filter equations and extended Kalman filter equations are very similar. The main differences are:

* the $F$ matrix will be replaced by $F_j$ when calculating $P'$.
* the $H$ matrix in the Kalman filter will be replaced by the Jacobian matrix $H_j$ when calculating $S$, $K$, and $P$.
* to calculate $x'$, the prediction update function, $f$, is used instead of the $F$ matrix.
* to calculate $y$, the $h$ function is used instead of the $H$ matrix.


For this project, however, we do not need to use the $f$ function or $F_j$. If we had been using a non-linear model in the prediction step, we would need to replace the $F$ matrix with its Jacobian, $F_j$. However, we are using a linear model for the prediction step. So, for the prediction step, we can still use the regular Kalman filter equations and the $F$ matrix rather than the extended Kalman filter equations.

The measurement update for lidar will also use the regular Kalman filter equations, since lidar uses linear equations. Only the measurement update for the radar sensor will use the extended Kalman filter equations.

One important point to reiterate is that the equation $y = z - Hx'$ for the Kalman filter does not become $y = z - H_jx$ for the extended Kalman filter. Instead, for extended Kalman filters, we'll use the $h$ function directly to map predicted locations $x'$ from Cartesian to polar coordinates.

![](https://video.udacity-data.com/topher/2017/February/58b4d569_algorithm-generalization-900/algorithm-generalization-900.jpg)

## Clarification of $u = 0$
In the above image, the prediction equation is written as $x' = Fx + u$ and $x' = f(x,u)$. Previously the equation was written $x' = Fx + \nu$.

It is just a question of notation where $\nu$ is the greek letter "nu" and "u" is used in the code examples. Remember that $\nu$ is represented by a gaussian distribution with mean zero. The equation $x' = Fx + u$ or the equivalent equation $x' = Fx + \nu$ calculates the mean value of the state variable $x$; hence we set $u = 0$. The uncertainty in the gaussian distribution shows up in the $Q$ matrix.

## More Details About Calculations with Radar Versus Lidar
In the radar update step, the Jacobian matrix $H_j$ is used to calculate $S$, $K$ and $P$. To calculate $y$, we use the equations that map the predicted location $x'$ from Cartesian coordinates to polar coordinates:
$$
h(x')= \begin{pmatrix} \sqrt{ p{'}_{x}^2 + p{'}_{y}^2 }\\ \arctan(p_y' / p_x')\\ \frac{p_x' v_x' + p_y' v_y'}{\sqrt{p{'}_{x}^2 + p{'}_{y}^2}} \end{pmatrix}
$$ 

The predicted measurement vector $x'$ is a vector containing values in the form $\begin{bmatrix} p_x, p_y, v_x, v_y \end{bmatrix}$. The radar sensor will output values in polar coordinates:
$$
\begin{pmatrix} \rho\\ \phi\\ \dot{\rho} \end{pmatrix} 
$$

In order to calculate $y$ for the radar sensor, we need to convert $x'$ to polar coordinates. In other words, the function $h(x)$ maps values from Cartesian coordinates to polar coordinates. So the equation for radar becomes $y = z_{radar} - h(x')$.

One other important point when calculating $y$ with radar sensor data: the second value in the polar coordinate vector is the angle $\phi$. You'll need to make sure to normalize $\phi$ in the $y$ vector so that its angle is between $-\pi$ and $\pi$; in other words, add or subtract $2\pi$ from $\phi$ until it is between $-\pi$ and $\pi$.

To summarize:

* for measurement updates with lidar, we can use the $H$ matrix for calculating $y$, $S$, $K$ and $P$.
* for radar, $H_j$ is used to calculate SS, KK and PP.
## QUIZ QUESTION
Compared to Kalman Filters, how would the Extended Kalman Filter result differ when the prediction function and measurement function are both linear?
- [x] The Extended Kalman Filter's result would be the same as the standard Kalman Filter's result.
- [ ] The Extended Kalman Filter's result would vary unpredictably compared to the kalman filter's result.
- [ ] The Extended Kalman Filter's result would be less accurate than the kalman filter's result.
- [ ] The Extended Kalman Filter's result would be more accurate than the kalman filter's result.

> If f and h are linear functions, then the Extended Kalman Filter generates exactly the same result as the standard Kalman Filter. Actually, if f and h are linear then the Extended Kalman Filter F_j turns into f and H_j turns into h. All that's left is the same ol' standard Kalman Filter!

> In our case we have a linear motion model, but a nonlinear measurement model when we use radar observations. So, we have to compute the Jacobian only for the measurement function.

# 23: Evaluating KF Performance
```cpp
#include <iostream>
#include <vector>
#include "Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

int main() 
{
  /**
   * Compute RMSE
   */
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // the input list of estimations
  VectorXd e(4);
  e << 1, 1, 0.2, 0.1;
  estimations.push_back(e);
  e << 2, 2, 0.3, 0.2;
  estimations.push_back(e);
  e << 3, 3, 0.4, 0.3;
  estimations.push_back(e);

  // the corresponding list of ground truth values
  VectorXd g(4);
  g << 1.1, 1.1, 0.3, 0.2;
  ground_truth.push_back(g);
  g << 2.1, 2.1, 0.4, 0.3;
  ground_truth.push_back(g);
  g << 3.1, 3.1, 0.5, 0.4;
  ground_truth.push_back(g);

  // call the CalculateRMSE and print out the result
  cout << CalculateRMSE(estimations, ground_truth) << endl;


  return 0;
}

VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) 
{

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) 
  {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) 
  {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}
```
















