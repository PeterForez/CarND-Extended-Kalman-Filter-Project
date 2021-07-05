# Main Functions
* Gaussina Distribution
$$
f(x)=\frac{1}{\sqrt{2\pi \sigma^2}} exp^{-\frac{1}{2}\frac{(x-\mu)^2}{\sigma^2}} 
$$
$$
X\sim\mathcal{N}(\mu, \sigma^2)
$$

* The Product of two Gaussian
$$
\sigma^2 = \frac{1}{\frac{1}{\sigma_1^2}+\frac{1}{\sigma_2^2}}
$$
$$
\mu= \frac{\mu_1 \sigma_2^2 + \mu_2 \sigma_1^2}{\sigma_1^2+\sigma_2^2}
$$

* The Sum of two Gaussian
$$
\sigma^2 = \sigma_1^2 + \sigma_2^2
$$
$$
\mu= \mu_1 + \mu_2
$$

* Kalman Filter Equations: Predicton
$$
\begin{aligned}
x' &= f(x) + \nu \\ 
&= Fx+\underbrace{Bu}_{=0}+\nu
\end{aligned}
$$
$$
P' = FPF^T
$$
* Kalman Filter Equations: Measurement Update
$$
y = z-Hx
$$
$$
S=HPH^T+R
$$
$$
K=PH^TS^{-1}
$$
$$
x'=x+Ky
$$
$$
P'=(I-KH)P
$$
Symbol|Meaning
--|--
x|estimate
P|Uncertainity Covariance
F|State Trasnition matrix
u|Montion Vector
z|Measurement
H|Measurement Functon
R|Measurement Noise
I|Identity Matrix



  



# Codes
## Gaussian Function in Python 
```Python
from math import *
def f(mu, sigma2, x):
    return 1/sqrt(2.*pi*sigma2) * exp(-.5*(x-mu)**2 / sigma2)
```
## Update the mean and variance of gaussian distribution
```Python
def update(mean1, var1, mean2, var2):
    new_mean = (mean1 * var2 + mean2 * var1)/(var1 + var2)
    new_var = 1/(1/var1 + 1/var2)
    return [new_mean, new_var]
```
```Python
def update(mean1, var1, mean2, var2):
    """
    # Write a program to update your mean and variance
    # when given the mean and variance of your belief
    # and the mean and variance of your measurement.
    # This program will update the parameters of your
    # belief function.
    """
    new_mean = (mean1 * var2 + mean2 * var1)/(var1 + var2)
    new_var = 1/(1/var1 + 1/var2)
    return [new_mean, new_var]
print update(10.,8.,13., 2.)
```
## Update Prediction code
```Python 
def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1/(1/var1 + 1/var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]

print predict(10., 4., 12., 4.)
```

## One Dimentional Kalman Filter
```Python
# Write a program that will iteratively update and
# predict based on the location measurements 
# and inferred motions shown below. 

def update(mean1, var1, mean2, var2):
    new_mean = float(var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1./(1./var1 + 1./var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0.
sig = 10000.

#Please print out ONLY the final values of the mean
#and the variance in a list [mu, sig]. 

# Insert code here
for n in range(len(measurements)):
    [mu, sig] = update(mu, sig, measurements[n], motion_sig)
    print("Update = " + str([mu, sig]))
    [mu, sig] = predict(mu, sig, motion[n], motion_sig)
    print("Predict = "+ str([mu, sig]))
```
## Kalman Matrices
```Python 
# Write a function 'kalman_filter' that implements a multi-
# dimensional Kalman Filter for the example given

from math import *


class matrix:
    
    # implements basic operations of a matrix class
    
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # multiply if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                try:
                   res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
                except:
                   raise ValueError, "Zero diagonal"
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)


########################################

# Implement the filter function below

def kalman_filter(x, P):
    for n in range(len(measurements)):
        z = matrix([[measurements[n]]])
        y = z-(H*x)
        S = H*P*H.transpose()+R
        K = P*H.transpose()*S.inverse()
        # measurement update
        x = x + (K*y)
        P = (I-(K*H))*P
        # prediction
        x = (F*x) + u
        P = F*P*F.transponse()
        
    return x,P

############################################
### use the code below to test your filter!
############################################

measurements = [1, 2, 3]

x = matrix([[0.], [0.]]) # initial state (location and velocity)
P = matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = matrix([[0.], [0.]]) # external motion
F = matrix([[1., 1.], [0, 1.]]) # next state function
H = matrix([[1., 0.]]) # measurement function
R = matrix([[1.]]) # measurement uncertainty
I = matrix([[1., 0.], [0., 1.]]) # identity matrix

print(kalman_filter(x, P))
# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]

```
# Multivaraiate Kalman Filter



# Notes
* The Kalman Filter represents our distributions by Gaussians and iterates on two main cycles.  
  * The first cycle is the Measurement Update.
    * Requires a product
    * Uses Bayes rule.
  * The second cycle is the Motion Update or Prediction.
    * Involves a convolution
    * Uses total probability.
* The new belief will be somewhere between the previous belief and the new measurement.
* Since the Gaussian's have the same width (which means same certainty), than their product will be a Gaussian with a mean that is right in the middle.
* This can be hard to wrap your head around, but multiple measurements ALWAYS gives us a more certain (and therefore taller and narrower) belief.



# Quiz
* ***Suppose you have a pedestrian state X. I want you to compare two scenarios: in the first predict the state 0.1s into the future and in the second 5s into the future. Which of these two scenarios leads to a higher uncertainty? In answering this, consider whether or not random noise has an increasing effect with increasing gaps between prediction times.*** 
- [ ] A time difference of 0.1s leads to a higher uncertainty.
- [x] A time difference of 5s leads to a higher uncertainty.
- [ ] Both time differences have the same uncertainty.

> Right! Here's another way of thinking about it: if you split the 5s time difference into several intermediate predictions with a difference of 0.1s, then compared to the first case, you will predict the same state many more times without receiving any feedback about the object's new position. Thus, the uncertainty increases.

* ***Let's say we use our linear motion model with fixed time increments, but the pedestrian is randomly changing her velocity (accelerating), sometimes speeding up, slowing down or changing direction. However, the overall mean change is zero. This introduces a noise in the tracking process - what kind of noise is it?***
- [ ] Measurement noise
- [x] Process noise
- [ ] Neither

> Correct! The prediction equation treats the pedestrian's velocity as constant. As such, a randomly accelerating pedestrian creates a process noise.

> From the examples I’ve just showed you we can clearly see that the process noise depends on both: the elapsed time and the uncertainty of acceleration. 

* Find the right $H$ matrix to project from a 4D state to a 2D observation space, as follows:
$$ 
\begin{pmatrix}
p_x\\p_y
\end{pmatrix} 
= H
\begin{pmatrix}
p'_x\\p'_y\\v'_x\\v'_y
\end{pmatrix}
$$
$$
A.
$$




# Kalman Filter Equations in C++ Part 1
* State $x$ is described by a position and velocity
$$
x=\begin{pmatrix}
p\\v 
\end{pmatrix}
$$

$$
x'=F*x+noise 
$$

$$
F=\begin{pmatrix}
1 & \Delta t\\ 
0 & 1
\end{pmatrix}
$$

$$
z=H*x+w
$$

Symbol | Meaning
--|--
$x$ |The State vector that is described by a position and vecloity
$x'$| The predicted state after time $\Delta t$
$w$ | The Sensor measurement noise



# References
1. https://video.udacity-data.com/topher/2018/June/5b327c11_sensor-fusion-ekf-reference/sensor-fusion-ekf-reference.pdf
2. 
