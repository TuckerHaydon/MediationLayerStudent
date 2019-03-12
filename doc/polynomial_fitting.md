# Polynomial Fitting

## Description
A core component of the Mediation Layer is it's ability to create smooth
trajectories for the quadcopters to follow given a set of waypoints and times.
The Mediation Layer fits a set of piecewise polynomials over the waypoints,
optimizing over a specified derivative of the polynomials.

## Polynomial Fitting
The input is a set of waypoints and times at which the quadcopters should reach
the waypoints. The goal is to find a set of piecewise polynomials that connect
and smoothly cover the set of waypoints. The solution should be of the form:

p\_0(t) = p\_00\*(1/0! t^0) + p\_01\*(1/1! t^1) + p\_02\*(1/2! t^2) ... 
p\_1(t) = p\_10\*(1/0! t^0) + p\_11\*(1/1! t^1) + p\_12\*(1/2! t^2) ... 
p\_2(t) = p\_20\*(1/0! t^0) + p\_21\*(1/1! t^1) + p\_22\*(1/2! t^2) ... 
...

These polynomials can be expressed in vector notation:
p\_0(t) = dot(c\_0, T)
p\_0(t) = dot(c\_1, T)
p\_0(t) = dot(c\_2, T)
  
