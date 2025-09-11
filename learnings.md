# Learnings
Unstructured learnings so I don't lose them for later in full writeup.

- Regularization is important to prevent wild jumps in estimates. This massively improved the stability of the estimator (before, it would jump around a lot and be very noisy).
- Optimizer confidence can be estimated based on the variance of the residuals. If the variance is low, we can be more confident in the estimates. This can be used to adjust the regularization strength dynamically.
- Intuition about the system in general (glider dynamics, thermal dynamics) proved to be super useful in generally understanding the math and what was going on.