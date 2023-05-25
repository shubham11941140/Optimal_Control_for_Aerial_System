# Optimal_Control_for_Aerial_System
This repo involves writing code to solve optimality control in aerial system in various real-world situations

## Part 1: Steepest Descent Method and Conjugate Gradient Method

### **Problem Statement:**

Implement Steepest Descent Method and Conjugate Gradient Method for finding the minimum of the function $f(x_{1}, x_{2}) = 2x_{1}^2 + 4x_{1}x_{2} + 4x_{2}^2 + 2x_{2} - 4x_{1} + 16$

**Steps for Steepest Descent Algorithm:**

1. Choose Starting Point $x$ = $ \begin{bmatrix} x_{1} \\ x_{2} \end{bmatrix} $ and evalute the function at $x = \begin{bmatrix} x_{1} \\ x_{2} \end{bmatrix} $. Also define the stopping criterias $\delta_1, \delta_2$ and $\delta_3$.

2. Find $f(x^{(k)})$ and $\nabla f(x^{(k)})$

3. Find $d_{k} = -\nabla f(x^{(k)})$

4. Find out $-\nabla^{2} f(x^{(k)})$

5. Find $\lambda_{k}$ = $\dfrac{- \nabla f(x^{(k)})^{T}d_{k}}{d_{k}^{T}\nabla^{2} f(x^{(k)})d_{k}}$

6. Find $x^{(k+1)} = x^{(k)} + \lambda_{k}d_{k}$

7. Evaluate $\triangle f = |f(x^{(k+1)}) - f(x^{(k)})|$

8. If $\triangle f < \delta_1$ then stop, If $\triangle x^{T} \triangle x < \delta_2$ then stop, If $\nabla f(x^{(k+1)})^{T} \nabla f(x^{(k+1)}) < \delta_3$ then stop, else go to step 2.

**In Congugate Gradient Method:**

We just update the direction vector $d_{k}$ in each iteration as:

$d_{k} = -\nabla f(x^{(k)}) + \beta_{k}d_{k-1}$

where $\beta_{k}$ is calculated as:

$\beta_{k} = \dfrac{\nabla f(x^{(k)})^{T}\nabla f(x^{(k)})}{\nabla f(x^{(k-1)})^{T}\nabla f(x^{(k-1)})}$

**Some Necessary Calculations**

$f(x) = f(x_1, x_2) = 2x_1^2 + 4x_1x_2 + 4x_2^2 + 2x_2 - 4x_1 + 16$

$\nabla f(x) = \nabla f(x_1, x_2) = \begin{bmatrix} 4x_1 + 4x_2 - 4 \\ 4x_1 + 8x_2 + 2 \end{bmatrix}$

$\nabla f(x)^T = \nabla f(x_1, x_2)^T = \begin{bmatrix} 4x_1 + 4x_2 - 4 & 4x_1 + 8x_2 + 2 \end{bmatrix}$

$\nabla f(x)^2 = \nabla^{2} f(x_1, x_2) = \begin{bmatrix} 4 & 4 \\ 4 & 8 \end{bmatrix}$

**Plotting the Contour Plot of the Function**



**Print the values of x and min(f(x)) using Steepest Descent Method**

```
The minimum point is:  [2.50001475369140 -1.49998524630860]
The minimum value is:  9.50000000217671
```

**Print the values of x and min(f(x)) using Conjugate Gradient Method**

```
The minimum point is:  [2.50001475369140 -1.49998524630860]
The minimum value is:  9.50000000217671
```
