# Optimal_Control_for_Aerial_System
This repo involves writing code to solve optimality control in aerial system in various real-world situations

## Part 1: Steepest Descent Method and Conjugate Gradient Method

### **Problem Statement:**

Implement Steepest Descent Method and Conjugate Gradient Method for finding the minimum of the function $f(x_{1}, x_{2}) = 2x_{1}^2 + 4x_{1}x_{2} + 4x_{2}^2 + 2x_{2} - 4x_{1} + 16$

**Steps for Steepest Descent Algorithm:**

1. Choose Starting Point $x$ and evalute the function at

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/3329a6d7-a8b2-4886-9957-f595ff375af0)

Also define the stopping criterias $\delta_1, \delta_2$ and $\delta_3$.

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

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/ad91c629-95a6-4d4d-a2d8-03dd9ff270a1)

**Plotting the Contour Plot of the Function**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/f9f741fa-2c04-40a8-b8bf-9258f8329117)


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

**Plotting the Contour Plot of the Function with the Minima Point**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/c2129048-9ff0-4303-ae14-7676949678c1)

## Part 2: Generalized Lagrangian Solver

### **Generalized $n$ variable Template Function for any given mathematical problem with given constraints**

#### **Generalized Optimizer** - **Lagrangian Solver** - All feasible solutions are given along with local minima/maxima

### **Input:**

#### **1.** **Minimize/Maximize Objective Function**

$$f(x_1,x_2,...,x_n)$$

#### **2.** **Equality Constraints** -

$$h_i(x_1,x_2,...,x_n) = 0 \quad i = 1,2,...,p$$

#### **3.** **Inequality Constraints**

$$g_j(x_1,x_2,...,x_n) \leq 0 \quad j = 1,2,...,m$$

### **Solver Variables:**

#### **1.** **Lagrange Function** - $L(f, x, \lambda, h, \mu, g, s)$

$$L(f, x, \lambda, h, \mu, g, s) = f(x) + \sum_{i=1}^{p}\lambda_i h_i(x) + \sum_{j=1}^{m}\mu_i (g_j(x) + s_{j}^{2})$$

#### **2.** **Lagrange Multipliers** - $\lambda_i, \mu_j, s_j$

$$ \mu_j \geq 0, s_j \geq 0 \quad j = 1,2,...,m$$

### **Necessary Conditions (Stationarity):**

#### **1.** **Lagrange Equation**

$$ \frac{\partial L}{\partial x_k} = 0 \quad k = 1,2,...,n$$

#### **2.** **KKT Conditions**

$$ \frac{\partial L}{\partial \lambda_i} = 0 \quad i = 1,2,...,p$$

$$ \frac{\partial L}{\partial \mu_j} = 0 \quad j = 1,2,...,m$$

$$ \frac{\partial L}{\partial s_j} = 0 \quad j = 1,2,...,m$$

### **Feasibility Conditions:**

$$s_j^2 \geq 0 \quad j = 1,2,...,m$$

$$g_j(x) \leq 0 \quad j = 1,2,...,m$$

### **Switching/Orthogonality Conditions:**

$$\mu_j \times g_j(x) = 0 \quad j = 1,2,...,m$$

### **Sufficient Conditions:**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/b51f3790-5eb4-40af-8973-71970f79ec22)


### **Example 1:**

#### **Minimize** - $f(x_1,x_2) = x_1^2 + x_2^2 - 4x_1 - 6x_2$ subject to $x_1 + x_2 - 2 \leq 0$ and $2x_1 + 3x_2 -12 \leq 0$ and $x_1 \geq 0$ and $x_2 \geq 0$

#### **Solution:**

```
[{x2: 3/2, x1: 1/2, u_0: 3, u_1: 0, 'type': 'min', 'F': -17/2}]
```

### **Example 2:**

#### **Minimize** - $f(x_1,x_2) = (x_1 - 2.5)^2 + (x_2 - 2.5)^2$ subject to $2x_1 + 2x_2 - 3 \leq 0$ and $x_1 \geq 0$ and $x_2 \geq 0$

#### **Solution:**

```
[{x2: 0.750000000000000,
  x1: 0.750000000000000,
  u_0: 1.75000000000000,
  'type': 'min',
  'F': 6.12500000000000}]
```

### **Example 3:**

#### **Minimize** - $f(x_1,x_2) = (x_1 - 2.5)^2 + (x_2 - 2.5)^2$ subject to $2x_1 + 2x_2 = 3$ and $x_1 \geq 0$ and $x_2 \geq 0$

#### **Solution:**

```
[{x2: 0.750000000000000,
  x1: 0.750000000000000,
  u_0: 1.75000000000000,
  'type': 'min',
  'F': 6.12500000000000}]
```

### **If you wish to test more examples, you can input you functions f, g, h and run it on this code.**

## **Solving Example 1 Step-by-Step to show the execution of the code**

### **Problem Statement:**

Write a code to minimise $f(x) = x_{1}^2 + x_{2}^2 - 4x_{1} - 6x_{2}$ and subject to the constraints $x_{1} + x_{2} \leq 2$ and $2x_{1} + 3x_{2} \leq 12$ and $x_{1} \geq 0$ and $x_{2} \geq 0$.

**Solution:**

We will use the **Lagrange Multiplier Method** to solve this problem.

**Step 1:** We will first define the function $f(x)$ and the constraints $g(x)$ and $h(x)$.

**Step 2:** We will then define the Lagrangian function $L(x, \lambda, \mu)$.

**Step 3:** We will then find the partial derivatives of the Lagrangian function with respect to $x_{1}$, $x_{2}$ and put them equal to $0$ to get some conditions.

**Step-4:** We will create switching conditions using the equation: $\mu_{j}g_{j}(x) = 0$.

**Step-5:** Solve all switch cases using Constraints, and another inequality which tells $\mu_{j} \geq 0$.

**Step-6:** We will then compare solutions of all switch cases and find the optimal solution.

### Solving Example 1 Step-by-Step

**Step-1:** Define the function $f(x)$ and the constraints $g(x)$ and $h(x)$.

**Step-2:** Define the Lagrangian function $L(x, \lambda, \mu)$.

$$L(x, \lambda, \mu) = f(x) + \sum_{i=1}^{p}\lambda_i h_i(x) + \sum_{j=1}^{m}\mu_i (g_j(x) + s_{j}^{2})$$

$$L(x, \lambda, \mu) = x_{1}^2 + x_{2}^2 - 4x_{1} - 6x_{2} + \mu_{1}(x_{1} + x_{2} - 2 + s_{1}^2) + \mu_{2}(2x_{1} + 3x_{2} - 12 + s_{2}^2)$$

(We do not need to consider $\lambda$ as we do not have equality constraints.)

(Also we do not need to code this function as we will be using the partial derivatives of this function.)

**Step-3:** Find the partial derivatives of the Lagrangian function with respect to $x_{1}$, $x_{2}$ and put them equal to $0$ to get some conditions.

$$\frac{\partial L}{\partial x_{1}} = 2x_{1} - 4 + \mu_{1} + 2\mu_{2} = 0$$

$$\frac{\partial L}{\partial x_{2}} = 2x_{2} - 6 + \mu_{1} + 3\mu_{2} = 0$$


**Step-4:** Create switching conditions using the equation: $\mu_{j}g_{j}(x) = 0$.

$\mu_{1}g_{1}(x) = 0$ and $\mu_{2}g_{2}(x) = 0$

**CASE 1:** $\mu_{1} = 0$ ($g_{1}(x) < 0$) and $\mu_{2} = 0$ ($g_{2}(x) < 0$)

**CASE 2:** $\mu_{1} = 0$ ($g_{1}(x) < 0$) and $g_{2}(x) = 0$ ($\mu_{2} > 0$)

**CASE 3:** $g_{1}(x) = 0$ ($\mu_{1} > 0$) and $\mu_{2} = 0$ ($g_{2}(x) < 0$)

**CASE 4:** $g_{1}(x) = 0$ ($\mu_{1} > 0$) and $g_{2}(x) = 0$ ($\mu_{2} > 0$)

**Step-5:** Solve all switch cases using Constraints, and another inequality which tells $\mu_{j} \geq 0$.

**CASE 1:**

$\mu_{1} = 0$ ($g_{1}(x) < 0$) and $\mu_{2} = 0$ ($g_{2}(x) < 0$)

$$2x_{1} + 3x_{2} - 12 = 0$$

$$x_{1} + x_{2} - 2 = 0$$

$$x_{1} \geq 0$$

$$x_{2} \geq 0$$

$$\mu_{1} \geq 0$$

$$\mu_{2} \geq 0$$

**Solution**

```
{x1: 2, x2: 3, m1: 0, m2: 0}
```

**Case 1 is not feasible**

**CASE 2:**

$\mu_{1} = 0$ ($g_{1}(x) < 0$) and $g_{2}(x) = 0$ ($\mu_{2} > 0$)

$$2x_{1} + 3x_{2} - 12 = 0$$

$$x_{1} + x_{2} - 2= 0$$

$$x_{1} \geq 0$$

$$x_{2} \geq 0$$

$$\mu_{1} \geq 0$$

$$\mu_{2} \geq 0$$

**Solution**

```
{x1: 24/13, x2: 36/13, m1: 0, m2: 2/13}
```
**Case 2 is not feasible**

**CASE 3:**

$g_{1}(x) = 0$ ($\mu_{1} > 0$) and $\mu_{2} = 0$ ($g_{2}(x) < 0$)

$$2x_{1} + 3x_{2} - 12 = 0$$

$$x_{1} + x_{2} - 2 = 0$$

$$x_{1} \geq 0$$

$$x_{2} \geq 0$$

$$\mu_{1} \geq 0$$

$$\mu_{2} \geq 0$$


**Solution**

```
{x1: 1/2, x2: 3/2, m1: 3, m2: 0}
```

**Since Case 3 is feasible, we will solve it.**

Finding the function value at $x_{1}, x_{2} = (\dfrac{1}{2}, \dfrac{3}{2})$

The function value is **$-8.5$**

Checking for local minima/maxima.

**To check this, we will use the following condition:**

$$\triangledown^{2}f(x) + \mu_1 \triangledown^{2}g_{1}(x) + \mu_2 \triangledown^{2}g_{2}(x) \geq 0$$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/68baa43d-06ad-4f77-bc48-89287d1bfa97)


#### **The solution is a local minimum.**

**CASE 4:**

$g_{1}(x) = 0$ ($\mu_{1} > 0$) and $g_{2}(x) = 0$ ($\mu_{2} > 0$)

$$2x_{1} + 3x_{2} - 12 = 0$$

$$x_{1} + x_{2} - 2 = 0$$

$$x_{1} \geq 0$$

$$x_{2} \geq 0$$

$$\mu_{1} \geq 0$$

$$\mu_{2} \geq 0$$


**Solution**

```
{x1: -6, x2: 8, m1: 68, m2: -26}
```

**Case 4 is not feasible**

**Step-6:** Find the true minimum value returned by this function by checking all the possible solutions.

Since, we have gotten only one feasible situation, and that **solution is a local minimum, we can say that this is the true minimum value of this function.**

Hence, the **true minimum value of this function is $-8.5$ at $x_{1}, x_{2} = (\dfrac{1}{2}, \dfrac{3}{2})$**

## Part 3: Finding Extremal Trajectory of Functional

### **Problem Statement:**

### To find the **Extremal Trajectory** for functional

$$J = \int_{0}^{t_f} \{2 \left( \dot{x}(t) \right)^2 + 24 t x(t) \} dt$$

### given the **initial condition** $x(0) = 0$ and the **final condition** $x(t_f) = 2$.

##### Let $x(t) = x$ and $\dot{x}(t) = \dot{x}$, so

$$V(.) = 2 \dot{x}^2 + 24 tx $$

### **Partially Differentiate** with respect to $x$ and $\dot{x}$

$$\frac{\delta V}{\delta x} = 24 t$$

$$\frac{\delta V}{\delta \dot{x}} = 4 \dot{x}$$

### **Solving we get 2 optimized trajectories**

$$x^*(t) = t^3 - 1.117t$$

$$x^*(t) = t^3 + 2.687t$$

### **We need to compute 2 quantities**

#### **Hessian Matrix** $H$ and **Second Variation** $\delta^2 J$

### **Hessian Matrix** $H$ is given by

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/b9061b27-d47b-4827-a23b-619c3353c943)


### **Second Variation**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/9d21b6dd-ed33-424c-acc0-29eb1b8d8530)


### **Compute Hessian Matrix**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/0d321390-654b-4a3b-8402-36ee86631822)


This means our solution is a minimum if the **Hessian Matrix is positive definite.**

### So, this is a **minima trajectory.**

### **Compute Second Variation**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/710c7037-1ad3-4770-b29c-7dac1849ea66)


at Optimal Trajectory $x^*(t)$,

$$x^*(t) = t^3 - 1.117t$$

$$x^*(t) = t^3 + 2.687t$$

### **Optimal Trajectory 1:**

$$x^*(t) = t^3 - 1.117t$$

$$t_0 = 0$$

$$t_f = 1.55$$

$$\delta^2 J = 89.373$$

#### **Plotting the Optimal Trajectory 1**

$$t = 0:0.01:1.55$$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/464ed2d2-2560-46b2-bb63-c6406822ca61)


### **Optimal Trajectory 2:**

$$x^*(t) = t^3 + 2.687t$$

$$t_0 = 0$$

$$t_f = 0.64$$

$$\delta^2 J = 6.291456$$

#### **Plotting the Optimal Trajectory 2**

$$t = 0:0.01:0.64$$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/f250d7e2-99b1-46b2-9864-986bdca47667)

## Part 4: Solve the Matrix Differential Riccati Equation (MDRE) for a Finite Linear Quadratic Regulator (LQR)

### **Problem Statement:**

**State-Space Definition of the given System:**

$$\dot{x_1}(t) = x_2(t)$$

$$\dot{x_2}(t) = -2x_1(t) + 2x_2(t) + 2u(t)$$

**Boundary Conditions:**

$$t \in [0,6]$$

$$x_1(0) = 1$$

$$x_2(0) = -2$$

**Performance Index:**

$$PI = \dfrac{1}{2} \left[ x_{1}^2(6) + 2x_1(6)x_2(6) + 2x_2^2(6) \right] $$

$$+ \int_0^6 \left[ 2x_{1}^2(t) + 3x_1(t)x_2(t) + 2x_2^2(t) + \dfrac{1}{2} u^2(t) \right]dt$$

**Find the optimal control input $u(t)$ for the given system to minimize the performance index.**

**Solution**:

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/cc9730e7-2f08-42b3-bb34-ebd910664637)

### **Eigen-Vales of $A$ are:**

$$\lambda_1 = 1 + j$$

$$\lambda_2 = 1 - j$$

**Since the Real-Part of both the Eigen-Values is positive, the system is unstable.**

### **Matrix Differential Riccati Equation:**

$$\dot{P} = -(A^TP + PA - PBR^{-1}B^TP + Q)$$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/6c85f65e-6082-42a5-bf03-b935e76eebd6)

**Simplifying**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/58620187-0f77-4e21-b421-313a0e62d1c6)

$$\dot{P_{11}}(t) = 4P_{12}^2(t) + 4P_{12}(t) - 4$$

$$\dot{P_{12}}(t) = -P_{11}(t) + 4P_{12}(t)P_{22}(t) - 2P_{12}(t) + 2P_{22}(t) - 3$$

$$\dot{P_{22}}(t) = -2P_{12}^2(t) + 4P_{22}^2(t) - 4P_{22}(t) - 4$$

$$P_{11}(6) = 1$$

$$P_{12}(6) = 1$$

$$P_{22}(6) = 2$$

Solving above equations using **Runge-Kutta Method** and **Euler's Method** with a small time-step.

### **Runge-Kutta Method**:

Solving the above equations using Runge-Kutta Method:

$$\dot{P_{11}}(t) = 4P_{12}^2(t) + 4P_{12}(t) - 4$$

$$\dot{P_{12}}(t) = -P_{11}(t) + 4P_{12}(t)P_{22}(t) - 2P_{12}(t) + 2P_{22}(t) - 3$$

$$\dot{P_{22}}(t) = -2P_{12}^2(t) + 4P_{22}^2(t) - 4P_{22}(t) - 4$$

$$P_{11}(6) = 1$$

$$P_{12}(6) = 1$$

$$P_{22}(6) = 2$$

Let the time-step be

$$\Delta t = 0.001$$

**Converting to vector form:**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/ebb1e990-9ea4-4d8b-91be-7b194b6ad336)



**Therefore,**


![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/7d413a2b-065a-4fd5-a423-8c33ded0f730)

Clearly, we have values for $\vec{X}$ from $t \in [0, 6]$ at a time-step of $\Delta t = 0.001$.

Now, let us plot $P_{11}(t)$, $P_{12}(t)$ and $P_{22}(t)$ vs $t$

($P_{11}(t)$, $P_{12}(t)$ and $P_{22}(t)$ are the values of $\vec{X}$ at each time-step)

**Plotting**:

$P_{11}(t)$ vs $t$:

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/be4ce487-d88f-494c-938a-57beb26cf06d)


$P_{12}(t)$ vs $t$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/41c5f3db-c813-45e0-aecb-70ec81743792)



$P_{22}(t)$ vs $t$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/b17ea8b6-0730-4e1d-9ab4-69955d38f93b)

### **Plotting them together:**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/975872b1-ed0d-4961-8455-8f01f914eecc)

### **Euler's Method**:

$$\dot{P_{11}}(t) = 4P_{12}^2(t) + 4P_{12}(t) - 4$$

$$\dot{P_{12}}(t) = -P_{11}(t) + 4P_{12}(t)P_{22}(t) - 2P_{12}(t) + 2P_{22}(t) - 3$$

$$\dot{P_{22}}(t) = -2P_{12}^2(t) + 4P_{22}^2(t) - 4P_{22}(t) - 4$$

$$P_{11}(6) = 1$$

$$P_{12}(6) = 1$$

$$P_{22}(6) = 2$$

Solving the above equations using Euler's Method:

Let the time-step be

$$\Delta t = 0.001$$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/01083969-87e7-4e19-a0cd-c33508bdb10e)

Clearly, we have values for $\vec{X}$ from $t \in [0, 6]$ at a time-step of $\Delta t = 0.001$.

Now, let us plot $P_{11}(t)$, $P_{12}(t)$ and $P_{22}(t)$ vs $t$

($P_{11}(t)$, $P_{12}(t)$ and $P_{22}(t)$ are the values of $\vec{X}$ at each time-step)

**Plotting**:

$P_{11}(t)$ vs $t$:

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/be4ce487-d88f-494c-938a-57beb26cf06d)


$P_{12}(t)$ vs $t$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/41c5f3db-c813-45e0-aecb-70ec81743792)



$P_{22}(t)$ vs $t$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/b17ea8b6-0730-4e1d-9ab4-69955d38f93b)

### **Plotting them together:**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/975872b1-ed0d-4961-8455-8f01f914eecc)


## Part 5: Solve the Matrix Differential Riccati Equation (MDRE) for an Infinite Linear Quadratic Regulator (LQR)

### **Problem Statement:**

**State-Space Definition of the given System:**

$$\dot{x_1}(t) = x_2(t)$$

$$\dot{x_2}(t) = -2x_1(t) + 2x_2(t) + 2u(t)$$

**Boundary Conditions:** 

$$t \in [0,6]$$

$$x_1(0) = 1$$

$$x_2(0) = -2$$

**Performance Index:**

$$PI = \dfrac{1}{2} \left[ x_{1}^2(6) + 2x_1(6)x_2(6) + 2x_2^2(6) \right] + \int_0^6 \left[ 2x_{1}^2(t) + 3x_1(t)x_2(t) + 2x_2^2(t) + \dfrac{1}{2} u^2(t) \right]dt$$

**Find the optimal control input $u(t)$ for the given system to minimize the performance index.**

**Solution**:

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/db3170a2-58bd-4f3c-b22e-9e550b38c1e0)

### **Eigen-Vales of $A$ are:**

$$\lambda_1 = 1 + j$$

$$\lambda_2 = 1 - j$$

**Since the Real-Part of both the Eigen-Values is positive, the system is unstable.**

### **Matrix Differential Riccati Equation:**

$$\dot{P} = -(A^TP + PA - PBR^{-1}B^TP + Q)$$

### **Non-Linear Algebraic Riccati Equation:**

$$ \dot{P} = 0$$

**P is a Constant Matrix**

$$A^TP + PA - PBR^{-1}B^TP + Q = 0$$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/0db93963-a9b0-48de-8c36-f85993613ce2)

We have **4 distinct solutions for P**, they are

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/a8cbdcd2-8cba-4d80-a6fc-7fb9b5733bf2)

**Calculating**

$$K = -R^{-1}B^TP$$

$$u(t) = -Kx(t)$$

**For $P_1$:**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/1e3e2f3a-286d-4a3d-8c7a-768c942b5e99)


**For $P_2$:**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/ed672a3e-9449-42c0-a82d-79998eeef28c)


**For $P_3$:**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/39b7ab69-a352-4dc7-ab9b-c4e8413f84d2)


**For $P_4$:**

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/481a26ad-e730-49f4-b197-b2f79dc06a92)

## **Optimal Cost**

$$C = \dfrac{1}{2} x^T(t_0)Px(t_0)$$

$$t_0 = 0$$

![image](https://github.com/shubham11941140/Optimal_Control_for_Aerial_System/assets/63910248/b96d85f8-cd3c-45ec-a72b-abd3df8598e8)

**For $P_1$:**

$$C_1 = -\sqrt{4 - \sqrt{5}} + 1 + \dfrac{\sqrt{20 - 5\sqrt{5}}}{2} + \sqrt{5} = 3.39283258009334$$

**For $P_2$:**

$$C_2 = -\sqrt{5} + 1 + \sqrt{\sqrt{5} + 4} + \dfrac{\sqrt{20 + 5\sqrt{5}}}{2} = 4.05311200236228$$

**For $P_3$:**

$$C_3 = -\dfrac{\sqrt{20 - 5\sqrt{5}}}{2} + 1 + \sqrt{4 - \sqrt{5}} + \sqrt{5} = 3.07930337490624 $$

**For $P_4$:**

$$C_4 = -\dfrac{\sqrt{20 + 5\sqrt{5}}}{2} - \sqrt{4 + \sqrt{5}} - \sqrt{5} + 1 = −6.52524795736186$$

#### **Since $C_4 = -6.52524795736186$ is the minimum cost, we can say that $P_4$ is the optimal solution.**

