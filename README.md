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

$$ \exists \lambda^* = [\lambda_1^*, \lambda_2^*, ..., \lambda_p^*] \quad \exists \mu^* = [\mu_1^*, \mu_2^*, ..., \mu_m^*] $$

$$ \triangledown_x^{2} f(x^*) + \sum_{i=1}^{p}\lambda_i^* \triangledown_x^{2} h_i(x^*) + \sum_{j=1}^{m}\mu_j^* \triangledown_x^{2} g_j(x^*) \geq 0$$


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




