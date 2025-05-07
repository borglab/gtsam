# Naive Way

The constructor $f$ is a binary function that constructs an element of SE(3):

$$
f(R,t) = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}
$$

We want to derive the relationship between a perturbation $ \delta \in \mathbb{R}^3 $ in the translation component $ t $ and the corresponding perturbation $ \xi \in \mathfrak{se}(3)$. We equate

$$
T \exp(\hat{\xi}) = f(R, t + \delta)
$$

For small perturbations, we approximate:
$$
\exp(\hat{\xi}) \approx \begin{bmatrix} I + \hat{\omega} & v \\ 0 & 1 \end{bmatrix},
$$

and the perturbed pose $ T \exp(\hat{\xi}) $ becomes:
$$
T \exp(\hat{\xi}) = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} \begin{bmatrix} I + \hat{\omega} & v \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} R + R\hat{\omega} & t + Rv \\ 0 & 1 \end{bmatrix}.
$$

On the other hand, the result of just perturbed $t$ with $\delta$ is simply:
$$
f(R, t + \delta) = \begin{bmatrix} R & t + \delta \\ 0 & 1 \end{bmatrix}.
$$

We see that 
$$
   t + Rv = t + \delta.
$$

And hence

$$
v = R^T \delta.
$$

Hence, the differential $D_t f$ of the constructor $f$ with respect to the translation $t$ at $(R, t)$ is:
$$
D_t f(R, t) \, \delta = \begin{bmatrix} 0 \\ R^T \delta \end{bmatrix}.
$$

# Differential Geometry

1. **Setup:**
   We have the function:
   $$
   f(R, t) = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}
   $$
   mapping $(R,t) \in SO(3) \times \mathbb{R}^3$ into $SE(3)$.

2. **View as a Group Operation:**
   Consider $SE(3) = SO(3) \ltimes \mathbb{R}^3$ with group law:
   $$
   (R_1, t_1) \cdot (R_2, t_2) = (R_1 R_2, \; t_1 + R_1 t_2).
   $$
   The function $f$ can be identified with left-translation at $(R,t)$:
   $$
   f(R,t) = L_{(R,t)}(I,0).
   $$

3. **Differentiating the Left-Translation:**
   The differential of $L_{(R,t)}$ at the identity acts on $(\hat{\omega}, v) \in \mathfrak{se}(3)$ by:
   $$
   D(L_{(R,t)})(I,0)(\hat{\omega}, v) = (R \hat{\omega}, \; R v).
   $$

4. **Extracting the Translation Contribution:**
   For a pure translation increment $\delta$, we set $\hat{\omega}=0$ and $v = R^T \delta$. Thus:
   $$
   D_t f(R, t) \delta = \begin{bmatrix}0 \\ R^T \delta\end{bmatrix}.
   $$

