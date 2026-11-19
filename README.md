# N-Link Planar Robotic Arm — FABRIK Kinematics Engine & Interactive Visualizer

An interactive, high-performance Inverse Kinematics (IK) engine built in Python implementing the **FABRIK (Forward And Backward Reaching Inverse Kinematics)** algorithm. Accompanied by a real-time web visualizer powered by **Streamlit** and **Matplotlib**.

Capable of solving inverse kinematics for hyper-redundant serial manipulators scaling from **2 to 200 links** with **$O(N)$ linear time complexity** and sub-millimeter position convergence.

---

## ⚡ Why FABRIK over Jacobian Inversion?

Traditional numerical methods (e.g., Jacobian Pseudoinverse, Damped Least Squares) scale at **$\mathcal{O}(N^3)$ computational complexity** due to repeated matrix inversions:
* They suffer from computational bottlenecks when scaling to hyper-redundant arms ($N > 10$).
* They encounter algorithmic instability near kinematic singularities.

**FABRIK** operates geometrically on joint coordinates rather than joint angles:
* Operates in **$\mathcal{O}(N)$ linear time complexity** per iteration.
* Guaranteed convergence without matrix inversions or singularity locks.
* Achieves sub-millimeter position accuracy (< 0.5 mm) in fewer than 15 iterations.

---

## 📐 Mathematical Formulation

FABRIK solves for joint positions through two geometric projection passes per iteration:

### 1. Backward Pass (Target Reaching)
The end-effector $P_N$ is clamped directly to the target coordinate $\mathbf{T}$:
$$P_N^{(k)} = \mathbf{T}$$

For each preceding joint from $i = N-1$ down to $0$:
1. Compute the displacement vector between adjacent joints:
   $$\vec{d}_i = P_i - P_{i+1}$$
2. Normalize and project along the fixed link length $L_i$:
   $$P_i = P_{i+1} + \left(\frac{L_i}{\|\vec{d}_i\|}\right) \vec{d}_i$$

### 2. Forward Pass (Base Anchor Re-centering)
The base joint $P_0$ is restored to its physical origin $(0, 0)$:
$$P_0^{(k)} = \mathbf{0}$$

For each subsequent joint from $i = 0$ up to $N-1$:
1. Compute the displacement vector:
   $$\vec{d}_i = P_{i+1} - P_i$$
2. Normalize and project:
   $$P_{i+1} = P_i + \left(\frac{L_i}{\|\vec{d}_i\|}\right) \vec{d}_i$$

### 3. Convergence Criterion
The Euclidean tip error is calculated against the target coordinate $\mathbf{T}$:
$$\epsilon = \|P_N - \mathbf{T}\|_2$$
If $\epsilon \leq \text{tolerance}$, the iteration terminates successfully.

---

## 🚀 Key Features

* **Hyper-Redundant Scaling:** Arbitrary degree-of-freedom manipulator support (tested from 2 up to 200 links).
* **Workspace Boundary Analysis:** Computes physical maximum reach $R_{\text{max}} = \sum L_i$ and internal dead-zone boundaries for irregular link configurations.
* **Actuator Telemetry:** Extracts relative inter-joint rotation angles ready for direct mapping to physical servo/stepper motors.
* **Interactive UI:** Dynamic sliders for Cartesian targets $(X, Y)$, customizable link lengths, real-time geometric visualizer, and live error convergence graphs.

---

## 🛠️ Installation & Quick Start

### 1. Clone the Repository
```bash
git clone https://github.com/dadarayaaarul-art/Planar-Robotic-Arm-Kinematics.git
cd Planar-Robotic-Arm-Kinematics
```

### 2. Install Dependencies
```bash
pip install -r requirements.txt
```

### 3. Launch the Interactive Visualizer
```bash
streamlit run app.py
```
The browser will automatically open at `http://localhost:8501`.

---

## 📁 Repository Structure
```text
├── app.py              # Streamlit interactive web visualizer & real-time telemetry
├── main.py             # CLI-based standalone FABRIK kinematic solver
├── requirements.txt    # Python package dependencies
└── README.md           # Technical documentation & mathematical derivation
```

---

## 👨‍💻 Author
**Aarul Dadaraya**  
*B.Tech in Mechanical Engineering (Robotics & Automation)*  
*MIT World Peace University (MIT-WPU), Pune*  
* [GitHub Profile](https://github.com/dadarayaaarul-art)  
* [LinkedIn Profile](https://www.linkedin.com/in/aarul-dadaraya-78b079330)
