"""
Interactive N-Link Robotic Arm Kinematics Visualizer (FABRIK Engine)
Author: Aarul Dadaraya
Framework: Streamlit + Matplotlib
"""

import streamlit as st
import matplotlib.pyplot as plt
import numpy as np
import math

# Page Configuration
st.set_page_config(
    page_title="FABRIK Robotic Arm Visualizer",
    page_icon="🦾",
    layout="wide"
)

# Custom Styling
st.markdown("""
<style>
    .main-title { font-size: 2.2rem; font-weight: 700; color: #1E3A8A; margin-bottom: 0px; }
    .sub-title { font-size: 1.0rem; color: #64748B; margin-bottom: 20px; }
</style>
""", unsafe_allow_html=True)

st.markdown('<div class="main-title">🦾 N-Link Planar Robotic Arm — FABRIK Engine</div>', unsafe_allow_html=True)
st.markdown('<div class="sub-title">Forward And Backward Reaching Inverse Kinematics with Real-Time Geometric Convergence</div>', unsafe_allow_html=True)

# ----------------- SIDEBAR CONTROLS -----------------
st.sidebar.header("⚙️ Manipulator Configuration")

# Number of links input (unlimited up to 200)
num_links = st.sidebar.number_input(
    "Number of Links (N)",
    min_value=1,
    max_value=200,
    value=4,
    step=1
)

lengths = []

# Mode selection for setting lengths
if num_links <= 10:
    st.sidebar.subheader("📏 Individual Link Lengths")
    for i in range(num_links):
        L = st.sidebar.number_input(f"Link {i+1} Length", min_value=0.01, value=10.0, step=1.0, key=f"link_{i}")
        lengths.append(L)
else:
    st.sidebar.markdown(f"**ℹ️ {num_links} Links Selected**")
    length_mode = st.sidebar.radio(
        "Choose Length Input Method:",
        ["Option A: Uniform Length for All Links", "Option B: Customize Each Individual Link"],
        index=0
    )

    if length_mode.startswith("Option A"):
        uniform_len = st.sidebar.number_input("Uniform Length per Link", min_value=0.01, value=5.0, step=0.5)
        lengths = [uniform_len] * num_links
    else:
        st.sidebar.info("Expand the section below to set each link length with zero limits.")
        with st.sidebar.expander(f"⚙️ Individual Inputs for all {num_links} Links", expanded=True):
            for i in range(num_links):
                L = st.number_input(f"Link {i+1} Length", min_value=0.01, value=5.0, step=0.5, key=f"link_large_{i}")
                lengths.append(L)

# Workspace limits
max_reach = sum(lengths)
max_single_link = max(lengths)
other_links_sum = max_reach - max_single_link
min_reach = max(0.0, max_single_link - other_links_sum)

st.sidebar.markdown("---")
st.sidebar.subheader("🎯 End-Effector Target (X, Y)")
st.sidebar.caption("Enter any positive, negative, or large coordinates:")
target_x = st.sidebar.number_input("Target X Coordinate", value=round(max_reach * 0.4, 2), step=1.0)
target_y = st.sidebar.number_input("Target Y Coordinate", value=round(max_reach * 0.5, 2), step=1.0)

tolerance = st.sidebar.number_input("Convergence Tolerance (cm)", min_value=0.00001, value=0.01, step=0.005, format="%.4f")
max_iterations = st.sidebar.number_input("Max Allowed Iterations", min_value=5, max_value=500, value=40, step=5)

st.sidebar.markdown("---")
solve_btn = st.sidebar.button("🚀 Calculate & Solve IK", type="primary", use_container_width=True)

# ----------------- FABRIK CORE SOLVER -----------------
def move_point(pointA, pointB, desired_len):
    dx = pointA[0] - pointB[0]
    dy = pointA[1] - pointB[1]
    dist = math.sqrt(dx**2 + dy**2)
    if dist == 0:
        return pointA
    ratio = desired_len / dist
    return [pointB[0] + dx * ratio, pointB[1] + dy * ratio]

def solve_fabrik(lengths, target, tolerance, max_iter):
    # Initialize arm straight along X
    joints = [[0.0, 0.0]]
    curr_x = 0.0
    for L in lengths:
        curr_x += L
        joints.append([curr_x, 0.0])
        
    history = []
    
    for it in range(max_iter):
        # 1. Backward Pass
        joints[-1] = [target[0], target[1]]
        for i in range(len(joints) - 2, -1, -1):
            joints[i] = move_point(joints[i], joints[i+1], lengths[i])
            
        # 2. Forward Pass
        joints[0] = [0.0, 0.0]
        for i in range(len(lengths)):
            joints[i+1] = move_point(joints[i+1], joints[i], lengths[i])
            
        err = math.sqrt((joints[-1][0] - target[0])**2 + (joints[-1][1] - target[1])**2)
        history.append(err)
        
        if err <= tolerance:
            return joints, it + 1, err, history
            
    return joints, max_iter, err, history

# ----------------- EXECUTION & DISPLAY -----------------
target_dist = math.sqrt(target_x**2 + target_y**2)

if solve_btn:
    st.session_state.calculated = True

if "calculated" not in st.session_state or not st.session_state.calculated:
    st.info("👈 Configure your links and target coordinates in the sidebar, then click **'🚀 Calculate & Solve IK'** to run the simulation!")
    # Show initial straight arm on load
    joints_init = [[0.0, 0.0]]
    curr_x = 0.0
    for L in lengths:
        curr_x += L
        joints_init.append([curr_x, 0.0])
        
    fig, ax = plt.subplots(figsize=(7, 7))
    theta = np.linspace(0, 2*np.pi, 200)
    ax.plot(max_reach * np.cos(theta), max_reach * np.sin(theta), color='#94A3B8', linestyle='--', linewidth=1, label=f'Max Reach ({max_reach:.1f})')
    xs = [pt[0] for pt in joints_init]
    ys = [pt[1] for pt in joints_init]
    ax.plot(xs, ys, color='#94A3B8', linewidth=2.5, label='Initial Rest Pose')
    ax.scatter(xs, ys, color='white', edgecolor='#94A3B8', s=40, zorder=3)
    ax.scatter([target_x], [target_y], color='#EF4444', marker='x', s=120, label=f'Target ({target_x}, {target_y})')
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.legend(loc='upper right')
    st.pyplot(fig)
else:
    col_plot, col_info = st.columns([1.6, 1.0])

    is_reachable = (target_dist <= max_reach) and (target_dist >= min_reach)

    if not is_reachable:
        with col_plot:
            if target_dist > max_reach:
                st.error(f"❌ Target is out of reach! Distance = {target_dist:.2f} cm (Max Reach = {max_reach:.2f} cm)")
            else:
                st.error(f"❌ Target is inside the inner dead zone! Distance = {target_dist:.2f} cm (Min Reach = {min_reach:.2f} cm)")
                
            fig, ax = plt.subplots(figsize=(8, 8))
            theta = np.linspace(0, 2*np.pi, 200)
            ax.plot(max_reach * np.cos(theta), max_reach * np.sin(theta), 'r--', label=f'Max Reach ({max_reach:.1f})')
            if min_reach > 0:
                ax.plot(min_reach * np.cos(theta), min_reach * np.sin(theta), 'm--', label=f'Min Reach ({min_reach:.1f})')
            ax.scatter([0], [0], color='black', s=120, label='Base (0,0)')
            ax.scatter([target_x], [target_y], color='red', marker='x', s=160, label='Target (Unreachable)')
            ax.set_aspect('equal')
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.legend()
            st.pyplot(fig)
    else:
        # Run Solver
        joints, iterations, final_err, history = solve_fabrik(lengths, [target_x, target_y], tolerance, max_iterations)
        
        with col_plot:
            fig, ax = plt.subplots(figsize=(8.5, 8.5))
            
            # Draw Workspace Bounds
            theta = np.linspace(0, 2*np.pi, 300)
            ax.plot(max_reach * np.cos(theta), max_reach * np.sin(theta), color='#94A3B8', linestyle='--', linewidth=1.2, label=f'Max Workspace (r={max_reach:.1f})')
            if min_reach > 0:
                ax.plot(min_reach * np.cos(theta), min_reach * np.sin(theta), color='#F43F5E', linestyle='--', linewidth=1.2, label=f'Inner Dead Zone (r={min_reach:.1f})')
                
            # Draw EVERY link as a TRUE RIGID STRAIGHT SEGMENT from joint to joint
            link_linewidth = max(2.0, min(5.0, 60.0 / num_links))
            joint_size = max(20.0, min(80.0, 500.0 / num_links))
            
            for i in range(len(joints) - 1):
                ax.plot(
                    [joints[i][0], joints[i+1][0]], 
                    [joints[i][1], joints[i+1][1]], 
                    color='#0284C7', 
                    linewidth=link_linewidth, 
                    solid_capstyle='round',
                    zorder=2
                )
                
            # Draw every individual joint vertex (white circle with blue border)
            xs = [pt[0] for pt in joints]
            ys = [pt[1] for pt in joints]
            ax.scatter(xs[:-1], ys[:-1], color='#FFFFFF', edgecolor='#0284C7', s=joint_size, linewidth=1.8, zorder=3, label=f'Joints (N={num_links})')
                
            # Base Joint
            ax.scatter([0], [0], color='#0F172A', s=150, zorder=4, label='Base (0,0)')
            
            # End Effector Tip
            ax.scatter([joints[-1][0]], [joints[-1][1]], color='#10B981', s=140, zorder=4, label=f'Tip: ({joints[-1][0]:.2f}, {joints[-1][1]:.2f})')
            
            # Target Point
            ax.scatter([target_x], [target_y], color='#EF4444', marker='x', s=180, linewidth=3, zorder=5, label=f'Target: ({target_x:.2f}, {target_y:.2f})')
            
            # Plot styling
            pad = max_reach * 1.15
            ax.set_xlim(-pad, pad)
            ax.set_ylim(-pad, pad)
            ax.set_aspect('equal')
            ax.grid(True, linestyle=':', alpha=0.5)
            ax.set_title(f"Manipulator Pose (Solved in {iterations} iterations | Error: {final_err*10:.2f} mm)", fontsize=13, fontweight='bold', color='#1E293B')
            ax.legend(loc='upper right', fontsize=8)
            
            st.pyplot(fig)

        with col_info:
            st.subheader("📊 Convergence Analytics")
            st.metric(label="Status", value="Converged ✓", delta=f"{iterations} Iterations")
            
            col_m1, col_m2 = st.columns(2)
            col_m1.metric("Final Tip Error", f"{final_err*10:.3f} mm")
            col_m2.metric("Target Distance", f"{target_dist:.2f} cm")
            
            # Relative Joint Angles calculation
            st.subheader("📐 Extracted Motor Angles")
            angles_deg = []
            alpha_prev = math.atan2(joints[1][1] - joints[0][1], joints[1][0] - joints[0][0])
            angles_deg.append(math.degrees(alpha_prev))
            
            for i in range(1, len(lengths)):
                dx = joints[i+1][0] - joints[i][0]
                dy = joints[i+1][1] - joints[i][1]
                alpha_curr = math.atan2(dy, dx)
                rel_ang = (math.degrees(alpha_curr - alpha_prev) + 180) % 360 - 180
                angles_deg.append(rel_ang)
                alpha_prev = alpha_curr
                
            if num_links <= 10:
                angle_table = [{"Joint": f"Motor {i+1} (J{i})", "Relative Angle": f"{ang:+.2f}°"} for i, ang in enumerate(angles_deg)]
                st.table(angle_table)
            else:
                with st.expander(f"📋 Full Angle Table for All {num_links} Joints", expanded=False):
                    angle_table_full = [{"Joint": f"Motor {i+1}", "Relative Angle": f"{ang:+.2f}°"} for i, ang in enumerate(angles_deg)]
                    st.table(angle_table_full)
                st.info(f"Summary: Motor 1 = {angles_deg[0]:+.2f}°, Motor {num_links} = {angles_deg[-1]:+.2f}°")

            # Convergence curve
            st.subheader("📉 Convergence Curve")
            st.line_chart(history)

