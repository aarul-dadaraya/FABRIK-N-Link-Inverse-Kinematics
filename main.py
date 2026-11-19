#FABRIK 

import math

while True:
    try:
        num_links = int(input("Enter number of links: "))
        if num_links < 1:
            print("Must have at least 1 link.")
        else:
            break
    except ValueError:
        print("Please enter a valid integer.")


lengths = []
for i in range(num_links):
    while True:
        try:
            length = float(input(f"What is the length of Link {i+1}: "))
            if length <= 0:
                print("Length has to be greater than 0.")
            else:
                lengths.append(length)
                print(f"Link of length {length} has been added.")
                break
        except ValueError:
            print("Please enter a valid number.")



# --- GET TARGET COORDINATES (x, y) ---
while True:
    try:
        target_x = float(input("\nEnter target X coordinate: "))
        break
    except ValueError:
        print("Please enter a valid number.")

while True:
    try:
        target_y = float(input("Enter target Y coordinate: "))
        break
    except ValueError:
        print("Please enter a valid number.")

target = [target_x, target_y]
print(f"Target set to: ({target[0]}, {target[1]})")

max_reach = sum(lengths)
max_single_link = max(lengths)
other_links_sum = max_reach - max_single_link
min_reach = max(0.0, max_single_link - other_links_sum)

dist_to_target = math.sqrt(target[0]**2 + target[1]**2)

if dist_to_target > max_reach:
    print(f"\n[!] Target unreachable! Too far: Distance ({dist_to_target:.2f}) > Max Reach ({max_reach:.2f})")
    exit()
elif dist_to_target < min_reach:
    print(f"\n[!] Target unreachable! Inside inner dead zone: Distance ({dist_to_target:.2f}) < Min Reach ({min_reach:.2f})")
    exit()
else:
    print(f"\n[✓] Target is reachable! Distance = {dist_to_target:.2f} (Workspace: [{min_reach:.2f}, {max_reach:.2f}])")




#making the links lying straight on the ground
joints=[[0.0,0.0]]#this is the fixed base
current_x=0.0
for length in lengths:
    current_x+=length
    joints.append([current_x,0.0])

print("\nInitial joint position.")
for i,points in enumerate(joints):
    print(f" p{i}|px{points[0]:.2f}|px{points[1]:.2f}")



def move_point(point_A,point_B,desired_length):
    x=point_A[0]-point_B[0]
    y=point_A[1]-point_B[1]

    current_length=math.sqrt(x**2+y**2)

    if current_length==0:
        return point_A

    lambda_ratio=desired_length/current_length


    new_x=point_B[0]+(lambda_ratio*x)
    new_y=point_B[1]+(lambda_ratio*y)

    return [new_x, new_y]


max_iterations=20
tolerance=0.01
for iteration in range(max_iterations):
    # --- 1. BACKWARD PASS ---
    joints[-1]=[target_x,target_y]

    for i in range(len(joints)-2,-1,-1):
        joints[i]=move_point(point_A=joints[i],point_B=joints[i+1],desired_length=lengths[i])

    # --- 2. FORWARD PASS ---
    joints[0]=[0.0,0.0]

    for i in range(len(lengths)):
        joints[i+1]=move_point(point_A=joints[i+1],point_B=joints[i],desired_length=lengths[i])

    # --- 3. CONVERGENCE CHECK ---
    error = math.sqrt((joints[-1][0] - target_x)**2 + (joints[-1][1] - target_y)**2)
    print(f"Iteration {iteration + 1} complete | Tip Error: {error:.4f} cm")

    if error <= tolerance:
        print(f"\n==================================================")
        print(f"  [✓] SUCCESS: Solved in {iteration + 1} iterations!")
        print(f"  Final Tip Position: ({joints[-1][0]:.3f}, {joints[-1][1]:.3f})")
        print(f"  Target Position   : ({target_x:.3f}, {target_y:.3f})")
        print(f"  Final Error       : {error:.6f} cm")
        print(f"==================================================")
        print("\nFinal Joint Coordinates:")
        for idx, pt in enumerate(joints):
            print(f"  Joint P{idx}: ({pt[0]:.3f}, {pt[1]:.3f})")

        # --- EXTRACT RELATIVE JOINT MOTOR ANGLES ---
        angles_deg = []
        alpha_prev = math.atan2(joints[1][1] - joints[0][1], joints[1][0] - joints[0][0])
        angles_deg.append(math.degrees(alpha_prev))

        for i in range(1, len(lengths)):
            dx = joints[i+1][0] - joints[i][0]
            dy = joints[i+1][1] - joints[i][1]
            alpha_current = math.atan2(dy, dx)
            
            relative_angle = alpha_current - alpha_prev
            relative_deg = (math.degrees(relative_angle) + 180) % 360 - 180
            angles_deg.append(relative_deg)
            
            alpha_prev = alpha_current

        print("\nCalculated Motor Joint Angles (Degrees):")
        for idx, ang in enumerate(angles_deg):
            print(f"  Motor Joint {idx+1}: {ang:8.2f}°")
        break





