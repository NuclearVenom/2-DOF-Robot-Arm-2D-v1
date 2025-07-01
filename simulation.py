import math
import tkinter as tk
from tkinter import ttk

class RoboticArmSimulation:
    def __init__(self, root):
        self.root = root
        self.root.title("2-Joint Robotic Arm Simulation")
        
        # Arm parameters
        self.base_pos = (300, 400)
        self.link1_length = 150
        self.link2_length = 120
        self.joint1_angle = math.pi/4  # Initial angle (45 degrees)
        self.joint2_angle = math.pi/4  # Initial angle (45 degrees)
        self.target = (300, 200)
        self.speed = 0.02
        self.moving = False
        
        # Canvas setup
        self.canvas = tk.Canvas(root, width=600, height=500, bg='white')
        self.canvas.pack(pady=10)
        
        # Control panel
        control_frame = tk.Frame(root)
        control_frame.pack(pady=5)
        
        tk.Label(control_frame, text="Speed:").pack(side=tk.LEFT)
        self.speed_slider = ttk.Scale(control_frame, from_=0.005, to=0.1, value=self.speed, 
                                    command=lambda e: self.update_speed())
        self.speed_slider.pack(side=tk.LEFT, padx=5)
        
        self.angle_label = tk.Label(root, text="Joint Angles: ")
        self.angle_label.pack()
        
        # Click to set target
        self.canvas.bind("<Button-1>", self.set_target)
        
        # Draw initial state
        self.draw_arm()
        self.update_angle_display()
    
    def update_speed(self):
        self.speed = float(self.speed_slider.get())
    
    def set_target(self, event):
        self.target = (event.x, event.y)
        self.moving = True
        self.move_arm()
    
    def calculate_inverse_kinematics(self, target):
        x, y = target
        x -= self.base_pos[0]
        y = self.base_pos[1] - y  # Convert to math coordinates
        
        # Calculate joint angles using inverse kinematics
        try:
            # Distance from base to target
            d = math.sqrt(x**2 + y**2)
            
            # Law of cosines to get joint2 angle
            cos_angle2 = (d**2 - self.link1_length**2 - self.link2_length**2) / (2 * self.link1_length * self.link2_length)
            angle2 = math.acos(cos_angle2)
            
            # Angle between link1 and the line from base to target
            angle_a = math.atan2(y, x)
            
            # Angle between link1 and link2
            angle_b = math.atan2(self.link2_length * math.sin(angle2), 
                                self.link1_length + self.link2_length * math.cos(angle2))
            
            angle1 = angle_a - angle_b
            
            return angle1, angle2
        except:
            # Target unreachable, return current angles
            return self.joint1_angle, self.joint2_angle
    
    def move_arm(self):
        if not self.moving:
            return
        
        # Calculate target angles
        target_angle1, target_angle2 = self.calculate_inverse_kinematics(self.target)
        
        # Move current angles toward target angles
        angle1_diff = target_angle1 - self.joint1_angle
        angle2_diff = target_angle2 - self.joint2_angle
        
        if abs(angle1_diff) < self.speed and abs(angle2_diff) < self.speed:
            self.joint1_angle = target_angle1
            self.joint2_angle = target_angle2
            self.moving = False
        else:
            self.joint1_angle += math.copysign(min(self.speed, abs(angle1_diff)), angle1_diff)
            self.joint2_angle += math.copysign(min(self.speed, abs(angle2_diff)), angle2_diff)
            self.root.after(20, self.move_arm)
        
        self.draw_arm()
        self.update_angle_display()
    
    def draw_arm(self):
        self.canvas.delete("all")
        
        # Draw base
        self.canvas.create_oval(self.base_pos[0]-15, self.base_pos[1]-15,
                               self.base_pos[0]+15, self.base_pos[1]+15, fill="gray")
        
        # Calculate joint positions
        joint1_x = self.base_pos[0] + self.link1_length * math.cos(self.joint1_angle)
        joint1_y = self.base_pos[1] - self.link1_length * math.sin(self.joint1_angle)
        
        joint2_x = joint1_x + self.link2_length * math.cos(self.joint1_angle + self.joint2_angle)
        joint2_y = joint1_y - self.link2_length * math.sin(self.joint1_angle + self.joint2_angle)
        
        # Draw links
        self.canvas.create_line(self.base_pos[0], self.base_pos[1], joint1_x, joint1_y, width=8, fill="blue")
        self.canvas.create_line(joint1_x, joint1_y, joint2_x, joint2_y, width=6, fill="green")
        
        # Draw joints
        self.canvas.create_oval(joint1_x-10, joint1_y-10, joint1_x+10, joint1_y+10, fill="red")
        self.canvas.create_oval(joint2_x-8, joint2_y-8, joint2_x+8, joint2_y+8, fill="orange")
        
        # Draw target
        self.canvas.create_oval(self.target[0]-5, self.target[1]-5,
                               self.target[0]+5, self.target[1]+5, fill="red")
    
    def update_angle_display(self):
        angle1_deg = round(math.degrees(self.joint1_angle))
        angle2_deg = round(math.degrees(self.joint2_angle))
        self.angle_label.config(text=f"Joint Angles: Base-Elbow: {angle1_deg}°, Elbow-Wrist: {angle2_deg}°")

if __name__ == "__main__":
    root = tk.Tk()
    app = RoboticArmSimulation(root)
    root.mainloop()