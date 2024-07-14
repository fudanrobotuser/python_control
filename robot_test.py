import time
import subprocess

# Define your program or command to execute
program_command = ["/home/fudanrobotuser/fd_humanoid_robot/build/robot_control_module/robot_control_node"]

# Run the program cyclically
while True:
    print("Starting program...")
    subprocess.run(program_command)
    print("Program finished, waiting 5 seconds before restarting...")
    time.sleep(5)  # Adjust the interval as needed
