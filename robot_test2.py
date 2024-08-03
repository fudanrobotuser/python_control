import time
import subprocess

# Define your program or command to execute
program_command = ["ethercat sl"]

# Run the program cyclically
while True:
    print("Starting program...")
    subprocess.run(program_command)
    print("Program finished, waiting 5 seconds before restarting...")
    time.sleep(4)  # Adjust the interval as needed
