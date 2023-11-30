import pandas as pd
import matplotlib.pyplot as plt

# Define the CSV file and field names
output_dir = 'Gen_Data'
output_file = f'{output_dir}/saved_data.csv'
fieldnames = ["num", "x", "y", "targetX", "targetY", "errorX", "errorY", "tot_error", "PID_x", "PID_y"]

# Read data using pandas
data = pd.read_csv(output_file)

# Select the columns for plotting
x_values = data['x']
y_values = data['y']
targetX_values = data['targetX']
targetY_values = data['targetY']
errorX_values = data['errorX']
errorY_values = data['errorY']
errortot_values = data['tot_error']
Pidx_values = data['PID_x']
Pidy_values = data['PID_y']

# Plotting individual figures
plt.figure(figsize=(8, 6))

# Plot x vs y
plt.plot(x_values, y_values, label='Position (x,y)')
plt.plot(targetX_values, targetY_values, label='Target (x,y)')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.title('Position (x,y) and Target (x,y)')

# Show the plot
plt.show()

# Create a new figure for the next plot
plt.figure(figsize=(8, 6))

# Plot errorX vs errorY
plt.plot(errorX_values, errorY_values, label='Error (x,y)')
plt.xlabel('ErrorX')
plt.ylabel('ErrorY')
plt.legend()
plt.title('Error (x,y)')

# Show the plot
plt.show()

# Create a new figure for the third plot
plt.figure(figsize=(8, 6))

# Plot errorX and errorY separately vs Time
plt.plot(errorX_values, label='ErrorX', color='green')
plt.plot(errorY_values, label='ErrorY', color='blue')
plt.xlabel('Time')
plt.ylabel('Error')
plt.legend()
plt.title('ErrorX and ErrorY vs Time')

# Show the plot
plt.show()
plt.figure(figsize=(8, 6), dpi=900)

# Plot x vs y
plt.plot(Pidx_values, label='Pådrag PID x')
plt.plot(Pidy_values,  label='Pådrag PID Y')
plt.xlabel('time')
plt.ylabel('gain')
plt.legend()
plt.title('Effect of PID regulator ')

# Show the plot
plt.show()
