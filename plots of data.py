import pandas as pd
import matplotlib.pyplot as plt

# Define the CSV file and field names
output_dir = 'Gen_Data'
output_file = f'{output_dir}/saved_data.csv'
fieldnames = ["num", "x", "y", "targetX", "targetY", "errorX", "errorY"]

# Read data using pandas
data = pd.read_csv(output_file)

# Select the columns for plotting
x_values = data['x']
y_values = data['y']
targetX_values = data['targetX']
targetY_values = data['targetY']
errorX_values = data['errorX']
errorY_values = data['errorY']

# Plotting
plt.figure(figsize=(18, 14))
plt.grid()
plt.style.use('ggplot')
# Plot x vs y
plt.subplot(3, 1, 1)
plt.plot(x_values, y_values, label='Position (x,y)')
plt.plot(targetX_values, targetY_values, label='Target (x,y)')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()


# Plot errorX vs errorY
plt.subplot(3, 1, 2)
plt.plot(errorX_values, errorY_values, label='Error (x,y)')
plt.xlabel('ErrorX')
plt.ylabel('ErrorY')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot( errorX_values, label='ErrorX', color='green')
plt.plot( errorY_values, label='ErrorY',color='blue')
plt.xlabel('Time')
plt.ylabel('Error')
plt.legend()
plt.title('ErrorX and ErrorY vs Time')
plt.tight_layout()
plt.show()
