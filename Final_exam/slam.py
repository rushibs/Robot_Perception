import matplotlib.pyplot as plt
import numpy as np

# Define the ground truth scene
ground_truth_positions = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]])
ground_truth_colors = ['red', 'green', 'blue', 'yellow']

# Define the estimated scene
estimated_positions = np.array([[0.1, 0.1, 0.1], [0.9, 0.1, 0.1], [0.1, 0.9, 0.1], [0.9, 0.9, 0.1]])
estimated_colors = ['red', 'green', 'blue', 'yellow']

# Plot the ground truth scene
plt.scatter(ground_truth_positions[:, 0], ground_truth_positions[:, 1], c=ground_truth_colors)
plt.title('Ground Truth Scene')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

# Plot the estimated scene
plt.scatter(estimated_positions[:, 0], estimated_positions[:, 1], c=estimated_colors)
plt.title('Estimated Scene')
plt.xlabel('X')                                                                                                                                                            
plt.ylabel('Y')
plt.show()                                                                                                                                                              