
import math
import matplotlib.pyplot as plt
import numpy as np
from navigation.repulsorFieldPlanner import RepulsorFieldPlanner
from navigation.navConstants import *
from matplotlib import cm
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
from scipy.signal import argrelextrema
from utils.constants import FIELD_X_M, FIELD_Y_M
from wpimath.geometry import Translation2d

class VectorPlotter:
    def __init__(self, widthMetersIn, heightMetersIn, figSizeIn=(10, 8), dpi=100):
        """
        Initializes the VectorPlotter.

        Parameters:
        - widthMeters (float): Width of the space in meters.
        - heightMeters (float): Height of the space in meters.
        - figSize (tuple): Size of the figure in inches (width, height).
        - dpi (int): Dots per inch for the figure.
        """
        print("Initializing VectorPlotter...")
        self.widthMeters = widthMetersIn
        self.heightMeters = heightMetersIn
        self.figSize = figSizeIn
        self.dpi = dpi
        self.vectors = []  # List to store vectors as tuples
        print(f"Defined space: {self.widthMeters}m (width) x {self.heightMeters}m (height)")

        # Initialize the plot
        print("Setting up the plot...")
        self.fig, self.ax = plt.subplots(figsize=self.figSize, dpi=self.dpi)
        self.ax.set_xlim(0, self.widthMeters)
        self.ax.set_ylim(0, self.heightMeters)
        self.ax.set_aspect('equal')  # Ensure equal scaling for x and y
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Vector Plotter with Gradient Lines and Local Minima')

        # Optional: Add grid
        self.ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        print("Plot setup complete.\n")

    def addVector(self, anchorX, anchorY, vectorX, vectorY):
        """
        Adds a vector to the plot.

        Parameters:
        - anchor_x (float): X-coordinate of the anchor point in meters.
        - anchor_y (float): Y-coordinate of the anchor point in meters.
        - vector_x (float): X-component of the vector in arbitrary units.
        - vector_y (float): Y-component of the vector in arbitrary units.
        """
        print(f"Adding vector: Anchor=({anchorX}, {anchorY}), Vector=({vectorX}, {vectorY})")
        self.vectors.append((anchorX, anchorY, vectorX, vectorY))

    def plot(self):
        """
        Plots all added vectors with color-coding based on magnitude,
        gradient lines, and marks local minima.
        """
        print("\nStarting plot process...")
        if not self.vectors:
            print("No vectors to plot. Exiting plot process.")
            return

        print(f"Number of vectors to plot: {len(self.vectors)}")

        # Extract vector components
        print("Extracting vector components...")
        vectorsNP = np.array(self.vectors)
        anchorX = vectorsNP[:, 0].astype(float)
        anchorY = vectorsNP[:, 1].astype(float)
        vectorX = vectorsNP[:, 2].astype(float)
        vectorY = vectorsNP[:, 3].astype(float)
        print("Extraction complete.")

        # Calculate magnitudes
        print("Calculating vector magnitudes...")
        magnitudes = np.linalg.norm(vectorsNP[:, 2:4], axis=1)
        maxMagnitude = np.max(magnitudes)
        minMagnitude = np.min(magnitudes)
        print(f"Maximum magnitude: {maxMagnitude}")
        print(f"Minimum magnitude: {minMagnitude}")

        if maxMagnitude == 0:
            print("All vectors have zero magnitude. Exiting plot process.")
            return

        # Normalize magnitudes for color mapping
        print("Normalizing magnitudes for color mapping...")
        norm = plt.Normalize(vmin=minMagnitude, vmax=maxMagnitude)
        cmap = cm.get_cmap('jet')  # Blue -> Green -> Red
        colors = cmap(norm(magnitudes))
        print("Color normalization complete.")

        # Plot each vector with color based on magnitude
        print("Plotting vectors...")
        for i in range(len(self.vectors)):
            mag = math.sqrt(vectorX[i]**2 + vectorY[i]**2)
            self.ax.arrow(anchorX[i], anchorY[i],
                          vectorX[i]/mag*0.2, vectorY[i]/mag*0.2,
                          head_width=0.06, head_length=0.1,
                          fc=colors[i], ec=colors[i],
                          length_includes_head=True)
            if (i + 1) % 10 == 0 or (i + 1) == len(self.vectors):
                print(f"Plotted {i + 1}/{len(self.vectors)} vectors.")
        print("All vectors plotted.")

        # Create a grid for gradient lines (streamlines)
        # print("Creating grid for gradient lines...")
        # gridSize = 20  # Adjust for resolution
        # gridX, gridY = np.mgrid[0:self.widthMeters:complex(gridSize),
        #                           0:self.heightMeters:complex(gridSize)]
        # print(f"Grid created with size {gridSize}x{gridSize}.")

        # Interpolate vector components onto the grid
        # print("Interpolating vector components onto the grid...")
        # gridU = griddata((anchorX, anchorY), vectorX, (gridX, gridY), method='cubic', fill_value=0)
        # gridV = griddata((anchorX, anchorY), vectorY, (gridX, gridY), method='cubic', fill_value=0)
        # print("Interpolation complete.")

        # Detect local minima
        print("Detecting local minima in vector magnitudes...")
        # Smooth magnitudes for better minima detection
        magnitudesSmooth = gaussian_filter(magnitudes, sigma=1)

        # Find indices of local minima
        localMinIndices = argrelextrema(magnitudesSmooth, np.less)[0]
        print(f"Found {len(localMinIndices)} potential local minima.")

        # Define a threshold to avoid marking trivial minima
        threshold = minMagnitude + (maxMagnitude - minMagnitude) * 0.1  # 10% above min
        print(f"Applying threshold: {threshold} to filter minima.")

        # Filter minima based on threshold
        filteredMinIndices = [i for i in localMinIndices if magnitudes[i] <= threshold]
        print(f"{len(filteredMinIndices)} local minima passed the threshold.")

        # Plot local minima
        if filteredMinIndices:
            print("Marking local minima on the plot...")
            minimaX = anchorX[filteredMinIndices]
            minimaY = anchorY[filteredMinIndices]
            self.ax.scatter(minimaX, minimaY, color='magenta', marker='X',
                            s=10, label='Local Minima')
            for x, y in zip(minimaX, minimaY):
                self.ax.annotate('', (x, y), textcoords="offset points",
                                  ha='center', color='magenta')
            print("Local minima marked.")
        else:
            print("No significant local minima found.")

        print("Finalizing and displaying the plot...")
        plt.show()
        print("Plot displayed successfully.\n")

    def clearVectors(self):
        """
        Clears all vectors from the plot.
        """
        print("Clearing all vectors and resetting the plot...")
        self.vectors = []
        self.ax.cla()
        self.ax.set_xlim(0, self.widthMeters)
        self.ax.set_ylim(0, self.heightMeters)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Vector Plotter with Gradient Lines and Local Minima')
        self.ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        print("Plot reset complete.\n")

# Example Usage
if __name__ == "__main__":
    print("=== Vector Plotter Program Started ===\n")

    # Create a VectorPlotter instance
    print("\nCreating VectorPlotter instance...")
    plotter = VectorPlotter(widthMetersIn=FIELD_X_M, heightMetersIn=FIELD_Y_M,
                            figSizeIn=(12, 9), dpi=100)
    print("VectorPlotter instance created.\n")

    # Add vectors: add_vector(anchor_x, anchor_y, vector_x, vector_y)
    print("Adding vectors...")
    rpp = RepulsorFieldPlanner()
    rpp.setGoal(GOAL_1A)

    XCount = 0.1
    YCount = 0.1
    while YCount < 8.1:
        while XCount < 16.4:
            force = rpp._getForceAtTrans(Translation2d(XCount, YCount))
            plotter.addVector(XCount, YCount, force.x, force.y)
            XCount += .2
        YCount += .2
        XCount = 0.1

    print("All vectors added.\n")

    # Plot the vectors along with gradient lines and local minima
    print("Initiating plotting of vectors...")
    plotter.plot()
    print("=== Vector Plotter Program Finished ===")
