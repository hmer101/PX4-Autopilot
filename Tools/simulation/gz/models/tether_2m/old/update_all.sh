#!/bin/bash
# This script generates the SDF file for the tether model and updates the world SDF the required drone and tether poses to match

# Run tether SDF generation
./generate_sdf.sh

# Get values from the generated SDF file
tether_sdf_path="/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/models/tether_2m/model.sdf"


# Calculate total length of tether 
pi=3.141592653589793

# Dist connection points: 1.4= 2.19 m cable, 1.45 = 2.27 m cable, 1.475 = 2.31 m cable, 1.48 = 2.32 m cable, 1.49 = 2.33 m cable, 1.5 = 2.35 m cable, 1.521 = 2.37957 m cable, 1.522 = 2.381 m cable, 1.523 = 2.383 m cable, 1.524 = 2.384 m cable, 1.525 = 2.3858 m cable, 1.53 = 2.3936 m cable, 1.535 = 2.40147 m cable, 1.545 = 2.4171 m cable, 1.5475 = 2.4210 m cable, 1.55 = 2.4249 m cable, 1.6 = 2.50 m cable, 1.65 = 2.58 m cable, 
elem_length_main=0.24210000000000001 # Replace with actual value or source
num_elements=10       # Replace with actual value or source
elem_length_ends=0.0001 # Replace with actual value or source

total_length=$(echo "$elem_length_main * $num_elements + 2 * $elem_length_ends" | bc)
distance_connection_points=$(echo "scale=10; $elem_length_main / (s($pi/(2*$num_elements)))" | bc -l)

# Print total length of tether
#echo "Total length of tether (update using data from SDF): $total_length"

# Print distance connection points
echo "Distance between connection points (update using data from SDF, also run calculate_drone_poses and update world sdf): $distance_connection_points"
