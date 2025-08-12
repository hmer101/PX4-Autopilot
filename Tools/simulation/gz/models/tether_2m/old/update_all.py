import numpy as np
import xml.etree.ElementTree as ET
import re
import subprocess

# Function to extract num_elements and elem_length_main from model.xacro
def extract_from_xacro(xacro_file_path):
    with open(xacro_file_path, 'r') as xacro_file:
        content = xacro_file.read()
        
    # Use regular expressions to find num_elements
    num_elements_match = re.search(r'<xacro:property name="num_elements" value="([^"]+)" />', content)
    num_elements = int(num_elements_match.group(1)) if num_elements_match else None
    
    return num_elements

# Function to extract elem_length_main and elem_length_ends from model.sdf
def extract_from_sdf(sdf_file_path):
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()

    # Extract elem_length_main from the length of the cylinder in the visual element for element_1
    elem_length_main = None
    elem_length_ends = None

    # Find the visual element for element_1 to get elem_length_main
    for visual in root.findall(".//link"):
        if visual.get('name') == "element_1":
            length_elem = visual.find(".//geometry/cylinder/length")
            if length_elem is not None:
                elem_length_main = float(length_elem.text)

        if visual.get('name') == "element_first":
            length_elem = visual.find(".//geometry/cylinder/length")
            if length_elem is not None:
                elem_length_ends = float(length_elem.text)

    return elem_length_main, elem_length_ends

# Function to run the bash script
def run_bash_script(bash_command):
    try:
        # Run the bash script
        result = subprocess.run(bash_command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Print the output of the script
        #print("Script output:", result.stdout.decode())
        
    except subprocess.CalledProcessError as e:
        # If the script fails, print the error
        print(f"Error executing script: {e}")
        print(f"Error message: {e.stderr.decode()}")

# Function to run another python script
def run_python_script(script_path, dist_connection_points):    
    try:
        # Running the Python script
        result = subprocess.run(['python3', script_path, str(dist_connection_points)], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Print the output from the script
        #print("Script output:", result.stdout.decode())
        
    except subprocess.CalledProcessError as e:
        # If the script fails, print the error
        print(f"Error executing script: {e}")
        print(f"Error message: {e.stderr.decode()}")

# Function to calculate the distance between connection points
def calculate_distance_connection_points(elem_length_main, num_elements):
    return elem_length_main / (np.sin(np.pi / (2 * num_elements)))


# Main function to update the SDF with the correct poses
def main():
    ### 0. Define file paths and parameters ###
    script_generate_tether = "/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/models/tether_2m/generate_sdf.sh"
    script_update_poses = "/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/worlds/update_drone_poses.py"

    xacro_file_path = "/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/models/tether_2m/model.xacro"
    tether_sdf_path="/multi_drone_slung_load_master/repos/PX4-Autopilot/Tools/simulation/gz/models/tether_2m/model.sdf"
    

    ### 1. GENERATE NEW TETHER SDF ###
    run_bash_script(script_generate_tether)

    print("1. Tether SDF generated successfully!")


    ### 2. CALCULATE DISTANCE BETWEEN CONNECTION POINTS ###
    # Extract values from model.xacro and model.sdf
    num_elements = extract_from_xacro(xacro_file_path)
    elem_length_main, elem_length_ends = extract_from_sdf(tether_sdf_path)

    if num_elements is None or elem_length_main is None or elem_length_ends is None:
        print("Failed to extract necessary data from the files.")
        return

    # Calculate distance between connection points
    distance_connection_points = calculate_distance_connection_points(elem_length_main, num_elements)
    print(f"Distance between connection points: {distance_connection_points}")

    print("2. Distance between connection points calculated successfully!")


    ### 3. UPDATE DRONE AND TETHER POSES IN WORLD SDF ###
    run_python_script(script_update_poses, distance_connection_points)
    print("3. Drone and tether poses updated successfully!")


    ### 4. END ###
    print("COMPLETE: All tasks finished successfully!")


if __name__ == '__main__':
    main()
