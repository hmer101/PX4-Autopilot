# Parameters
# Universal joints
JOINT2_AXIS="0 0 1"
JOINT2_FRICTION=0
JOINT2_DAMPING=0.01
JOINT2_SPRING_STIFFNESS=0

# Prismatic joints
PRISMATIC_DAMPING=0.1
PRISMATIC_FRICTION=0.0
PRISMATIC_SPRING_REFERENCE=0.0
PRISMATIC_SPRING_STIFFNESS=25.0

FIRST_JOINT_NAME="joint_p1_to_p2"
FIRST_DAMPING=0.0
FIRST_FRICTION=0.0
FIRST_SPRING_REFERENCE=0.0
FIRST_SPRING_STIFFNESS=0.0

# Generate URDF from XACRO
echo "1. Generating URDF from XACRO..."
xacro model.xacro > model.urdf

# Check that the urdf is valid
echo "2. Checking URDF validity..."
check_urdf model.urdf

# Generate SDF from URDF
echo "3. Generating SDF from URDF..."
gz sdf -p model.urdf > model.sdf

# Find and replace in the sdf file to add features to the SDF not available in URDF
# Swap to universal joint
sed -i 's/revolute/universal/g' model.sdf

# Add axis2 only to universal joints
perl -i -0777 -pe '
  s|(<joint[^>]*type=["'\'']universal["'\''][^>]*>.*?<axis>.*?</axis>)|
    $1 . qq{
      <axis2>
        <xyz>'"$JOINT2_AXIS"'</xyz>
        <limit>
          <lower>-inf</lower>
          <upper>inf</upper>
        </limit>
        <dynamics>
          <damping>'"$JOINT2_DAMPING"'</damping>
          <friction>'"$JOINT2_FRICTION"'</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>'"$JOINT2_SPRING_STIFFNESS"'</spring_stiffness>
        </dynamics>
      </axis2>}
  |ges
' model.sdf

# Add prismatic joint properties
# First prismatic joint
perl -i -0777 -pe '
  s|(<joint[^>]*name=["'"'"']'"$FIRST_JOINT_NAME"'["'"'"'][^>]*>.*?<axis>.*?<dynamics>).*?</dynamics>|$1
        <damping>'"$FIRST_DAMPING"'</damping>
        <friction>'"$FIRST_FRICTION"'</friction>
        <spring_reference>'"$FIRST_SPRING_REFERENCE"'</spring_reference>
        <spring_stiffness>'"$FIRST_SPRING_STIFFNESS"'</spring_stiffness>
      </dynamics>|gs
' model.sdf

# Other prismatic joints
perl -i -0777 -pe '
  s|(<joint(?![^>]*name=["'"'"']'"$FIRST_JOINT_NAME"'["'"'"'])[^>]*type=["'"'"']prismatic["'"'"'][^>]*>.*?<axis>.*?<dynamics>).*?</dynamics>|$1
        <damping>'"$PRISMATIC_DAMPING"'</damping>
        <friction>'"$PRISMATIC_FRICTION"'</friction>
        <spring_reference>'"$PRISMATIC_SPRING_REFERENCE"'</spring_reference>
        <spring_stiffness>'"$PRISMATIC_SPRING_STIFFNESS"'</spring_stiffness>
      </dynamics>|gs
' model.sdf


# Calculate total length of tether
echo "4. Calculating total length of tether..."
elem_length_main=1.4  # Replace with actual value or source
num_elements=1       # Replace with actual value or source
elem_length_ends=0.0001 # Replace with actual value or source

total_length=$(echo "$elem_length_main * $num_elements + 2 * $elem_length_ends" | bc)

# Print total length of tether
echo "Total length of tether (update using data from SDF): $total_length"