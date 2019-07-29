# Clone the gym duckietown repository
if [ -d "simulation" ]; then
  echo "simulation/ directory exists, skipping clone of gym-duckietown..";
else
  git clone https://github.com/montrealrobotics/gym-duckietown simulation
fi 

# pip install requirements for this class
pip install -r requirements.txt

