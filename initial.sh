# Clone the gym duckietown repository
if [ -d "simulation" ]; then
  echo "simulation/ directory exists, skipping clone of gym-duckietown..";
else
  git clone https://github.com/montrealrobotics/gym-duckietown simulation
fi 

# Clone the duckietown/Software repository
if [ -d "software" ]; then
  echo "software/ directory exists, skipping clone of duckietown/Software..";
else
  git clone https://github.com/duckietown/Software software
fi 

# pip install requirements for this class
pip install -r requirements.txt

