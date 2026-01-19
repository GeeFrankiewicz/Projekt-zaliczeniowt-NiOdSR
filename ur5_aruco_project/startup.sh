#!/bin/bash

# 1. Budowanie kodu
echo "--- BUILDING WORKSPACE ---"
colcon build

# 2. Odświeżenie środowiska
source install/setup.bash

# 3. Uruchomienie symulacji
echo "--- LAUNCHING SYSTEM ---"
ros2 launch ur_control system.launch.py