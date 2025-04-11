# drone-project
drone navigation project on AirSim simulator 

# Drone Navigation Project

## Overview

This project focuses on developing and testing autonomous drone navigation algorithms within the [AirSim](https://github.com/microsoft/AirSim) simulator. It aims to simulate realistic drone flight scenarios, enabling the evaluation of various path-planning strategies in environments populated with obstacles.

## Features

- **Path Planning Algorithms**: Implementation of multiple algorithms including:
  - A* Search
  - Dijkstra's Algorithm
  - Tangent Bug Algorithm
  - Line-of-Sight Target Guidance (LTG)

- **Obstacle Handling**: Utilizes real-world obstacle data to simulate complex environments, enhancing the realism of navigation scenarios.

- **Modular Architecture**: Designed with modularity in mind, allowing for easy integration of new algorithms and components.

## Project Structure

```
├── main.py              # Entry point for running simulations
├── Agent.py             # Defines the drone agent's behavior and interactions
├── PathPlanner.py       # Core path planning module
├── astar.py             # A* algorithm implementation
├── Dijkstra.py          # Dijkstra algorithm implementation
├── TangentBug.py        # Tangent Bug algorithm
├── LTG.py               # Line-of-Sight Target Guidance
├── Obstacles.py         # Obstacle management
├── Obstacle.py          # Individual obstacle representation
├── MapDrawer.py         # Visualization tools
├── Config.py            # Configuration settings
├── utils.py             # Utility functions
├── *.csv, *.json        # Obstacle/environment data
```

## Getting Started

### Prerequisites

- Python 3.x
- [AirSim Simulator](https://github.com/microsoft/AirSim)
- Required Python packages (listed in `requirements.txt`)

### Installation

```bash
git clone https://github.com/noy-shargal/drone-project.git
cd drone-project
pip install -r requirements.txt
```

### Running a Simulation

1. Ensure AirSim is running and properly configured.
2. Run the main script:
   ```bash
   python main.py
   ```

## Configuration

Adjust simulation parameters in `Config.py`, including:
- Drone speed and behavior
- Obstacle density
- Selected navigation algorithm

## Contributions

Contributions are welcome!

```bash
# Fork & clone the repo
git checkout -b feature/your-feature-name
git commit -m "Add your message here"
git push origin feature/your-feature-name
```

Then open a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [Microsoft AirSim](https://github.com/microsoft/AirSim) for the simulator framework.
- Open-source contributors for tools and inspiration.
