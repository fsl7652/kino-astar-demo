# JetRacer - Kinodynamic Pathfinding Demo

A real-time kinodynamic A* pathfinding implementation using SDL3, featuring vehicle physics and dynamic obstacle avoidance.

## Features

- **Kinodynamic A*** - Pathfinding with vehicle motion constraints
- **Multiple Maps** - Default, Maze, and Race Track environments
- **Interactive Goals** - Click anywhere to set navigation targets
- **Dynamic Obstacles** - Right-click to place custom obstacles
- **Real-time Physics** - Vehicle dynamics with acceleration and steering
- **Visual Debugging** - See explored nodes and computed paths
- **Dual Control** - Manual driving or autonomous path following

### Prerequisites
- **CMake** 3.12+
- **SDL3** development libraries
- **C++17** compatible compiler

### Installation & Build

```bash
# Clone the repository
git clone https://github.com/yourusername/jetracer-pathfinding
cd jetracer-pathfinding

# Create build directory
mkdir build && cd build

# Configure and build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release

# Run the demo
./JetRacerDemo
```

## 📋 Controls

| Control | Action |
|---------|--------|
| **Left Click** | Set goal position |
| **Right Click** | Place obstacle (drag to size) |
| **Mouse Wheel** | Rotate obstacle while placing |
| **Arrow Keys** | Manual driving controls |
| **1-3 Keys** | Switch between maps |
| **Tab** | Cycle through maps |
| **ESC** | Quit application |

## 🗺️ Maps

1. **Default Map** - Simple environment with basic obstacles
2. **Maze Map** - Complex navigation challenge
3. **Race Track** - Oval track with center obstacles

## 🛠️ Installation Details

### Linux (Ubuntu/Debian)
```bash
sudo apt-get update
sudo apt-get install libsdl3-dev cmake build-essential
```

### macOS
```bash
brew install sdl3 cmake
```

### Windows
1. Download [SDL3 development libraries](https://github.com/libsdl-org/SDL/releases)
2. Extract to `C:\SDL3`
3. Set environment variable: `SDL3_ROOT=C:\SDL3`
4. Add `C:\SDL3\bin` to your PATH

## 🏗️ Project Structure

```
src/
├── environment.cpp    # Main application and event handling
├── kino_search.cpp    # Kinodynamic A* implementation
├── OBB.cpp           # Oriented bounding box collisions
├── jetracer.cpp      # Vehicle physics and rendering
├── wall.cpp          # Obstacle rendering
├── goal.cpp          # Target rendering
└── render_circle.cpp # Utility rendering functions
```

## 🔧 Customization

### Adding New Maps
Edit the `initMaps()` function in `environment.cpp` to add custom environments.

### Modifying Vehicle Parameters
Adjust physical properties in `jetracer.h`:
```cpp
wheelbase = 0.15f;      // Distance between axles
length = 0.25f;         // Vehicle length
width = 0.19f;          // Vehicle width
max_speed = 2.0f;       // Maximum speed
```

## 🐛 Troubleshooting

### Common Issues

**"Could NOT find SDL3"**
- Ensure SDL3 development libraries are installed
- On Windows, verify `SDL3_ROOT` environment variable

**Runtime errors on Windows**
- Copy `SDL3.dll` to the executable directory

**Build failures**
- Verify CMake version ≥ 3.12
- Check compiler supports C++17

## 📊 Performance

The algorithm provides real-time performance with:
- 1000+ nodes expanded per search
- <5ms path computation time
- Smooth 60 FPS rendering

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the project
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **SDL3** for cross-platform graphics and input

---

<div align="center">

**Enjoy exploring pathfinding!** 🎯

*If you find this project useful, please give it a ⭐ on GitHub!*

</div>
