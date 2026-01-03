# SpiderBot - Quadruped Robot Controller

A quadruped (4-legged) crawling robot with 12 servo motors running on Arduino Pro 5V 16MHz. This project implements inverse kinematics and coordinated leg movements to move the whole bot.

## 🤖 Overview

This robot features:

- **4 legs** with **3 servos each** (coxa, femur, tibia)
- **Inverse kinematics** for natural movement
- **Multiple gaits**: walking forward/backward, turning, sitting/standing
- **Demo movements**: hand waving, hand shaking, body dancing
- Real-time servo interpolation at 50Hz for smooth motion

![the_bot.jpg]

## 🛠️ Hardware

- **Microcontroller**: Arduino Pro 5V 16MHz (ATmega328P)
- **Servos**: 12x standard hobby servos
- **Power**: External 12.6V Li-Pol battery with DC converter
- **Communication**: Bluetooth module (optional & not yet implemented in code)

## Software

- PlatformIO or Arduino IDE
- Libraries:
  - `Arduino.h`
  - `Servo.h`

## 🚀 Getting Started

1. **Clone the repository**

   ```bash
   git clone https://github.com/davidstrasak/SpiderBot.git
   cd SpiderBot/SpiderBot_Code
   ```

2. **Calibration** (Important!)

   - Before first use, calibrate the robot for accurate movement
   - See calibration instructions in the code header

3. **Build and upload**

   ```bash
   platformio run --target upload
   ```

## Movement Functions

- `stand()` / `sit()` - Basic postures
- `step_forward()` / `step_back()` - Walking gaits
- `turn_left()` / `turn_right()` - Rotation
- `body_left()` / `body_right()` - Weight shifting
- `hand_wave()` / `hand_shake()` - Demonstration movements
- `body_dance()` - Fun dance routine

## 📚 Learning Experience

This project was a learning experience in:

- Embedded systems programming
- Inverse kinematics implementation
- Real-time control systems
- Hardware-software integration
- Multi-servo coordination

## 🔧 Configuration

Key parameters can be adjusted in the code:

- Leg dimensions (`length_a`, `length_b`, `length_c`)
- Movement speeds (`leg_move_speed`, `body_move_speed`, etc.)
- Servo pin assignments
- Default positions and step sizes

## 📝 License

This project builds upon open-source robotics code. See the source file headers for attribution details.

## ⚠️ Notes

- Ensure adequate battery and DC converter for all 12 servos
- Print a good and tight enclosure for best results
- Consider upgrading to an STM board or ESP32 for more complex features
- The RTOS implementation is experimental on ATmega328P due to limited RAM (2KB)

---

**Built with**: PlatformIO • Arduino • C++ • Love for Embedded Systems 🦿
# SpiderBot_Code
