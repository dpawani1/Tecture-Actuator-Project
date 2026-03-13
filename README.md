# 🎨 Tecture Actuator Project — Interactive Kinetic Art Control System

**Creative Tech Installation Art — MAE156, UC San Diego**  
**Sponsor:** Tecture Design Studio, San Diego, CA  

**Team:**  
Darsh Pawani · Anthony Mark · Abdulla Zidan · Youngyen Lin  

---

## 📌 Overview

Sponsored by Tecture Design Studio, the system demonstrates how electronically controlled actuators can produce synchronized physical motion in response to environmental sensing and AI vision input.

The prototype consists of a **3×3 grid of linear actuators** housed within a portable enclosure and controlled by a Raspberry Pi–based architecture. The system integrates depth sensing, object/person detection, and programmable motion patterns to enable responsive kinetic artwork.

The design emphasizes:

- Replicability  
- Modularity  
- Scalability    
- Real-time interaction  
- Ease of deployment for artistic applications  

---
## 🎯 Project Goals

- Provide a modular motion platform for kinetic art  
- Enable interactive installations using AI vision  
- Deliver a fully documented, replicable prototype  
- Expand sponsor capabilities into mechanized artwork

---

## ⚙️ System Architecture

**Primary Controller:** Raspberry Pi 5  
**Vision Sensor:** OAK-D Lite Depth AI Camera  
**Microcontroller:** Arduino Uno 
**Actuators:** Actuonix L12 Linear Servos 100:1
**Driver:** PCA9685 16-Channel PWM Driver  
**Communication:** USB Serial (9600 baud)  

### Data Flow

1. Sensors capture environmental input (vision / motion)
2. Raspberry Pi processes AI models and control logic
3. Commands sent via serial to Arduino
4. Arduino drives actuators through PWM driver
5. Physical motion produced in 3×3 grid

---

## 🤖 Interaction Modes

The system supports multiple operating modes:

- 🖐️ AI Palm Tracking  
- 📏 Spatial Depth Interaction  
- 👤 Object / Person Detection  
- 🚶 PIR Motion Trigger  
- 🎛️ GUI Manual Control  
- 🔁 Autonomous Motion Patterns  


---

## 💻 Software Stack

- Python 3 (Raspberry Pi OS)
- DepthAI SDK
- OpenCV
- NumPy
- PySerial
- Custom GUI application
- Arduino firmware (C++)

All Python dependencies are listed in:

requirements.txt

---

## 📂 Repository Structure

Tecture-Actuator-Project/
│
├── wall_gui/              # Main GUI and control application (Pi)
├── raspberrypi_test_code/     # Folder containing testing of the vision processing and control scripts
├── assorted_arduino_test_codes/  # Folder containing assorted test codes used in development of the arduino/actuator firmware (.ino)
├── requirements.txt       # Python dependencies
├── README.md              # Project documentation
├── docs/                  # Setup instructions & system architecture

---

## ▶️ Setup and Installation

1. Clone the repository:

git clone https://github.com/dpawani1/Tecture-Actuator-Project  
cd Tecture-Actuator-Project  

2. Install Python dependencies:

pip install -r requirements.txt  

3. Upload the Arduino firmware using the Arduino IDE

4. Connect hardware components (additional information in the docs folder)


---

## ▶️ Running the System

python wall_gui/gui.py

---

## 📖 Documentation

Additional documents include full Raspberry Pi setup, wiring, system architecture, and replication guidelines.

---

## 📊 Performance Highlights

- Visible actuator stroke ≈ 95 mm  
- Load capacity ≈ 22 N per actuator  
- Acoustic output ≈ 50 dB  
- Portable footprint ≈ 46 cm × 46 cm  


---

## 🧠 AI Model Credits

### ✋ Palm Detection Model

MediaPipe Palm Detection (palm_detection_128x128)  
Source: Luxonis DepthAI Model Zoo  
Original Framework: Google MediaPipe  

DepthAI Model Zoo:  
https://github.com/luxonis/depthai-model-zoo  

MediaPipe:  
https://github.com/google/mediapipe  

### 👤 Object / Person Detection Model

MobileNet‑SSD (Spatial Detection Network)  
Source: Luxonis DepthAI Model Zoo  

---

## 🏢 Sponsor

Tecture Design Studio — San Diego, California  

---

## 👨‍🎓 Course Information

MAE156 — Creative Tech Installation Art  
University of California, San Diego  

---

## 📄 License

Provided for academic and demonstration purposes.
For commercial or reproduction inquiries, contact the sponsor.

