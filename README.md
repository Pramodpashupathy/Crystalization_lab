# Crystalization Lab – UR RTDE Control (C++)

This repository contains C++ code for controlling a **Universal Robots (UR)** manipulator using **RTDE (Real-Time Data Exchange)**, along with vision-based components (OpenCV) and calibration utilities.

The purpose of this README is to document **how to set up and build this project on a new system**.

---

## 1. System Requirements

- Ubuntu 18.04 / 20.04 / 22.04 (recommended)
- C++ compiler (GCC ≥ 9)
- CMake ≥ 3.16
- Universal Robots controller with:
  - RTDE enabled
  - Remote Control mode enabled
  - Network access to robot IP

---

## 2. Required Dependencies

### 2.1 System build tools
Install basic build dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  libopencv-dev \
  libeigen3-dev
