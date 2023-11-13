# Drone-based Volcanic Plume Monitoring

## Overview

Welcome to the repository for the drone-based volcanic plume monitoring project. This project is the result of my master's thesis that explores the integration of machine vision and path planning techniques to enhance the capabilities of drones in monitoring volcanic activities, specifically targeting plume interceptions at Volcano Fuego in Guatemala.

![Fuego](https://theworldtravelguy.com/wp-content/uploads/2022/12/DSCF8237_1200-3.jpg)


## Table of Contents

1. [Introduction](#introduction)
2. [Machine Vision](#machine-vision)
    - [Plume Detection Model](#plume-detection-model)
    - [Object Detection Model](#object-detection-model)
    - [Training Data](#training-data)
3. [Path Planning](#path-planning)
    - [Simulation Bias](#simulation-bias)
    - [Trajectory Generation](#trajectory-generation)
    - [Integration with Machine Vision](#integration-with-machine-vision)
4. [Limitations](#limitations)
5. [Contributions](#contributions)
6. [Future Work](#future-work)
    - [Hardware Optimization](#hardware-optimization)
    - [Model Refinement](#model-refinement)
    - [Real-time Integration](#real-time-integration)
7. [Getting Started](#getting-started)
    - [Dependencies](#dependencies)
    - [Installation](#installation)
    - [Usage](#usage)
8. [License](#license)
9. [Acknowledgments](#acknowledgments)

## 1. Introduction

The primary goal of this project is to advance the capabilities of drone-based volcanic monitoring, particularly in the context of Fuego. By combining machine vision for plume detection and innovative path planning, the project aims to provide a robust system for autonomous plume sampling.

## 2. Machine Vision

### Plume Detection Model

The plume detection model is designed to identify and locate volcanic plumes in the drone's field of view. It has been trained using a convolutional neural network, leveraging a dataset that includes various levels of cloud cover. However, the model's effectiveness could be further improved by incorporating infrared images into the training data.

### Object Detection Model

For general environmental awareness, an object detection model based on YOLOv8 has been implemented. Trained on images captured in 2019 around Fuego, this model aids the drone in navigating its surroundings.

### Training Data

The labeled databases, comprising Object Detection and Classification datasets, are valuable resources that can be used for other machine vision projects. However, it's important to note that the training data represents a specific season, and future work could involve expanding the database to include diverse environmental conditions.

<iframe width="560" height="315" src="https://www.youtube.com/embed/pSGYUPancfA" frameborder="0" allowfullscreen></iframe>


## 3. Path Planning

The path planning component addresses the trajectory generation for the drone during plume interception missions. The simulation, facilitated by ArduPilot, introduces potential biases that might not precisely represent real-world wind impacts on the drone. The proposed trajectory generation method assumes constant wind conditions throughout the flight, which might pose challenges in dynamic meteorological scenarios.

### Simulation Bias

To validate the accuracy of the simulation, real flight tests are recommended. Additionally, an alternative approach involving real-time updates to waypoints using the Extended Kalman Filter (EKF) is proposed to enhance the system's robustness against changing wind conditions.

### Integration with Machine Vision

While the plume detection method shows promise, the current trajectory generation method does not leverage the full potential of machine vision. Integration possibilities include using plume detection as input for a servo-controlled system to guide the drone to interception. However, challenges, such as the need for a second camera to estimate plume distance, should be considered.

## 4. Limitations

- The machine vision models are trained on a dataset representing specific seasonal conditions at Fuego.
- The trajectory generation method assumes constant wind conditions, which may not hold true in real-world scenarios.

## 5. Contributions

This project has led to the creation of labeled databases, machine vision models, and algorithms to facilitate the processing of flight data. The contributions include:

- Labeled databases for Object Detection and Classification.
- Machine vision models for plume detection and general object detection.
- Algorithms for processing flight data, including displaying logs on videos and maps.
- Waypoint generation for volcanic monitoring missions using Python and the MAVLink protocol.

## 6. Future Work

### 6.1 Hardware Optimization

Determining the suitable hardware for running machine vision models on the drone's embedded electronics is a crucial next step. The goal is to maximize detection frequency while minimizing weight.

### 6.2 Model Refinement

Further work on the plume detection model involves considering clouds in the training process to address false positives. Implementing a filtration mechanism for predictions, especially when the model generates multiple bounding boxes for a single plume, is another avenue for improvement.

### 6.3 Real-time Integration

Investigate the possibility of generating a control law based on data extracted by machine vision algorithms in real-time. This includes exploring the integration of machine vision algorithms into the drone's control loop for dynamic response to changing environmental conditions.

## 7. Getting Started

### 7.1 Dependencies

List the dependencies, including libraries, frameworks, and tools, necessary for running the code and simulations.

### 7.2 Installation

Provide step-by-step instructions for installing the required dependencies.

### 7.3 Usage

Guide users through the process of running the code, simulations, and experiments.

## 8. License

This project is licensed under the [MIT License](LICENSE).



