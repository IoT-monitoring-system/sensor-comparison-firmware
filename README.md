# About this repository

This repository contains the firmware developed for the **Gas Sensor Evaluation Prototype** — a dedicated setup to compare and evaluate the performance of different VOC sensors under controlled exposure conditions.

## Repository structure
- **Components/** – Shared libraries and reusable components used by the evaluation firmware.
- **Main/** – Core application modules which implement the sensor test logic and data management.

## Features
- **VOC Sensor Evaluation** – Integrates and compares different volatile organic compound (VOC) sensors:
  - **Bosch BME688**
    - Heating profiles can be configured by supplying a `.bmeconfig` file in `main/littlefs/`
  - **Bosch BME690**
    - Heating profiles can also be configured by supplying a `.bmeconfig` file in `main/littlefs/`
  - **Figaro TGS2602** (sampled via an **ADS1115 ADC**)
- **Data Transmission** – Sends collected data using **MQTT over Wi-Fi Access Point**.
  - The MQTT client requires `caRoot.pem`, `client.crt`, and `client.key` for mutual TLS (mTLS).
