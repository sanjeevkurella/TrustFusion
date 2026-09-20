# TrustFusion

## Adaptive Trust-Aware Physiological Sensor Fusion on STM32F401CCU6

TrustFusion is a physiological sensor fusion system designed to maintain reliable monitoring when individual physiological sensors experience noise, missing data, signal corruption, electrode movement, or temporary sensor failure.

The system combines real physiological signals from hardware sensors with additional physiological datasets. Each sensor is evaluated based on its signal quality, and a dynamic trust value is assigned to determine how reliable the sensor is at a given time.

---

## Problem Statement

Physiological monitoring systems can be affected by:

- Sensor noise
- Electrode movement or disconnection
- Missing samples
- Sensor dropout
- Signal corruption
- Different sampling rates
- Communication packet loss

TrustFusion addresses these problems using signal-quality analysis, dynamic sensor trust, and trust-aware sensor fusion.

---

## System Overview

```text
                PHYSIOLOGICAL SIGNALS
                         |
          +--------------+--------------+
          |                             |
     REAL SENSORS                  DATASETS
          |                             |
     +----+----+                  +-----+-----+
     |         |                  |           |
   AD8232   DS18B20             SpO2     Respiration
     |         |                  |           |
    ECG    Temperature           Dataset     Dataset
     |         |                  |           |
     +----+----+                  +-----+-----+
          |                             |
          +--------------+--------------+
                         |
                         v
                  STM32F401CCU6
                         |
                  Signal Processing
                         |
                         v
                       SQI
              (Signal Quality Index)
                         |
                         v
                  Trust Estimation
                         |
                         v
                 Trust-Aware Fusion
                         |
                         v
              Patient Stability Index
                         |
                +--------+--------+
                |                 |
           Confidence        Fault Status
