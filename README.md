# Robotics_ATHENS_UPM

Code developed during ATHENS course Robotics Applications with ROS2: From Basics to Integration, held in march 2025 at Universidad Politecnica de Madrid (UPM).

Hardware:
- Microcontroller (Nucleo-STM32-F466RE)
- Motor Driver (X-NUCLEO-IHM04A1)
- Brushed DC Motor (Pololu 25mm Metal Gearmotor LP)
- Encoder (Magnetic Encoder: 12 pulses per revolution)
- Inertial Measurement Unit (MPU6050)

Architecture:
- pubsub node: handles the serial connection with the microcontroller. Receives control messages from the ros2 environment and publish the motor state. Can be set as a master or a slave;
- control node: receives the state of a master-node (used as a setpoint) and computes the control signals to control a slave-node on the same network to replicate the movement of the master. It implements parameters and services to handle the namespaces and set the master and the slave.
