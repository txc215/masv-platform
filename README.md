# Simulation and Verification Environment

## System structure and Data flow diagram
```mermaid
graph TD
    A[main.py] --> B[core_algorithms]
    A --> C[motion_models]
    A --> D[simulation_core]
    A --> E[data]
    A --> F[model_viewer]
    A --> G[result_viewer]
    A --> H[ros2_ws]

    subgraph Algorithm Layer
        B --> B1[EKF/UKF]
        B --> B2[RNN/PINN/Transformer]
    end

    subgraph Motion Modeling
        C --> C1[BicycleModel]
        C --> C2[Kinematics Utils]
    end

    subgraph Simulation Layer
        D --> D1[simulator.py]
        D --> C
        D --> B
        D --> E
    end

    subgraph Data Layer
        E --> E1[sample_inputs]
        E --> E2[rosbag_logs]
        E --> E3[sample_outputs]
    end

    subgraph Viewer
        F --> F1[wxPython viewer]
        G --> G1[Flask result viewer]
    end

    subgraph ROS2 Integration
        H --> H1[ros2_interface]
        H --> H2[ai_models_node]
        H --> H3[visualization]
        H --> C
        H --> B
    end

```