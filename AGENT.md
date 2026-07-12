# Agent Directives

1. **Always Update README**: Whenever adding new features, scripts, nodes, or modifying the architecture, the `README.md` file must be updated to reflect how to run and test these new components.
2. **ROS 2 Standard Best Practices**: Ensure the use of standard ROS 2 message types (`sensor_msgs/msg/Imu`), actions, and nodes as specified in the project plan.
3. **C++ & Eigen**: Numerical computations should be implemented in C++ using the Eigen library.
4. **Documentation**: Keep code well-documented with comments and clear explanations.
5. **Version Control**: Always split implementations into small, logical commits. Commit messages must strictly follow the Linux kernel style format (e.g., `subsystem: Short descriptive summary (50 chars)` followed by a blank line and a detailed description wrapped at 72 characters explaining the 'why' and 'how').
