# Romea Core Path Matching Library

This library provides two implementations of path matching algorithms:

1. **"Static" Path Matching**: This implementation is used to follow a predefined, static trajectory. It is ideal for trajectory tracking tasks.
2. **"On th fly" Path Matching**: This implementation builds the path dynamically by considering the successive positions of a leading robot. It is designed for robot following and platooning purposes.

In addition, the library includes monitoring tools to track localization data and map matching results. It can generate diagnostic reports to help detect and troubleshoot potential issues in real-time.