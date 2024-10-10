# Romea Core Path Matching Library

This library provides two implementations of path matching algorithms:

1. **"Static" Path Matching**: This implementation is used to follow a predefined, static trajectory. It is ideal for trajectory tracking tasks.
2. **"On the fly" Path Matching**: This implementation builds the path dynamically by considering the successive positions of a leading robot. It is designed for robot following and platooning purposes.

In addition, the library includes monitoring tools to track localization data and map matching results. It can generate diagnostic reports to help detect and troubleshoot potential issues in real-time.

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea_core_path_matching/refs/heads/main/romea_path_matching_public.repos
5. vcs import src < romea_path_matching_public.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to this library, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

This project is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea Core Path Matching library, written by **Jean Laneurit** and **Cyrille Pierre**, was developed during ANR Baudet Rob 2 and ANR Tiara projects. Several individuals contributed scientifically to the development of this library:

**Jean Laneurit**  
**Roland Lenain**  
**Cyrille Pierre**  
**Vincent Rousseau**  
**Benoît Thuilot**    

## **Contact**

If you have any questions or comments about Romea Core Path Matching library, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** or **[Cyrille Pierre](mailto:cyrille.pierre@inrae.fr)**.