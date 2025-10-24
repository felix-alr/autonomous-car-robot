HSAR Robot           {#mainpage}
==========

## About this Documentation

This documentation aims to explain the example project for the HSAR seminar.
The code is meant to be continuously expanded in order to fulfill all tasks of the robot, but you can modify everything as you seem fit to meet the requirements.
To help with getting development started, the principal structure of the codebase has been laid out and
some functionality, like Bluetooth communication, is sufficiently implemented.

Documentation of code components is compiled directly from the source code with doxygen.

## Structure

Generally there is a Python module (file) for each of the project modules Guidance, Navigation, Perception and Control. Code for the Android-HMI is a separate project.
The main.py file is the entry point of the program and gets automatically loaded and executed when the robot gets powered or reset, the remaining files are support functions or parameters.


