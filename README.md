# FBX2md5
---------

FBX to idTech4.x md5mesh / md5anim CLI converter

FIXME: this doesn't quite work for complex characters with deep hierarchy. It only works for 1- or 2- bones skeletons.

Tested with official Autodesk FBX SDK 2020.3.7

# Setup:
1. Install Cmake 4.x
2. Install MSVC2022
3. Install FBX SDK into default location
4. Clone this repo into a folder
5. Run .bat file to generate VS solution

# How to use (CLI):

fbx2md5.exe <your_fbx_file_name>.fbx -md5v10
