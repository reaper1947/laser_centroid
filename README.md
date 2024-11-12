# laser_midpoint
Use ROS2 Humble 

Lidar: SLAMTEC LPX-T1

i'm use intensities from reflection sticker to detect 2 point.

then i have to convert intensities to TF.

then group all in area that i call proximity_threshold.

You can adjust intensity_threshold to your intensitie and proximity_threshold your area or you need to use laser_filters to filters intensities before use this repo.

then determine middle each group then determine midpoint and public TF.


# HOPE THIS REPO IS HELPFUL THANK YOU !!!


# With TF
![image](https://github.com/user-attachments/assets/59f67bed-6f9e-4d15-bae1-4284efcae5b0)

# With Nav2
![image](https://github.com/user-attachments/assets/48e78ebb-3f1e-46bd-9f1c-e7c6c206b142)

# With MarkerArray
![image](https://github.com/user-attachments/assets/8dc258a0-c717-4372-94cd-c85195b4b939)

# Define diameter from Group1 to Group2
![image](https://github.com/user-attachments/assets/e1e7fd45-3e08-42cb-9962-ac159d9bbab6)

