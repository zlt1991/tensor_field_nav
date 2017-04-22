sensor_msgs/Image 	image  			# texture 
geometry_msgs/Pose 	pose   			# 6DOF pose of the center of the quad to be displayed
float32 			width		    # physical width of the quad in Rviz unit (usually meters) 
float32 			height			# physical height of the quad in Rviz unit (usually meters) 
string 				caption		    # [OPTIONAL] text description to appear below the quad
float32 			border_size     # [OPTIONAL] size of the quad border
float32[] 			border_color    # [OPTIONAL] (r,g,b,a) color value of the quad border (Size = 4) 