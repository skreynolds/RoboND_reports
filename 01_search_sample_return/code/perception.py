import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    
    # 1) Define source and destination points for perspective transform
    img = Rover.img
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # Threshold image for terrain
    nav = color_thresh(warped, (165, 165, 165))
    
    # Threshold image for gold rocks
    hsv_warped = cv2.cvtColor(warped, cv2.COLOR_RGB2HSV)
    lower_thres = np.array([0,120,120])
    upper_thres = np.array([40,255,255])
    rock = cv2.inRange(hsv_warped, lower_thres, upper_thres).astype(bool).astype(int)
    
    # Threshold image for obstacles
    obstacles_temp = color_thresh(warped, (135, 135, 135))
    obstacles_temp ^= 1
    
    # Threshold image for cone
    # Cone derivation
    nav_thresh = color_thresh(img, (165, 165, 165))
    obstacles_thresh = nav_thresh.copy()
    obstacles_thresh ^= 1
    cone = perspect_transform(obstacles_thresh, source, destination)
    cone ^= 1
    cone = np.logical_and(obstacles_temp, cone)
    
    # Thresholded image for obstacles (with cone and rocks removed)
    obstacles = obstacles_temp - cone - rock
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles*255
    Rover.vision_image[:,:,1] = rock*255
    Rover.vision_image[:,:,2] = nav*255
    
    # 5) Convert map image pixel values to rover-centric coords
    # Terrain
    xpix_nav, ypix_nav = rover_coords(nav)
    
    # Obstacles
    xpix_obstacles, ypix_obstacles = rover_coords(obstacles)
    
    # Rocks
    xpix_rock, ypix_rock = rover_coords(rock)
    
    # 6) Convert rover-centric pixel values to world coordinates
    # Define parameters which will transform image
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    world_size = 200
    scale = 10
    
    # Terrain
    nav_x_world, nav_y_world = pix_to_world(xpix_nav, ypix_nav, xpos, ypos, yaw, world_size, scale)
    
    # Obstacles
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obstacles, ypix_obstacles, xpos, ypos, yaw, world_size, scale)
    
    # Rocks
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, yaw, world_size, scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    
    # Add a small amount of colour to obstacles
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 2
    # Reset obstacle channel that rover currently sees as navigable terrain
    Rover.worldmap[nav_y_world, nav_x_world, 0] = 0
    
    # Update any rocks on the rock colour channel
    Rover.worldmap[rock_y_world, rock_x_world, 1] = 255 
    
    # Reset anything on the navigable terrain channel that the rover currently sees as an obstacle
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 2] = 0
    Rover.worldmap[nav_y_world, nav_x_world, 2] = 255
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
    # If the rover detects a rock, and it is not in stuck mode, the Rover will
    # switch to the rocking mode (i.e. the mode used to search for rocks)
    if (sum(sum(rock)) > 20) and Rover.mode != "stuck":
        Rover.mode = "rocking"
        Rover.rock_spot_count = 0 # The rover will reset the rock spot counter
        distances, angles = to_polar_coords(xpix_rock, ypix_rock)
        Rover.rock_dists = distances
        Rover.rock_angles = angles
    # If the Rover loses track of the rock, the Rover still remains in the rocking
    # mode, however, will start to count down to switching back to forward mode
    # provided the Rover cannot relocate the rock
    elif Rover.mode == "rocking":
        Rover.rock_spot_count += 1
        Rover.rock_angles = 0
    
    # The code will always take the updated navigation distances and angles
    # irrespective of whether or not a rock is detected
    distances, angles = to_polar_coords(xpix_nav, ypix_nav)
    Rover.nav_dists = distances
    Rover.nav_angles = angles
    
    return Rover