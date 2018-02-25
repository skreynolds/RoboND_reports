import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    print(Rover.mode)
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # If the Rover is no longer stuck, then 
            if abs(Rover.vel) > 0.08:
                Rover.vel_count = 0 # Reset the stuck counter
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    # If the rover is stuck
                    if abs(Rover.vel) < 0.05:
                        # Some hysteresis so the robot doesn't switch modes rapidly
                        Rover.vel_count += 1
                        if Rover.vel_count > 400:
                            Rover.vel_count = 0 # Reset the zero velocity count
                            Rover.stuck_count = 0 # Start the stuck counter
                            Rover.throttle = 0
                            Rover.brake = 0
                            Rover.steer = 0
                            Rover.mode = 'stuck'
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                
                # Set steering to average angle clipped to the range +/- 15
                # Note that the steering angle has been smoothed by using a
                # two time period weighted average
                Rover.steer = (2*Rover.steer + np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15))/3
                
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
            
        # If the rover is in the vicinity of a rock - this is set in perception 
        elif Rover.mode == 'rocking':
            if abs(Rover.vel) > 0.08:
                Rover.vel_count = 0 # Reset the stuck counter
            if Rover.rock_spot_count > 200:
                Rover.mode = "forward"
            else:
                if Rover.vel < 0.5:
                    Rover.throttle = 0.1
                    Rover.brake = 0
                    if abs(Rover.vel) < 0.05:
                        # Some hysteresis so the robot doesn't switch modes rapidly
                        Rover.vel_count += 1
                        if Rover.vel_count > 400:
                            Rover.vel_count = 0 # Reset the zero velocity count
                            Rover.stuck_count = 0 # Start the stuck counter
                            Rover.throttle = 0
                            Rover.brake = 0
                            Rover.steer = 0
                            Rover.mode = 'stuck'
                elif Rover.vel >= 0.5:
                    Rover.throttle = 0
                
                # This will direct the rover towards the rock
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                
                # This will stop the Rover if it is near a rock sample
                # Note this is a condition for picking up samples
                if Rover.near_sample:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
        
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
        
        # This is the Rover stuck mode - that is if the wheels are caught on the terrain        
        elif Rover.mode == 'stuck':
            Rover.stuck_count += 1 # Add a count to the hysteresis
            Rover.steer = 0
            Rover.throttle = -0.1 # This will reverse the rover with 0 steer angle
            Rover.brake = 0
            
            # This behaviour, once triggered only needs to occur for 300 processed frames
            # which is less than 10 seconds
            if Rover.stuck_count > 300:
                Rover.stuck_count = 0 # Reset the stuck counter
                Rover.steer = 0 # Reset the Rover steering
                Rover.mode = 'forward' # Hopefully the rover is unstuck
                
                    
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

