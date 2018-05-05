def pr2_mover(object_list):

    # TODO: Initialize variables
    test_scene_num = Int32()
    arm_name = String()
    object_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    labels = []
    centriods = []
    dict_list = []

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables
    for detected_object in object_list:
        labels.append(detected_object.label)
        points_arr = ros_to_pcl(detected_object.cloud).to_array()
        centriods.append(np.mean(points_arr, axis=0)[:3])

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for i in range(0, len(object_list_param)):

        # Specify variables to be fed into the yaml dict

        # Specify the test scene
        test_scene_num.data = 1

        # Specify the arm name for the object (note that the arm name
        # is dependent on the clour assigned to the arm)
        if object_list_param[i]['group'] == 'red':
            arm_name.data = 'left'
        else:
            arm_name.data = 'right'

        # Specify the object name
        object_name.data = str(object_list_param[i]['name'])

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        if object_list_param[i]['name'] in labels:
        	index = labels.index(object_list_param[i]['name'])
        	centroid = centriods[index]
        else:
        	centroid = (0,0,0)

        # assign the pick_pose
        pick_pose.position.x = float(centroid[0])
        pick_pose.position.y = float(centroid[1])
        pick_pose.position.z = float(centroid[2])

        # TODO: Create 'place_pose' for the object
        if arm_name.data == 'left':
        	place_pose.position.x = float(dropbox_param[0]['position'][0])
        	place_pose.position.y = float(dropbox_param[0]['position'][1])
        	place_pose.position.z = float(dropbox_param[0]['position'][2])
        else:
        	place_pose.position.x = float(dropbox_param[1]['position'][0])
        	place_pose.position.y = float(dropbox_param[1]['position'][1])
        	place_pose.position.z = float(dropbox_param[1]['position'][2])

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num,
                                   arm_name,
                                   object_name,
                                   pick_pose,
                                   place_pose)
        
        dict_list.append(yaml_dict)
        '''
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
		'''
    # TODO: Output your request parameters into output yaml file
    #print(dict_list) # Uncomment to see the dictionary list that is being created
    send_to_yaml('yaml_out_1',dict_list)