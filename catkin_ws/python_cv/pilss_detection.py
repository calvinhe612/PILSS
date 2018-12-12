# USAGE
# python pilss_tracker.py

# import CentroidTracker to keep track of all unique pills
from pyimagesearch.centroidtracker import CentroidTracker

import numpy as np
import cv2
import sys
import math

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray

from sklearn.cluster import KMeans
from clustering.equal_groups import EqualGroupsKMeans

from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform
from shapely.geometry import Point


# Instantiate CvBridge
bridge = CvBridge()

# initialize our centroid tracker and frame dimensions
ct = CentroidTracker()

# Current Gazebo world file contains 12 pills
num_clusters = 4
NUM_PILLS = 12
desired_num_pills = NUM_PILLS
pills_per_cluster = NUM_PILLS/num_clusters

# Global variable to keep track of current paths and clusters
path = None
center_points = None
kmeans_clf = None
kmeans_space = None
current_center = None
current_radius = None
current_centroids_num  = 0

# Allows system to wait for motion to complete before detecting/clustering again
motion_completed = True

#CONTOUR_THRESH = 800
CONTOUR_THRESH = 1000

# pixel per metric ratio to convert from pixel-space to robot space
PIXEL_METRIC_RATIO = 322/0.5

# Defining the largest range and workspace manipulator can reacg to
#ROBOT_RANGE = 0.9
ROBOT_RANGE = 0.7
ROBOT_CENTER = Point(800,800)
ROBOT_WORKSPACE = ROBOT_CENTER.buffer(ROBOT_RANGE*PIXEL_METRIC_RATIO).boundary

# Defining circular boundary as destination area
destination_point_pixels = (478,1122)
destination_point_meters = (-0.5,-0.5)
destination_radius_pixels = 161
destination_radius_meters = 0.25

# Global variables to help determine when manipulator needs to take care of extra 
# remaining pills in the environment
notAdjusted = True
needToDecrease = True

# ROS publisher to publish path points to MoveIt!
moveit_publisher = rospy.Publisher('/rrbot/moveLocation', Float32MultiArray, queue_size=10)



# Image call back to process image feed for pill detection
def image_callback(msg):
#    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
	cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        pill_detection(cv2_img)



# Callback when motion planning has finished executing and need to reset all global variables
# before next computer vision and motion planning execution
def completed_callback(msg):
    print "Inside completed motion callback"
    global motion_completed, path, center_points, num_clusters
    motion_completed = True
    path = None
    center_points = None
    num_clusters = num_clusters - 1
    print "Num clusters inside complete:", num_clusters
    
    global current_center, current_radius
    #current_center = None
    current_radius = None
    
    global needToDecrease
    if needToDecrease:
        global desired_num_pills
        desired_num_pills = desired_num_pills - pills_per_cluster

    global notAdjusted
    notAdjusted = True



# Apply thresholding plus contouring to the white colored objectd
def pill_detection(image):
    ## PILL DETECTION ###################################################################
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    retval, threshold = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
    im2, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    for cnt in contours:
        
        startX, startY, width, height = cv2.boundingRect(cnt)
        endX = startX + width
        endY = startY + height
        
        centerX = startX + (width/2)
        centerY = startY + (height/2)
        
        # If pills are within destination area, they will not be detected
        distance_to_destination = math.sqrt(math.pow((centerX-destination_point_pixels[0]),2)+math.pow((centerY-destination_point_pixels[1]),2))
        if distance_to_destination <= destination_radius_pixels:
            continue

        # Bounding box of detection
        box = np.array([startX, startY, endX, endY])
        rects.append(box.astype("int"))
        cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 3)

    # Update our centroid tracker using the computed set of bounding box rectangles
    objects = ct.update(rects)

    centroids = []
    # Loop over the tracked objects
    for (objectID, centroid) in objects.items():
        #print "ObjectID =", objectID, "Centroid =", centroid
        centroids.append(centroid)

        # Draw both the ID of the object and the centroid of the object on the output frame
        #text = "ID {}".format(objectID)
        #cv2.putText(image, text, (centroid[0] - 10, centroid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.circle(image, (centroid[0], centroid[1]), 5, (255, 0, 0), -1)

    centroids = np.asarray(centroids)
    global current_centroids_num
    current_centroids_num = len(centroids)


    ## CLUSTERING PILLS #################################################################
    if motion_completed:
        
        # The following logic decides whether to use Equal Groups K-Means when pills can be divided
        # into clusters evenly, or use standard K-Means when there are extra remaining pills detected
        # that need to be pushed into destination first
        if current_centroids_num > desired_num_pills:
            #print "Checking pill number to desired"
            global notAdjusted
            if notAdjusted:
                global num_clusters
                num_clusters = num_clusters + 1
                print "Anomaly, current_centroids_num:", current_centroids_num, ", desired:", desired_num_pills, ", num_clusters:", num_clusters
                notAdjusted = False
                global needToDecrease
                needToDecrease = False
         

            global num_clusters
            if num_clusters == 0:
                print"Inside pill detection, num_clusters =", num_clusters
                # Need a more robust system exiting when arm completely finishes 
                sys.exit("No more clusters to detect and move")

            print "Standard k means, num_clusters:", num_clusters
            clf = KMeans(n_clusters=num_clusters)
            space = clf.fit_transform(centroids)
            
            global kmeans_clf, kmeans_space
            kmeans_clf = clf
            kmeans_space = space
         
        else:
            global num_clusters
            if num_clusters == 0:
                print"Inside pill detection, num_clusters =", num_clusters
                sys.exit("No more clusters to detect and move")

            print "Calculating size same clusters, num_clusters:", num_clusters
            #num_clusters = 2
            clf = EqualGroupsKMeans(n_clusters=num_clusters)
            space = clf.fit_transform(centroids)
            
            global kmeans_clf, kmeans_space
            kmeans_clf = clf
            kmeans_space = space    
            
            global needToDecrease
            needToDecrease = True
            
            
        center_distances = squareform(pdist(clf.cluster_centers_))
        np.fill_diagonal(center_distances, np.inf)
        #closest_index = center_distances.argmin(axis=0)[0]
        #closest_distance = center_distances[0][closest_index]
    
        # All cluster centers
        cluster_centers = clf.cluster_centers_
        
        # Leftmost clusters
        first_label = cluster_centers[:,0].argmin()
        first_center = cluster_centers[first_label,:]
        
        # Upmost clusters
#        first_label = cluster_centers[:,1].argmin()
#        first_center = cluster_centers[first_label,:]

        # Keeping track of current center
        global current_center
        current_center = first_center
        
        distances = space[:,first_label]
        point_distances = distances[np.where(clf.labels_ == first_label)]
        largest_radius = point_distances.max()
        global current_radius
        current_radius = largest_radius
    

    # Draw all centers of each cluster    
    for center in kmeans_clf.cluster_centers_:
        cv2.circle(image, (int(center[0]), int(center[1])), 10, (0, 0, 0), -1)
        
    # Draw circles of pills belonging to their respective cluster
    for i in range(kmeans_clf.labels_.shape[0]):
        label = kmeans_clf.labels_[i]
        distance = kmeans_space[i][label]
        color = (127,127,127)
        cv2.circle(image, (int(kmeans_clf.cluster_centers_[label][0]), int(kmeans_clf.cluster_centers_[label][1])), int(distance), color, 5)
    


    ## STRAIGHT LINE PATH PLANNING ######################################################
    # Getting vector from current cluster to destination center and determining starting point
    vector = (current_center[0] - destination_point_pixels[0], current_center[1] - destination_point_pixels[1])
    vector_norm = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1],2))
    vector_normalized = tuple([x/vector_norm for x in vector])
    vector_normalized_orthogonal = (-1*vector_normalized[1], vector_normalized[0])
    
    additional_offset_pixels = 15
    largest_distance = current_radius
    extended_vector = tuple([x*(largest_distance+additional_offset_pixels) for x in vector_normalized])
    
    new_path_start = current_center.copy()
    new_path_start = (current_center[0]+extended_vector[0], current_center[1]+extended_vector[1])    
    
    # Allow offset to keep blade centered along path
    blade_measurement_pixels = 161
    #blade_measurement_meters = 0.25
    blade_offset_vector = tuple([x*(blade_measurement_pixels/2) for x in vector_normalized_orthogonal])
    new_start_point_blade_offset = (int(new_path_start[0]-blade_offset_vector[0]), int(new_path_start[1]-blade_offset_vector[1]))
    destination_blade_offset = (int(destination_point_pixels[0]-blade_offset_vector[0]),int(destination_point_pixels[1]-blade_offset_vector[1]))    
    
    # Setting global variable for current path
    global path
    if path == None:
        path = (new_path_start, destination_point_pixels)
    else:
        # Creating Float32MultiArray for ROS topic
        mat = Float32MultiArray()
#            mat.layout.dim.append(MultiArrayDimension())
#            mat.layout.dim.append(MultiArrayDimension())
#            mat.layout.dim[0].label = "height"
#            mat.layout.dim[1].label = "width"
#            mat.layout.dim[0].size = 1
#            mat.layout.dim[1].size = 6
#            mat.layout.dim[0].stride = 1*1*6
#            mat.layout.dim[1].stride = 1*1
#            mat.layout.data_offset = 0
        
        # Path points in pixel-space
        pixel_points = [new_start_point_blade_offset[0], new_start_point_blade_offset[1], 0.2, destination_blade_offset[0], destination_blade_offset[1], 0.0]

        # Calibration to transform from pixel space to robot space
        meter_points = []
        for i in range(0,len(pixel_points)):
            coord = pixel_points[i]
            if i%3 == 0:
                meter_points.append((coord-800)/PIXEL_METRIC_RATIO)
            elif i%3 == 1:
                meter_points.append((800-coord)/PIXEL_METRIC_RATIO)
            else:
                meter_points.append(coord)
                
        
        ## ROS PUBLISHING ###############################################################
        mat.data = meter_points            
        global motion_completed
        if motion_completed:            
            moveit_publisher.publish(mat)
            print "Published to moveLocation:", mat.data
            global motion_completed
            # Wait until motion is completed before motion planning calls the completed_callback function
            motion_completed = False
            
            

    # Drawing functions on the camera view to visualize detections, clusterings, path lines, and text
    path_start = path[0]
    path_end = path[1]
    path_start = (int(path_start[0]),int(path_start[1]))
    path_end = (int(path_end[0]),int(path_end[1]))        
    cv2.line(image, path_start, path_end, (0,0,255), 10)  

    cv2.circle(image, destination_point_pixels, 10, (255,0,255), -1)
    cv2.circle(image, (int(current_center[0]),int(current_center[1])), int(current_radius), (0,255,255), 5)
    cv2.circle(image, destination_point_pixels, destination_radius_pixels, (255,0,255), 5)
    
    cv2.circle(image, new_start_point_blade_offset, 10, (255,0,255), -1)
    cv2.line(image, new_start_point_blade_offset, destination_blade_offset, (255,0,255), 10)
    
    pill_text =    "Number of detected pills:   %d"%current_centroids_num
    cv2.putText(image, pill_text, (100, 125), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2)
    cluster_text = "Number of clusters to sort: %d"%num_clusters
    cv2.putText(image, cluster_text, (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2)
    desired_text = "Desired pills left to sort: %d"%desired_num_pills
    cv2.putText(image, desired_text, (100, 275), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2)
    counted_text = "Pill count: %d"%(NUM_PILLS-current_centroids_num)    
    cv2.putText(image, counted_text, (destination_point_pixels[0]-150, destination_point_pixels[1]-180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 2)
    
    
    # Showing all detection, clusters, and paths
    cv2.imshow("Frame", cv2.resize(image, (0,0), fx=0.5, fy=0.5))
    #cv2.imshow("Threshold", cv2.resize(threshold, (0,0), fx=0.5, fy=0.5))
    
    # Allows image to key updating in cv2.imshow()
    key = cv2.waitKey(1) & 0xFF




# Main method
if __name__=="__main__":
    # Start ROS node
    rospy.init_node('PILSS_Detection')

    # Image topic in Gazebo   
    image_topic = "/rrbot/camera3/image_raw"
    # Topic to indicate when motion planning execution finishes
    completed_topic = "/rrbot/motionComplete"
    
    # Intializing subscribers with its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(completed_topic, Float64, completed_callback)
    
    # Spin until Ctrl + C
    rospy.spin()
