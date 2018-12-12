# USAGE
# python object_tracker.py --prototxt deploy.prototxt --model res10_300x300_ssd_iter_140000.caffemodel

# import the necessary packages
from pyimagesearch.centroidtracker import CentroidTracker
#from imutils.video import VideoStream
import numpy as np
import cv2
import sys

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float64
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

from sklearn.cluster import KMeans
from clustering.equal_groups import EqualGroupsKMeans

from sklearn import svm
from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform
from shapely.geometry import LineString
from shapely.geometry import Point
from shapely.geometry import MultiPoint

import math

# Instantiate CvBridge
bridge = CvBridge()

# initialize our centroid tracker and frame dimensions
ct = CentroidTracker()

num_clusters = 4
NUM_PILLS = 12
desired_num_pills = NUM_PILLS
pills_per_cluster = NUM_PILLS/num_clusters

path = None
center_points = None

decision_boundary_line = None
divide_path = None
center_path = None

kmeans_clf = None
kmeans_space = None

motion_completed = True
#CONTOUR_THRESH = 800
CONTOUR_THRESH = 1000
PIXEL_METRIC_RATIO = 322/0.5

current_center = None
current_radius = None

#ROBOT_RANGE = 0.9
ROBOT_RANGE = 0.7
ROBOT_CENTER = Point(800,800)
ROBOT_WORKSPACE = ROBOT_CENTER.buffer(ROBOT_RANGE*PIXEL_METRIC_RATIO).boundary


destination_point_pixels = (478,1122)
destination_point_meters = (-0.5,-0.5)
destination_radius_pixels = 161
destination_radius_meters = 0.25

current_centroids_num  = 0

notAdjusted = True

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return (x,y)

pointX_publisher = rospy.Publisher('pointX', Float64, queue_size=10)
pointY_publisher = rospy.Publisher('pointY', Float64, queue_size=10)
points_publisher = rospy.Publisher('points', Float32MultiArray, queue_size=10)
moveit_publisher = rospy.Publisher('/rrbot/moveLocation', Float32MultiArray, queue_size=10)

def image_callback(msg):
#    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
	cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
	   #cv2.imwrite('camera_image.jpeg', cv2_img)
        global num_clusters
        if num_clusters == 0:
            sys.exit("No more clusters to detect and move")
        else:
            pill_detection(cv2_img)
        #time.sleep(5)


def completed_callback(msg):
    print "Inside completed motion callback"
    global motion_completed, decision_boundary_line, divide_path, center_path, path, center_points, num_clusters
    motion_completed = True
    path = None
    center_points = None
    decision_boundary_line = None
    divide_path = None
    center_path = None
    num_clusters = num_clusters - 1
    print "Num clusters inside complete:", num_clusters
    
    global current_center, current_radius
    #current_center = None
    current_radius = None

    global desired_num_pills
    desired_num_pills = desired_num_pills - pills_per_cluster

    global notAdjusted
    notAdjusted = True



def pill_detection(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    retval, threshold = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
    im2, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    for cnt in contours:
        #x,y,w,h = cv2.boundingRect(cnt)
        
        area = cv2.contourArea(cnt)
        if area <= CONTOUR_THRESH:
        
            startX, startY, width, height = cv2.boundingRect(cnt)
            endX = startX + width
            endY = startY + height
            
            centerX = startX + (width/2)
            centerY = startY + (height/2)
            
            distance_to_destination = math.sqrt(math.pow((centerX-destination_point_pixels[0]),2)+math.pow((centerY-destination_point_pixels[1]),2))
            if distance_to_destination <= destination_radius_pixels:
                continue

            box = np.array([startX, startY, endX, endY])
            rects.append(box.astype("int"))
            cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 3)

    # update our centroid tracker using the computed set of bounding
    # box rectangles
    #print "Number of rectangles =", len(rects)
    objects = ct.update(rects)

    centroids = []
    # loop over the tracked objects
    for (objectID, centroid) in objects.items():
        #print "ObjectID =", objectID, "Centroid =", centroid
        centroids.append(centroid)

        # draw both the ID of the object and the centroid of the
        # object on the output frame
        text = "ID {}".format(objectID)
        #cv2.putText(image, text, (centroid[0] - 10, centroid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        #cv2.putText(image, text, (centroid[0] - 100, centroid[1] - 100), cv2.FONT_HERSHEY_SIMPLEX, 1000, (255, 0, 0), 2)
        cv2.circle(image, (centroid[0], centroid[1]), 5, (255, 0, 0), -1)

    centroids = np.asarray(centroids)
    global current_centroids_num
    current_centroids_num = len(centroids)
           
    #print "Centroid Lists:\n", centroids
    
    
    
    
    if motion_completed:
        #if current_centroids_num%num_clusters != 0:
        #    global num_clusters
        #    num_clusters = num_clusters+1

        if current_centroids_num > desired_num_pills:
            global notAdjusted
            if notAdjusted:
                global num_clusters
                num_clusters = num_clusters + 1
                print "Anomaly, num_clusters:", num_clusters
                notAdjusted = False
         
        
        print "Calculating clusters, num_clusters:", num_clusters
        #num_clusters = 2
        clf = EqualGroupsKMeans(n_clusters=num_clusters)
        space = clf.fit_transform(centroids)
        
        global kmeans_clf, kmeans_space
        kmeans_clf = clf
        kmeans_space = space        
        
    
        center_distances = squareform(pdist(clf.cluster_centers_))
        np.fill_diagonal(center_distances, np.inf)
        #print "Center_distances:\n", center_distances, "\nShape of center_distances:", center_distances.shape
        closest_index = center_distances.argmin(axis=0)[0]
        #print "Closest =", closest_index
        closest_distance = center_distances[0][closest_index]
        #print "Closest dist=", closest_distance
    
    
        # TODO need first center to be the leftmost and next center is next 
        cluster_centers = clf.cluster_centers_
        #print "Cluster centers:", cluster_centers
        #print "Cluster centers shape:", cluster_centers.shape, "Type:", type(cluster_centers)
        
        # Leftmost clusters
        first_label = cluster_centers[:,0].argmin()
        first_center = cluster_centers[first_label,:]
#        cluster_centers = np.delete(cluster_centers, (first_label), axis=0)
#        next_label = cluster_centers[:,0].argmin()
#        next_center = cluster_centers[next_label,:]
        
        # Upmost clusters
#        first_label = cluster_centers[:,1].argmin()
#        first_center = cluster_centers[first_label,:]
#        cluster_centers = np.delete(cluster_centers, (first_label), axis=0)
#        next_label = cluster_centers[:,1].argmin()
#        next_center = cluster_centers[next_label,:]
                
        
        #first_center = clf.cluster_centers_[0]
        #next_center = clf.cluster_centers_[closest_index]
        #first_label = 0
        #next_label = closest_index
        
        global current_center
        current_center = first_center
        
        #print "Space:", space
        distances = space[:,first_label]
        #print "first label:", first_label
        #print "all distances:", distances
        #print "all labels:", clf.labels_
        point_distances = distances[np.where(clf.labels_ == first_label)]
        largest_radius = point_distances.max()
        global current_radius
        current_radius = largest_radius
    

    
        
    for center in kmeans_clf.cluster_centers_:
        cv2.circle(image, (int(center[0]), int(center[1])), 10, (0, 0, 0), -1)
        
    for i in range(kmeans_clf.labels_.shape[0]):
        label = kmeans_clf.labels_[i]
        distance = kmeans_space[i][label]
        color = (127,127,127)
        cv2.circle(image, (int(kmeans_clf.cluster_centers_[label][0]), int(kmeans_clf.cluster_centers_[label][1])), int(distance), color, 5)
    

    

    vector = (current_center[0] - destination_point_pixels[0], current_center[1] - destination_point_pixels[1])
    vector_norm = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1],2))
    vector_normalized = tuple([x/vector_norm for x in vector])
    vector_normalized_orthogonal = (-1*vector_normalized[1], vector_normalized[0])
    
    additional_offset_pixels = 15
    largest_distance = current_radius
    extended_vector = tuple([x*(largest_distance+additional_offset_pixels) for x in vector_normalized])
    
    new_path_start = current_center.copy()
#    print "v x:", vector[0]
#    print "v y:", vector[1]
#    if vector[0] > 0:
#        new_path_start[0] = new_path_start[0] + extended_vector[0]
#    elif vector[0] < 0:
#        new_path_start[0] = new_path_start[0] - extended_vector[0]
#
#    if vector[1] > 0:
#        new_path_start[1] = new_path_start[1] + extended_vector[1]
#    elif vector[1] < 0:
#        new_path_start[1] = new_path_start[1] - extended_vector[1]    
    
    
    new_path_start = (current_center[0]+extended_vector[0], current_center[1]+extended_vector[1])    
    
    
    blade_measurement_pixels = 161
    blade_measurement_meters = 0.25
    blade_offset_vector = tuple([x*(blade_measurement_pixels/2) for x in vector_normalized_orthogonal])
    new_start_point_blade_offset = (int(new_path_start[0]-blade_offset_vector[0]), int(new_path_start[1]-blade_offset_vector[1]))    
    
    
    destination_blade_offset = (int(destination_point_pixels[0]-blade_offset_vector[0]),int(destination_point_pixels[1]-blade_offset_vector[1]))    
    
    global path
    if path == None:
        path = (new_path_start, destination_point_pixels)
        
    else:
            
        path_start = path[0]
        path_end = path[1]
        path_start = (int(path_start[0]),int(path_start[1]))
        path_end = (int(path_end[0]),int(path_end[1]))        
        cv2.line(image, path_start, path_end, (0,0,255), 10)            
            
            # Publishing Float32MultiArray
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
        #pixel_points = [new_path_start.x, new_path_start.y, 0.2, intersection[0], intersection[1], 0, center1[0], center1[1], 0]
        #pixel_points = [new_path_start[0], new_path_start[1], 0.2, destination_point_pixels[0], destination_point_pixels[1], 0, current_center[0], current_center[1], 0]

#        pixel_points = [new_start_point_blade_offset[0], new_start_point_blade_offset[1], 0.2, destination_point_pixels[0], destination_point_pixels[1], 0.0]
        pixel_points = [new_start_point_blade_offset[0], new_start_point_blade_offset[1], 0.2, destination_blade_offset[0], destination_blade_offset[1], 0.0]

        meter_points = []
        for i in range(0,len(pixel_points)):
            coord = pixel_points[i]
            if i%3 == 0:
                meter_points.append((coord-800)/PIXEL_METRIC_RATIO)
            elif i%3 == 1:
                meter_points.append((800-coord)/PIXEL_METRIC_RATIO)
            else:
                meter_points.append(coord)
                
            #points = [x / PIXEL_METRIC_RATIO for x in points]
#            print "POINTSSSSSSSS:", meter_points
                
        #meter_points[6] = 0.7
        #meter_points[7] = 0
        
        mat.data = meter_points

        #my_array_for_publishing = Int64MultiArray(data=array)
        points_publisher.publish(mat)
            
        global motion_completed
        if motion_completed:
#            pointX_publisher.publish(new_path_start_tuple[0])
#            pointY_publisher.publish(new_path_start_tuple[1])
#            pointX_publisher.publish(intersection[0])
#            pointY_publisher.publish(intersection[1])
            
            moveit_publisher.publish(mat)
            print "Published to moveLocation:", mat.data
            global motion_completed
            motion_completed = False
            
            
#        else:
#            
#            cv2.line(image, (int(path_xx[0]),int(path_yy[0])), (int(path_xx[-1]),int(path_yy[-1])), (0,0,255), 10)
        
    #cv2.line(image, (int(xx[0]),int(yy[0])), (int(xx[-1]),int(yy[-1])), (0,0,255), 10)

    cv2.circle(image, destination_point_pixels, 10, (255,0,255), -1)
    cv2.circle(image, (int(current_center[0]),int(current_center[1])), int(current_radius), (0,255,255), 5)
    cv2.circle(image, destination_point_pixels, destination_radius_pixels, (255,0,255), 5)
    
    cv2.circle(image, new_start_point_blade_offset, 10, (255,0,255), -1)
    cv2.line(image, new_start_point_blade_offset, destination_blade_offset, (255,0,255), 10)
    
    pill_text =    "Number of detected pills:   %d"%current_centroids_num
    cv2.putText(image, pill_text, (100, 125), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2)
    cluster_text = "Number of clusters to sort: %d"%num_clusters
    cv2.putText(image, cluster_text, (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2)
    
    # show the output frame
    cv2.imshow("Frame", cv2.resize(image, (0,0), fx=0.5, fy=0.5))
    #cv2.imshow("Threshold", cv2.resize(threshold, (0,0), fx=0.5, fy=0.5))
    key = cv2.waitKey(1) & 0xFF







if __name__=="__main__":
    rospy.init_node('pill_tracker')
    # Define your image topic
    #image_topic = "/rrbot/camera2/image_raw"
    image_topic = "/rrbot/camera3/image_raw"
    completed_topic = "/rrbot/motionComplete"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(completed_topic, Float64, completed_callback)
    # Spin until ctrl + c
    rospy.spin()
