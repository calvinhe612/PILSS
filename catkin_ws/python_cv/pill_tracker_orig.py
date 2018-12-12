# USAGE
# python object_tracker.py --prototxt deploy.prototxt --model res10_300x300_ssd_iter_140000.caffemodel

# import the necessary packages
from pyimagesearch.centroidtracker import CentroidTracker
#from imutils.video import VideoStream
import numpy as np
import argparse
import imutils
import time
import cv2
import random

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

import matplotlib
from matplotlib import pyplot as plt
import scipy
from sklearn import svm
from sklearn.metrics.pairwise import euclidean_distances
from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform

# construct the argument parse and parse the arguments
'''ap = argparse.ArgumentParser()
ap.add_argument("-p", "--prototxt", required=True,
	help="path to Caffe 'deploy' prototxt file")
ap.add_argument("-m", "--model", required=True,
	help="path to Caffe pre-trained model")
ap.add_argument("-c", "--confidence", type=float, default=0.5,
	help="minimum probability to filter weak detections")
args = vars(ap.parse_args())
'''
# initialize our centroid tracker and frame dimensions
ct = CentroidTracker()
(H, W) = (None, None)

# load our serialized model from disk
'''print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])

# initialize the video stream and allow the camera sensor to warmup
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)
'''

# Instantiate CvBridge
bridge = CvBridge()

# initialize our centroid tracker and frame dimensions
ct = CentroidTracker()

path = None
center_points = None
move_completed = True
CONTOUR_THRESH = 800
PIXEL_METRIC_RATIO = 322/0.5
from shapely.geometry import LineString
from shapely.geometry import Point

#ROBOT_RANGE = 0.9
ROBOT_RANGE = 0.7
ROBOT_CENTER = Point(800,800)
ROBOT_WORKSPACE = ROBOT_CENTER.buffer(ROBOT_RANGE*PIXEL_METRIC_RATIO).boundary

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
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
	cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
	#cv2.imwrite('camera_image.jpeg', cv2_img)
        pill_detections(cv2_img)
        #time.sleep(5)


def completed_callback(msg):
    global move_completed
    move_completed = True



def pill_detections(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    retval, threshold = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
    '''cv2.imshow('original', image)
    cv2.imshow('threshold', threshold)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    '''
    im2, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(image, contours, -1, (0,255,0), 3)

    rects = []
    for cnt in contours:
        #x,y,w,h = cv2.boundingRect(cnt)
        
        area = cv2.contourArea(cnt)
        #print "AREAAAAAAAAA:", area
        if area <= CONTOUR_THRESH:
        
            startX, startY, width, height = cv2.boundingRect(cnt)
            endX = startX + width
            endY = startY + height
            #image = cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),5)

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
    #print "Centroid Lists:\n", centroids
    
    
    
    
    
    num_clusters = 2
    clf = EqualGroupsKMeans(n_clusters=num_clusters)
    space = clf.fit_transform(centroids)
    #print "Clf type:", type(clf)
    #print "Labels:", clf.labels_, "Type:", type(clf.labels_), "Shape:", clf.labels_.shape
    #print "Cluster centers:", clf.cluster_centers_

    #print "Space:", space 

    for center in clf.cluster_centers_:
        cv2.circle(image, (int(center[0]), int(center[1])), 10, (0, 0, 0), -1)
        
    for i in range(clf.labels_.shape[0]):
        label = clf.labels_[i]
        distance = space[i][label]
        color = (127,127,127)
        cv2.circle(image, (int(clf.cluster_centers_[label][0]), int(clf.cluster_centers_[label][1])), int(distance), color, 5)



    ## Clustering and Boundary Decision
    
    '''first_center = clf.cluster_centers_[0]
    remaining_center = clf.cluster_centers_[1:]
    center_distances = euclidean_distances(remaining_center, [first_center])
    print "Center_distances:", center_distances, "\nShape of center_distances:", center_distances.shape
    min_index = np.where(center_distances == center_distances.min())[0]
    next_center = clf.cluster_centers_[min_index][0]

    first_center_index = np.where(clf.cluster_centers_ == first_center)
    next_center_index = np.where(clf.cluster_centers_ == next_center)
    print "first index =", first_center_index, "next index =", next_center_index

    print "First:", first_center
    print "Next:", next_center
    '''

    center_distances = squareform(pdist(clf.cluster_centers_))
    np.fill_diagonal(center_distances, np.inf)
    #print "Center_distances:\n", center_distances, "\nShape of center_distances:", center_distances.shape
    closest_index = center_distances.argmin(axis=0)[0]
    #print "Closest =", closest_index
    closest_distance = center_distances[0][closest_index]
    #print "Closest dist=", closest_distance

    first_center = clf.cluster_centers_[0]
    next_center = clf.cluster_centers_[closest_index]

    first_label = 0
    next_label = closest_index

    first_cluster_points = centroids[np.where(clf.labels_ == first_label)]
    next_cluster_points = centroids[np.where(clf.labels_ == next_label)]

    print "Centroid Lists:\n", centroids
    #print "First cluster points =\n", first_cluster_points
    #print "Next cluster points =\n", next_cluster_points

    #print "Shape of first cluster =", first_cluster_points.shape
    
    X = np.concatenate((first_cluster_points,next_cluster_points), axis = 0)
    Y = np.array([0]*first_cluster_points.shape[0] + [1]*next_cluster_points.shape[0])

    C = 1.0  # SVM regularization parameter
    clf_svm = svm.SVC(kernel = 'linear',  gamma=0.7, C=C )
    clf_svm.fit(X, Y)

    w = clf_svm.coef_[0]
    a = -w[0] / w[1]
    #xx = np.linspace(-5, 5)
    print "Image shape =", image.shape

    image_width = image.shape[0]
    '''print "***********************:", first_center
    if first_center[0] <= (image_width/2):
        xx = np.linspace(0, image.shape[0]/2)
    else:
        xx = np.linspace(image_width/2, image_width)
    '''
    
    xx = np.linspace(0, image_width)
    yy = a * xx - (clf_svm.intercept_[0]) / w[1]
    #print "xx =", xx
    #print "yy =", yy
    
    global center_points
    if center_points == None:
        center_points = (first_center, next_center)
    else:
        center1 = center_points[0]
        center2 = center_points[1]
        cv2.line(image, (int(center1[0]),int(center1[1])), (int(center2[0]),int(center2[1])), (0,255,0), 10)
    
    global path
    if path == None:
        path = (xx, yy)
        
    else:
        path_xx = path[0]
        path_yy = path[1]
        path_start = (int(path_xx[0]),int(path_yy[0]))
        path_end = (int(path_xx[-1]),int(path_yy[-1]))
        
        if center_points:
            center1 = center_points[0]
            center2 = center_points[1]
            center1 = (int(center1[0]),int(center1[1]))
            center2 = (int(center2[0]),int(center2[1]))
            
            intersection = line_intersection((path_start, path_end), (center1, center2))
            
            current_path = LineString([path_start, intersection])
            new_path_start = ROBOT_WORKSPACE.intersection(current_path)
            #print "NEW PATH STARTTTTTTTTTTTTTTTTT:", new_path_start
            #print "TYPEEEEEEEEEE:", type(new_path_start)
            
            new_path_start_tuple = (int(new_path_start.x), int(new_path_start.y))
            cv2.line(image, path_start, intersection, (255,0,0), 10)
            cv2.line(image, new_path_start_tuple, intersection, (0,0,255), 10)
            
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
            pixel_points = [new_path_start.x, new_path_start.y, 0.2, intersection[0], intersection[1], 0, center1[0], center1[1], 0]
            
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
            print "POINTSSSSSSSS:", meter_points
            mat.data = meter_points

            #my_array_for_publishing = Int64MultiArray(data=array)
            points_publisher.publish(mat)
            
            global move_completed
            if move_completed:
                pointX_publisher.publish(new_path_start_tuple[0])
                pointY_publisher.publish(new_path_start_tuple[1])
                pointX_publisher.publish(intersection[0])
                pointY_publisher.publish(intersection[1])
                
                moveit_publisher.publish(mat)
                
                move_completed = False
            
            
        else:
            
            cv2.line(image, (int(path_xx[0]),int(path_yy[0])), (int(path_xx[-1]),int(path_yy[-1])), (0,0,255), 10)
        
    #cv2.line(image, (int(xx[0]),int(yy[0])), (int(xx[-1]),int(yy[-1])), (0,0,255), 10)

    # show the output frame
    cv2.imshow("Frame", cv2.resize(image, (0,0), fx=0.5, fy=0.5))
    #cv2.imshow("Threshold", cv2.resize(threshold, (0,0), fx=0.5, fy=0.5))
    key = cv2.waitKey(1) & 0xFF






'''
# loop over the frames from the video stream
while True:
	# read the next frame from the video stream and resize it
	frame = vs.read()
	frame = imutils.resize(frame, width=400)

	# if the frame dimensions are None, grab them
	if W is None or H is None:
		(H, W) = frame.shape[:2]

	# construct a blob from the frame, pass it through the network,
	# obtain our output predictions, and initialize the list of
	# bounding box rectangles
	blob = cv2.dnn.blobFromImage(frame, 1.0, (W, H),
		(104.0, 177.0, 123.0))
	net.setInput(blob)
	detections = net.forward()
	print "DETECTIONS", detections
        print "TYPE", type(detections)
        print "SHAPE", detections.shape
        rects = []

	# loop over the detections
	for i in range(0, detections.shape[2]):
		# filter out weak detections by ensuring the predicted
		# probability is greater than a minimum threshold
		if detections[0, 0, i, 2] > args["confidence"]:
			# compute the (x, y)-coordinates of the bounding box for
			# the object, then update the bounding box rectangles list
			box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
			rects.append(box.astype("int"))

			# draw a bounding box surrounding the object so we can
			# visualize it
			(startX, startY, endX, endY) = box.astype("int")
			cv2.rectangle(frame, (startX, startY), (endX, endY),
				(0, 255, 0), 2)

	# update our centroid tracker using the computed set of bounding
	# box rectangles
	objects = ct.update(rects)

	# loop over the tracked objects
	for (objectID, centroid) in objects.items():
		# draw both the ID of the object and the centroid of the
		# object on the output frame
		text = "ID {}".format(objectID)
		cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
    image_topic = "/rrbot/camera2/image_raw"

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
'''

if __name__=="__main__":
    rospy.init_node('pill_tracker')
    # Define your image topic
    #image_topic = "/rrbot/camera2/image_raw"
    image_topic = "/rrbot/camera3/image_raw"
    completed_topic = "/rrbot/moveCompleted"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(completed_topic, Float64, completed_callback)
    # Spin until ctrl + c
    rospy.spin()
