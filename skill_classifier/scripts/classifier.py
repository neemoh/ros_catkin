#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Twist
import numpy as np
import pickle
from sklearn.linear_model import LogisticRegression
from sklearn.svm import SVC
from sklearn import preprocessing

import matplotlib.pyplot as plt
from os import getcwd
# -----------------------------------------------------------------------------
# Load data and initialize
# -----------------------------------------------------------------------------

# make array printing more readable
np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

metrics = Pose()
new_metrics_msg = False


def poseMsgToNp(pose):
    return np.array([
    pose.position.x,    pose.position.y,    pose.position.z,
    pose.orientation.x,    pose.orientation.y,
    pose.orientation.z,    pose.orientation.w
    ]).reshape(1,7)

def npToTwistMsg(nparray):
    out = Twist()
    out.linear.x = nparray[0]
    out.linear.y = nparray[1]
    out.linear.z = nparray[2]
    out.angular.x = nparray[3]
    out.angular.y = nparray[4]
    return out



# -----------------------------------------------------------------------------
# Classifier class
# -----------------------------------------------------------------------------
class classifier():
    def __init__(self):
        with open('classifiers_and_scalers.pickle') as f:
            c_s = pickle.load(f)
        self.scalers = c_s[0]
        self.classifiers = c_s[1]

    # skill Classifier
    def classify(self, X):

        user_profiles = np.zeros( (X.shape[0], 5) )
        features = [range(X.shape[1]) , [0,6], [1,2], [3,4], [5]]

        for i in range(0, 5):
            key = str(features[i])
            X_scaled = self.scalers[key].transform( X[:, features[i]] )

            if i in [1, 2, 4]: # Classifiers with probability
                user_profiles[:,i] = self.classifiers[key].predict_proba(X_scaled)[:,0]
            else: # Classifiers with decision
                user_profiles[:,i] = self.classifiers[key].predict(X_scaled)

        user_profile = user_profiles[0,:]
        return user_profile


def callback(data):
    global metrics
    global new_metrics_msg
    metrics = data
    new_metrics_msg = True


def rosGO():
    global new_metrics_msg
    global metrics
    cl = classifier()

    rospy.init_node('skillClassifier')
    #directory = rospy.get_param("~classifiers_file_dir")
    rospy.Subscriber('metrics', Pose, callback)

    pub = rospy.Publisher('user_profile', Twist, queue_size=10)
    #rospy.loginfo("Publishing static odometry from \"%s\" to \"%s\"", "2", "3")
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        if(new_metrics_msg):

            new_metrics_msg = False
            # do the classification
            profile = cl.classify(poseMsgToNp(metrics))
            # Log the input and outputs
            rospy.loginfo("received sample  : %s" % poseMsgToNp(metrics).ravel())
            rospy.loginfo("estimated profile: %s\n" % profile)
            # Publish the profile
            pub.publish( npToTwistMsg(profile) )

        r.sleep()

if __name__ == '__main__':
    try:
        rosGO()
    except rospy.ROSInterruptException:
        pass
