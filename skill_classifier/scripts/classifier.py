#!/usr/bin/env python
import rospy
from active_guidance.msg import perfmetrics, skillProbabilities
import numpy as np
import pickle
from sklearn.linear_model import LogisticRegression
from sklearn.svm import SVC
from sklearn import preprocessing

import matplotlib.pyplot as plt
from os import getcwd
from os.path import join
from os import linesep
import json

# -----------------------------------------------------------------------------
# Load data and initialize
# -----------------------------------------------------------------------------
proj_dir = '/media/nima/Data/Dropbox/2_Nima_Work_Pub/5_User_profiling/data/'
user = 11;
skill_level = 0;

# make array printing more readable
np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

metrics = perfmetrics()
new_metrics_msg = False


# -----------------------------------------------------------------------------
# Classifier class
# -----------------------------------------------------------------------------
class classifier():
    def __init__(self):
        file_name = 'classifiers_and_scalers_scaled_by_aclength.pickle'
        file_address =  join(proj_dir, "pickles", file_name)

        with open(file_address, 'rb') as f:
            c_s = pickle.load(f)
        self.scalers = c_s[0]
        self.classifiers = c_s[1]

    # skill Classifier
    def classify(self, X):

        user_profiles = np.zeros( (X.shape[0], 5) )
        features = [range(X.shape[1]), [0,6], [1,2], [5], [3,4]]

        for i in range(0, 5):
            key = str(features[i])
            X_scaled = self.scalers[key].transform( X[:, features[i]] )
            user_profiles[:,i] = self.classifiers[key].predict_proba(X_scaled)[:,1]

        user_profile = user_profiles[0,:]
        return user_profile



# -----------------------------------------------------------------------------
# skill prob message to array conv function
# -----------------------------------------------------------------------------
def perfMetricsMsgToNp(msg):
    # out of laziness put the path length in the time stamp! :D
    # The length is used to scale the last three metrics
    path_length = msg.acpathlength
    print path_length
    # first three metrics are converted from m to mm
    return np.array([
    msg.rmse * 1000,
    msg.maxerror*1000,
    msg.maxvel *1000,
    msg.ws,
    msg.totdisp/path_length,
    float(msg.numsegs)/path_length,
    msg.time/path_length,
    msg.other ]).reshape(1,8)


# -----------------------------------------------------------------------------
# array to skill prob message conv function
# -----------------------------------------------------------------------------
def npToSkillProbabilitiesMsg(nparray):
    out = skillProbabilities()
    out.skillclass          = int(nparray[0]+ 0.5)
    out.accuracy            = nparray[1]
    out.motionconsistency   = nparray[2]
    out.depthperception     = nparray[3]
    out.mstrworkspace       = nparray[4]
    return out


# -----------------------------------------------------------------------------
# callback function
# -----------------------------------------------------------------------------
def callback(data):
    global metrics
    global new_metrics_msg
    metrics = data
    new_metrics_msg = True


# -----------------------------------------------------------------------------
# awrite results function
# -----------------------------------------------------------------------------
def append_result(result):
    file_name = str(user) + '_results'
    file_address =  (join(proj_dir, "2016_09_tests", file_name) )
    with open(file_address, 'a') as f:
        json.dump(result, f)
        f.write(linesep)

# -----------------------------------------------------------------------------
# awrite results function
# -----------------------------------------------------------------------------
def append_metrics(msg):
    file_name = str(user) + '_metrics'
    file_address =  (join(proj_dir, "2016_09_tests", file_name) )
    data =  [
    skill_level,
    msg.rmse * 1000,
    msg.maxerror*1000,
    msg.maxvel *1000,
    msg.ws,
    msg.totdisp,
    msg.numsegs,
    msg.time,
    msg.acpathlength,
    msg.other ]

    with open(file_address, 'a') as f:
        json.dump(data, f)
        f.write(linesep)

# -----------------------------------------------------------------------------
# ROS function
# -----------------------------------------------------------------------------
def rosGO():
    global new_metrics_msg
    global metrics
    cl = classifier()

    rospy.init_node('skillClassifier')
    #directory = rospy.get_param("~classifiers_file_dir")
    rospy.Subscriber('metricsAll', perfmetrics, callback)

    pub = rospy.Publisher('user_profile', skillProbabilities, queue_size=10)
    #rospy.loginfo("Publishing static odometry from \"%s\" to \"%s\"", "2", "3")
    r = rospy.Rate(10)

    # We do 3 tests and average their classification resuls
    num_of_placement_acqs = 3;
    acq_num = 0;

    placement_results = np.zeros( (num_of_placement_acqs, 5) )
    while not rospy.is_shutdown():
        if(new_metrics_msg):

            new_metrics_msg = False
            metrics_arr = perfMetricsMsgToNp(metrics)

            # Chech if the message contains NaNs by counting the number of Nans
            if(len([x for x in np.isnan(metrics_arr).ravel() if x]) > 0):
                print "ERROR the metrics message has a NaN element"
            else:
                # do the classification
                placement_results[acq_num] = cl.classify(metrics_arr[:,:7])
                # Log the input and outputs
                rospy.loginfo("received sample  : %s" % metrics_arr.ravel())
                rospy.loginfo("estimated profile: %s\n" % placement_results[acq_num])
                append_result( [skill_level] +  placement_results[acq_num].tolist() )
                append_metrics(metrics)

                # Publish the profile
                acq_num += 1
                if(acq_num==3):
                    profile = np.mean(placement_results, axis=0)
                    print "sending profile: " , profile
                    pub.publish( npToSkillProbabilitiesMsg(profile) )
                    acq_num = 0

        r.sleep()



# -----------------------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        rosGO()
    except rospy.ROSInterruptException:
        pass
