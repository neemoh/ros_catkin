#!/usr/bin/env python
import numpy as np
import pickle
from sklearn.linear_model import LogisticRegression
from sklearn.svm import SVC
from sklearn import preprocessing
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt

with open('classifiers_and_scalers.pickle') as f:
    c_s = pickle.load(f)
scalers = c_s[0]
classifiers = c_s[1]


with open('train_data_mat_1to7.pickle', 'rb') as f:
    exp1_data_mat = pickle.load(f)

# exp1_data_mat comprises:
# 0:users  1:labels
# 2:rmse   3:max_err   4:WS_boundary
# 5:mstr_displ   6:cut_segments   7:time ]
n_users = 6
y = exp1_data_mat[:,1]
user_labels = exp1_data_mat[:,0]
X_train = exp1_data_mat[:,2:]



metrics = Twist()
print metrics
def callback(data):
    metrics = data
    print metrics

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('skillClassifier')
    rospy.Subscriber('metrics', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    plt.figure(figsize=(6 * 3, 8))

    mesh_size = 400  #  size mesh
    for i in range(0,6):

        X_1d_non_scaled = np.reshape(X_train[:,i], (-1, 1))

        robust_scaler = preprocessing.RobustScaler()
        X_1d_scaled = robust_scaler.fit_transform( X_1d_non_scaled)



        # use classifier
        classifier = classifiers[i]
        y_pred = classifier.predict(X_1d_scaled)
        probas = classifier.predict_proba(X_1d_scaled)
        #    probas[y_pred.ravel() == 1]

        # saving the scaler and the classifier in a list to be dumbed later
        scalers.append(robust_scaler)
        classifiers.append(classifier)

        # MESH coloring to see the desicion boundaries
        x_min = X_1d_non_scaled.min() - 0.1 * X_1d_non_scaled.max()
        x_max = X_1d_non_scaled.max() + 0.1 * X_1d_non_scaled.max()
        xx = np.reshape(np.linspace(x_min, x_max, mesh_size), (-1, 1))
        prob_mesh_xx = classifier.predict_proba(robust_scaler.transform(xx))

        prob_mesh = np.repeat((prob_mesh_xx[:,0]), mesh_size)
        prob_mesh = prob_mesh.reshape(mesh_size, mesh_size)


        # PLOT
        classif_rate = np.mean(y_pred.ravel() == y.ravel()) * 100
        print("classif_rate for %s : %f " % (i, classif_rate))

        plt.subplot(1, 6, i+1 )
        plt.pcolormesh(np.linspace(0, 1, mesh_size), xx.ravel(), prob_mesh, cmap='RdBu')
        if i == 5:
            plt.colorbar()
        plt.title(str(i) + ": %" + str(int(classif_rate)))

        # Plot also the training points
        plt.scatter(np.random.uniform(0.4, 0.6, size=(X_1d_non_scaled.shape)), X_1d_non_scaled, c=y, edgecolors='w', linewidths =0.4)
        plt.ylim(x_min , x_max)
        plt.xlim(0, 1)
    plt.show()
