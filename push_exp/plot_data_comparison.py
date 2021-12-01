# Alexie Pogue, Nov 2021
# Plot processed trajectories

import h5py
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from config.shape_db import ShapeDB
from tf import transformations as tfm
from ik.helper import *
from config.helper import *


# what is in the data
# raw data
# tip_pose: time (s), x, y position (meters) wrt robot frame, orientation (?) tip in rad
# object_pose: time (s), x, y position (meters) wrt robot frame, orientation (?) tip in rad
# ft_wrench: time (s), force x, y (Newtons), torque z (Newton*meter)

def data_readout(data_raw, data_proc):

    print data_raw.keys()
    print data_proc.keys()
    print data_raw['object_pose'], 'object pose'
    print data_proc['object']

    print data_raw['tip_pose'], 'tip_pose'
    print data_proc['tip']


# plot the data
def plot_processed_object_pose(data_proc):
    object = data_proc['object']
    dt = 1 / 180.0  # this value is set in preprocess.py
    starttime = 0.0
    endtime = (np.shape(object)[0]) * dt
    timearray = np.arange(0, endtime, dt) - starttime

    print timearray[0], timearray[-1], 'processed initial and final time'
    print dt, 'timediff'

    object = np.array(object)
    x_pos = object[:, 0:1]*100 # in cm
    y_pos = object[:, 1:2]*100 # in cm
    pose = object[:, 2:3]

    plt.figure(1)
    ax1 = plt.subplot(311)
    ax1.plot(timearray, x_pos)
    ax1.set_xlabel('time (sec)')
    ax1.set_ylabel('x_pos (cm)')

    ax2 = plt.subplot(312)
    ax2.plot(timearray, y_pos)
    ax2.set_xlabel('time (sec)')
    ax2.set_ylabel('y_pos (cm)')

    ax2 = plt.subplot(313)
    ax2.plot(timearray, pose)
    ax2.set_xlabel('time (sec)')
    ax2.set_ylabel('pose (rad)')

    ax1.set_title('object position processed data')

    plt.show()


def _transform_back_frame2d(pts, f):
    # TODO: this will go in helper file
    # transform_back_frame2d (pts, f)
    # pts: row vectors [x,y]
    # f: row vector [x,y,theta] b frame wrt origin
    # return: row point vectors in frame b, [x,y]
    # T = np.array([[c, s, f[0]],
    #               [-s, c, f[1]],
    #               [0, 0, 1]])
    # above put in the frame of the origin, we want the frame of the block (intro to robotics pg 36)
    pts_array = np.array(pts).reshape(-1, 2)
    theta = f[2]
    c = cos(theta)
    s = sin(theta)
    R = np.array([[c, -s],
                  [s, c]])
    f_from_b = np.dot(-R.transpose(), f[0:2])
    T = np.vstack((np.hstack((R.transpose(), f_from_b)), np.array([0, 0, 1])))
    pts_ret = np.dot(T, np.vstack((pts_array.transpose(), np.ones((1, pts_array.shape[0])))))
    return pts_ret[0:2, :].transpose()


def plot_raw_object_pose(data_raw):
    # see preprocess _zero_initial_position function to understand how the data is preprocessed
    # the world frame moves from the robot frame to the initial position of the block

    object = data_raw['object_pose']
    object = np.array(object)
    starttime = object[0, 0:1]
    #  TODO: startx and starty should be handled within the function (f not as input)
    startx = object[0, 1:2]*100  # cm
    starty = object[0, 2:3]*100  # cm
    startpose = object[0, 3:4]

    timearray = object[:, 0:1] - starttime
    timediff = (object[1:, 0:1] - object[0:-1, 0:1])  # 1: includes last element
    pose = object[:, 3:4] - startpose

    print timearray[0], timearray[-1], 'raw initial and final time'
    print timediff, 'raw timediff'

    pts = object[:, 1:3]*100  # cm
    f = np.array([startx, starty, startpose])
    pts_world_frame2d = _transform_back_frame2d(pts, f)

    x_pos = pts_world_frame2d[:, 0:1]
    y_pos = pts_world_frame2d[:, 1:2]

    plt.figure(1)
    ax1 = plt.subplot(311)
    ax1.plot(timearray, x_pos)
    ax1.set_xlabel('time (sec)')
    ax1.set_ylabel('x_pos (cm)')

    ax2 = plt.subplot(312)
    ax2.plot(timearray, y_pos)
    ax2.set_xlabel('time (sec)')
    ax2.set_ylabel('y_pos (cm)')

    ax2 = plt.subplot(313)
    ax2.plot(timearray, pose)
    ax2.set_xlabel('time (sec)')
    ax2.set_ylabel('pose (rad)')

    ax1.set_title('object position raw data')

    plt.show()


def plot_processed_tip_pose(data_proc):
    #  in preprocess script the tip pose is wrt initial object pose
    tip = data_proc['tip']
    dt = 1 / 180.0  # this value is set in preprocess.py
    starttime = 0.0
    endtime = (np.shape(tip)[0]) * dt
    timearray = np.arange(0, endtime, dt) - starttime

    print timearray[0], timearray[-1], 'processed initial and final time'
    print dt, 'timediff'

    tip = np.array(tip)
    x_pos = tip[:, 0:1]*100 # in cm
    y_pos = tip[:, 1:2]*100 # in cm

    plt.figure(1)
    ax1 = plt.subplot(211)
    ax1.plot(timearray, x_pos)
    ax1.set_xlabel('time (sec)')
    ax1.set_ylabel('x_pos (cm)')

    ax2 = plt.subplot(212)
    ax2.plot(timearray, y_pos)
    ax2.set_xlabel('time (sec)')
    ax2.set_ylabel('y_pos (cm)')

    ax1.set_title('tip position processed data')

    plt.show()


def plot_raw_tip_pose(data_raw):
    # see preprocess _zero_initial_position function to understand how the data is preprocessed
    # the world frame moves from the robot frame to the initial position of the block
    # not going to worry about the pose for now although the pose is available
    object = data_raw['object_pose']
    object = np.array(object)
    startx = object[0, 1:2]*100  # cm
    starty = object[0, 2:3]*100  # cm
    startpose = object[0, 3:4]

    tip = data_raw['tip_pose']
    tip = np.array(tip)
    starttime = tip[0, 0:1]

    timearray = tip[:, 0:1] - starttime
    timediff = (tip[1:, 0:1] - tip[0:-1, 0:1])  # 1: includes last element

    print timearray[0], timearray[-1], 'raw initial and final time'
    print timediff, 'raw timediff'

    pts = tip[:, 1:3]*100  # cm
    f = np.array([startx, starty, startpose])  # cm
    pts_world_frame2d = _transform_back_frame2d(pts, f)  # put it all in the perspective of the block

    x_pos = pts_world_frame2d[:, 0:1]
    y_pos = pts_world_frame2d[:, 1:2]

    plt.figure(1)
    ax1 = plt.subplot(211)
    ax1.plot(timearray, x_pos)
    ax1.set_xlabel('time (sec)')
    ax1.set_ylabel('x_pos (cm)')

    ax2 = plt.subplot(212)
    ax2.plot(timearray, y_pos)
    ax2.set_xlabel('time (sec)')
    ax2.set_ylabel('y_pos (cm)')

    ax1.set_title('object position raw data')

    plt.show()


def main(argv):
    # TODO: need to deal with forces as well
    # TODO: need to run over many scripts including scripts with acceleration
    # if len(argv) < 2:
    #     print 'Usage: plot_raw_json.py *.h5 tip_speed_profile/forceprofile/snapshots'
    #     return
    h5_filepath = argv[1]
    dir_name = os.path.dirname(h5_filepath)
    name = os.path.basename(h5_filepath)

    data_raw = h5py.File(h5_filepath, "r", driver='core')
    data_proc = h5py.File(os.path.join(dir_name, name[name.find('_a=')+1:]), "r", driver='core')

    # figname = h5_filepath.replace('.h5', '.png')
    shape_id = 'rect1'

    data_readout(data_raw, data_proc)
    plot_processed_object_pose(data_proc)
    plot_raw_object_pose(data_raw)

    # TODO: is this reasonable? check block sizes (add to the blog post)
    plot_processed_tip_pose(data_proc)
    plot_raw_tip_pose(data_raw)
    data_raw.close()
    data_proc.close()

    # length data
    # initial and final time
    # time step
    # tip speed
    # pose snapshots
    # object position
    # tip position
    # png of rotation stuff
    # object photo from the mcube website


if __name__=='__main__':
    # TODO: currently dumps output into input file

    import sys
    main(sys.argv)


