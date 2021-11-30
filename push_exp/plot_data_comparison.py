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
    print data_raw['object_pose']
    print data_proc['object']




# plot the data
def plot_processed_object_pose(data_proc):
    object = data_proc['object']
    dt = 1 / 180.0
    starttime = 0.0
    endtime = (np.shape(object)[0]+1) * dt
    timearray = np.arange(0, endtime, dt) - starttime
    # print(np.size(timearray))

    object = np.array(object)
    x_pos = object[:, 0:1]*100 # in cm
    y_pos = object[:, 1:2]*100 # in cm
    pose = object[:, 2:3]

    # print(np.size(tip))
    f, ax = plt.subplots(1, sharex=True)
    plt.figure(1)
    ax1 = plt.subplot(311)
    ax1.plot(timearray[1:], x_pos)
    ax1.set_xlabel('time (sec)')
    ax1.set_ylabel('x_pos (cm)')

    ax2 = plt.subplot(312)
    ax2.plot(timearray[1:], y_pos)
    ax2.set_xlabel('time (sec)')
    ax2.set_ylabel('y_pos (cm)')


    ax2 = plt.subplot(313)
    ax2.plot(timearray[1:], pose)
    ax2.set_xlabel('time (sec)')
    ax2.set_ylabel('pose (rad)')

    ax1.set_title('object position processed data')

    plt.show()









# def plot_tip_pose(data_raw, data_proc):




def main(argv):
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

    # plot(data, shape_id, figname)
    # plot_speed_profile(data, shape_id, figname, multidim=True)

    data_raw.close()
    data_proc.close()


if __name__=='__main__':
    # TODO: currently dumps output into input file

    import sys
    main(sys.argv)


