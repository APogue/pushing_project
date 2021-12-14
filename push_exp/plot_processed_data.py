


# Alexie Pogue, Nov 2021
# Plot processed trajectories

import h5py
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from config.shape_db import ShapeDB
from tf import transformations as tfm
from ik.helper import *
from config.helper import *

def plot(data, shape_id, figfname):
    # TODO: double check probe size
    probe_radii = {'probe1' : 0.00626/2, 'probe2': 0.004745, 'probe3': 0.00475, 'probe4': 0.00475}
    probe_radius = probe_radii['probe4']

    fig, ax = plt.subplots()
    fig.set_size_inches(7, 7)

    v = int(getfield_from_filename(os.path.basename(figfname), 'v'))
    try:
        a = int(getfield_from_filename(os.path.basename(figfname), 'a'))
    except:
        a = 0

    if a != 0:
        sub = int((2500.0 ** 2) * 2 / (a ** 2))
        if sub < 1: sub = 1
    elif v != -1:
        sub = int(30 * 20 / (v))  # subsample rate
        if sub < 1: sub = 1

    tip = data['tip']

    # add the object as a polygon
    shape_db = ShapeDB()
    shape = shape_db.shape_db[shape_id]['shape']  # shape of the objects presented as polygon.
    shape_type = shape_db.shape_db[shape_id]['shape_type']
    if shape_type == 'poly':
        shape_polygon_3d = np.hstack((np.array(shape), np.zeros((len(shape), 1)), np.ones((len(shape), 1))))
    elif shape_type == 'ellip':
        shape = shape[0]
    elif shape_type == 'polyapprox':
        shape_polygon_3d = np.hstack((np.array(shape[0]), np.zeros((len(shape[0]), 1)), np.ones((len(shape[0]), 1))))

    object = data['object']

    # print data['push_point']

    if len(object) > 0:
        invT0 = np.linalg.inv(matrix_from_xyzrpy(object[0][0:2].tolist() + [0], [0,0,object[0][2]]))
    elif len(tip) > 0:
        invT0 = np.linalg.inv(matrix_from_xyzquat(tip[0][0:2].tolist() +[0], [0, 0, 0, 1]))

    print 'object_pose', len(object), 'tip_pose', len(tip)

    # add the object as shape
    r = []
    if len(object) > 0:
        r = (range(0, len(object), sub)) + [len(object) - 1]
    for i in r:

        T = matrix_from_xyzrpy(object[i][0:2].tolist() + [0], [0, 0, object[i][2]])

        if i == 0:
            alpha, fill = (0.3, True)
        elif i == r[-1]:
            alpha, fill = (0.6, True)
        else:
            alpha, fill = (0.6, False)

        ec, fc = 'black', 'orangered'
        if shape_type == 'poly' or shape_type == 'polyapprox':
            shape_polygon_3d_world = np.dot(np.dot(invT0, T), shape_polygon_3d.T)
            obj = mpatches.Polygon(shape_polygon_3d_world.T[:, 0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill,
                                   linewidth=1, linestyle='solid')
        elif shape_type == 'ellip':
            T_T0 = np.dot(invT0, T)
            scale, shear, angles, trans, persp = tfm.decompose_matrix(T_T0)
            obj = mpatches.Ellipse(trans[0:2], shape[0] * 2, shape[1] * 2, angle=angles[2], fc=fc, ec=ec, alpha=alpha,
                                   fill=fill, linewidth=1, linestyle='solid')

        ax.add_patch(obj)

        # add the probes as circle
        r = []
        if len(tip) > 0:
            r = (range(0, len(tip), sub)) + [len(tip) - 1]
        for i in r:
            tip_pose_0 = np.dot(invT0, tip[i][0:2].tolist() + [0, 1])
            if i == 0:
                alpha, fill = (0.8, False)
            elif i == r[-1]:
                alpha, fill = (0.8, False)
            else:
                alpha, fill = (0.8, False)
            circle = mpatches.Circle(tip_pose_0[0:2], probe_radius, color='black', alpha=alpha, fill=fill, linewidth=1,
                                     linestyle='solid')

            ax.add_patch(circle)

        # render it
        plt.axis([-0.15, 0.15, -0.15, 0.15])
        plt.axis('off')

        if figfname is not None:
            plt.savefig(figfname)


def plot_speed_profile(data, shape_id, figfname, multidim):
    tip = data['tip']
    dt = 1/180.0
    starttime = 0.0
    endtime = np.shape(tip)[0]*dt
    timearray = np.arange(0, endtime, dt) - starttime
    # print(np.size(timearray))

    tip = np.array(tip)
    # print(np.size(tip))
    timediff = dt

    tip_vel = (tip[1:, 0:2] - tip[0:-1, 0:2]) / np.hstack((timediff, timediff))
    tip_speed = np.sqrt(tip_vel[:, 0:1] ** 2 + tip_vel[:, 1:2] ** 2)

    f, ax = plt.subplots(1, sharex=True)
    ax.plot(timearray[1:], tip_speed)
    ax.set_xlabel('time (sec)')
    ax.set_ylabel('speed (m/s)')
    ax.set_ylim(0, .016)
    plt.show()


def plot_force_profile(data):
    # for several files: plot the unrotated raw data
    # plot the unrotated processed data
    # assuming the processed data is in a different frame the two should be different
    # then manually process the raw data (using the rot function here) to look like the processed data

    force_obj = data['force']

    dt = 1/180.0
    starttime = 0.0
    endtime = np.shape(force_obj)[0]*dt
    timearray = np.arange(0, endtime, dt) - starttime

    f, axarr = plt.subplots(2, sharex=True)
    axarr[0].plot(timearray, np.array(force_obj)[:, 0])
    axarr[1].plot(timearray, np.array(force_obj)[:, 1])
    axarr[1].set_xlabel('time (sec)')
    axarr[0].set_ylabel('force x (N)')
    axarr[1].set_ylabel('force y (N)')
    axarr[0].set_title('Processed Force Data')
    # TODO: find a better way to do the figure size
    f.set_figheight(6)
    f.set_figwidth(8)
    plt.show()

def main(argv):
    if len(argv) < 2:
        print 'Usage: plot_raw_json.py *.h5 tip_speed_profile/forceprofile/snapshots'
        return

    h5_filepath = argv[1]

    data = h5py.File(h5_filepath, "r", driver='core')
    figname = h5_filepath.replace('.h5', '.png')
    shape_id = 'rect1'

    # plot(data, shape_id, figname)
    # plot_speed_profile(data, shape_id, figname, multidim=True)
    plot_force_profile(data)
    data.close()



if __name__=='__main__':
    # TODO: currently dumps output into input file

    import sys
    main(sys.argv)





























