import os
import argparse
import sys
import numpy as np
import h5py
import pandas as pd
import zipfile
import shutil
import logging

filename = 'data/a=0_v=10_i=2.000_s=0.500_t=0.000.h5'
data_h5 = h5py.File(filename, "r", driver='core')
tip_pose = data_h5['tip']
object_pose = data_h5['object']
force = data_h5['force']
print('tip_pose', tip_pose)
print('object_pose', object_pose)
print('force', force)
data_h5.close()
