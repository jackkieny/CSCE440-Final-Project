from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import scipy as sp
import scipy.interpolate
import open3d as o3d
from geomdl import BSpline
from geomdl import utilities
from geomdl import exchange
from geomdl.visualization import VisMPL
from plyfile import PlyData, PlyElement
import pandas as pd
import random

x = [0,2,0,-2,-2,2]
y = [0,3,0,-3,3,-3]
z = [20,30,40,30,30,30]
fig = plt.figure(figsize=(10,6))
ax = axes3d.Axes3D(fig)
ax.scatter3D(x, y, z, c='r')
plt.show()

surf = BSpline.Surface()
surf.degree_u = 1
surf.degree_v = 1
control_points = [[0,0,20],
                  [-2,-3,30],
                  [2,3,30],
                  [-2,3,30],
                  [2,-3,30],
                  [0,0,20],
                  [0,0,40],
                  [-2,-3,30],
                  [2,3,30],
                  [-2,3,30],
                  [2,-3,30],
                  [0,0,40]]


surf.set_ctrlpts(control_points, 6, 2)
surf.knotvector_u = utilities.generate_knot_vector(surf.degree_u, 6)
surf.knotvector_v = utilities.generate_knot_vector(surf.degree_v, 2) 

 # Evaluate surface
surf.evaluate()
vis_comp = VisMPL.VisSurface()
surf.vis = vis_comp
surf.render()
 # Save surface as a .obj file
exchange.export_obj(surf, "b-spline_surf.obj")
