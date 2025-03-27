from argparse import ArgumentParser
import math
import numpy as np
import matplotlib.path as mplpath
from scipy.spatial import Delaunay, delaunay_plot_2d
from matplotlib import pyplot as plt

from calc_barycentric_coordinate import calc_wachpress_coordinate

eps = 1e-8

parser = ArgumentParser()
parser.add_argument("-n", required=True, type=int, help="顶点数")
parser.add_argument("--r_interval", "-r", required=True, type=float, help="半径间隔")
parser.add_argument("--c_interval", "-c", required=True, type=float, help="周向间隔")
parser.add_argument("--b_interval", "-b", required=True, type=float, help="边界间隔")
parser.add_argument("--filename", "-f", type=str, default="ngon.off", help="输出文件名")
parser.add_argument("--coordinates", "-cd", nargs="+", type=float, help="顶点坐标")

args = parser.parse_args()

vertex_count : int = args.n
theta = 2* math.pi / vertex_count
thetas = theta * np.arange(vertex_count)
v_xs = np.cos(thetas)
v_ys = np.sin(thetas)
vertex_points = []
for i in range(vertex_count):
    vertex_points.append(np.array([v_xs[i], v_ys[i]]))
polygon_path = mplpath.Path([point * (1 - eps) for point in vertex_points])
total_vertex_count = len(vertex_points)

boundary_interval : float = args.b_interval
edgh_length = 2 * math.sin(math.pi / vertex_count)
boundary_count = math.ceil(edgh_length / boundary_interval)
weight1 = np.arange(1, boundary_count) / boundary_count
weight2 = 1 - weight1
boundary_points = []
for i in range(vertex_count):
    points = []
    from_v = vertex_points[i]
    to_v = vertex_points[(i+1)%vertex_count]
    xs = from_v[0] * weight1 + to_v[0] * weight2
    ys = from_v[1] * weight1 + to_v[1] * weight2
    for x, y in zip(xs, ys):
        points.append(np.array([x, y]))
    boundary_points = boundary_points + points
total_boundary_count = len(boundary_points)

r_interval : float = args.r_interval
c_interval : float = args.c_interval
r_count = math.ceil(1 / r_interval)
internal_points = [np.array([0.0, 0.0])]
for i in range(1, r_count):
    points = []
    r = i * r_interval
    c_count = math.ceil(np.pi / (np.arcsin(c_interval / r)))
    theta_interval = 2 * np.pi / c_count
    thetas = theta_interval * np.arange(c_count)
    xs = r * np.cos(thetas)
    ys = r * np.sin(thetas)
    for i in range(c_count):
        point = (xs[i], ys[i])
        if polygon_path.contains_point(point):
            points.append(np.array(point))
    internal_points = internal_points + points
total_internal_count = len(internal_points)

total_point_count = total_vertex_count + total_boundary_count + total_internal_count

delaunay = Delaunay(vertex_points + boundary_points + internal_points)

simplices = delaunay.simplices
simplices_count = len(simplices)

off_filename = args.filename + ".off"
coord_filename = args.filename + ".coord"
off_file = open(off_filename, "w")
coord_file = open(coord_filename, "w")
off_file.write("OFF\n")
off_file.write(f"{total_point_count} {simplices_count} 0\n")
for p in vertex_points + boundary_points + internal_points:
    off_file.write(f"{p[0]} {p[1]} 0\n")
    weights = calc_wachpress_coordinate(p, vertex_points)
    coord_file.write(" ".join(map(str, weights)) + "\n")
for s in simplices:
    off_file.write("3")
    for i in s:
        off_file.write(f" {i}")
    off_file.write("\n")
off_file.close()
