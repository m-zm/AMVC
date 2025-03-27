from typing import List
import numpy as np
import numpy.typing as npt

eps = 1e-8

# assuming all points are 1d array like [x, y] 
Point = npt.NDArray[np.floating]

# distance between point and edge, positive if inside, assuming counter clockwise
def inside_distance(point : Point, from_point : Point, to_point : Point) -> np.floating:
    from_vec = from_point - point
    to_vec = to_point - point
    normal = np.cross(to_vec, from_vec)
    length = np.linalg.norm(from_point - to_point)
    height = normal / length
    return np.float64(height)

# assuming counter clockwise is positive
def calc_area(p0, p1, p2) -> np.float64:
    from_vec = p1 - p0
    to_vec = p2 - p0
    normal = np.cross(to_vec, from_vec)
    return  np.float64(normal)

def calc_wachpress_coordinate(point: Point, vertex_points: List[Point]) -> npt.NDArray[np.floating]:
    vertex_count = len(vertex_points)

    for i in range(vertex_count):
        from_point = vertex_points[i]
        to_point = vertex_points[(i+1)%vertex_count]
        distance = inside_distance(point, from_point, to_point)
        if np.abs(distance) < eps:
            result = np.zeros(vertex_count)
            from_distance = np.linalg.norm(from_point - point)
            to_distance = np.linalg.norm(to_point - point)
            total_distance = from_distance + to_distance
            result[i] = 1 - from_distance / total_distance
            result[(i+1)%vertex_count] = 1 - to_distance / total_distance
            return result

    As = np.zeros(vertex_count)
    for i in range(vertex_count):
        area = calc_area(point, vertex_points[i], vertex_points[(i+1)%vertex_count])
        As[i] = area
    Bs = np.zeros(vertex_count)
    for i in range(vertex_count):
        area = calc_area(vertex_points[(i-1)%vertex_count], vertex_points[i], vertex_points[(i+1)%vertex_count])
        Bs[i] = area
    ws = np.zeros(vertex_count)
    for i in range(vertex_count):
        ws[i] = Bs[i] / (As[(i-1)%vertex_count] * As[i])
    sum = ws.sum()
    return ws / sum
    
def calc_mean_value_coordinate(point : Point, vertex_points: List[Point])-> npt.NDArray[np.floating]:
    vertex_count = len(vertex_points)

    for i in range(vertex_count):
        from_point = vertex_points[i]
        to_point = vertex_points[(i+1)%vertex_count]
        distance = inside_distance(point, from_point, to_point)
        if np.abs(distance) < eps:
            result = np.zeros(vertex_count)
            from_distance = np.linalg.norm(from_point - point)
            to_distance = np.linalg.norm(to_point - point)
            total_distance = from_distance + to_distance
            result[i] = 1 - from_distance / total_distance
            result[(i+1)%vertex_count] = 1 - to_distance / total_distance
            return result
    
    tan_halfs = np.zeros(vertex_count)
    for i in range(vertex_count):
        from_point = vertex_points[i]
        to_point = vertex_points[(i+1)%vertex_count]
        from_vec = from_point - point
        to_vec = to_point - point
        from_distance = np.linalg.norm(from_vec)
        to_distance = np.linalg.norm(to_vec)
        cos = np.dot(from_vec, to_vec) / (from_distance * to_distance)
        alpha = np.arccos(cos)
        tan_half = np.tan(alpha/2)
        tan_halfs[i] = tan_half

    lengths = np.zeros(vertex_count)
    for i in range(vertex_count):
        opp_point = vertex_points[i]
        length = np.linalg.norm(opp_point - point)
        lengths[i] = length

    ws = np.zeros(vertex_count)
    for i in range(vertex_count):
        ws[i] = (tan_halfs[(i-1)%vertex_count] + tan_halfs[i]) / lengths[i]
    
    sum = ws.sum()
    return ws / sum