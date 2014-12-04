#! /usr/bin/python

"""
A library for computing Bezier Cubic
for an arbitrary set of control points
in R2 or R3 space

Cubic Segment:
C(t) = (1 - t)^3*b0 + 3(1 - t)*b1 + 3(1 - t)*t^2*b2 + t^3*b3

Bezier Spline of Cubic Segments:
B(t) = C_(i)(t-i+1), i-1 <= t <= i
where C0 continuity exists: C_(i)(1) = C_(i+1)(0)
where C1 continuity exists: C'_(i)(1) = C'_(i+1)(0)
and where C2 continuity exists: C"_(i)(1) = C"_(i+1)(0)

ex. usage:
import numpy
import bezier
points_array = numpy.array([[1, 2, 3], [4, 4, 4],
                            [6, 4, 6], [2, 5, 6],
                            [5, 6, 7]])
d_pts = bezier.compute_de_boor_control_pts(points_array)
bvals = bezier.compute_bvals(points_array, d_pts)
b_curve = bezier.compute_curve(bvals, 50)
#  plotting example
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.gca(projection='3d')
#plot bezier curve
ax.plot(b_curve[:,0], b_curve[:,1], b_curve[:,2])
#plot specified points
ax.plot(points_array[:,0], points_array[:,1], points_array[:,2], 'g*')
ax.set_title("Cubic Bezier Spline")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend(["Bezier Curve", "Control Points"], loc=2)
plt.show()
"""
import numpy as np


def compute_de_boor_control_pts(points_array, d0=None,
                                dN=None, natural=True):
    """
    Compute the de Boor control points for a given
    set for control points

    params:
        points_array: array of control points
            numpy.array of size N by k
            N is the number of input control points
            k is the number of dimensions for each point

        d0: the first control point - None if "natural"
            numpy.array of size 1 by k

        dN: the last control point - None if "natural"
            numpy.array of size 1 by k

        natural: flag to signify natural start/end conditions
            bool

    returns:
        d_pts: array of de Boor control points
            numpy.array of size N+3 by k
    """
    # N+3 auxiliary points required to compute d_pts
    # dpts_(-1) = x_(0)
    # dpts_(N+1) = x_(N)
    # so it is only necessary to find N+1 pts, dpts_(0) to to dpts_(N)
    (rows, k) = np.shape(points_array)
    N = rows - 1  # minus 1 because list includes x_(0)
    # Compute A matrix
    if natural:
        if N > 2:
            A = np.zeros((N-1, N-1))
            A[np.ix_([0], [0, 1])] = [4, 1]
            A[np.ix_([N-2], [N-3, N-2])] = [1, 4]
        else:
            A = 4.0
    else:
        if N > 2:
            A = np.zeros((N-1, N-1))
            A[np.ix_([0], [0, 1])] = [3.5, 1]
            A[np.ix_([N-2], [N-3, N-2])] = [1, 3.5]
        else:
            A = 3.5
    for i in range(1, N-2):
        A[np.ix_([i], [i-1, i, i+1])] = [1, 4, 1]
    # Construct de Boor Control Points from A matrix
    d_pts = np.zeros((N+3, k))
    for col in range(0, k):
        x = np.zeros((N-1, 1))
        # Compute start / end conditions
        if natural:
            x[N-2, 0] = 6*points_array[-2, col] - points_array[-1, col]
            x[0, 0] = 6*points_array[1, col] - points_array[0, col]
        else:
            x[N-2, 0] = 6*points_array[-2, col] - 1.5*dN[0, col]
            x[0, 0] = 6*points_array[1, col] - 1.5*d0[0, col]
        x[range(1, N-3+1), 0] = 6*points_array[range(2, N-2+1), col]
        # Solve bezier interpolation
        if N > 2:
            d_pts[2:N+1, col] = np.linalg.solve(A, x).T
        else:
            d_pts[2, col] = x / A
    # Store off start and end positions
    d_pts[0, :] = points_array[0, :]
    d_pts[-1, :] = points_array[-1, :]
    # Compute the second to last de Boor point based on end conditions
    if natural:
        one_third = (1.0/3.0)
        two_thirds = (2.0/3.0)
        d_pts[1, :] = (two_thirds)*points_array[0, :] + (one_third)*d_pts[2, :]
        d_pts[N+1, :] = ((one_third)*d_pts[-3, :] +
                         (two_thirds)*points_array[-1, :])
    else:
        d_pts[1, :] = d0
        d_pts[N+1, :] = dN
    return d_pts


def compute_bvals(points_array, d_pts):
    """
    Compute the Bezier control points for a given
    set for control pts and de Boor control pts.

    These B values are used to compute the cubic
    splines as follows:
    For each cubic spline segment:
    C(t) = (1 - t)^3*b0 + 3(1 - t)*b1
            + 3(1 - t)*t^2*b2 + t^3*b3

    params:
        points_array: array of control points
            numpy.array of size N by k
            N is the number of control points
            k is the number of dimensions for each point

        d_pts: array of de Boor control points
            numpy.array of size N+3 by k

    returns:
        bvals: 3D array of 4 Bezier control points
            for every control point, for every dimension
            of the control points
            numpy.array of size N by 4 by k
    """
    (rows, k) = np.shape(points_array)
    N = rows - 1  # N minus 1 because points array includes x_0
    bvals = np.zeros(shape=(k, N, 4))
    for i in range(0, N):
        points_array_i = i+1
        d_pts_i = i + 2
        if i == 0:
            for axis_pos in range(0, k):
                bvals[axis_pos, i, 0] = points_array[points_array_i - 1,
                                                     axis_pos]
                bvals[axis_pos, i, 1] = d_pts[d_pts_i - 1, axis_pos]
                bvals[axis_pos, i, 2] = (0.5 * d_pts[d_pts_i - 1, axis_pos]
                                         + 0.5 * d_pts[d_pts_i, axis_pos])
                bvals[axis_pos, i, 3] = points_array[points_array_i, axis_pos]
        elif i == N-1:
            for axis_pos in range(0, k):
                bvals[axis_pos, i, 0] = points_array[points_array_i - 1,
                                                     axis_pos]
                bvals[axis_pos, i, 1] = (0.5 * d_pts[d_pts_i - 1, axis_pos]
                                         + 0.5 * d_pts[d_pts_i, axis_pos])
                bvals[axis_pos, i, 2] = d_pts[d_pts_i, axis_pos]
                bvals[axis_pos, i, 3] = points_array[points_array_i, axis_pos]
        else:
            for axis_pos in range(0, k):
                bvals[axis_pos, i, 0] = points_array[points_array_i - 1,
                                                     axis_pos]
                bvals[axis_pos, i, 1] = (2.0/3.0 * d_pts[d_pts_i - 1, axis_pos]
                                         + 1.0/3.0 * d_pts[d_pts_i, axis_pos])
                bvals[axis_pos, i, 2] = (1.0/3.0 * d_pts[d_pts_i - 1, axis_pos]
                                         + 2.0/3.0 * d_pts[d_pts_i, axis_pos])
                bvals[axis_pos, i, 3] = points_array[points_array_i, axis_pos]

    return bvals


def compute_point(bvals, b_index, t):
    """
    Finds the k values that describe the current
    position along the bezier curve for k dimensions.

    params:
        bvals: 3D array of 4 Bezier control points
            for every control point, for every dimension
            of the control points
            numpy.array of size N by 4 by k
            N is the number of control points
            k is the number of dimensions for each point
        b_index: index position out between two of
            the N control points for this point in time
            int
        t: percentage of time that has passed between
            the two control points
            0 <= int <= 1.0

    returns:
        b_point: current position in k dimensions
            numpy.array of size 1 by k
    """
    if b_index < 0:
        b_point = bvals[:, 0, 0]
    elif b_index > bvals.shape[1]:
        b_point = bvals[:, -1, -1]
    else:
        t = 0.0 if t < 0.0 else t
        t = 1.0 if t > 1.0 else t
        num_axes = bvals.shape[0]
        b_point = np.zeros(num_axes)
        for current_axis in range(0, num_axes):
            (b0, b1, b2, b3) = bvals[current_axis, b_index-1, range(4)]
            b_point[current_axis] = (pow((1-t), 3)*b0 +
                                     3*pow((1-t), 2)*t*b1 +
                                     3*(1-t)*pow(t, 2)*b2 +
                                     pow(t, 3)*b3
                                     )
    return b_point


def compute_curve(bvals, num_intervals):
    """
    Iterpolation of the entire Bezier curve at once,
    using a specified number of intervals between
    control points.

    params:
        bvals: 3D array of 4 Bezier control points
            for every control point, for every dimension
            of the control points
            numpy.array of size N by 4 by k
            N is the number of control points
            k is the number of dimensions for each point
        num_intervals: the number of intervals between
            control points
            int > 0

    returns:
        b_curve: positions along the bezier curve in k dimensions
            numpy.array of size N*num_interval+1  by k
            (the +1 is to include the start position on the curve)
    """
    assert num_intervals > 0,\
        "Invalid number of intervals chosen (must be greater than 0)"
    interval = 1.0 / num_intervals
    (num_axes, num_bpts, _) = np.shape(bvals)
    b_curve = np.zeros((num_bpts*num_intervals+1, num_axes))
    # Copy out initial point
    b_curve[0, :] = bvals[:, 0, 0]
    for current_bpt in range(0, num_bpts):
        for current_axis in range(0, num_axes):
            (b0, b1, b2, b3) = bvals[current_axis, current_bpt, range(0, 4)]
            for iteration, t in enumerate(np.linspace(interval, 1,
                                                      num_intervals)):
                b_curve[(current_bpt*num_intervals+iteration+1),
                        current_axis] = (
                    pow((1-t), 3)*b0 + 3*pow((1-t), 2)*t*b1
                    + 3*(1-t)*pow(t, 2)*b2 + pow(t, 3)*b3
                )
    return b_curve
