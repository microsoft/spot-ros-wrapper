import numpy as np
import math

import rospy
import geometry_msgs.msg
import std_msgs.msg
import visualization_msgs.msg

from bosdyn.api import local_grid_pb2
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME


### Set of helper functions for local_grid processing
def get_terrain_markers(local_grid_proto):
    '''Receives raw proto from grid_client.get_local_grids(...) and returns marker array msg'''
    for local_grid in local_grid_proto:
        if local_grid.local_grid_type_name == "terrain": #TODO: Support parsing the terrain_valid and the intensity fields in the proto
            vision_tform_local_grid = get_a_tform_b(
                local_grid.local_grid.transforms_snapshot,
                VISION_FRAME_NAME,
                local_grid.local_grid.frame_name_local_grid_data).to_proto()
                
            cell_size = local_grid.local_grid.extent.cell_size
            terrain_pts = get_terrain_grid(local_grid)

    # terrain_pts is [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ...] (a list of lists, each of which is a point)
    # in the correct relative pose to Spot's body
    terrain_pts = offset_grid_pixels(terrain_pts, vision_tform_local_grid, cell_size)

    # Parse terrain_pts into Marker (cube_list)
    # Note: Using a single Marker of type CUBE_LIST allows for batch rendering, whereas MarkerArray of CUBEs does not
    marker = visualization_msgs.msg.Marker()
    marker.header.seq=0
    marker.id=0
    marker.header.stamp= rospy.Time()
    marker.header.frame_id= "vision_odometry_frame" #Must be a frame that exists (e.g. Spot's vision_odometry_frame)
    marker.type = visualization_msgs.msg.Marker.CUBE_LIST
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = cell_size
    marker.scale.y = cell_size
    marker.scale.z = cell_size
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1

    for terrain_pt in terrain_pts:
        p = geometry_msgs.msg.Point()
        c = std_msgs.msg.ColorRGBA()

        # Cube location
        p.x = terrain_pt[0]
        p.y = terrain_pt[1]
        p.z = terrain_pt[2]

        #Make color change with distance from Spot
        # max_dist = math.sqrt(local_grid_proto[0].local_grid.extent.num_cells_x**2 + local_grid_proto[0].local_grid.extent.num_cells_y**2)/2.0 * local_grid_proto[0].local_grid.extent.cell_size
        # c.r = math.sqrt(p.x**2+p.y**2)/(max_dist+0.1)
        # c.g = 1 - math.sqrt(p.x**2+p.y**2)/(max_dist)
        # c.b = math.sqrt(p.x**2+p.y**2)/(max_dist+0.1)
        # c.a = 1.0

        marker.points.append(p)
        # marker.colors.append(c)
        
    return marker

def get_terrain_grid(local_grid_proto):
    """Generate a 3xN set of points representing the terrain local grid."""
    cells_pz_full = unpack_grid(local_grid_proto).astype(np.float32)
    # Populate the x,y values with a complete combination of all possible pairs for the dimensions in the grid extent.
    ys, xs = np.mgrid[0:local_grid_proto.local_grid.extent.num_cells_x, 0:local_grid_proto.
                    local_grid.extent.num_cells_y]
    # Numpy vstack makes it so that each column is (x,y,z) for a single terrain point. The height values (z) come from the
    # terrain grid's data field.
    pts = np.vstack(
        [np.ravel(xs).astype(np.float32),
        np.ravel(ys).astype(np.float32), cells_pz_full]).T
    pts[:, [0, 1]] *= (local_grid_proto.local_grid.extent.cell_size,
                    local_grid_proto.local_grid.extent.cell_size)
    return pts

def unpack_grid(local_grid_proto):
    """Unpack the local grid proto."""
    # Determine the data type for the bytes data.
    data_type = get_numpy_data_type(local_grid_proto.local_grid)
    if data_type is None:
        print("Cannot determine the dataformat for the local grid.")
        return None
    # Decode the local grid.
    if local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RAW:
        full_grid = np.fromstring(local_grid_proto.local_grid.data, dtype=data_type)
    elif local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RLE:
        full_grid = expand_data_by_rle_count(local_grid_proto, data_type=data_type)
    else:
        # Return nothing if there is no encoding type set.
        return None
    # Apply the offset and scaling to the local grid.
    if local_grid_proto.local_grid.cell_value_scale == 0:
        return full_grid
    full_grid_float = full_grid.astype(np.float64)
    full_grid_float *= local_grid_proto.local_grid.cell_value_scale
    full_grid_float += local_grid_proto.local_grid.cell_value_offset
    return full_grid_float

def get_numpy_data_type(local_grid_proto):
    """Convert the cell format of the local grid proto to a numpy data type."""
    if local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT16:
        return np.uint16
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT16:
        return np.int16
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT8:
        return np.uint8
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT8:
        return np.int8
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT64:
        return np.float64
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT32:
        return np.float32
    else:
        return None

def expand_data_by_rle_count(local_grid_proto, data_type=np.int16):
    """Expand local grid data to full bytes data using the RLE count."""
    cells_pz = np.fromstring(local_grid_proto.local_grid.data, dtype=data_type)
    cells_pz_full = []
    # For each value of rle_counts, we expand the cell data at the matching index
    # to have that many repeated, consecutive values.
    for i in range(0, len(local_grid_proto.local_grid.rle_counts)):
        for j in range(0, local_grid_proto.local_grid.rle_counts[i]):
            cells_pz_full.append(cells_pz[i])
    return np.array(cells_pz_full)

def offset_grid_pixels(pts, vision_tform_local_grid, cell_size):
    """Offset the local grid's pixels to be in the world frame instead of the local grid frame."""
    x_base = vision_tform_local_grid.position.x + cell_size * 0.5
    y_base = vision_tform_local_grid.position.y + cell_size * 0.5
    pts[:, 0] += x_base
    pts[:, 1] += y_base
    return pts

