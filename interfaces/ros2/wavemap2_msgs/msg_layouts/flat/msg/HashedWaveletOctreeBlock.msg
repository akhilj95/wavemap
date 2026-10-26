Index3D root_node_offset
float32 root_node_scale_coefficient

# The block's octree nodes in depth-first preorder, stored as one array per
# field instead of the nested layout's WaveletOctreeNode[] nodes. Node i's
# detail coefficients are node_detail_coefficients[8*i : 8*i+8], and bit j of
# node_allocated_children_bitsets[i] is set if its j-th child follows.
# NOTE: Nodes have 7 detail coefficients. The 8th is always 0 and only kept
#       for parity with the nested layout's float32[8] detail_coefficients.
float32[] node_detail_coefficients
uint8[] node_allocated_children_bitsets
