float32 min_cell_width
float32 min_log_odds
float32 max_log_odds

int32 tree_height

float32 root_node_scale_coefficient

# The octree's nodes in depth-first preorder, stored as one array per field
# instead of the nested layout's WaveletOctreeNode[] nodes. Node i's detail
# coefficients are node_detail_coefficients[8*i : 8*i+8], and bit j of
# node_allocated_children_bitsets[i] is set if its j-th child follows.
# NOTE: Nodes have 7 detail coefficients. The 8th is always 0 and only kept
#       for parity with the nested layout's float32[8] detail_coefficients.
float32[] node_detail_coefficients
uint8[] node_allocated_children_bitsets
