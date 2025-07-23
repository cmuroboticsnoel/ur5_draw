import numpy as np
import scipy.interpolate as interp
from tqdm import tqdm

#-------------------------------------
# Point manipulation and smoothing functions
#-------------------------------------
def remove_duplicate_points(points):
    """Remove consecutive duplicate points"""
    if not points:
        return points
        
    unique_points = [points[0]]
    for i in range(1, len(points)):
        if points[i] != points[i-1]:
            unique_points.append(points[i])
    return unique_points

def fit_spline_smoothing(points, smoothing_factor=0.1):
    """Fit B-spline with cumulative distance parameterization"""
    if len(points) < 4:
        return points  # Not enough points for spline fitting
    
    points = np.array(points)
    
    # Remove any duplicate points that might cause issues
    _, unique_indices = np.unique(points, axis=0, return_index=True)
    if len(unique_indices) < 4:
        return points.tolist()
    
    points = points[np.sort(unique_indices)]
    
    # Calculate cumulative distance for better parameterization
    dists = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
    cum_dists = np.concatenate(([0], np.cumsum(dists)))
    total_dist = cum_dists[-1]
    
    if total_dist < 1e-5:  # Avoid division by zero
        return points.tolist()
    
    t = cum_dists / total_dist
    
    try:
        # Fit smoothing spline with distance-based parameterization
        tck, u = interp.splprep(points.T, u=t, s=smoothing_factor * len(points), k=3)
        
        # Evaluate the spline at more points for smoothness
        num_points = max(10, int(total_dist))
        u_new = np.linspace(0, 1, num_points)
        smooth_points = np.array(interp.splev(u_new, tck)).T
        
        return smooth_points.tolist()
    except Exception as e:
        print(f"Spline fitting error: {e} - using original points")
        return points.tolist()

def douglas_puecker_simplify(points, epsilon=0.1):
    """Optimized Douglas-Peucker with vectorized distance calculation"""
    if len(points) <= 2:
        return points
    
    points = np.array(points)
    start, end = points[0], points[-1]
    
    if len(points) == 2:
        return [start.tolist(), end.tolist()]
    
    # Vectorized perpendicular distance calculation
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)
    
    if line_len < 1e-8:
        return [start.tolist(), end.tolist()]
    
    # Handle case where there are no intermediate points
    if len(points) == 2:
        return [points[0].tolist(), points[-1].tolist()]
    
    point_vecs = points[1:-1] - start
    cross_products = np.abs(np.cross(point_vecs, line_vec))
    distances = cross_products / line_len
    
    # Handle case where all distances are zero
    if len(distances) == 0:
        return [points[0].tolist(), points[-1].tolist()]
    
    max_idx = np.argmax(distances) + 1
    max_dist = distances[max_idx - 1]
    
    if max_dist > epsilon:
        # Convert to lists before recursion to avoid array shape issues
        left_segment = douglas_puecker_simplify(points[:max_idx + 1].tolist(), epsilon)
        right_segment = douglas_puecker_simplify(points[max_idx:].tolist(), epsilon)
        
        # Combine segments properly
        if not left_segment or not right_segment:
            return left_segment or right_segment
        return left_segment[:-1] + right_segment
    else:
        return [points[0].tolist(), points[-1].tolist()]

def refine_sequences(edge_sequences, min_length=3, simplify_epsilon=0.1):
    """Enhanced sequence refinement pipeline"""
    refined_sequences = []
    
    if not edge_sequences:
        print("No sequences to refine!")
        return []
    
    print(f"Refining {len(edge_sequences)} sequences...")
    
    for sequence in tqdm(edge_sequences, desc="Refining sequences"):
        if len(sequence) < min_length:
            continue
        
        try:
            # Remove duplicate points first
            sequence = remove_duplicate_points(sequence)
            
            # Only try to smooth sequences with enough points
            if len(sequence) >= 4:
                smoothed = fit_spline_smoothing(sequence)
            else:
                smoothed = sequence
                
            simplified = douglas_puecker_simplify(smoothed, simplify_epsilon)
            
            if len(simplified) >= min_length:
                refined_sequences.append(simplified)
        except Exception as e:
            print(f"Error refining sequence: {e} - using original")
            refined_sequences.append(sequence)
    
    print(f"Refined to {len(refined_sequences)} sequences")
    return refined_sequences