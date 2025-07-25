import cv2
import os
import json
import shutil
from simple_term_menu import TerminalMenu
import numpy as np
from edge_processing import (
    multi_scale_scharr_detection, 
    extract_edges_contours,
    adaptive_preprocessing,
)
from point_processing import refine_sequences
from constants import IMAGES_PATH, IMAGE_DESCRIPTION_PATH, DEBUG_COUNTER


def analyze_image_properties(image):
    """Analyze image properties to determine optimal parameters"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    
    # Calculate various metrics
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    mean_intensity = np.mean(gray)
    std_intensity = np.std(gray)
    
    # Estimate edge density
    edges_canny = cv2.Canny(gray, 50, 150)
    edge_density = np.sum(edges_canny > 0) / (gray.shape[0] * gray.shape[1])
    
    print(f"\nImage analysis:")
    print(f"  Laplacian variance: {laplacian_var:.1f}")
    print(f"  Edge density: {edge_density:.3f}")
    print(f"  Mean intensity: {mean_intensity:.1f}")
    print(f"  Std intensity: {std_intensity:.1f}")
    
    return {
        'laplacian_var': laplacian_var,
        'edge_density': edge_density,
        'mean_intensity': mean_intensity,
        'std_intensity': std_intensity
    }

def get_optimal_threshold(edges):
    """Automatically determine optimal edge threshold using Otsu's method"""
    # Normalize edges to 0-255 range
    edges_norm = cv2.normalize(edges, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    
    # Apply Otsu's thresholding
    otsu_thresh, _ = cv2.threshold(edges_norm, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # Adjust based on edge statistics
    edge_mean = np.mean(edges_norm[edges_norm > 20])
    
    # Use a weighted combination
    optimal_thresh = int(0.7 * otsu_thresh + 0.3 * edge_mean)
    
    # Clamp to reasonable range
    optimal_thresh = np.clip(optimal_thresh, 30, 80)
    
    return optimal_thresh

def transform_points_to_canvas(sequences, original_width, original_height):
    """
    Transform extracted points to fit within 800x600 landscape canvas
    with padding and centered positioning
    """
    # Target canvas dimensions
    canvas_width = 800
    canvas_height = 600
    
    # Calculate aspect ratios
    original_ratio = original_width / original_height
    canvas_ratio = canvas_width / canvas_height
    
    # Determine scaling factor
    if original_ratio > canvas_ratio:
        # Landscape image - scale to width
        scale = canvas_width / original_width
    else:
        # Portrait image - scale to height
        scale = canvas_height / original_height
    
    # Apply scaling
    scaled_width = original_width * scale
    scaled_height = original_height * scale
    
    # Calculate padding to center image
    pad_x = (canvas_width - scaled_width) / 2
    pad_y = (canvas_height - scaled_height) / 2
    
    # Transform all points
    transformed_sequences = []
    for sequence in sequences:
        transformed_sequence = []
        for point in sequence:
            x, y = point
            # Scale and add padding
            x_new = (x * scale) + pad_x
            y_new = (y * scale) + pad_y
            transformed_sequence.append([x_new, y_new])
        transformed_sequences.append(transformed_sequence)
    
    return transformed_sequences, scale, pad_x, pad_y

def export_to_json(sequences, image_path):
    """
    Export drawing sequences to JSON format for UR5 drawing node
    
    Parameters:
    - sequences: List of drawing sequences (each sequence is a list of [x, y] points)
    - image_path: Path to the source image (for metadata)
    - IMAGE_DESCRIPTION_PATH: Directory to save the JSON file
    """
    
    # Expand user path and ensure it exists
    output_path = os.path.expanduser(IMAGE_DESCRIPTION_PATH)
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    else:
        shutil.rmtree(output_path)
        os.makedirs(output_path)
    
    # Prepare data for JSON export
    # Ensure all points are in the correct format [x, y] as regular Python lists
    json_sequences = []
    for sequence in sequences:
        json_sequence = []
        for point in sequence:
            # Convert numpy arrays or other formats to standard Python lists
            if hasattr(point, '__iter__') and len(point) >= 2:
                json_sequence.append([float(point[0]), float(point[1])])
        
        if len(json_sequence) > 0:  # Only add non-empty sequences
            json_sequences.append(json_sequence)
    
    # Get base filename without extension
    image_name = os.path.splitext(os.path.basename(image_path))[0]
    
    # Prepare metadata
    metadata = {
        'source_image': os.path.basename(image_path),
        'total_sequences': len(json_sequences),
        'total_points': sum(len(seq) for seq in json_sequences),
        'image_dimensions': {
            'width': 800,  # Should match the parameters in the ROS node
            'height': 600
        },
        'processing_timestamp': __import__('datetime').datetime.now().isoformat()
    }
    
    # Save sequences JSON file
    json_filename = f"image_description.json"
    json_path = os.path.join(output_path, json_filename)
    
    try:
        with open(json_path, 'w') as f:
            json.dump(json_sequences, f, indent=2)  # Export just the sequences array as expected by the node
        
        print(f"\n✓ Exported {len(json_sequences)} sequences to: {json_path}")
        print(f"  Total drawing points: {sum(len(seq) for seq in json_sequences)}")
        print(f"  File size: {os.path.getsize(json_path)} bytes")
        
        # Also save metadata separately for reference
        metadata_filename = f"{image_name}_metadata.json"
        metadata_path = os.path.join(output_path, metadata_filename)
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"  Metadata saved to: {metadata_path}")
        
        return json_path
        
    except Exception as e:
        print(f"✗ Error exporting JSON: {e}")
        return None

def validate_sequences_for_robot(sequences, image_width=800, image_height=600):
    """
    Validate that sequences are suitable for robot drawing
    
    Parameters:
    - sequences: List of drawing sequences
    - image_width: Expected image width
    - image_height: Expected image height
    """
    print(f"\nValidating sequences for robot drawing:")
    
    valid_sequences = []
    total_points = 0
    out_of_bounds_count = 0
    
    for i, sequence in enumerate(sequences):
        valid_points = []
        
        for point in sequence:
            x, y = point[0], point[1]
            
            # Check if point is within image bounds
            if 0 <= x < image_width and 0 <= y < image_height:
                valid_points.append(point)
            else:
                out_of_bounds_count += 1
        
        if len(valid_points) >= 2:  # Need at least 2 points to draw a line
            valid_sequences.append(valid_points)
            total_points += len(valid_points)
    
    if out_of_bounds_count > 0:
        print(f"  Warning: {out_of_bounds_count} points were out of bounds and filtered")
    
    print(f"  ✓ {len(valid_sequences)} valid sequences ({len(sequences) - len(valid_sequences)} filtered out)")
    print(f"  ✓ {total_points} total valid points")
    if len(valid_sequences) > 0:
        print(f"  ✓ Average points per sequence: {total_points / len(valid_sequences):.1f}")
    
    return valid_sequences

def main():
    # Get images from 'images' directory
    images = os.listdir(IMAGES_PATH)
    if not images:
        print("No images found in the 'images' directory.")
        return
    
    image_index = TerminalMenu(images, title="select an image").show()
    if image_index is None:
        print("Error: Image not found.")
        return
    
    image_path = os.path.join(IMAGES_PATH, images[image_index])    
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not read image {image_path}")
        return

    print(f"Processing image: {os.path.basename(image_path)}")

    # Analyze image properties
    image_props = analyze_image_properties(image)
    
    # Get original image dimensions
    original_height, original_width = image.shape[:2]
    
    # Auto-detect noise level based on image properties
    if image_props['laplacian_var'] > 1000000 or image_props['edge_density'] > 1.50:
        noise_level = 'high'
    elif image_props['laplacian_var'] > 50000 or image_props['edge_density'] > 1.00:
        noise_level = 'medium'
    else:
        noise_level = 'low'

    print(f"\nAuto-detected noise level: {noise_level}")
    
    # Apply adaptive preprocessing
    preprocessed = adaptive_preprocessing(image, noise_level)
    
    print("\nDetecting edges with multi-scale Scharr...")
    
    # Adjust edge detection parameters based on image properties
    if noise_level == 'high':
        scales = [2, 4, 6]  # Larger scales for noisy images
        weights = [0.6, 1.0, 0.8]
    elif noise_level == 'medium':
        scales = [1, 3, 5]
        weights = [0.8, 1.0, 0.6]
    else:
        scales = [1, 2, 3]  # Finer scales for clean images
        weights = [1.0, 0.8, 0.5]
    
    edges = multi_scale_scharr_detection(
        preprocessed,
        scales=scales,
        weights=weights
    )
    
    # Automatically determine optimal threshold
    threshold = get_optimal_threshold(edges)
    print(f"\nAuto-selected edge threshold: {threshold}")
    
    # Adjust other parameters based on noise level
    if noise_level == 'high':
        min_length = 20
        min_component_size = 80
    elif noise_level == 'medium':
        min_length = 15
        min_component_size = 50
    else:
        min_length = 10
        min_component_size = 15
    
    print(f"Edge extraction parameters: min_length={min_length}, min_component_size={min_component_size}")
    
    edge_sequences = extract_edges_contours(
        edges, 
        threshold=threshold, 
        min_length=min_length,
        min_component_size=min_component_size
    )
    
    if not edge_sequences:
        print("Warning: No edge sequences found! Trying with lower threshold...")
        # Automatically retry with lower threshold
        threshold = max(10, threshold - 20)
        min_length = max(5, min_length - 5)
        min_component_size = max(20, min_component_size - 20)
        
        print(f"Retrying with: threshold={threshold}, min_length={min_length}, min_component_size={min_component_size}")
        edge_sequences = extract_edges_contours(
            edges, 
            threshold=threshold, 
            min_length=min_length,
            min_component_size=min_component_size
        )
    
    if not edge_sequences:
        print("Still no sequences found. Exiting.")
        return
    
    print("\nRefining sequences with spline smoothing...")
    
    # Adjust refinement parameters based on number of sequences
    if len(edge_sequences) > 500:
        # Many sequences - be more aggressive with simplification
        simplify_epsilon = 2.50
        min_refined_length = 15
    elif len(edge_sequences) > 200:
        simplify_epsilon = 0.25
        min_refined_length = 7
    else:
        # Few sequences - preserve more detail
        simplify_epsilon = 0.05
        min_refined_length = 1
    
    print(f"Refinement parameters: simplify_epsilon={simplify_epsilon}, min_length={min_refined_length}")
    
    refined_sequences = refine_sequences(
        edge_sequences, 
        min_length=min_refined_length,
        simplify_epsilon=simplify_epsilon
    )
    
    if not refined_sequences:
        print("Warning: Refining removed all sequences! Using original sequences.")
        refined_sequences = edge_sequences
    
    print(f"\nSequence Processing Summary:")
    print(f"  Original sequences: {len(edge_sequences)}")
    print(f"  Refined sequences: {len(refined_sequences)}")
    print(f"  Reduction: {len(edge_sequences) - len(refined_sequences)} sequences removed")
    
    # ====== NEW SECTION: Transform points to canvas ======
    print("\nTransforming points to fit 800x600 canvas...")
    
    # Transform points to fit canvas
    transformed_sequences, scale, pad_x, pad_y = transform_points_to_canvas(
        refined_sequences,
        original_width,
        original_height
    )
    
    print(f"  Applied scaling factor: {scale:.4f}")
    print(f"  Horizontal padding: {pad_x:.1f} pixels")
    print(f"  Vertical padding: {pad_y:.1f} pixels")
    
    # ====== END NEW SECTION ======
    
    # Automatically validate and export sequences for robot use
    print("\nPreparing sequences for UR5 robot...")
    
    # Validate and clean sequences for robot use
    robot_sequences = validate_sequences_for_robot(transformed_sequences, 
                                                 image_width=800, 
                                                 image_height=600)
    
    if robot_sequences:
        # Export to sequences directory
        json_path = export_to_json(robot_sequences, image_path)
        
        if json_path:
            print(f"\nProcessing complete for {os.path.basename(image_path)}")
            
            # Show transformation summary
            print("\nCanvas Transformation Summary:")
            print(f"  Original dimensions: {original_width} x {original_height}")
            print(f"  Scaled dimensions: {original_width*scale:.1f} x {original_height*scale:.1f}")
            print(f"  Canvas dimensions: 800 x 600")
            print(f"  Horizontal padding: {pad_x:.1f} pixels")
            print(f"  Vertical padding: {pad_y:.1f} pixels")
            
            # Print sample point before/after transformation
            if refined_sequences and refined_sequences[0]:
                original_point = refined_sequences[0][0]
                transformed_point = robot_sequences[0][0]
                print(f"\nSample point transformation:")
                print(f"  Original: ({original_point[0]:.1f}, {original_point[1]:.1f})")
                print(f"  Transformed: ({transformed_point[0]:.1f}, {transformed_point[1]:.1f})")
        else:
            print("✗ Failed to export JSON file!")
    else:
        print("✗ No valid sequences for robot drawing!")

if __name__ == "__main__":
    main()