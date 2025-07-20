import cv2
import os
from time import sleep
from simple_term_menu import TerminalMenu
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from edge_processing import (
    multi_scale_scharr_detection, 
    extract_edges_contours,
    adaptive_preprocessing,
    estimate_noise_level
)
from point_processing import refine_sequences

def draw_edges(image, edges, edge_sequences, max_sequences=10000, pause=False):
    if not edge_sequences:
        print("No sequences to draw!")
        return
    
    plt.figure(figsize=(20, 5))
    plt.ion()
    
    # Original image
    plt.subplot(1, 3, 1)
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title('Original Image')
    plt.axis('off')
    
    # Edge detection result
    plt.subplot(1, 3, 2)
    plt.imshow(edges, cmap='gray')
    plt.title('Multi-Scale Edge Detection')
    plt.axis('off')
    sleep(0.5)
    
    # Draw sequenced edges
    plt.subplot(1, 3, 3)
    plt.title('Drawing')
    plt.axis('off')
    plt.imshow(np.ones((image.shape[0], image.shape[1], 3), dtype=np.uint8) * 255)
    plt.xlim(0, image.shape[1])
    plt.ylim(image.shape[0], 0)
    
    # Sort sequences by length for better visualization
    sorted_sequences = sorted(edge_sequences, key=len, reverse=True)
    
    # Generate random colors for each sequence
    num_sequences = min(len(sorted_sequences), max_sequences)

    colors = np.random.rand(num_sequences, 3)  # Random RGB colors
    
    with tqdm(total=num_sequences, desc="Drawing") as pbar:
        for i, sequence in enumerate(sorted_sequences[:max_sequences]):
            if len(sequence) > 1:
                seq_array = np.array(sequence)
            
                # Draw the sequence point by point
                for j in range(len(seq_array) - 1):
                    # Draw line segment from point j to point j+1
                    plt.plot([seq_array[j, 0], seq_array[j+1, 0]], 
                            [seq_array[j, 1], seq_array[j+1, 1]], 
                            color=colors[i], linewidth=1)
                    if pause:
                        plt.pause(0.0001)
                    plt.draw()
                    pbar.update(1)
    
    
    plt.ioff()
    print("Drawing complete.")
    plt.show()

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

def main():
    # Get images from 'images' directory
    images = os.listdir('images')
    if not images:
        print("No images found in the 'images' directory.")
        return
    
    image_index = TerminalMenu(images, title="select an image").show()
    if image_index is None:
        print("Error: Image not found.")
        return

    
    pause = not TerminalMenu(["Yes", "No"], title="pause when drawing?").show() # Index 0 or 1
    image_path = os.path.join('images', images[image_index])
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not read image {image_path}")
        return

    # Analyze image properties
    image_props = analyze_image_properties(image)
    
    # Auto-detect noise level based on image properties
    if image_props['laplacian_var'] > 2000 or image_props['edge_density'] > 0.15:
        noise_level = 'high'
    elif image_props['laplacian_var'] > 500 or image_props['edge_density'] > 0.08:
        noise_level = 'medium'
    else:
        noise_level = 'low'
    
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
        simplify_epsilon = 2.5
        min_refined_length = 5
    elif len(edge_sequences) > 200:
        simplify_epsilon = 1.5
        min_refined_length = 3
    else:
        # Few sequences - preserve more detail
        simplify_epsilon = 1.0
        min_refined_length = 3
    
    print(f"Refinement parameters: simplify_epsilon={simplify_epsilon}, min_length={min_refined_length}")
    
    refined_sequences = refine_sequences(
        edge_sequences, 
        min_length=min_refined_length,
        simplify_epsilon=simplify_epsilon
    )
    
    if not refined_sequences:
        print("Warning: Refining removed all sequences! Adjust refinement parameters.")
        return
    
    print(f"\nSummary:")
    print(f"  Original sequences: {len(edge_sequences)}")
    print(f"  Refined sequences: {len(refined_sequences)}")
    print(f"  Reduction: {len(edge_sequences) - len(refined_sequences)} sequences removed")
    
    print("\nVisualizing Edge Sequences:")
    draw_edges(image, edges, refined_sequences, pause=pause)

if __name__ == "__main__":
    main()