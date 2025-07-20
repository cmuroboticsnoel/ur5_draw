import cv2
import numpy as np
import os

# Create debug directory if it doesn't exist
if not os.path.exists('debug'):
    os.makedirs('debug')

#-------------------------------------
# Image preprocessing functions
#-------------------------------------
def preprocess_image(image, denoise_strength=10, bilateral_d=9, 
                     bilateral_sigma_color=75, bilateral_sigma_space=75,
                     morph_kernel_size=3, use_clahe=True):
    """
    Comprehensive preprocessing to reduce noise and simplify image
    
    Parameters:
    - denoise_strength: Non-local means denoising strength (higher = more denoising)
    - bilateral_d: Diameter of bilateral filter
    - bilateral_sigma_color: Bilateral filter color similarity
    - bilateral_sigma_space: Bilateral filter spatial similarity
    - morph_kernel_size: Size of morphological operations kernel
    - use_clahe: Whether to apply CLAHE for contrast enhancement
    """
    print("Preprocessing image...")
    
    # Convert to grayscale if needed
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
    
    # Save original
    cv2.imwrite('debug/01_original_gray.png', gray)
    
    # Step 1: Non-local means denoising
    print(f"  1. Applying non-local means denoising (h={denoise_strength})...")
    denoised = cv2.fastNlMeansDenoising(gray, h=denoise_strength)
    cv2.imwrite('debug/02_denoised.png', denoised)
    
    # Step 2: Bilateral filter for edge-preserving smoothing
    print(f"  2. Applying bilateral filter (d={bilateral_d})...")
    bilateral = cv2.bilateralFilter(denoised, bilateral_d, bilateral_sigma_color, bilateral_sigma_space)
    cv2.imwrite('debug/03_bilateral.png', bilateral)
    
    # Step 3: CLAHE for contrast enhancement (optional)
    if use_clahe:
        print("  3. Applying CLAHE for contrast enhancement...")
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(bilateral)
        cv2.imwrite('debug/04_clahe.png', enhanced)
    else:
        enhanced = bilateral
    
    # Step 4: Morphological operations to remove small features
    if morph_kernel_size > 0:
        print(f"  4. Applying morphological operations (kernel={morph_kernel_size})...")
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel_size, morph_kernel_size))
        
        # Opening to remove small bright spots
        opened = cv2.morphologyEx(enhanced, cv2.MORPH_OPEN, kernel)
        
        # Closing to fill small dark gaps
        morphed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        cv2.imwrite('debug/05_morphological.png', morphed)
    else:
        morphed = enhanced
    
    # Step 5: Additional Gaussian smoothing
    print("  5. Final Gaussian smoothing...")
    final = cv2.GaussianBlur(morphed, (5, 5), 1.0)
    cv2.imwrite('debug/06_final_preprocessed.png', final)
    
    print("Preprocessing complete.")
    return enhanced

def adaptive_preprocessing(image, noise_level='medium'):
    """
    Adaptive preprocessing based on estimated noise level
    
    Parameters:
    - noise_level: 'low', 'medium', or 'high'
    """
    presets = {
        'low': {
            'denoise_strength': 5,
            'bilateral_d': 5,
            'bilateral_sigma_color': 50,
            'bilateral_sigma_space': 50,
            'morph_kernel_size': 0,
            'use_clahe': False
        },
        'medium': {
            'denoise_strength': 10,
            'bilateral_d': 9,
            'bilateral_sigma_color': 75,
            'bilateral_sigma_space': 75,
            'morph_kernel_size': 3,
            'use_clahe': True
        },
        'high': {
            'denoise_strength': 20,
            'bilateral_d': 15,
            'bilateral_sigma_color': 100,
            'bilateral_sigma_space': 100,
            'morph_kernel_size': 5,
            'use_clahe': True
        }
    }
    
    params = presets.get(noise_level, presets['medium'])
    return preprocess_image(image, **params)

def estimate_noise_level(image):
    """
    Estimate noise level in the image using Laplacian variance
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    variance = laplacian.var()
    
    # Thresholds for noise levels (adjust based on your images)
    if variance < 500:
        return 'low'
    elif variance < 2000:
        return 'medium'
    else:
        return 'high'

#-------------------------------------
# Zhang-Suen thinning algorithm
#-------------------------------------
def zhang_suen_thinning(binary_image):
    """Zhang-Suen thinning algorithm"""
    # Convert to uint8
    binary_image = binary_image.astype(np.uint8)
    
    # Initialize the thinning
    thinned = np.zeros(binary_image.shape, dtype=np.uint8)
    
    # Apply the thinning algorithm
    thinned = cv2.ximgproc.thinning(
        binary_image, 
        thinningType=cv2.ximgproc.THINNING_ZHANGSUEN
    )
    
    return thinned

#------------------------------------- 
# Image processing functions
#-------------------------------------
def multi_scale_scharr_detection(preprocessed_gray, scales=[1, 2, 4], weights=None):
    """Multi-scale Scharr edge detection for better connectivity"""
    
    if weights is None:
        weights = [1.0, 0.7, 0.4]
    
    combined_edges = np.zeros(preprocessed_gray.shape, dtype=np.float64)
    
    print("Performing multi-scale edge detection:")
    for i, scale in enumerate(scales):
        if scale > 1:
            sigma = scale * 0.8
            ksize = int(6 * sigma) | 1  # Ensure odd kernel size
            blurred = cv2.GaussianBlur(preprocessed_gray, (ksize, ksize), sigma)
        else:
            blurred = cv2.GaussianBlur(preprocessed_gray, (3, 3), 0)
        
        Gx = cv2.Scharr(blurred, cv2.CV_64F, 1, 0)
        Gy = cv2.Scharr(blurred, cv2.CV_64F, 0, 1)
        edges = np.sqrt(Gx**2 + Gy**2)
        
        if i < len(weights):
            weight = weights[i]
        else:
            weight = 0.2
        
        print(f"  Scale {scale}: weight={weight}, max={np.max(edges):.1f}")
        combined_edges += weight * edges
    
    combined_edges = cv2.normalize(combined_edges, None, 0, 255, cv2.NORM_MINMAX)
    
    # Save debug image
    cv2.imwrite('debug/combined_edges.png', combined_edges.astype(np.uint8))
    
    print(f"Combined edges: min={np.min(combined_edges):.1f}, max={np.max(combined_edges):.1f}")
    return combined_edges

def thin_edges_zhang_suen(edges, threshold=50):
    """Zhang-Suen thinning"""
    binary_edges = (edges > threshold).astype(np.uint8) * 255
    
    # Save debug image
    cv2.imwrite('debug/binary_edges.png', binary_edges)
    
    thinned = zhang_suen_thinning(binary_edges)
    
    # Save debug image
    cv2.imwrite('debug/thinned_edges.png', thinned)
    
    print(f"Thinned edges: {np.sum(thinned > 0)} pixels")
    return thinned

def post_process_edges(edges, min_edge_length=5):
    """
    Post-process edges to remove small components
    
    Parameters:
    - min_edge_length: Minimum edge length to keep (in pixels)
    """
    # Find connected components
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        (edges > 0).astype(np.uint8), connectivity=8
    )
    
    # Create output image
    filtered_edges = np.zeros_like(edges)
    
    # Keep only components larger than threshold
    for i in range(1, num_labels):  # Skip background (label 0)
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_edge_length:
            filtered_edges[labels == i] = edges[labels == i]
    
    removed = num_labels - 1 - np.sum(stats[1:, cv2.CC_STAT_AREA] >= min_edge_length)
    print(f"Removed {removed} small components (< {min_edge_length} pixels)")
    
    return filtered_edges

def extract_edges_contours(edges, threshold=50, min_length=10, min_component_size=5):
    """Edge extraction using OpenCV contour tracing"""
    print(f"Thresholding edges at {threshold}...")
    thinned = thin_edges_zhang_suen(edges, threshold)
    
    if np.sum(thinned) == 0:
        print("Warning: No edge pixels after thinning!")
        return []
    
    # Post-process to remove small components
    filtered = post_process_edges(thinned, min_component_size)
    cv2.imwrite('debug/filtered_edges.png', filtered)
    
    # Find contours using OpenCV
    contours, _ = cv2.findContours(
        filtered, 
        cv2.RETR_LIST, 
        cv2.CHAIN_APPROX_NONE
    )
    
    print(f"Found {len(contours)} contours")
    
    edge_sequences = []
    
    for contour in contours:
        # Convert contour to list of points
        points = contour.squeeze()
        
        # Handle single point contours
        if points.ndim == 1:
            points = [points.tolist()]
        else:
            points = points.tolist()
        
        # Remove duplicate endpoint for closed contours
        if len(points) > 2 and points[0] == points[-1]:
            points = points[:-1]
        
        if len(points) >= min_length:
            edge_sequences.append(points)
    
    print(f"Total sequences found: {len(edge_sequences)}")
    return edge_sequences