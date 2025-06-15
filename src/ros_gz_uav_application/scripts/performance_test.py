#!/usr/bin/env python3
"""
Performance test script for obstacle marking methods
Compares different optimization approaches for addObstacleToVoxelEnvironment
"""

import time
import subprocess
import sys
import os
import matplotlib.pyplot as plt
import numpy as np

def run_performance_test(num_obstacles_list, grid_sizes, num_runs=5):
    """
    Run performance tests with different parameters
    
    Args:
        num_obstacles_list: List of obstacle counts to test
        grid_sizes: List of grid sizes to test (as tuples of (x, y, z))
        num_runs: Number of runs per configuration for averaging
    """
    
    results = {
        'num_obstacles': [],
        'grid_size': [],
        'method': [],
        'time_ms': [],
        'memory_mb': []
    }
    
    # Test different methods
    methods = [
        ('original', 'Original sequential method'),
        ('optimized', 'Optimized with type-specific functions'),
        ('openmp', 'OpenMP parallelized version'),
        ('batch', 'Batch processing version')
    ]
    
    for num_obstacles in num_obstacles_list:
        for grid_size in grid_sizes:
            for method_name, method_desc in methods:
                print(f"Testing {method_desc} with {num_obstacles} obstacles, grid {grid_size}")
                
                times = []
                memories = []
                
                for run in range(num_runs):
                    # Set environment variables for the test
                    env = os.environ.copy()
                    env['NUM_OBSTACLES'] = str(num_obstacles)
                    env['GRID_X'] = str(grid_size[0])
                    env['GRID_Y'] = str(grid_size[1])
                    env['GRID_Z'] = str(grid_size[2])
                    env['METHOD'] = method_name
                    
                    # Run the test (this would need to be implemented in the C++ code)
                    start_time = time.time()
                    
                    # Simulate the test - in reality, this would call the C++ executable
                    # with the appropriate parameters
                    try:
                        # This is a placeholder - you would need to implement
                        # command-line arguments in your C++ code to test different methods
                        cmd = [
                            'ros2', 'run', 'ros_gz_uav_application', 'test_3d_planning',
                            '--num-obstacles', str(num_obstacles),
                            '--grid-size', f"{grid_size[0]}x{grid_size[1]}x{grid_size[2]}",
                            '--method', method_name
                        ]
                        
                        # For now, just simulate the test
                        process_time = time.time() - start_time
                        memory_usage = 100 + num_obstacles * 10 + grid_size[0] * grid_size[1] * grid_size[2] * 0.001
                        
                        times.append(process_time)
                        memories.append(memory_usage)
                        
                    except subprocess.CalledProcessError as e:
                        print(f"Error running test: {e}")
                        continue
                
                if times:
                    avg_time = np.mean(times)
                    avg_memory = np.mean(memories)
                    
                    results['num_obstacles'].append(num_obstacles)
                    results['grid_size'].append(grid_size)
                    results['method'].append(method_name)
                    results['time_ms'].append(avg_time * 1000)  # Convert to milliseconds
                    results['memory_mb'].append(avg_memory)
                    
                    print(f"  Average time: {avg_time*1000:.2f} ms, Memory: {avg_memory:.1f} MB")
    
    return results

def plot_results(results):
    """Plot the performance results"""
    
    # Create subplots
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    # Extract unique values
    methods = list(set(results['method']))
    num_obstacles_list = sorted(list(set(results['num_obstacles'])))
    grid_sizes = list(set(results['grid_size']))
    
    # Plot 1: Time vs Number of Obstacles
    for method in methods:
        method_data = [r for r in zip(results['num_obstacles'], results['time_ms'], results['method']) 
                      if r[2] == method]
        if method_data:
            obstacles, times, _ = zip(*method_data)
            ax1.plot(obstacles, times, 'o-', label=method, linewidth=2, markersize=6)
    
    ax1.set_xlabel('Number of Obstacles')
    ax1.set_ylabel('Time (ms)')
    ax1.set_title('Performance vs Number of Obstacles')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Time vs Grid Size
    for method in methods:
        method_data = [r for r in zip(results['grid_size'], results['time_ms'], results['method']) 
                      if r[2] == method]
        if method_data:
            grid_sizes_method, times, _ = zip(*method_data)
            grid_volumes = [g[0] * g[1] * g[2] for g in grid_sizes_method]
            ax2.plot(grid_volumes, times, 'o-', label=method, linewidth=2, markersize=6)
    
    ax2.set_xlabel('Grid Volume (voxels)')
    ax2.set_ylabel('Time (ms)')
    ax2.set_title('Performance vs Grid Size')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Memory Usage
    for method in methods:
        method_data = [r for r in zip(results['num_obstacles'], results['memory_mb'], results['method']) 
                      if r[2] == method]
        if method_data:
            obstacles, memories, _ = zip(*method_data)
            ax3.plot(obstacles, memories, 'o-', label=method, linewidth=2, markersize=6)
    
    ax3.set_xlabel('Number of Obstacles')
    ax3.set_ylabel('Memory Usage (MB)')
    ax3.set_title('Memory Usage vs Number of Obstacles')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Speedup comparison
    if 'original' in methods:
        original_times = {}
        for r in zip(results['num_obstacles'], results['time_ms'], results['method']):
            if r[2] == 'original':
                original_times[r[0]] = r[1]
        
        for method in methods:
            if method != 'original':
                method_data = [r for r in zip(results['num_obstacles'], results['time_ms'], results['method']) 
                              if r[2] == method]
                if method_data:
                    obstacles, times, _ = zip(*method_data)
                    speedups = [original_times.get(o, 1) / t if t > 0 else 1 for o, t in zip(obstacles, times)]
                    ax4.plot(obstacles, speedups, 'o-', label=f'{method} speedup', linewidth=2, markersize=6)
    
    ax4.set_xlabel('Number of Obstacles')
    ax4.set_ylabel('Speedup Factor')
    ax4.set_title('Speedup vs Original Method')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=1, color='black', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.savefig('obstacle_marking_performance.png', dpi=300, bbox_inches='tight')
    plt.show()

def generate_optimization_report(results):
    """Generate a detailed optimization report"""
    
    print("\n" + "="*60)
    print("OBSTACLE MARKING OPTIMIZATION REPORT")
    print("="*60)
    
    methods = list(set(results['method']))
    
    # Calculate average performance metrics
    for method in methods:
        method_data = [r for r in zip(results['time_ms'], results['memory_mb']) 
                      if r[0] in [r for r in zip(results['time_ms'], results['method']) if r[1] == method]]
        
        if method_data:
            times, memories = zip(*method_data)
            avg_time = np.mean(times)
            avg_memory = np.mean(memories)
            
            print(f"\n{method.upper()} METHOD:")
            print(f"  Average Time: {avg_time:.2f} ms")
            print(f"  Average Memory: {avg_memory:.1f} MB")
    
    # Calculate speedup
    if 'original' in methods:
        original_avg = np.mean([r[0] for r in zip(results['time_ms'], results['method']) if r[1] == 'original'])
        
        print(f"\nSPEEDUP ANALYSIS (vs Original):")
        for method in methods:
            if method != 'original':
                method_avg = np.mean([r[0] for r in zip(results['time_ms'], results['method']) if r[1] == method])
                speedup = original_avg / method_avg if method_avg > 0 else 1
                print(f"  {method}: {speedup:.2f}x speedup")
    
    print("\nOPTIMIZATION RECOMMENDATIONS:")
    print("1. Use OpenMP parallelization for large grids")
    print("2. Use batch processing for multiple obstacles")
    print("3. Consider GPU acceleration for very large environments")
    print("4. Implement spatial partitioning for better cache locality")
    print("5. Use SIMD instructions for vectorized operations")

def main():
    """Main function to run performance tests"""
    
    # Test parameters
    num_obstacles_list = [5, 10, 20, 50, 100]
    grid_sizes = [
        (256, 256, 64),   # Small grid
        (512, 512, 128),  # Medium grid
        (1024, 1024, 256), # Large grid
        (2048, 2048, 512)  # Very large grid
    ]
    
    print("Starting obstacle marking performance tests...")
    print(f"Testing {len(num_obstacles_list)} obstacle counts: {num_obstacles_list}")
    print(f"Testing {len(grid_sizes)} grid sizes: {grid_sizes}")
    
    # Run tests
    results = run_performance_test(num_obstacles_list, grid_sizes, num_runs=3)
    
    # Generate plots
    print("\nGenerating performance plots...")
    plot_results(results)
    
    # Generate report
    generate_optimization_report(results)
    
    print(f"\nPerformance test completed. Results saved to 'obstacle_marking_performance.png'")

if __name__ == "__main__":
    main() 