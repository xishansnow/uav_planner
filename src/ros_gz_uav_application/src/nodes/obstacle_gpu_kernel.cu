#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/execution_policy.h>

// 障碍物数据结构（GPU版本）
struct GPUMarker {
    int x, y, z;
    unsigned char cost;
};

// CUDA核函数：标记长方体障碍物
__global__ void markBoxObstacleKernel(
    unsigned char* voxel_grid,
    int grid_x, int grid_y, int grid_z,
    double resolution_xy, double resolution_z,
    double center_x, double center_y, double center_z,
    double size_x, double size_y, double size_z,
    double buffer_size,
    unsigned char cost_value
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;
    
    if (x >= grid_x || y >= grid_y || z >= grid_z) return;
    
    // 计算世界坐标
    double world_x = x * resolution_xy;
    double world_y = y * resolution_xy;
    double world_z = z * resolution_z;
    
    // 计算障碍物边界
    double min_x = center_x - size_x/2 - buffer_size;
    double max_x = center_x + size_x/2 + buffer_size;
    double min_y = center_y - size_y/2 - buffer_size;
    double max_y = center_y + size_y/2 + buffer_size;
    double min_z = center_z - size_z/2 - buffer_size;
    double max_z = center_z + size_z/2 + buffer_size;
    
    // 检查是否在障碍物内
    if (world_x >= min_x && world_x <= max_x &&
        world_y >= min_y && world_y <= max_y &&
        world_z >= min_z && world_z <= max_z) {
        
        int index = z * grid_x * grid_y + y * grid_x + x;
        voxel_grid[index] = cost_value;
    }
}

// CUDA核函数：标记圆柱体障碍物
__global__ void markCylinderObstacleKernel(
    unsigned char* voxel_grid,
    int grid_x, int grid_y, int grid_z,
    double resolution_xy, double resolution_z,
    double center_x, double center_y, double center_z,
    double radius, double height,
    double buffer_size,
    unsigned char cost_value
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;
    
    if (x >= grid_x || y >= grid_y || z >= grid_z) return;
    
    // 计算世界坐标
    double world_x = x * resolution_xy;
    double world_y = y * resolution_xy;
    double world_z = z * resolution_z;
    
    // 计算XY平面距离
    double dx = world_x - center_x;
    double dy = world_y - center_y;
    double dist_sq = dx*dx + dy*dy;
    double radius_sq = (radius + buffer_size) * (radius + buffer_size);
    
    // 检查Z轴范围
    double min_z = center_z - height/2 - buffer_size;
    double max_z = center_z + height/2 + buffer_size;
    
    // 检查是否在圆柱内
    if (dist_sq <= radius_sq && world_z >= min_z && world_z <= max_z) {
        int index = z * grid_x * grid_y + y * grid_x + x;
        voxel_grid[index] = cost_value;
    }
}

// CUDA核函数：标记球体障碍物
__global__ void markSphereObstacleKernel(
    unsigned char* voxel_grid,
    int grid_x, int grid_y, int grid_z,
    double resolution_xy, double resolution_z,
    double center_x, double center_y, double center_z,
    double radius,
    double buffer_size,
    unsigned char cost_value
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;
    
    if (x >= grid_x || y >= grid_y || z >= grid_z) return;
    
    // 计算世界坐标
    double world_x = x * resolution_xy;
    double world_y = y * resolution_xy;
    double world_z = z * resolution_z;
    
    // 计算3D距离
    double dx = world_x - center_x;
    double dy = world_y - center_y;
    double dz = world_z - center_z;
    double dist_sq = dx*dx + dy*dy + dz*dz;
    double radius_sq = (radius + buffer_size) * (radius + buffer_size);
    
    // 检查是否在球体内
    if (dist_sq <= radius_sq) {
        int index = z * grid_x * grid_y + y * grid_x + x;
        voxel_grid[index] = cost_value;
    }
}

// 主机端包装函数
extern "C" {
    // 标记长方体障碍物
    int markBoxObstacleGPU(
        unsigned char* voxel_grid,
        int grid_x, int grid_y, int grid_z,
        double resolution_xy, double resolution_z,
        double center_x, double center_y, double center_z,
        double size_x, double size_y, double size_z,
        double buffer_size,
        unsigned char cost_value
    ) {
        // 分配GPU内存
        unsigned char* d_voxel_grid;
        size_t grid_size = grid_x * grid_y * grid_z * sizeof(unsigned char);
        cudaMalloc(&d_voxel_grid, grid_size);
        
        // 复制数据到GPU
        cudaMemcpy(d_voxel_grid, voxel_grid, grid_size, cudaMemcpyHostToDevice);
        
        // 设置CUDA网格和块大小
        dim3 block_size(16, 16, 4);
        dim3 grid_size_cuda(
            (grid_x + block_size.x - 1) / block_size.x,
            (grid_y + block_size.y - 1) / block_size.y,
            (grid_z + block_size.z - 1) / block_size.z
        );
        
        // 启动核函数
        markBoxObstacleKernel<<<grid_size_cuda, block_size>>>(
            d_voxel_grid, grid_x, grid_y, grid_z,
            resolution_xy, resolution_z,
            center_x, center_y, center_z,
            size_x, size_y, size_z,
            buffer_size, cost_value
        );
        
        // 复制结果回主机
        cudaMemcpy(voxel_grid, d_voxel_grid, grid_size, cudaMemcpyDeviceToHost);
        
        // 清理GPU内存
        cudaFree(d_voxel_grid);
        
        return 0;
    }
    
    // 标记圆柱体障碍物
    int markCylinderObstacleGPU(
        unsigned char* voxel_grid,
        int grid_x, int grid_y, int grid_z,
        double resolution_xy, double resolution_z,
        double center_x, double center_y, double center_z,
        double radius, double height,
        double buffer_size,
        unsigned char cost_value
    ) {
        // 分配GPU内存
        unsigned char* d_voxel_grid;
        size_t grid_size = grid_x * grid_y * grid_z * sizeof(unsigned char);
        cudaMalloc(&d_voxel_grid, grid_size);
        
        // 复制数据到GPU
        cudaMemcpy(d_voxel_grid, voxel_grid, grid_size, cudaMemcpyHostToDevice);
        
        // 设置CUDA网格和块大小
        dim3 block_size(16, 16, 4);
        dim3 grid_size_cuda(
            (grid_x + block_size.x - 1) / block_size.x,
            (grid_y + block_size.y - 1) / block_size.y,
            (grid_z + block_size.z - 1) / block_size.z
        );
        
        // 启动核函数
        markCylinderObstacleKernel<<<grid_size_cuda, block_size>>>(
            d_voxel_grid, grid_x, grid_y, grid_z,
            resolution_xy, resolution_z,
            center_x, center_y, center_z,
            radius, height,
            buffer_size, cost_value
        );
        
        // 复制结果回主机
        cudaMemcpy(voxel_grid, d_voxel_grid, grid_size, cudaMemcpyDeviceToHost);
        
        // 清理GPU内存
        cudaFree(d_voxel_grid);
        
        return 0;
    }
    
    // 标记球体障碍物
    int markSphereObstacleGPU(
        unsigned char* voxel_grid,
        int grid_x, int grid_y, int grid_z,
        double resolution_xy, double resolution_z,
        double center_x, double center_y, double center_z,
        double radius,
        double buffer_size,
        unsigned char cost_value
    ) {
        // 分配GPU内存
        unsigned char* d_voxel_grid;
        size_t grid_size = grid_x * grid_y * grid_z * sizeof(unsigned char);
        cudaMalloc(&d_voxel_grid, grid_size);
        
        // 复制数据到GPU
        cudaMemcpy(d_voxel_grid, voxel_grid, grid_size, cudaMemcpyHostToDevice);
        
        // 设置CUDA网格和块大小
        dim3 block_size(16, 16, 4);
        dim3 grid_size_cuda(
            (grid_x + block_size.x - 1) / block_size.x,
            (grid_y + block_size.y - 1) / block_size.y,
            (grid_z + block_size.z - 1) / block_size.z
        );
        
        // 启动核函数
        markSphereObstacleKernel<<<grid_size_cuda, block_size>>>(
            d_voxel_grid, grid_x, grid_y, grid_z,
            resolution_xy, resolution_z,
            center_x, center_y, center_z,
            radius,
            buffer_size, cost_value
        );
        
        // 复制结果回主机
        cudaMemcpy(voxel_grid, d_voxel_grid, grid_size, cudaMemcpyDeviceToHost);
        
        // 清理GPU内存
        cudaFree(d_voxel_grid);
        
        return 0;
    }
} 