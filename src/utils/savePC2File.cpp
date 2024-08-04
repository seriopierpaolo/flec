#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <flec/datastructure.h>  // Ensure this includes the custom data structures

// Define an alias for convenience
using PointCloud = pcl::PointCloud<PointType>;

void saveData(const TfAccumulator& accumulator, const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);
    if (!outFile) {
        throw std::runtime_error("Could not open file for writing.");
    }

    // Save the number of TfPairs in the macroBatch
    std::size_t num_pairs = accumulator.macroBatch.currentBatch.size();
    outFile.write(reinterpret_cast<const char*>(&num_pairs), sizeof(num_pairs));

    // Iterate over each TfPair in the macroBatch
    for (const auto& tfPair : accumulator.macroBatch.currentBatch) {
        // Save transformation_F
        outFile.write(reinterpret_cast<const char*>(tfPair.transformation_F.data()), sizeof(tfPair.transformation_F));

        // Save point cloud pcl_F
        std::size_t pcl_F_size = tfPair.pcl_F.points.size();
        outFile.write(reinterpret_cast<const char*>(&pcl_F_size), sizeof(pcl_F_size));
        for (const auto& point : tfPair.pcl_F.points) {
            outFile.write(reinterpret_cast<const char*>(&point.x), sizeof(point.x));
            outFile.write(reinterpret_cast<const char*>(&point.y), sizeof(point.y));
            outFile.write(reinterpret_cast<const char*>(&point.z), sizeof(point.z));
        }

        // Save transformation_S
        outFile.write(reinterpret_cast<const char*>(tfPair.transformation_S.data()), sizeof(tfPair.transformation_S));

        // Save point cloud pcl_S
        std::size_t pcl_S_size = tfPair.pcl_S.points.size();
        outFile.write(reinterpret_cast<const char*>(&pcl_S_size), sizeof(pcl_S_size));
        for (const auto& point : tfPair.pcl_S.points) {
            outFile.write(reinterpret_cast<const char*>(&point.x), sizeof(point.x));
            outFile.write(reinterpret_cast<const char*>(&point.y), sizeof(point.y));
            outFile.write(reinterpret_cast<const char*>(&point.z), sizeof(point.z));
        }
    }

    outFile.close();
}
