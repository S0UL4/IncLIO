#include <spdlog/spdlog.h>
#include <pcl/io/pcd_io.h>

#include "common/utils/point_types.hpp"
#include "tools/ui/pangolin_window.h"

int main(int argc, char** argv) {
    std::string source = "./data/test.pcd";
    if (argc > 1) {
        source = argv[1];
    }

    IncLIO::ui::PangolinWindow ui;
    ui.Init();

    IncLIO::CloudPtr cloud(new IncLIO::PointCloudType);
    pcl::io::loadPCDFile(source, *cloud);

    spdlog::info("Loaded {} points, updating scan", cloud->size());
    ui.UpdateScan(cloud, SE3());

    spdlog::info("Displaying for 60 seconds...");
    sleep(60);
    ui.Quit();

    return 0;
}
