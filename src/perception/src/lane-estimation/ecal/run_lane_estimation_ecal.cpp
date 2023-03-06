#include "lane_estimation_node_ecal.hpp"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>


#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>

int main(int argc, char *argv[])
{
    eCAL::Initialize(argc, argv, "Lane-Estimation");

    LaneEstimationModule laneestimation;
    
    while (eCAL::Ok())
    {
        #ifdef RENDER_PERCEPTION_LANE_ESTIMATION
        laneestimation.render();
        #else
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        #endif
    }

    eCAL::Finalize();

    return 0;
}