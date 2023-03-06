#include "segmentation_node_ecal.hpp"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>


int main(int argc, char *argv[])
{
    eCAL::Initialize(argc, argv, "Segmentation");

    SegmentationModule segmentation;
    
    while (eCAL::Ok())
    {
        #ifdef RENDER_PERCEPTION_SEGMENTATION
        segmentation.render();
        #else
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        #endif
    }

    eCAL::Finalize();

    return 0;
}