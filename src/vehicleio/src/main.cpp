// #include "socketcan_cpp/socketcan_cpp.h"

#include "cpp_headers.hpp"

#include "vehicleio.hpp"

//commons
#include "commons.hpp"

//DS
#include "control/control-DS.hpp"
#include "vehicleio/vehicle_DS.hpp"

using namespace std;

void foo(vehicleIO& o)
{
    o.send_to_pix();
}

int main(int argc, char *argv[])
{   
    
    eCAL::Initialize(argc, argv, "VehicleIO");
    vehicleIO pix;
  
    thread t(foo,std::ref(pix));

    while (eCAL::Ok())
    {   
        FEEDBACK feed = pix.feed_from_pix_control();
        pix.send_to_control(feed);
        // pix.debug();
    }

    eCAL::Finalize();
    t.join();
    

    return 0;
} 