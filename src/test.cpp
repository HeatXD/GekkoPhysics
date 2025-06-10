#include <chrono>
#include "gekko_math.h"
#include "gekko_ds.h"
#include "gekko_physics.h"
using namespace GekkoPhysics;

int main()
{
    using namespace GekkoDS;
    using namespace GekkoMath;
    using namespace std::chrono;

    World world1, world2;
    MemStream stream2;

    for (size_t i = 0; i < 100; i++) {
        auto body_id = world1.CreateBody();
        for (size_t i = 0; i < 8; i++) {
            auto group_id = world1.AddShapeGroup(body_id);
            for (size_t i = 0; i < 8; i++) {
                auto shape_id = world1.AddShape(group_id, Shape::Sphere);
            }
        }
    }

    for (Identifier i = 10; i < 60; i++) {
        world1.RemoveBody(i);
    }

    auto t0 = high_resolution_clock::now();
    world1.Save(stream2);
    auto t1 = high_resolution_clock::now();

    stream2.rewind();

    auto t2 = high_resolution_clock::now();
    world2.Load(stream2);
    auto t3 = high_resolution_clock::now();

    auto save_us = duration_cast<microseconds>(t1 - t0).count();
    auto load_us = duration_cast<microseconds>(t3 - t2).count();

    // Only print after all timing is done
    std::cout << "Save: " << save_us << " us\n";
    std::cout << "Load: " << load_us << " us\n";

    return 0;
}
