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

    SparseSet<int16_t, uint32_t> set1a;
    SparseSet<int16_t, uint32_t> set1b;
    for (int i = 0; i < 1000; i++) {
        set1a.insert(i + 666);
        set1b.insert(i + 777);
    }

    MemStream stream;

    auto t0 = high_resolution_clock::now();
    set1a.save(stream);
    set1b.save(stream);
    auto t1 = high_resolution_clock::now();

    stream.rewind();

    SparseSet<int16_t, uint32_t> set2a;
    SparseSet<int16_t, uint32_t> set2b;

    auto t2 = high_resolution_clock::now();
    set2a.load(stream);
    set2b.load(stream);
    auto t3 = high_resolution_clock::now();

    auto save_us = duration_cast<microseconds>(t1 - t0).count();
    auto load_us = duration_cast<microseconds>(t3 - t2).count();

    // Only print after all timing is done
    std::cout << "Save: " << save_us << " us\n";
    std::cout << "Load: " << load_us << " us\n";

    World world;
    MemStream stream2;

    t0 = high_resolution_clock::now();
    world.Save(stream2);
    t1 = high_resolution_clock::now();

    stream2.rewind();

    t2 = high_resolution_clock::now();
    world.Load(stream2);
    t3 = high_resolution_clock::now();

    save_us = duration_cast<microseconds>(t1 - t0).count();
    load_us = duration_cast<microseconds>(t3 - t2).count();

    // Only print after all timing is done
    std::cout << "Save: " << save_us << " us\n";
    std::cout << "Load: " << load_us << " us\n";
    return 0;
}
