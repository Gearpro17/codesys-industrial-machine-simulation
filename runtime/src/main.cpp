#include <chrono>
#include <iostream>

int main() {
    using namespace std::chrono;
    const auto t0 = steady_clock::now();
    std::cout << "Palletizer runtime: toolchain OK\n";
    const auto dt = duration_cast<microseconds>(steady_clock::now() - t0);
    std::cout << "Elapsed: " << dt.count() << " us\n";
}