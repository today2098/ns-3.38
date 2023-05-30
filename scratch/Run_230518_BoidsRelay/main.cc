#include <iostream>
#include <string>

int main() {
    system("./ns3 run scratch/230518_BoidsRelay/main.cc -- --enemy --ws=0.0 --wa=0.0 --wc=0.0 --dist=35.0 --flag");
    for(double a = 0.1; a < 1.0; a += 0.2) {
        for(double b = 0.1; b < 1.0; b += 0.2) {
            for(double c = 0.1; c < 1.0; c += 0.2) {
                std::string cmd = "./ns3 run scratch/230518_BoidsRelay/main.cc -- --enemy --ws=" + std::to_string(a) + " --wa=" + std::to_string(b) + " --wc=" + std::to_string(c) + " --dist=35.0 --flag";
                system(cmd.c_str());
            }
        }
    }
}
