#include <thread>
#include <chrono>
#include <iostream>

int main(int argc, char** argv){
  std::chrono::seconds sleep_time(5);
  std::this_thread::sleep_for(sleep_time);
  std::srand(std::time(NULL));
  int rand_int = std::rand();
  if (rand_int%3 == 1)
    std::cout<<" PASSED: SUCCESFULLY PASSED THROUGH GATE"<<std::endl;
  else if (rand_int%3 == 2 )
    std::cout<<" FAILED: DID NOT PASS THROUGH GATE IN TIME"<<std::endl;
  else
    std::cout<<" FAILED: HIT GATE "<<std::endl;
  return 0;
}
