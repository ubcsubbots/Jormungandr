#include <thread>
#include <chrono>
#include <iostream>

int main(int argc, char** argv){
  std::chrono::seconds sleep_time(15);
  std::this_thread::sleep_for(sleep_time);
  std::srand(std::time(NULL));
  int rand_int = std::rand();
  if (rand_int%3 == 1)
    std::cout<< "PASSED" <<std::endl;
  else if (rand_int%3 == 2 )
    std::cout<<"FAILED"<<std::endl;
  else
    std::cout<<"FAILED"<<std::endl;
  return 0;
}
