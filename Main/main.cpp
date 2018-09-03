#include"ImgProdCons.h"
#include<thread>

using namespace std;

bool rm::ImgProdCons::_quit_flag = false;

int main()
{
    rm::ImgProdCons imgProdCons;

    imgProdCons.init();

    std::thread produceThread(&rm::ImgProdCons::produce, &imgProdCons);
    std::thread consumeThread(&rm::ImgProdCons::consume, &imgProdCons);
    std::thread senseThread(&rm::ImgProdCons::sense, &imgProdCons);

    produceThread.join();
    consumeThread.join();
    senseThread.join();

    return 0;
}
