#include "common.h"

const pin_pair * pin_pair_init(uint8_t first, uint8_t second){
    pin_pair * pp;
    pp = (pin_pair *) malloc(sizeof(pin_pair));

    pp->first = first;
    pp->second = second;
    
    return pp;
}