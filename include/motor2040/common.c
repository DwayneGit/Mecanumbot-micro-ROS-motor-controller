#include "common.h"

const struct pin_pair * pin_pair_init(uint8_t first, uint8_t second){
    struct pin_pair * pp;
    pp = (struct pin_pair *) malloc(sizeof(struct pin_pair));

    pp->first = first;
    pp->second = second;
    
    return pp;
}