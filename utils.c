void wait(unsigned volatile long how_long){  // ### Зачем здесь volatile?
 for(; how_long > 0; how_long--){}
}
