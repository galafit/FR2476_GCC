void wait(unsigned volatile long how_long){
 for(; how_long > 0; how_long--){}
}
